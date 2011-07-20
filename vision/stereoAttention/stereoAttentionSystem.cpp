/*
 *  stereoAttentionSystem.cpp
 *
 * 	Logan Niehaus
 * 	7/19/11
 * 	rudimentary stereo attention system controller module. takes in a
 * 	composite (weighted sum) ego-sphere salience map for both left and right channels.
 *  the module switches b/w two basic behavioral modes: first, the module picks
 *  the most salient point from either l/r ego image. an 'estimate' focus is made
 *  by directing gaze at its az/el. after this, the unwarped salience maps are
 *  used to more accurately fixate on the salient region. an IOR point is registered
 *  for this region, and control is switched back to the first state.
 *
 *
 *  inputs:
 *  	/stereoAttention/ego:l	-- ego-mapped left weighted salience sum map
 *  	/stereoAttention/ego:r	-- ego-mapped right weighted salience sum map
 *  	/stereoAttention/sal:l	-- unwarped left weighted salience sum
 *  	/stereoAttention/sal:r	-- unwarped right weighted salience sum
 *  	/iKinGazeCtrl/head/		-- not connected, but must be running in order to function
 *
 *
 *  params: ([R]equired/[D]efault <Value>/[O]ptional)
 *
 *		azrange					-- total range of azimuth angles on incoming maps (R, ex: 0 180)
 *		elrange					-- total range of elevation angles on incoming maps(R, ex: -90 90)
 *  	tol						-- stereo convergence tolerance (D 5.0)
 *  	nt, et					-- solver trajectory times; lower values -> faster head movement (D 1.0, 0.5 for icub)
 *  	beta					-- IOR object decay time
 *  	iorthr					-- threshold applied to determine what pixels belong to the 'object' to be inhibited
 *  	name					-- module port basename (D /stereoAttention)
 *  	verbose					-- setting flag makes the module shoot debug info to stdout
 *
 *  outputs:
 *  	/stereoAttention/foc:t	-- trigger signal consisting of a junk bottle. produced in fixation state
 *  								once the target point has been converged upon.
 *
 *  TODO:
 *  	state 0 (minus IOR) needs to be tests
 *  	state 1 behavior unfinished
 *  	IOR unimplemented
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>

#include <cv.h>

#include <string>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

YARP_DECLARE_DEVICES(icubmod);

class stereoAttentionThread : public Thread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelFloat> > *portEgoL;
	BufferedPort<ImageOf<PixelFloat> > *portEgoR;
	BufferedPort<ImageOf<PixelFloat> > *portSalL;
	BufferedPort<ImageOf<PixelFloat> > *portSalR;

	//ikingaze objects and params
	PolyDriver clientGazeCtrl;
	IGazeControl *igaze;
	double neckTT, eyeTT;

	//ego map params
	double azlo, azhi;
	double ello, elhi;

	int state;


public:

	stereoAttentionThread(ResourceFinder &_rf) : rf(_rf)
	{ }

	void topDownGazeCmd(yarp::sig::Vector &ang, bool wait = false) {

		igaze->lookAtAbsAngles(ang);
		if (wait)
			igaze->waitMotionDone(0.1, 10.0);

	}

	virtual bool threadInit()
	{

		name=rf.check("name",Value("stereoAttention")).asString().c_str();
		neckTT = rf.check("nt",Value(1.0)).asDouble();
		eyeTT = rf.check("et",Value(0.5)).asDouble();

		//get output map size
		Bottle arng = rf.findGroup("azrange");
		Bottle erng = rf.findGroup("elrange");
		if (arng.isNull() || arng.size() < 3 || erng.isNull() || erng.size() < 3) {
			return false;
		} else {
			azlo = arng.get(1).asDouble(); azhi = arng.get(2).asDouble();
			ello = erng.get(1).asDouble(); elhi = erng.get(2).asDouble();
		}

		//open up ports
		portEgoL=new BufferedPort<ImageOf<PixelFloat> >;
		string portEgolName="/"+name+"/ego:l";
		portEgoL->open(portEgolName.c_str());

		portEgoR=new BufferedPort<ImageOf<PixelFloat> >;
		string portEgorName="/"+name+"/ego:r";
		portEgoR->open(portEgorName.c_str());

		portSalL=new BufferedPort<ImageOf<PixelFloat> >;
		string portSallName="/"+name+"/sal:l";
		portSalL->open(portSallName.c_str());

		portSalR=new BufferedPort<ImageOf<PixelFloat> >;
		string portSalrName="/"+name+"/sal:r";
		portSalR->open(portSalrName.c_str());

		//start up gaze control client interface
		Property option("(device gazecontrollerclient)");
		option.put("remote","/iKinGazeCtrl");
		option.put("local","/client/gaze");
		clientGazeCtrl.open(option);
		igaze=NULL;
		if (clientGazeCtrl.isValid()) {
			clientGazeCtrl.view(igaze);
		} else {
			printf("could not initialize gaze control interface, failing...\n");
			return false;
		}

		//set gaze control interface params
		igaze->setNeckTrajTime(neckTT);
		igaze->setEyesTrajTime(eyeTT);
		igaze->bindNeckPitch(-30,30);
		igaze->bindNeckYaw(-25,25);
		igaze->bindNeckRoll(-10,10);

		//state = 0;
		state = 1;

		return true;

	}

	virtual void run()
	{

		Mat * Esl, * Esr, * iorMaskL, * iorMaskR;

		while (isStopping() != true) {


			//state 0: find most salient point, perform pre-focus
			if (!state) {

				ImageOf<PixelFloat> *pEgoL = portEgoL->read(true);
				ImageOf<PixelFloat> *pEgoR = portEgoR->read(true);
				Esl = new Mat(pEgoL->height(), pEgoL->width(), CV_32F, (void *)pEgoL->getRawImage());
				Esr = new Mat(pEgoR->height(), pEgoR->width(), CV_32F, (void *)pEgoR->getRawImage());


				//apply all inhibition of return events to the salience images
				iorMaskL = new Mat(pEgoL->height(), pEgoL->width(), CV_32F);
				iorMaskR = new Mat(pEgoR->height(), pEgoR->width(), CV_32F);


				//find point of highest salience across both pairs
				Point * mxDxL = new Point;
				Point * mxDxR = new Point;
				double * mxVal = new double[2];
				minMaxLoc(*Esl, NULL, mxVal, NULL, mxDxL, *iorMaskL);
				minMaxLoc(*Esr, NULL, mxVal+1, NULL, mxDxR, *iorMaskR);
				if (mxVal[1] > mxVal[0])
					mxDxL = mxDxR;

				//find the az/el location of the point
				double az, el;
				az = azlo + ((azhi-azlo)/(float)pEgoL->width())*(mxDxL->x);
				el = elhi - ((elhi-ello)/(float)pEgoL->height())*(mxDxL->y);

				//move head to that az/el
				yarp::sig::Vector tang(3);
				tang[0] = az; tang[1] = el; tang[2] = 1;
				igaze->lookAtAbsAngles(tang);

				//wait for head move to complete, then transition to local search state
				igaze->waitMotionDone(0.1, 10.0); //wait a max of 10s for motion to finish
				state = 1;

				delete Esl, Esr;
				delete iorMaskL, iorMaskR;
				delete mxDxL, mxVal;


			}
			//state 1: lock in on most salient point in both images, track while verging
			else {

				Time::delay(0.5);


			}

		}

	}

	virtual void threadRelease()
	{

		clientGazeCtrl.close();

		portEgoL->interrupt();
		portEgoR->interrupt();
		portSalL->interrupt();
		portSalR->interrupt();

		portEgoL->close();
		portEgoR->close();
		portSalL->close();
		portSalR->close();

		delete portEgoL, portEgoR, portSalL, portSalR;

	}

};

class stereoAttentionModule: public RFModule
{
protected:

	stereoAttentionThread *thr;
	Port * rpcPort;
	string name;

public:

	stereoAttentionModule() { }

	bool respond(const Bottle& command, Bottle& reply) {

		string msg(command.get(0).asString().c_str());
		if (msg == "ang") {
			if (command.size() < 2) {
				reply.add(-1);
			}
			else {
				yarp::sig::Vector azelr(3);
				azelr[0] = command.get(1).asDouble();
				azelr[1] = command.get(2).asDouble();
				azelr[2] = command.get(3).asDouble();
				thr->topDownGazeCmd(azelr,true);
				reply.add(1);
			}
		}
		else {
			reply.add(-1);
		}

		return true;

	}

	virtual bool configure(ResourceFinder &rf)
	{

		//set up the rpc port
		name=rf.check("name",Value("stereoAttention")).asString().c_str();
		rpcPort = new Port;
		string portRpcName="/"+name+"/rpc";
		rpcPort->open(portRpcName.c_str());
		attach(*rpcPort);


		thr=new stereoAttentionThread(rf);
		if (!thr->start())
		{
			delete thr;
			return false;
		}

		return true;
	}

	virtual bool close()
	{
		thr->stop();
		delete thr;

		return true;
	}

	virtual double getPeriod()    { return 1.0;  }
	virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{

	YARP_REGISTER_DEVICES(icubmod);

	Network yarp;

	if (!yarp.checkNetwork())
		return -1;

	ResourceFinder rf;

	rf.configure("ICUB_ROOT",argc,argv);

	stereoAttentionModule mod;

	return mod.runModule(rf);
}



