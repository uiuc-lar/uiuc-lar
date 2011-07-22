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
 *  used to more accurately fixate on the salient region. to do this, the maps are
 *  rectified using the current kinematic configuration, and a windowed search is
 *  performed in the neighborhood of the salient region to register the correspondence.
 *  the gaze is then directed towards the xyz location triangulated from the match.
 *  a signal containing the object locations in each image [ul vl ur vr] is
 *  generated for the IOR module. fixation lasts some user-defined amount of time
 *  and then the algorithm returns to the global search state
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
 *  	wsize					-- scanning window size for stereo matching (D 25)
 *  	nlags					-- number of lags for stereo matching. will scan from -nlags to +nlags (D 50)
 *  	fixtime					-- object fixation duration, in seconds (D 5s)
 *  	<camera parameters>		-- intrinsic calibration params for each camera under [CAMERA_CALIBRATION_LEFT/RIGHT].
 *										see iKinGazeCtrl docs for example of how to do this
 *		voffset					-- vertical offset from left to right image in pixels; this adjusts for
 *									vertical camera misalignment on the icub (ex -20 shifts the right image up by 20)
 *  	name					-- module port basename (D /stereoAttention)
 *  	verbose					-- setting flag makes the module shoot debug info to stdout
 *
 *  outputs:
 *  	/stereoAttention/fxl:o	-- bottle containing fixation location in terms of image coordinates [ul vl ur vr].
 *  								generated when second-pass fixation point is determined
 *
 *  TODO:
 *  	complete system behavior needs testing
 *  	also need to basic stereo function using gaze control interface instead of joint port
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
#include <yarp/math/SVD.h>

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

#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::ctrl;

YARP_DECLARE_DEVICES(icubmod);

#define PI 3.14159


class stereoAttentionThread : public Thread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelFloat> > *portEgoL;
	BufferedPort<ImageOf<PixelFloat> > *portEgoR;
	BufferedPort<ImageOf<PixelFloat> > *portSalL;
	BufferedPort<ImageOf<PixelFloat> > *portSalR;
	Port *portFxlOut;

	//ikingaze objects and params
	PolyDriver clientGazeCtrl;
	IGazeControl *igaze;
	double neckTT, eyeTT;

	//ego map params
	double azlo, azhi;
	double ello, elhi;

	//camera projection matrix/params
	Mat Pl;
	Mat Pr;
	double fxl, fyl, cxl, cyl;
	double fxr, fyr, cxr, cyr;
	int voffset;

	//aux. data for stereo processing
	Mat R1, R2, P1, P2, Q;
	Mat R, Rl, Rr;
	Mat mpxL, mpyL, mpxR, mpyR;
	Point * mxDxL, * mxDxR;
	int wsize, nlags;

	double fT;
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
		fT = rf.check("fixtime",Value(5.0)).asDouble();
		voffset = rf.check("voffset",Value(0)).asInt();
		wsize = (int)rf.check("wsize",Value(25)).asInt()/2;
		nlags = rf.check("nlags",Value(50)).asInt();

		//get output map size
		Bottle arng = rf.findGroup("azrange");
		Bottle erng = rf.findGroup("elrange");
		if (arng.isNull() || arng.size() < 3 || erng.isNull() || erng.size() < 3) {
			return false;
		} else {
			azlo = arng.get(1).asDouble(); azhi = arng.get(2).asDouble();
			ello = erng.get(1).asDouble(); elhi = erng.get(2).asDouble();
		}

		//get camera calibration parameters
		Pl = Mat::eye(3, 3, CV_64F);
		Pr = Mat::eye(3, 3, CV_64F);
		Bottle ipsL = rf.findGroup("CAMERA_CALIBRATION_LEFT");
		Bottle ipsR = rf.findGroup("CAMERA_CALIBRATION_RIGHT");
		if (ipsL.size() && ipsR.size()) {

			Pl.at<double>(0,2) = cxl = ipsL.find("w").asDouble()/2.0;
			Pl.at<double>(1,2) = cyl = ipsL.find("h").asDouble()/2.0;
			Pl.at<double>(0,0) = fxl = ipsL.find("fx").asDouble();
			Pl.at<double>(1,1) = fyl = ipsL.find("fy").asDouble();

			Pr.at<double>(0,2) = cxr = ipsR.find("w").asDouble()/2.0;
			Pr.at<double>(1,2) = cyr = ipsR.find("h").asDouble()/2.0;
			Pr.at<double>(0,0) = fxr = ipsR.find("fx").asDouble();
			Pr.at<double>(1,1) = fyr = ipsR.find("fy").asDouble();

		}
		else {

			fprintf(stdout,"Could not find calibration parameters for one of the cameras\n");
			return false;

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

		portFxlOut=new Port;
		string portFxlName="/"+name+"/fxl:o";
		portFxlOut->open(portFxlName.c_str());
		portFxlOut->setTimeout(0.5);

		//start up gaze control client interface
		Property option("(device gazecontrollerclient)");
		option.put("remote","/iKinGazeCtrl");
		option.put("local","/client/gaze");
		clientGazeCtrl.open(option);
		igaze=NULL;
		if (clientGazeCtrl.isValid()) {

			clientGazeCtrl.view(igaze);

			//set gaze control interface params
			igaze->setNeckTrajTime(neckTT);
			igaze->setEyesTrajTime(eyeTT);
			igaze->bindNeckPitch(-30,30);
			igaze->bindNeckYaw(-25,25);
			igaze->bindNeckRoll(-10,10);

		} else {

			printf("could not initialize gaze control interface, failing...\n");
			return false;

		}

		//state = 0;
		state = 1;

		return true;

	}

	virtual void run()
	{

		Mat * Esl, * Esr, * Sl, * Sr, * Scl, * Scr;

		while (isStopping() != true) {


			//state 0: find most salient point, perform pre-focus
			if (!state) {

				ImageOf<PixelFloat> *pEgoL = portEgoL->read(true);
				ImageOf<PixelFloat> *pEgoR = portEgoR->read(true);
				Esl = new Mat(pEgoL->height(), pEgoL->width(), CV_32F, (void *)pEgoL->getRawImage());
				Esr = new Mat(pEgoR->height(), pEgoR->width(), CV_32F, (void *)pEgoR->getRawImage());


				//find point of highest salience across both pairs
				mxDxL = new Point;
				mxDxR = new Point;
				double * mxVal = new double[2];
				minMaxLoc(*Esl, NULL, mxVal, NULL, mxDxL);
				minMaxLoc(*Esr, NULL, mxVal+1, NULL, mxDxR);
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
				delete mxDxL, mxVal;


			}
			//state 1: lock in on most salient point in both images, track while verging
			else {


				//get the normal salience maps after pre-focusing
				ImageOf<PixelFloat> *pSalL = portSalL->read(true);
				ImageOf<PixelFloat> *pSalR = portSalR->read(true);

				Sl = new Mat(pSalL->height(), pSalL->width(), CV_32F, (void *)pSalL->getRawImage());
				Sr = new Mat(pSalR->height(), pSalR->width(), CV_32F, (void *)pSalR->getRawImage());
				Scl = new Mat(pSalL->height(), pSalL->width(), CV_32F);
				Scr = new Mat(pSalR->height(), pSalR->width(), CV_32F);

				//get head and l/r eye matrices, assuming fixed torso
				Matrix Hl, Hr, H;
				Mat R(3,3, CV_64F);
				vector<double> T(3);
				yarp::sig::Vector eo, ep;
				igaze->getLeftEyePose(eo, ep);
				Hl = axis2dcm(ep);
				Hl(0,3) = eo[0]; Hl(1,3) = eo[1]; Hl(2,3) = eo[2];
				igaze->getRightEyePose(eo, ep);
				Hr = axis2dcm(ep);
				Hr(0,3) = eo[0]; Hr(1,3) = eo[1]; Hr(2,3) = eo[2];

				//get the transform matrix from the left image to the right image
				H = SE3inv(Hr)*Hl;
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 3; j++) {
						R.at<double>(i,j) = H(i,j);
					}
					T.at(i) = H(i,3);
				}

				//rectify the images
				stereoRectify(Pl, Mat::zeros(4,1,CV_64F), Pr, Mat::zeros(4,1,CV_64F), Size(cxl*2, cyl*2), R, Mat(T), R1, R2, P1, P2, Q);
				initUndistortRectifyMap(Pl, Mat(), R1, P1, Size(cxl*2, cyl*2), CV_32FC1, mpxL, mpyL);
				initUndistortRectifyMap(Pr, Mat(), R2, P2, Size(cxr*2, cyr*2), CV_32FC1, mpxR, mpyR);
				remap(*Sl, *Scl, mpxL, mpyL, INTER_LINEAR);
				remap(*Sr, *Scr, mpxR, mpyR, INTER_LINEAR);

				//locate the most salient point again
				mxDxL = new Point;
				mxDxR = new Point;
				int winimg = 0;
				double * mxVal = new double[2];
				minMaxLoc(*Scl, NULL, mxVal, NULL, mxDxL);
				minMaxLoc(*Scr, NULL, mxVal+1, NULL, mxDxR);
				if (mxVal[1] > mxVal[0]) {
					mxDxL->x = mxDxR->x; mxDxL->y = mxDxR->y;
					winimg = 1;
				}

				//adjust for vertical offset from left to right image
				int vamin, vamax, vadj;
				if (voffset < 0) {
					vamin = -voffset; vamax = Scr->rows-1; vadj = 0;
				} else if (voffset > 0) {
					vamin = 0; vamax = Scr->rows-1-voffset; vadj = voffset;
				} else {
					vamin = 0; vamax = Scr->rows-1; vadj = 0;
				}

				//calc the xcorr b/w left/right for various shifts of window around object
				Mat salImg, srcImg, corr;
				Rect salWin, srcWin;
				if (!winimg) {
					copyMakeBorder(*Scl, salImg, wsize, wsize, wsize, wsize, BORDER_CONSTANT, Scalar(0));
					copyMakeBorder(Scr->rowRange(vamin,vamax), srcImg, wsize+vadj, wsize+vamin+1, wsize+nlags, wsize+nlags, BORDER_CONSTANT, Scalar(0));
				} else {
					copyMakeBorder(Scr->rowRange(vamin,vamax), salImg, wsize+vadj, wsize+vamin, wsize, wsize, BORDER_CONSTANT, Scalar(0));
					copyMakeBorder(*Scl, srcImg, wsize, wsize, wsize+nlags, wsize+nlags, BORDER_CONSTANT, Scalar(0));
					mxDxL->y = mxDxL->y + voffset;
				}
				salWin = Rect(mxDxL->x, mxDxL->y, 2*wsize+1, 2*wsize+1);
				srcWin = Rect(mxDxL->x, mxDxL->y, 2*nlags+1, 2*wsize+1);
				matchTemplate(srcImg(srcWin), salImg(salWin), corr, CV_TM_CCORR_NORMED);

				//find max of xcorr and calculate disparity at interest point
				int mCorr; double mxCrVal = 0;
				for (int i = 0; i < corr.cols; i++) {
					if (corr.at<float>(0,i) > mxCrVal) {
						mxCrVal = corr.at<float>(0,i);
						mCorr = i;
					}
				}

				//remap corresponding point onto the unrectified image
				if (!winimg) {
					mxDxR->x = mxDxL->x+mCorr-nlags-1; mxDxR->y = mxDxL->y - voffset;
				} else {
					mxDxR->x = mxDxL->x; mxDxR->y = mxDxL->y - voffset;
					mxDxL->x = mxDxL->x+mCorr-nlags-1;
				}
				int ul = (int)mpxL.at<float>(*mxDxL);
				int vl = (int)mpyL.at<float>(*mxDxL);
				int ur = (int)mpxR.at<float>(*mxDxR);
				int vr = (int)mpyR.at<float>(*mxDxR);

				//triangulate the point (least squares)
				Matrix A(4,3);
				yarp::sig::Vector b(4);
				yarp::sig::Vector Xp(3);
				Matrix iTmp(3,4); iTmp.zero(); iTmp(2,2) = 1.0;
				iTmp(0,0) = fxl; iTmp(1,1) = fyl;
				iTmp(0,2) = cxl-ul; iTmp(1,2) = cyl-vl;
				Hl = iTmp*SE3inv(Hl);
				iTmp(0,0) = fxr; iTmp(1,1) = fyr;
				iTmp(0,2) = cxr-ur; iTmp(1,2) = cyr-vr;
				Hr = iTmp*SE3inv(Hr);
				for (int i = 0; i < 2; i++) {
					for (int j = 0; j < 3; j++) {
						A(i,j) = Hl(i,j);
						A(i+2,j) = Hr(i,j);
					}
				}
				b[0] = -Hl(0,3); b[1] = -Hl(1,3);
				b[2] = -Hr(0,3); b[3] = -Hr(1,3);
				Xp = pinv(A)*b;

				//generate signal for IOR
				Bottle fxloc;
				fxloc.addInt(ul); fxloc.addInt(vl); fxloc.addInt(ur); fxloc.addInt(vr);
				portFxlOut->write(fxloc);

				//direct gaze to fixate on point
				igaze->lookAtFixationPoint(Xp);
				igaze->waitMotionDone(0.1, 10.0); //wait a max of 10s for motion to finish
				Time::delay(fT); //wait some additional time while looking at the object

				state = 0;

				delete Sl, Sr, Scl, Scr;
				delete mxDxL, mxDxR, mxVal;


			}

		}

	}

	virtual void threadRelease()
	{

		if (clientGazeCtrl.isValid())
			clientGazeCtrl.close();

		portEgoL->interrupt();
		portEgoR->interrupt();
		portSalL->interrupt();
		portSalR->interrupt();
		portFxlOut->interrupt();

		portEgoL->close();
		portEgoR->close();
		portSalL->close();
		portSalR->close();
		portFxlOut->close();

		delete portEgoL, portEgoR, portSalL, portSalR;
		delete portFxlOut;

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



