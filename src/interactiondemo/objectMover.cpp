/*
 * Copyright (C) 2014 Logan Niehaus
 *
 * 	Author: Logan Niehaus
 * 	Email:  niehaula@gmail.com
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  objectMover.cpp
 *
 * 	Logan Niehaus
 * 	3/7/14
 * 	open-loop controller for robot to move objects around a table
 *
 *  inputs:
 *  	/stereoBlobTrack/img:l	-- robot's left camera image stream (calibrated)
 *  	/stereoBlobTrack/img:r	-- robot's right camera image stream (calibrated)
 *
 *  params: ([R]equired/[D]efault <Value>/[O]ptional)
 *  	red/green/blue/yellow	-- selected color channel to find blobs of (R)
 *
 *  outputs:
 *  	/stereoBlobTrack/img:o  -- debug image, shows right threshold image w/ target blob locations marked
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>


#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>

#include <string>
#include <time.h>
#include <stdio.h>
#include <math.h>

//namespaces
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace iCub::ctrl;


YARP_DECLARE_DEVICES(icubmod)


class ClickBuffer : public Bottle {

private:

	Semaphore mutex;

public:

	void lock()   { mutex.wait(); }
	void unlock() { mutex.post(); }

};



class ClickPort : public BufferedPort<Bottle> {

protected:

	ClickBuffer &buf;

public:

	ClickPort(ClickBuffer &_buf) : buf(_buf) { }

	//callback for incoming sequences
	virtual void onRead(Bottle& c) {

		buf.lock();
		buf.clear();
		buf.copy(c);
		buf.unlock();

	}

};

class objectMoverThread : public Thread {

protected:

	/* OBJECTS AND CONTAINERS */
	ResourceFinder &rf;
	ClickBuffer bfL, bfR;
	Vector tang, hang, aang;
	Vector objPos, targetPos;

	/* PORTS */
	ClickPort *cportL;
	ClickPort *cportR;
	Port *susPort;

	/* MOTOR INTERFACES */
	PolyDriver clientGazeCtrl;
	IGazeControl *gaze;

	PolyDriver clientArmCart;
	ICartesianControl *carm;

	PolyDriver robotTorso;
	PolyDriver robotHead;
	PolyDriver robotArm;
	IPositionControl * tPos; IEncoders * tEnc;
	IPositionControl * hPos; IEncoders * hEnc;
	IPositionControl * aPos; IEncoders * aEnc;

	/* PARAMETERS */
	string name;
	string robotname;
	string armname;
	double neckTT, eyeTT; //neck and eye trajectory times (see ikingazectrl docs)
	bool armInUse; //which arm to use 0-left, 1-right (D 0)
	double trajTime; //trajectory time of arm motions (D 0.4)
	double maxPitch; //maximum torso pitch (D 30)



public:


	objectMoverThread(ResourceFinder &_rf) : rf(_rf) {	}


	virtual bool loadParams() {

		name = rf.check("name",Value("objectMover")).asString().c_str();
		neckTT = rf.check("necktt",Value(2.0)).asDouble();
		eyeTT = rf.check("eyett",Value(1.2)).asDouble();
		trajTime = rf.check("trajtime",Value(4.0),"Solver trajectory time").asDouble();
		maxPitch = rf.check("maxpitch",Value(30.0),"Torso max pitch").asDouble();

		//get which arm to use. default to left if they didnt pass in left or right
		armname = rf.check("arm", Value("left"),"arm name").asString().c_str();
		if (armname == "right") {
			armInUse = true;
		}
		else {
			armInUse = false;
		}

		//get which robot target to use
		robotname = rf.check("robot", Value("icub"),"robot name").asString().c_str();

	}


	virtual bool threadInit() {


		/* Read in parameters from resource finder */
		loadParams();

		/* Start up gaze control client interface */
		Property option("(device gazecontrollerclient)");
		option.put("remote","/iKinGazeCtrl");
		option.put("local","/client/gaze");
		clientGazeCtrl.open(option);
		gaze=NULL;
		if (clientGazeCtrl.isValid()) {
			clientGazeCtrl.view(gaze);
		} else {
			printf("could not initialize gaze control interface, failing...\n");
			return false;
		}

		//set gaze control interface params
		gaze->setNeckTrajTime(neckTT);
		gaze->setEyesTrajTime(eyeTT);

		gaze->bindNeckPitch(-30,30);
		gaze->bindNeckYaw(-25,25);
		gaze->bindNeckRoll(-10,10);

		/* Start up arm cartesian control client interface */
		Property optiona("(device cartesiancontrollerclient)");
		if (armInUse) {
			string tmpcrname = "/" + robotname + "/cartesianController/right_arm";
			optiona.put("remote",tmpcrname.c_str());
			optiona.put("local","/cartesian_client/right_arm");
		}
		else {
			string tmpcrname = "/" + robotname + "/cartesianController/left_arm";
			optiona.put("remote",tmpcrname.c_str());
			optiona.put("local","/cartesian_client/left_arm");
		}

		clientArmCart.open(optiona);
		carm = NULL;
		if (clientArmCart.isValid())
		{
			clientArmCart.view(carm);
		} else {
			printf("could not initialize arm cartesian control interface, failing...\n");
			return false;
		}

		// set trajectory time
		carm->setTrajTime(trajTime);	//slow for safety

		// get the torso dofs
		Vector newDof, curDof;
		carm->getDOF(curDof);
		newDof=curDof;

		//enable torso pitch and yaw, also wrist movements
		newDof[0]=1;
		newDof[1]=0;
		newDof[2]=1;
		newDof[7]=1;
		newDof[8]=1;
		newDof[9]=1;

		// impose some restriction on the torso pitch
		double tpmin, tpmax;
		carm->getLimits(0,&tpmin,&tpmax);
		carm->setLimits(0,tpmin,maxPitch);

		// send the request for dofs reconfiguration
		carm->setDOF(newDof,curDof);


		string lname, rname;
		Property doption;
		doption.put("device", "remote_controlboard");

		lname = "/"+name+"/torso"; rname = "/"+robotname+"/torso";
		doption.put("local", lname.c_str());
		doption.put("remote", rname.c_str());

		robotTorso.open(doption);
		if (!robotTorso.isValid()) {
			printf("could not initialize torso control interface, failing...\n");
			return false;
		}
		robotTorso.view(tPos); robotTorso.view(tEnc);

		lname = "/"+name+"/head"; rname = "/"+robotname+"/head";
		doption.put("local", lname.c_str());
		doption.put("remote", rname.c_str());

		robotHead.open(doption);
		if (!robotHead.isValid()) {
			printf("could not initialize head control interface, failing...\n");
			return false;
		}
		robotHead.view(hPos); robotHead.view(hEnc);

		lname = "/"+name+"/"+armname+"_arm"; rname = "/"+robotname+"/"+armname+"_arm";
		doption.put("local", lname.c_str());
		doption.put("remote", rname.c_str());

		robotArm.open(doption);
		if (!robotArm.isValid()) {
			printf("could not initialize arm control interface, failing...\n");
			return false;
		}
		robotArm.view(aPos); robotArm.view(aEnc);

		for (int i=0; i<7; i++) {
			aPos->setRefSpeed(i, 10.0);
		}

		/* Create and open ports */
		cportL=new ClickPort(bfL);
		string cplName="/"+name+"/clk:l";
		cportL->open(cplName.c_str());
		cportL->useCallback();

		cportR=new ClickPort(bfR);
		string cprName="/"+name+"/clk:r";
		cportR->open(cprName.c_str());
		cportR->useCallback();

		susPort=new Port;
		string suspName="/"+name+"/sus:o";
		susPort->open(suspName.c_str());

		return true;

	}

	virtual void threadRelease() {

		/* Stop and close ports */
		cportL->interrupt();
		cportR->interrupt();
		susPort->interrupt();

		cportL->close();
		cportR->close();
		susPort->close();

		delete cportL;
		delete cportR;
		delete susPort;

		/* stop motor interfaces and close */
		gaze->stopControl();
		clientGazeCtrl.close();
		carm->stopControl();
		clientArmCart.close();

		robotTorso.close();
		robotHead.close();
		robotArm.close();

	}

	virtual void run() {

		while (isStopping() != true) {

			/* poll the click ports containers to see if we have left/right ready to go */
			bfL.lock();	bfR.lock();
			if (bfL.size() == 2 && bfR.size() == 2) {

				printf("got a hit!\n");

				/* if they are, raise the flag that action is beginning, save current joint configuration */
				Bottle susmsg;
				susmsg.addInt(1);
				susPort->write(susmsg);

				//get the current joint configuration for the torso, head, and arm
				tang.clear(); tang.resize(3);
				tEnc->getEncoders(tang.data());
				hang.clear(); hang.resize(6);
				hEnc->getEncoders(hang.data());
				aang.clear(); aang.resize(16);
				aEnc->getEncoders(aang.data());

				/* get the xyz location of the gaze point */
				Vector bvL(2); Vector bvR(2);
				bvL[0] = bfL.get(0).asDouble(); bvL[1] = bfL.get(1).asDouble();
				bvR[0] = bfR.get(0).asDouble(); bvR[1] = bfR.get(1).asDouble();
				objPos.clear(); objPos.resize(3);
				gaze->triangulate3DPoint(bvL,bvR,objPos);

				/* servo the head to gaze at that point */
				//gaze->lookAtStereoPixels(bvL,bvR);
				gaze->lookAtFixationPoint(objPos);
				gaze->waitMotionDone(1.0,10.0);
				gaze->stopControl();

				printf("object position estimated as: %f, %f, %f\n", objPos[0], objPos[1], objPos[2]);
				printf("is this ok?\n");
				string posResp = Network::readString().c_str();

				if (posResp == "yes" || posResp == "y") {

					/* move to hover the hand over the XY position of the target: [X, Y, Z=0.2], with palm upright */
					objPos[2] = 0.1;

					Vector axa(4);
					axa.zero();
					if (armInUse) {
						axa[2] = 1.0; axa[3] = M_PI;
					} else {
						axa[1] = 1.0; axa[3] = M_PI;
					}

					carm->goToPoseSync(objPos,axa);
					carm->waitMotionDone(1.0,10.0);
					Time::delay(2.0);

					//curl fingers and thumb slightly to hold object
					Vector armCur(16);
					aEnc->getEncoders(armCur.data());
					armCur[8] = 3; armCur[10] = 25;
					armCur[11] = 25; armCur[12] = 25;
					armCur[13] = 25; armCur[14] = 25;
					armCur[15] = 55;
					aPos->positionMove(armCur.data());
					Time::delay(2.0);

					/* wait for terminal signal from user that object has been moved to the hand */
					bool validTarg = false;
					printf("object position reached, place in hand and enter target xy position\n");
					while (!validTarg) {

						string objResp = Network::readString().c_str();

						/* ask the user to enter in an XY target location, or confirm use of previous one */
						Bottle btarPos(objResp.c_str());
						if (btarPos.size() < 2) {

							//if user enters no target position, try to use last entered position
							if (targetPos.length() != 3) {
								printf("no previous target position available, please re-enter:\n");
							} else {
								validTarg = true;
							}

						} else {

							targetPos.clear(); targetPos.resize(3);
							targetPos[0] = btarPos.get(0).asDouble();
							targetPos[1] = btarPos.get(1).asDouble();
							targetPos[2] = 0.1;
							validTarg = true;

						}
					}

					/* move the arm to above the target location */
					axa.zero();
					if (armInUse) {
						axa[2] = 1.0; axa[3] = M_PI;
					} else {
						axa[1] = 1.0; axa[3] = M_PI;
					}
					carm->goToPoseSync(targetPos,axa);
					//carm->goToPosition(targetPos);
					carm->waitMotionDone(1.0,10.0);
					Time::delay(2.0);

					/* wait for user signal that the object has been removed */
					printf("object has been moved to target location. please remove object and hit enter\n");
					string tarResp = Network::readString().c_str();


				}
				/* return to saved motor configuration, clear click buffers, lower flag signaling action done */

				printf("gaze done, attempting reset\n");
				tPos->positionMove(tang.data());
				hPos->positionMove(hang.data());
				aPos->positionMove(aang.data());

				bfL.clear(); bfR.clear();
				bfL.unlock(); bfR.unlock();

				susmsg.clear();
				susmsg.addInt(0);
				susPort->write(susmsg);


			}
			else {

				bfL.unlock(); bfR.unlock();

			}
		}
	}

};



class objectMoverModule: public RFModule
{
protected:

	objectMoverThread * thr;

public:

	objectMoverModule() { }

	virtual bool configure(ResourceFinder &rf)
	{
		Time::turboBoost();

		thr=new objectMoverThread(rf);
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

	YARP_REGISTER_DEVICES(icubmod)

																					Network yarp;

	if (!yarp.checkNetwork())
		return -1;

	ResourceFinder rf;

	rf.configure("ICUB_ROOT",argc,argv);

	objectMoverModule mod;

	return mod.runModule(rf);
}
