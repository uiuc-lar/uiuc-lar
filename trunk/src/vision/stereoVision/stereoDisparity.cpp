/*
 * Copyright (C) 2011 Logan Niehaus
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
 *  stereoDisparity.cpp
 *
 * 	Logan Niehaus
 *  08/08/2013
 * 	rudimentary stereo vision system module. takes in the left and right images, head angles (assuming fixed torso)
 * 	and produces a disparity image
 *
 *  inputs:
 *  	/stereoVision/img:l	-- left eye image
 *  	/stereoVision/img:r	-- right eye image
 *  	/iKinGazeCtrl/head/		-- not connected, but must be running in order to function
 *  		or
 *  	/stereoVision/head:i -- streaming head angle values
 *  	/stereoVision/disp:o	-- disparity image output
 *
 *
 *  params: ([R]equired/[D]efault <Value>/[O]ptional)
 *
 *		camera intrinsics - file containing the calibrated camera's intrinsic parameters
 *		following parameters used by opencv semi global block matching algorithm
 *			preFiltCap = rf.check("preFiltCap",Value(63)).asInt();
 *			blockSize = rf.check("blockSize",Value(7)).asInt();
 *			ps1 = rf.check("ps1",Value(8)).asInt();
 *			ps2 = rf.check("ps2",Value(32)).asInt();
 *			minDisp = rf.check("minDisp",Value(0)).asInt();
 *			nDisp = rf.check("nDisp",Value(16)).asInt();
 *			uniquenessRatio = rf.check("uniquenessRatio",Value(20)).asInt();
 *			speckWS = rf.check("speckWS",Value(150)).asInt();
 *			speckRng = rf.check("speckRng",Value(1)).asInt();
 *			dispMaxDiff = rf.check("dispMaxDiff",Value(0)).asInt();
 *			dp = rf.check("fullDP");
 *
 *
 *
 *  outputs:
 *  	/stereoVision/img:o --
 *
 *  TODO:
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>
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


class stereoVisionThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<yarp::sig::Vector > *portHead;
	BufferedPort<ImageOf<PixelRgb> > *portImgL;
	BufferedPort<ImageOf<PixelRgb> > *portImgR;
	BufferedPort<ImageOf<PixelBgr> > *portImgO;
	BufferedPort<ImageOf<PixelBgr> > *portImgLO;
	BufferedPort<ImageOf<PixelBgr> > *portImgRO;
	Port *portFxlOut;

	//ikingaze objects and params
	PolyDriver clientGazeCtrl;
	IGazeControl *igaze;
	double neckTT, eyeTT;

	//forward kinematics if gazectrl not used
	iCubEye * eyeL;
	iCubEye * eyeR;


	//camera projection matrix/params
	Mat Pl;
	Mat Pr;
	double fxl, fyl, cxl, cyl;
	double fxr, fyr, cxr, cyr;
	int wl, hl, wr, hr;
	Mat mpxL, mpyL, mpxR, mpyR;

	//aux. data for stereo processing
	Mat R1, R2, P1, P2, Q;
	Mat R, Rl, Rr;

	//parameters for stereo block matching algorithm
	int preFiltCap, blockSize, ps1, ps2;
	int minDisp, nDisp, uniquenessRatio, speckWS, speckRng;
	int dispMaxDiff;
	bool dp;



public:

	stereoVisionThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	void topDownGazeCmd(yarp::sig::Vector &ang, bool wait = false) {

		igaze->lookAtAbsAngles(ang);
		if (wait)
			igaze->waitMotionDone(0.1, 10.0);

	}

	virtual bool threadInit()
	{

		name=rf.check("name",Value("stereoVision")).asString().c_str();
		neckTT = rf.check("nt",Value(1.0)).asDouble();
		eyeTT = rf.check("et",Value(0.5)).asDouble();

		//get camera calibration parameters
		Pl = Mat::eye(3, 3, CV_64F);
		Pr = Mat::eye(3, 3, CV_64F);
		Bottle ipsL = rf.findGroup("CAMERA_CALIBRATION_LEFT");
		Bottle ipsR = rf.findGroup("CAMERA_CALIBRATION_RIGHT");
		if (ipsL.size() && ipsR.size()) {

			Pl.at<double>(0,2) = cxl = ipsL.find("cx").asDouble();
			Pl.at<double>(1,2) = cyl = ipsL.find("cy").asDouble();
			Pl.at<double>(0,0) = fxl = ipsL.find("fx").asDouble();
			Pl.at<double>(1,1) = fyl = ipsL.find("fy").asDouble();
			wl = ipsL.find("w").asInt();
			hl = ipsL.find("h").asInt();
			printf("Left cam parameters: %f, %f, %f, %f\n",cxl,cyl,fxl,fyl);

			Pr.at<double>(0,2) = cxr = ipsR.find("cx").asDouble();
			Pr.at<double>(1,2) = cyr = ipsR.find("cy").asDouble();
			Pr.at<double>(0,0) = fxr = ipsR.find("fx").asDouble();
			Pr.at<double>(1,1) = fyr = ipsR.find("fy").asDouble();
			wr = ipsR.find("w").asInt();
			hr = ipsR.find("h").asInt();
			printf("Right cam parameters: %f, %f, %f, %f\n",cxr,cyr,fxr,fyr);

		}
		else {

			fprintf(stdout,"Could not find calibration parameters for one of the cameras\n");
			return false;

		}

		preFiltCap = rf.check("preFiltCap",Value(63)).asInt();
		blockSize = rf.check("blockSize",Value(7)).asInt();
		ps1 = rf.check("ps1",Value(8)).asInt();
		ps2 = rf.check("ps2",Value(32)).asInt();
		minDisp = rf.check("minDisp",Value(0)).asInt();
		nDisp = rf.check("nDisp",Value(16)).asInt();
		uniquenessRatio = rf.check("uniquenessRatio",Value(20)).asInt();
		speckWS = rf.check("speckWS",Value(150)).asInt();
		speckRng = rf.check("speckRng",Value(1)).asInt();
		dispMaxDiff = rf.check("dispMaxDiff",Value(0)).asInt();
		dp = rf.check("fullDP");


		//open up ports
		portHead=new BufferedPort<yarp::sig::Vector >;
		string portHeadName="/"+name+"/head:i";
		portHead->open(portHeadName.c_str());

		portImgL=new BufferedPort<ImageOf<PixelRgb> >;
		string portImgLName="/"+name+"/img:l";
		portImgL->open(portImgLName.c_str());

		portImgR=new BufferedPort<ImageOf<PixelRgb> >;
		string portImgRName="/"+name+"/img:r";
		portImgR->open(portImgRName.c_str());

		portImgO=new BufferedPort<ImageOf<PixelBgr> >;
		string portImgOName="/"+name+"/img:o";
		portImgO->open(portImgOName.c_str());

		portImgLO=new BufferedPort<ImageOf<PixelBgr> >;
		string portImgLOName="/"+name+"/img:lo";
		portImgLO->open(portImgLOName.c_str());

		portImgRO=new BufferedPort<ImageOf<PixelBgr> >;
		string portImgROName="/"+name+"/img:ro";
		portImgRO->open(portImgROName.c_str());

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

			printf("Could not initialize gaze control interface, please use head angle input port...\n");
			//return false;

		}

		//initialize head kinematics
		eyeL = new iCubEye("left");
		eyeR = new iCubEye("right");
		eyeL->setAllConstraints(false);
		eyeR->setAllConstraints(false);
		eyeL->releaseLink(0); eyeR->releaseLink(0);
		eyeL->releaseLink(1); eyeR->releaseLink(1);
		eyeL->releaseLink(2); eyeR->releaseLink(2);


		return true;

	}

	virtual void run()
	{

		//get the normal salience maps after pre-focusing
		ImageOf<PixelRgb> *pImgL = portImgL->read(false);
		ImageOf<PixelRgb> *pImgR = portImgR->read(false);

		if (pImgL && pImgR) {

			Mat Sl, Sr;

			Sl = (IplImage *) pImgL->getIplImage();
			Sr = (IplImage *) pImgR->getIplImage();
			Mat Scl;
			Mat Scr;


			//set up head and l/r eye matrices, assuming fixed torso
			Matrix Hl, Hr, H;
			Mat R(3,3, CV_64F);
			vector<double> T(3);
			yarp::sig::Vector eo, ep;
			bool havePose;


			//look for joint information from gazectrl
			yarp::sig::Vector *headAng = portHead->read(false);
			if (clientGazeCtrl.isValid()) {

				igaze->getLeftEyePose(eo, ep);
				Hl = axis2dcm(ep);
				Hl(0,3) = eo[0]; Hl(1,3) = eo[1]; Hl(2,3) = eo[2];
				igaze->getRightEyePose(eo, ep);
				Hr = axis2dcm(ep);
				Hr(0,3) = eo[0]; Hr(1,3) = eo[1]; Hr(2,3) = eo[2];
				havePose = true;


			}
			//if gazectrl unavailable, check if head angle value available
			else if (headAng) {

				yarp::sig::Vector angles(8);

				//assuming torso fixed at home here
				angles[0] = 0.0; angles[1] = 0.0; angles[2] = 0.0;

				//get head angles (for left eye only at first)
				angles[3] = (*headAng)[0]; angles[4] = (*headAng)[1];
				angles[5] = (*headAng)[2]; angles[6] = (*headAng)[3];
				angles[7] = (*headAng)[4] + (*headAng)[5]/2.0;
				angles = PI*angles/180.0;

				Hl = eyeL->getH(angles);
				angles[7] = PI*((*headAng)[4] - (*headAng)[5]/2.0)/180.0;
				Hr = eyeR->getH(angles);
				havePose = true;

			}
			//if neither available, flag
			else {

				havePose = false;

			}


			//if we have a proper head/eye pose reading, do rectification and disparity map
			if (havePose) {

				//get the transform matrix from the left image to the right image
				H = Hr*SE3inv(Hl);
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 3; j++) {
						R.at<double>(i,j) = H(i,j);
					}
					T.at(i) = H(i,3);
				}

				//rectify the images
				stereoRectify(Pl, Mat::zeros(4,1,CV_64F), Pr, Mat::zeros(4,1,CV_64F), Size(wl, hl), R, Mat(T), R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1);
				initUndistortRectifyMap(Pl, Mat(), R1, P1, Size(wl, hl), CV_32FC1, mpxL, mpyL);
				initUndistortRectifyMap(Pr, Mat(), R2, P2, Size(wr, hr), CV_32FC1, mpxR, mpyR);
				remap(Sl, Scl, mpxL, mpyL, INTER_LINEAR);
				remap(Sr, Scr, mpxR, mpyR, INTER_LINEAR);



				StereoSGBM sgbm;

				sgbm.preFilterCap = preFiltCap; //63
				sgbm.SADWindowSize = blockSize;
				int cn = 3;
				sgbm.P1 = ps1*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
				sgbm.P2 = ps2*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
				sgbm.minDisparity = minDisp; //-15
				sgbm.numberOfDisparities = nDisp;
				sgbm.uniquenessRatio = uniquenessRatio; //22
				sgbm.speckleWindowSize = speckWS; //100
				sgbm.speckleRange = speckRng; //32
				sgbm.disp12MaxDiff = dispMaxDiff;
				sgbm.fullDP = dp; // alg == STEREO_HH


				Mat disp;
				sgbm(Scl, Scr, disp);

				Mat dispo, dispt;
				disp.convertTo(dispo, CV_32FC1, 255/(16*16.));
				dispo.convertTo(dispt, CV_8U);



				ImageOf<PixelBgr> &oImg = portImgO->prepare();
				ImageOf<PixelBgr> &loImg = portImgLO->prepare();
				ImageOf<PixelBgr> &roImg = portImgRO->prepare();

				oImg.resize(*pImgL);
				loImg.resize(*pImgL);
				roImg.resize(*pImgR);

				Mat om((IplImage *)oImg.getIplImage(), false);
				Mat lm((IplImage *)loImg.getIplImage(), false);
				Mat rm((IplImage *)roImg.getIplImage(), false);


				Mat dprep(disp.rows, disp.cols, CV_8UC3);
				cvtColor(dispt,om,CV_GRAY2RGB);

				Scl.copyTo(lm);
				Scr.copyTo(rm);

				portImgO->write();
				portImgLO->write();
				portImgRO->write();

			}

		}

	}

	virtual void threadRelease()
	{

		if (clientGazeCtrl.isValid())
			clientGazeCtrl.close();

		portImgL->interrupt();
		portImgR->interrupt();
		portImgO->interrupt();
		portImgLO->interrupt();
		portImgRO->interrupt();
		portFxlOut->interrupt();
		portHead->interrupt();

		portImgL->close();
		portImgR->close();
		portImgO->close();
		portImgLO->close();
		portImgRO->close();
		portFxlOut->close();
		portHead->close();

		delete portImgL, portImgR, portImgO, portImgLO, portImgRO, portHead;
		delete portFxlOut;


	}

};

class stereoVisionModule: public RFModule
{
protected:

	stereoVisionThread *thr;
	Port * rpcPort;
	string name;

public:

	stereoVisionModule() { }

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
		name=rf.check("name",Value("stereoVision")).asString().c_str();
		rpcPort = new Port;
		string portRpcName="/"+name+"/rpc";
		rpcPort->open(portRpcName.c_str());
		attach(*rpcPort);


		thr=new stereoVisionThread(rf);
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

	stereoVisionModule mod;

	return mod.runModule(rf);
}



