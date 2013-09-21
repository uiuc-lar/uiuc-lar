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
 * 	and produces a disparity image (as well as providing left/right rectified images). Via the RPC port, you can
 * 	query the module for the XYZ location of a pixel in the left (rectified) image, or its raw disparity value.
 * 	the module is based on the 3d camera calibration/stereo block matching toolbox available in opencv.
 *
 *  inputs:
 *  	/stereoVision/img:l	-- left eye image
 *  	/stereoVision/img:r	-- right eye image
 *  	/iKinGazeCtrl/head/		-- not connected, but must be running in order to function
 *  		or
 *  	/stereoVision/head:i -- streaming head angle values
 *  	/stereoVision/disp:o	-- disparity image output
 *  	/stereoVision/map:o	-- worldmap image output. XYZ values (root frame) for each pixel in the original image plane
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
 *		cType - variable denoting what kind of color conversion should be used to convert
 *			to a single channel image for block matching. 0 (default): simple grayscale;
 *			1: convert to HSV and take saturation channel. if already pre-converted to
 *			a single channel image, pass it as an RGB and set this to 0.
 *		mapMin, mapMax - if set, these parameters will cause the world output map to be scaled
 *			in a way that mapMin takes on value 0 and mapMax takes on value 255. Values outside of [mapMin,mapMax]
 *			will saturate to 0 (mapMin) or 255 (mapMax).
 *			This is useful for other modules that read or save the map as an 8bit 3-channel image.
 *			To reconstruct the original floating point XYZ image, use the following expression:
 *				IM_orig = IM_uint8 * (mapMax - mapMin)/255 + mapMin
 *			If not set, or if mapMin == mapMax, the map will remain in its original scaling (real XYZ coordinates)
 *			It is suggested that these parameters be set to be as close as possible for the application,
 *			as this will reduce rounding error for the reconstruction
 *
 *  outputs:
 *  	/stereoVision/img:o -- 8bit 3-channel disparity image. to get proper floating point disparity
 *  		map values, take one of the three identical channels and multiple by nDisp/255.
 *  	/stereoVision/img:lo -- Grayscale (3channel) view of rectified left camera image
 *  		used to calculate disparity values. May be a particular channel of non-RGB color space
 *  	/stereoVision/img:ro -- Grayscale (3channel) view of rectified right camera image
 *  	/stereoVision/rpc -- RPC port for module. Accepts the following commands:
 *
 *  		"loc u v" -- Get the XYZ coordinates (in root reference frame) of object corresponding to
 *  						pixel (u,v) in the left rectified image
 *  		"disp u v" -- Get the disparity value (32F) at the pixel (u,v)
 *  		"set <param> <val>" -- set the named parameter to the target value. allowable parameters
 *  						are those for the stereo block matching, as listed above
 *
 *  TODO:
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
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
	BufferedPort<ImageOf<PixelRgbFloat> > *portMapO;

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
	Mat impxL, impyL, impxR, impyR;

	//aux. data for stereo processing
	Mat R1, R2, P1, P2;
	Mat R, Rl, Rr;

	//parameters for stereo block matching algorithm
	int ctype;
	bool useSG;
	int preFiltCap, blockSize, ps1, ps2;
	int minDisp, nDisp, uniquenessRatio, speckWS, speckRng;
	int dispMaxDiff;
	bool dp;
	double mapMin, mapMax;

	//module parameters
	bool strict;

	//storage objects for current disparity map and perspective transform matrix
	Mat * disp;
	Mat * Q;
	Matrix Hl;
	Matrix Hr;
	Matrix Hl0, Hr0, H0;

	yarp::sig::Vector QL,QR;

	//semaphores for interaction with rpc port
	Semaphore * mutex;

public:

	stereoVisionThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	virtual float getDispVal(int u, int v) {

		float dval;

		//first find the location of the point in the rectified image
		Mat uvpt(1,1,CV_32FC2);
		uvpt.at<Point2f>(0,0) = Point2f(u,v);
		undistortPoints(uvpt, uvpt, Pl, Mat(), R1, P1);
		u = (int)uvpt.at<Point2f>(0,0).x;
		v = (int)uvpt.at<Point2f>(0,0).y;

		mutex->wait();
		dval = disp->at<float>(v,u);
		mutex->post();

		return dval;

	}


	/* get3dWorldLoc
	 * Desc: Get the 3d coordinates in the root frame for a point that projects
	 * to (u,v) in the left ORIGINAL camera image, using the disparity map
	 */
	virtual yarp::sig::Vector get3dWorldLoc(int u, int v) {

		yarp::sig::Vector loc(4);

		Matrix Hlt, Hrct, Qm;
		Mat Qt, Rrct;
		float dval;


		//first find the location of the point in the rectified image
		Mat uvpt(1,1,CV_32FC2);
		uvpt.at<Point2f>(0,0) = Point2f(u,v);
		undistortPoints(uvpt, uvpt, Pl, Mat(), R1, P1);
		u = (int)uvpt.at<Point2f>(0,0).x;
		v = (int)uvpt.at<Point2f>(0,0).y;


		mutex->wait();
		Hlt = Hl;
		Q->copyTo(Qt);
		dval = disp->at<float>(v,u);
		mutex->post();


		//get the homogeneous coordinates
		Qm.resize(4,4);
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				Qm(i,j) = (float) Qt.at<double>(i,j);
			}
		}

		loc[0] = (float) u; loc[1] = (float) v; loc[2] = (float) dval; loc[3] = 1.0;

		loc = Qm*loc;
		loc = loc/loc[3];


		//transform back to the unrectified image coordinate frame
		Hrct.resize(4,4);
		Hrct.zero();
		Rrct = R1.t();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				Hrct(i,j) = Rrct.at<double>(i,j);
			}
		}
		Hrct(3,3) = 1;

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				printf("%f,",Hlt(i,j));
			}
			printf("\n");
		}


		//transform to current world frame
		loc[3] = 1.0;

		loc = Hlt*Hrct*loc;

		loc = loc/loc[3];

		return loc;


	}

	/* triangulatePointMatch
	 * Desc: Get the 3d coordinates in the root frame for known corresponding points
	 * in left and right unrectified images.
	 */
	virtual yarp::sig::Vector triangulatePointMatch(Point2f lp, Point2f rp) {

		yarp::sig::Vector loc(4);
		Matrix Hlt, Hrct, Qm;
		Mat Qt, Rrct;
		float dval;

		//map points into rectified images
		Mat lpr(1,1,CV_32FC2);
		Mat rpr(1,1,CV_32FC2);
		lpr.at<Point2f>(0,0) = lp;
		rpr.at<Point2f>(0,0) = rp;
		undistortPoints(lpr, lpr, Pl, Mat(), R1, P1);
		undistortPoints(rpr, rpr, Pr, Mat(), R2, P2);

		printf("rectified l coord: %f, %f\n", lpr.at<Point2f>(0,0).x, lpr.at<Point2f>(0,0).y);
		printf("rectified r coord: %f, %f\n", rpr.at<Point2f>(0,0).x, rpr.at<Point2f>(0,0).y);

		mutex->wait();
		Hlt = Hl;
		Q->copyTo(Qt);
		dval = lpr.at<Point2f>(0,0).x-rpr.at<Point2f>(0,0).x;
		mutex->post();

		//get the homogeneous coordinates
		Qm.resize(4,4);
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				Qm(i,j) = (float) Qt.at<double>(i,j);
			}
		}


		loc[0] = (float) lpr.at<Point2f>(0,0).x; loc[1] = (float) lpr.at<Point2f>(0,0).y; loc[2] = (float) dval; loc[3] = 1.0;
		loc = Qm*loc;
		loc = loc/loc[3];


		//transform back to the unrectified image coordinate frame
		Hrct.resize(4,4);
		Hrct.zero();
		Rrct = R1.t();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				Hrct(i,j) = Rrct.at<double>(i,j);
			}
		}
		Hrct(3,3) = 1;


		//transform to current world frame
		loc[3] = 1.0;
		loc = Hlt*Hrct*loc;
		loc = loc/loc[3];

		return loc;

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

		printf("QL:\n");
		Bottle pars=rf.findGroup("STEREO_DISPARITY");
		if (Bottle *pXo=pars.find("QL").asList())
		{
			QL.resize(pXo->size());
			for (int i=0; i<(pXo->size()); i++) {
				QL[i]=pXo->get(i).asDouble();
				printf("%f,",QL(i));
			}

		}
		printf("\n");
		printf("QR:\n");

		if (Bottle *pXo=pars.find("QR").asList())
		{
			QR.resize(pXo->size());
			for (int i=0; i<(pXo->size()); i++) {
				QR[i]=pXo->get(i).asDouble();
				printf("%f,",QR(i));
			}
		}
		printf("\n");

		Bottle extrinsics=rf.findGroup("STEREO_DISPARITY");
		if (Bottle *pXo=extrinsics.find("HN").asList()) {
			H0.resize(4,4);
			for (int i=0; i<4; i++) {
				for (int j=0; j<4; j++) {
					H0(i,j) = pXo->get(i*4+j).asDouble();
				}
			}
		}

		printf("H0:\n");
		for (int i =0; i<4; i++) {
			for (int j=0; j<4; j++) {
				printf("%f,", H0(i,j));
			}
			printf("\n");
		}

		ctype = rf.check("cType",Value(0)).asInt();
		preFiltCap = rf.check("preFiltCap",Value(63)).asInt();
		blockSize = rf.check("blockSize",Value(5)).asInt();
		ps1 = rf.check("ps1",Value(8)).asInt();
		ps2 = rf.check("ps2",Value(32)).asInt();
		minDisp = rf.check("minDisp",Value(0)).asInt();
		nDisp = rf.check("nDisp",Value(32)).asInt();
		uniquenessRatio = rf.check("uniquenessRatio",Value(15)).asInt();
		speckWS = rf.check("speckWS",Value(50)).asInt();
		speckRng = rf.check("speckRng",Value(1)).asInt();
		dispMaxDiff = rf.check("dispMaxDiff",Value(0)).asInt();
		dp = rf.check("fullDP",Value(0)).asInt();
		useSG = rf.check("useSG",Value(1)).asInt();

		strict = rf.check("strict");

		mapMin = rf.check("mapMin",Value(0)).asDouble();
		mapMax = rf.check("mapMax",Value(0)).asDouble();

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

		portMapO=new BufferedPort<ImageOf<PixelRgbFloat> >;
		string portMapOName="/"+name+"/map:o";
		portMapO->open(portMapOName.c_str());

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

		//initialize semaphores
		mutex = new Semaphore;


		disp = new Mat(wl,hl,CV_32FC1);
		Q = new Mat(4,4,CV_64FC1);


		//set reader ports to do strict reads
		if (strict) {
			portImgL->setStrict(true);
			portImgR->setStrict(true);
		}

		Hl0 = eyeL->getH(QL);
		Hr0 = eyeR->getH(QR);


		return true;

	}

	virtual void run()
	{

		//make sure there are two images available before grabbing one

		if (portImgL->getPendingReads() > 0 && portImgR->getPendingReads() >0) {

			//get the normal salience maps after pre-focusing
			ImageOf<PixelRgb> *pImgL = portImgL->read(false);
			ImageOf<PixelRgb> *pImgR = portImgR->read(false);


			Mat Sl, Sr;

			Sl = (IplImage *) pImgL->getIplImage();
			Sr = (IplImage *) pImgR->getIplImage();
			Mat Scl;
			Mat Scr;
			Mat Qtmp;


			//set up head and l/r eye matrices, assuming fixed torso
			Matrix H, Hlt, Hrt;
			Mat R(3,3, CV_64F);
			vector<double> T(3);
			yarp::sig::Vector eo, ep;
			bool havePose;


			//look for joint information from gazectrl
			yarp::sig::Vector *headAng = portHead->read(false);
			if (clientGazeCtrl.isValid()) {

				igaze->getLeftEyePose(eo, ep);
				Hlt = axis2dcm(ep);
				Hlt(0,3) = eo[0]; Hlt(1,3) = eo[1]; Hlt(2,3) = eo[2];
				igaze->getRightEyePose(eo, ep);
				Hrt = axis2dcm(ep);
				Hrt(0,3) = eo[0]; Hrt(1,3) = eo[1]; Hrt(2,3) = eo[2];
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

				Hlt = eyeL->getH(angles);
				angles[7] = PI*((*headAng)[4] - (*headAng)[5]/2.0)/180.0;
				Hrt = eyeR->getH(angles);
				havePose = true;

			}
			//if neither available, flag
			else {

				havePose = false;

			}


			//if we have a proper head/eye pose reading, do rectification and disparity map
			if (havePose) {

				//get the transform matrix from the left image to the right image
				H = SE3inv(Hrt)*Hr0*H0*SE3inv(SE3inv(Hlt)*Hl0);
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 3; j++) {
						R.at<double>(i,j) = H(i,j);
					}
					T.at(i) = H(i,3);
				}

				//rectify the images
				stereoRectify(Pl, Mat::zeros(4,1,CV_64F), Pr, Mat::zeros(4,1,CV_64F), Size(wl, hl), R, Mat(T), R1, R2, P1, P2, Qtmp, CALIB_ZERO_DISPARITY, -1);
				initUndistortRectifyMap(Pl, Mat(), R1, P1, Size(wl, hl), CV_32FC1, mpxL, mpyL);
				initUndistortRectifyMap(Pr, Mat(), R2, P2, Size(wr, hr), CV_32FC1, mpxR, mpyR);
				remap(Sl, Scl, mpxL, mpyL, INTER_LINEAR);
				remap(Sr, Scr, mpxR, mpyR, INTER_LINEAR);


				//create an inverse mapping to unrectify the disparity map
				Mat P1n, P2n, Pln, Prn;
				P1n = P1.rowRange(0,3).colRange(0,3);
				P2n = P2.rowRange(0,3).colRange(0,3);
				Pln = Pl.t(); Pln.resize(4,0); Pln = Pln.t();
				Prn = Pr.t(); Prn.resize(4,0); Prn = Prn.t();
				Prn.at<double>(0,3) = -P2.at<double>(0,3);


				//initUndistortRectifyMap(P1n, Mat(), R1.t(), Pln, Size(wl, hl), CV_32FC1, impxL, impyL);
				initUndistortRectifyMap(P1n, Mat(), R1.t(), Pl, Size(wl, hl), CV_32FC1, impxL, impyL);
				initUndistortRectifyMap(P2n, Mat(), R2.t(), Prn, Size(wr, hr), CV_32FC1, impxR, impyR);


				//convert to HSV color space and get the S channel (for colored objects basically)
				Mat scl1(Scl.rows, Scl.cols, CV_8UC1);
				Mat scr1(Scr.rows, Scr.cols, CV_8UC1);
				Mat ctmp(Scl.rows, Scl.cols, CV_8UC3);


				if (ctype == 1) {

					//take an HSV conversion and grab the sat channel
					int * frto = new int[4];
					frto[0] = 1; frto[1] = 0;

					cvtColor(Scl,ctmp,CV_RGB2HSV);
					mixChannels(&ctmp, 1, &scl1, 1, frto, 1);

					cvtColor(Scr,ctmp,CV_RGB2HSV);
					mixChannels(&ctmp, 1, &scr1, 1, frto, 1);


				}
				else {

					//just convert it to grayscale
					cvtColor(Scl,scl1,CV_RGB2GRAY);
					cvtColor(Scr,scr1,CV_RGB2GRAY);

				}

				//TODO: convert to L*a*b color space, and take max of abs(C-255/2) between a and b channels
				/*
				Mat * abchn = new Mat[3];

				cvtColor(Scl,ctmp,CV_RGB2Lab);
				absdiff(ctmp,128,ctmp);
				ctmp = 2*ctmp;
				split(ctmp,abchn);
				max(abchn[1],abchn[2],scl1);

				cvtColor(Scr,ctmp,CV_RGB2Lab);
				absdiff(ctmp,128,ctmp);
				ctmp = 2*ctmp;
				split(ctmp,abchn);
				max(abchn[1],abchn[2],scr1);
				 */

				Mat dispo, dispt;


				//perform stereo block matching algorithm
				if (useSG) {

					StereoSGBM sgbm;

					int cn = 1;
					sgbm.preFilterCap = preFiltCap;
					sgbm.SADWindowSize = blockSize;
					sgbm.P1 = ps1*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
					sgbm.P2 = ps2*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
					sgbm.minDisparity = minDisp;
					sgbm.numberOfDisparities = nDisp;
					sgbm.uniquenessRatio = uniquenessRatio;
					sgbm.speckleWindowSize = speckWS;
					sgbm.speckleRange = speckRng;
					sgbm.disp12MaxDiff = dispMaxDiff;
					sgbm.fullDP = dp;

					sgbm(scl1, scr1, dispo);

					mutex->wait();
					dispo.convertTo(*disp, CV_32FC1, 1.0/16.0);
#if CV_MINOR_VERSION < 4
					Qtmp.at<double>(3,2) = -Qtmp.at<double>(3,2); //the disparity value for some versions of opencv should be negative
#endif
					Qtmp.copyTo(*Q);
					Hl = Hlt;
					Hr = Hrt;
					mutex->post();

				}
				else {

					StereoBM sbm(CV_STEREO_BM_BASIC, nDisp, blockSize);

					sbm.state->preFilterCap = preFiltCap;
					sbm.state->minDisparity = minDisp;
					sbm.state->numberOfDisparities = nDisp;
					sbm.state->uniquenessRatio = uniquenessRatio;
					sbm.state->speckleWindowSize = speckWS;
					sbm.state->speckleRange = speckRng;
					sbm.state->disp12MaxDiff = dispMaxDiff;

					mutex->wait();
					sbm(scl1,scr1,*disp,CV_32F);
#if CV_MINOR_VERSION < 4
					Qtmp.at<double>(3,2) = -Qtmp.at<double>(3,2); //the disparity value for some versions of opencv should be negative
#endif
					Qtmp.copyTo(*Q);
					Hl = Hlt;
					Hr = Hrt;
					mutex->post();

				}

				//disp->convertTo(dispt, CV_8U, 255/((double)nDisp));
				disp->convertTo(dispt, CV_8U, 1);
				dispt = (255.0/(double)nDisp)*dispt;

				//undo the rectification before sending out the disparity image
				dispo = Mat(dispt.rows,dispt.cols,CV_8U);
				remap(dispt, dispo, impxL, impyL, INTER_LINEAR);

				ImageOf<PixelBgr> &oImg = portImgO->prepare();
				oImg.resize(*pImgL);
				Mat om((IplImage *)oImg.getIplImage(), false);

				Mat dprep(disp->rows, disp->cols, CV_8UC3);
				cvtColor(dispo,om,CV_GRAY2RGB);

				if (strict) {
					portImgO->writeStrict();
				}
				else {
					portImgO->write();
				}

				//if it is connected to something, calculate the XYZ coordinates of all original image pixels
				if (portMapO->getOutputCount() > 0) {

					ImageOf<PixelRgbFloat> &mapImg = portMapO->prepare();
					mapImg.resize(wl,hl);
					Mat map((IplImage *)mapImg.getIplImage(), false);
					Mat tmap(map.rows, map.cols, CV_32FC3);
					reprojectImageTo3D(*disp, map, *Q);

					//transform back to the unrectified image coordinate frame
					Matrix Hrct;
					Hrct.resize(4,4);
					Hrct.zero();
					Mat Rrct = R1.t();
					for (int i = 0; i < 3; i++) {
						for (int j = 0; j < 3; j++) {
							Hrct(i,j) = Rrct.at<double>(i,j);
						}
					}
					Hrct(3,3) = 1;

					//add transform back to root reference frame
					Hrct = Hlt*Hrct;

					//apply transform
					Mat Hm(4,4,CV_32F);
					for (int i = 0; i < 4; i++) {
						for (int j = 0; j < 4; j++) {
							Hm.at<float>(i,j) = (float)Hrct(i,j);
						}
					}

					perspectiveTransform(map,tmap,Hm);

					//remap to the original (unrectified) image plane
					remap(tmap, map, impxL, impyL, INTER_LINEAR);

					//may need to be ranged from [mapMin, mapMax] -> [0,255] for saving
					//if mapMin == mapMax, do not scale
					//TODO: Allow for multiple image outputs (both scaled and not)
					if (mapMin != mapMax) {

						map = (map-mapMin)*255.0/(mapMax-mapMin);

					}

					if (strict) {
						portMapO->writeStrict();
					}
					else {
						portMapO->write();
					}


				}

				//only write to these ports if they are actually being watched
				if (portImgLO->getOutputCount() > 0) {

					ImageOf<PixelBgr> &loImg = portImgLO->prepare();
					loImg.resize(*pImgL);
					Mat lm((IplImage *)loImg.getIplImage(), false);
					cvtColor(scl1,Scl,CV_GRAY2RGB);
					Scl.copyTo(lm);
					portImgLO->write();

				}

				if (portImgRO->getOutputCount() > 0) {

					ImageOf<PixelBgr> &roImg = portImgRO->prepare();
					roImg.resize(*pImgR);
					Mat rm((IplImage *)roImg.getIplImage(), false);
					cvtColor(scr1,Scr,CV_GRAY2RGB);
					Scr.copyTo(rm);
					portImgRO->write();

				}
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
		portHead->interrupt();
		portMapO->interrupt();

		portImgL->close();
		portImgR->close();
		portImgO->close();
		portImgLO->close();
		portImgRO->close();
		portHead->close();
		portMapO->close();

		delete portImgL, portImgR, portImgO, portImgLO, portImgRO, portHead, portMapO;

		delete mutex;
		delete disp;
		delete Q;


	}

	virtual bool setParam(string pname, int pval) {

		bool success = true;

		if (pname == "useSG") {
			useSG = (bool)pval;
		}
		else if (pname == "preFiltCap") {
			preFiltCap = pval;
		}
		else if (pname == "blockSize") {
			blockSize = pval;
		}
		else if (pname == "ps1") {
			ps1 = pval;
		}
		else if (pname == "ps2") {
			ps2 = pval;
		}
		else if (pname == "minDisp") {
			minDisp = pval;
		}
		else if (pname == "nDisp") {
			nDisp = pval;
		}
		else if (pname == "uniquenessRatio") {
			uniquenessRatio = pval;
		}
		else if (pname == "speckWS") {
			speckWS = pval;
		}
		else if (pname == "speckRng") {
			speckRng = pval;
		}
		else if (pname == "dispMaxDiff") {
			dispMaxDiff = pval;
		}
		else if (pname == "fullDP") {
			dp = (bool)pval;
		}
		else if (pname == "cType") {
			ctype = pval;
		}
		else {
			success = false;
		}

		return success;

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
		if (msg == "loc") {

			if (command.size() == 3) {

				yarp::sig::Vector mloc;
				int u = command.get(1).asInt();
				int v = command.get(2).asInt();
				mloc = thr->get3dWorldLoc(u,v);
				reply.add(mloc[0]);
				reply.add(mloc[1]);
				reply.add(mloc[2]);

			}
			else if (command.size() == 5) {

				yarp::sig::Vector mloc;
				mloc = thr->triangulatePointMatch(Point2f(command.get(1).asInt(), command.get(2).asInt()),
						Point2f(command.get(3).asInt(),command.get(4).asInt()));
				reply.add(mloc[0]);
				reply.add(mloc[1]);
				reply.add(mloc[2]);

			}
			else {
				reply.add(-1);
			}
		}
		else if (msg == "disp") {
			if (command.size() < 3) {
				reply.add(-1);
			}
			else {

				int u = command.get(1).asInt();
				int v = command.get(2).asInt();
				float dval = thr->getDispVal(u,v);
				dval = dval*255.0/16.0;
				reply.add(dval);

			}
		}
		else if (msg == "set") {
			if (command.size() < 3) {
				reply.add(-1);
			}
			else {
				string param(command.get(1).asString().c_str());
				int pval = command.get(2).asInt();
				reply.add(thr->setParam(param,pval));
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



