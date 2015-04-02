/*
 *  depthMap.cpp
 *
 * 	Logan Niehaus
 * 	4/24/11
 * 	takes in a two stereo images and head position and produces a disparity map
 *
 *
 *  inputs:
 *  	/depthMap/img:l	-- left eye rgb image
 *  	/depthMap/img:r -- right eye rgb image
 *  	/depthMap/pos:h	-- streaming head position, to calculate extrinsic camera params
 *
 *
 *  params: ([R]equired/[D]efault <Value>/[O]ptional)
 *
 *  	<camera parameters>		-- intrinsic calibration params for each camera under [CAMERA_CALIBRATION_LEFT/RIGHT].
 *										see iKinGazeCtrl docs for example of how to do this
 *		voffset					-- vertical offset from left to right image in pixels; this adjusts for
 *									vertical camera misalignment on the icub (ex -20 shifts the right image up by 20)
 *  	name					-- module port basename (D /stereoAttention)
 *  	debug					-- setting flag makes the module shoot debug info to stdout, and opens up debug image ports
 *
 *  outputs:
 *  	/depthMap/omg:l	-- left eye rectified image
 *  	/depthMap/omg:r	-- right eye rectified image
 *
 *  TODO:
 *  	write this module
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


class depthMapThread : public Thread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *portImgL;
	BufferedPort<ImageOf<PixelRgb> > *portImgR;
	BufferedPort<ImageOf<PixelRgb> > *portOmgL;
	BufferedPort<ImageOf<PixelRgb> > *portOmgR;
	BufferedPort<yarp::sig::Vector> *portHeadPos;


	//ego map params
	double azlo, azhi, azalo, azahi;
	double ello, elhi, elalo, elahi;

	//camera projection matrix/params
	Mat Pl;
	Mat Pr;
	double fxl, fyl, cxl, cyl;
	double fxr, fyr, cxr, cyr;
	int voffset;
	iCubEye * eyeL, * eyeR;

	//aux. data for stereo processing
	Mat R1, R2, P1, P2, Q;
	Mat R, Rl, Rr;
	Mat mpxL, mpyL, mpxR, mpyR;
	Point * mxDxL, * mxDxR;

	//misc running params
	int ksize;
	double fsize;
	bool debug;


public:

	depthMapThread(ResourceFinder &_rf) : rf(_rf)
	{ }


	virtual bool threadInit()
	{

		name=rf.check("name",Value("depthMap")).asString().c_str();
		voffset = rf.check("voffset",Value(0)).asInt();
		debug = rf.check("debug");


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

		//set up kinematic chains
		eyeL = new iCubEye("left");
		eyeR = new iCubEye("right");
		eyeL->setAllConstraints(false);
		eyeR->setAllConstraints(false);
		eyeL->releaseLink(0); eyeR->releaseLink(0);
		eyeL->releaseLink(1); eyeR->releaseLink(1);
		eyeL->releaseLink(2); eyeR->releaseLink(2);


		//open up ports
		portImgL=new BufferedPort<ImageOf<PixelRgb> >;
		string portImglName="/"+name+"/img:l";
		portImgL->open(portImglName.c_str());

		portImgR=new BufferedPort<ImageOf<PixelRgb> >;
		string portImgrName="/"+name+"/img:r";
		portImgR->open(portImgrName.c_str());

		portOmgL=new BufferedPort<ImageOf<PixelRgb> >;
		string portOmglName="/"+name+"/omg:l";
		portOmgL->open(portOmglName.c_str());

		portOmgR=new BufferedPort<ImageOf<PixelRgb> >;
		string portOmgrName="/"+name+"/omg:r";
		portOmgR->open(portOmgrName.c_str());

		portHeadPos=new BufferedPort<yarp::sig::Vector>;
		string portHPName="/"+name+"/pos:h";
		portHeadPos->open(portHPName.c_str());

		return true;

	}

	virtual void run()
	{

		Mat * Sl, * Sr, * Scl, * Scr;

		while (isStopping() != true) {


			//get the left and right cam images
			ImageOf<PixelRgb> *pImgL = portImgL->read(true);
			ImageOf<PixelRgb> *pImgR = portImgR->read(true);
			ImageOf<PixelRgb> &pOmgL= portOmgL->prepare();
			ImageOf<PixelRgb> &pOmgR= portOmgR->prepare();
			pOmgL.copy(*pImgL);
			pOmgR.copy(*pImgR);

			Sl = new Mat((IplImage *)pImgL->getIplImage(),true);
			Sr = new Mat((IplImage *)pImgR->getIplImage(),true);
			Scl = new Mat((IplImage *)pOmgL.getIplImage(),false);
			Scr = new Mat((IplImage *)pOmgR.getIplImage(),false);

			//Scl = new Mat(pImgL->height(), pImgL->width(), CV_8UC3);
			//Scr = new Mat(pImgR->height(), pImgR->width(), CV_8UC3);


			//get current head configuration
			Matrix Hl, Hr, H;
			Mat R(3,3, CV_64F);
			vector<double> T(3);
			yarp::sig::Vector *headAng = portHeadPos->read(true);
			yarp::sig::Vector angles(8); angles.zero();
			angles[3] = (*headAng)[0]; angles[4] = (*headAng)[1];
			angles[5] = (*headAng)[2]; angles[6] = (*headAng)[3];
			angles[7] = (*headAng)[4] + (*headAng)[5]/2.0;
			angles = PI*angles/180.0;
			Hl = eyeL->getH(angles);
			angles[7] = PI*((*headAng)[4] - (*headAng)[5]/2.0)/180.0;
			Hr = eyeR->getH(angles);

			//get the transform matrix from the left image to the right image
			H = SE3inv(Hr)*Hl;
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					R.at<double>(i,j) = H(i,j);
				}
				T.at(i) = H(i,3);
			}

			//rectify the images
			stereoRectify(Pl, Mat::zeros(4,1,CV_64F), Pr, Mat::zeros(4,1,CV_64F), Size(cxl*2, cyl*2), R, Mat(T), R1, R2, P1, P2, Q,	0);
			initUndistortRectifyMap(Pl, Mat(), R1, P1, Size(cxl*2, cyl*2), CV_32FC1, mpxL, mpyL);
			initUndistortRectifyMap(Pr, Mat(), R2, P2, Size(cxr*2, cyr*2), CV_32FC1, mpxR, mpyR);
			remap(*Sl, *Scl, mpxL, mpyL, INTER_LINEAR);
			remap(*Sr, *Scr, mpxR, mpyR, INTER_LINEAR);

			/*
			//adjust for vertical offset from left to right image
			int vamin, vamax, vadj;
			if (voffset < 0) {
				vamin = -voffset; vamax = Scr->rows-1; vadj = 0;
			} else if (voffset > 0) {
				vamin = 0; vamax = Scr->rows-1-voffset; vadj = voffset;
			} else {
				vamin = 0; vamax = Scr->rows-1; vadj = 0;
			}
			 */


			/*
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
			 */

			portOmgL->write();
			portOmgR->write();

			delete Sl, Sr, Scl, Scr;

		}

	}

	virtual void threadRelease()
	{


		portImgL->interrupt();
		portImgR->interrupt();
		portOmgL->interrupt();
		portOmgR->interrupt();
		portHeadPos->interrupt();

		portImgL->close();
		portImgR->close();
		portOmgL->close();
		portOmgR->close();
		portHeadPos->close();

		delete portImgL, portImgR, portOmgL, portOmgR;
		delete portHeadPos;

	}

};

class depthMapModule: public RFModule
{
protected:

	depthMapThread *thr;
	string name;

public:

	depthMapModule() { }

	virtual bool configure(ResourceFinder &rf)
	{

		//set up the rpc port
		name=rf.check("name",Value("depthMap")).asString().c_str();

		thr=new depthMapThread(rf);
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

	depthMapModule mod;

	return mod.runModule(rf);
}



