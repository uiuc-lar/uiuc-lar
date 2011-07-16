/*
 *  egoRemapper.cpp
 *
 * 	Logan Niehaus
 * 	7/12/11
 * 	module for remapping a provided calibrated image to an egosphere image.
 * 	head and torso joint state ports must be running for this module to	function,
 * 	as they are used to get the current head pose estimates for remapping.
 *  input images should	be only pixelfloats (intended for salience maps)
 *
 *  inputs:
 *  	/egoRemapper/img:l	-- left camera image stream (calibrated)
 *  	/egoRemapper/img:r	-- right camera image stream (calibrated)
 *
 *
 *  params: ([R]equired/[D]efault <Value>/[O]ptional)
 *
 *		mapw, maph					-- egosphere image width/height (D 320 x 240)
 *		azrange					-- total range of azimuth angles (D, -180 180)
 *		elrange					-- total range of elevation angles (D, -180 180)
 *		<camera parameters>		-- intrinsic calibration parameters for each camera listed under [CAMERA_CALIBRATION_LEFT/RIGHT].
 *										see iKinGazeCtrl docs for example of how to do this
 *  	name					-- module port basename (D /egoRemapper)
 *  	verbose					-- setting flag makes the module shoot debug info to stdout
 *
 *  outputs:
 *  	/egoRemapper/img:lo  -- left egosphere remapped image
 *  	/egoRemapper/img:ro  -- right egosphere remapped image
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
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

#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>
#include <iCub/vis/spherical_projection.h>

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::ctrl;

#define PI 3.14159

//YARP_DECLARE_DEVICES(icubmod)

class iCubCFrame : public iCubEye {

public:

	iCubCFrame(const string &_type)	: iCubEye(_type) {

		(*this)[getN()-2].setD(0.0);
		this->blockLink(getN()-2,0.0);
		this->blockLink(getN()-1,0.0);

	}

};


class egoRemapperThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelFloat> > *portImgL;
	BufferedPort<ImageOf<PixelFloat> > *portImgR;

	BufferedPort<ImageOf<PixelFloat> > *portImgLO;
	BufferedPort<ImageOf<PixelFloat> > *portImgRO;

	BufferedPort<yarp::sig::Vector> *portHAngIn;

	iCubEye * eyeL;
	iCubEye * eyeR;
	iCubCFrame * eyeC;

	//camera projection matrix/params
	Matrix Pl;
	Matrix Pr;
	double fxl, fyl, cxl, cyl;
	double fxr, fyr, cxr, cyr;

	//ego map params
	int erows, ecols;
	double azlo, azhi;
	double ello, elhi;

	//important matrices
	Matrix rootToEgo, egoToRoot;
	Matrix xrVals, yrVals, zrVals;

public:

	egoRemapperThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	bool getCamPrj(ResourceFinder &rf, const string &type, Matrix &Prj)
	{
		Bottle parType=rf.findGroup(type.c_str());
		string warning="Intrinsic parameters for "+type+" group not found";
		if (parType.size())
		{
			if (parType.check("w") && parType.check("h") &&
					parType.check("fx") && parType.check("fy"))
			{
				Prj.resize(3,4);
				Prj.zero();

				// we suppose that the center distorsion is already compensated
				Prj(0,2) = parType.find("w").asDouble()/2.0;
				Prj(1,2) = parType.find("h").asDouble()/2.0;
				Prj(0,0) = parType.find("fx").asDouble();
				Prj(1,1) = parType.find("fy").asDouble();
				Prj(2,2)=1.0;

				return true;
			}
			else
				fprintf(stdout,"%s\n",warning.c_str());
		}
		else
			fprintf(stdout,"%s\n",warning.c_str());

		return false;
	}


	virtual bool threadInit()
	{

		name=rf.check("name",Value("egoRemapper")).asString().c_str();
		erows = rf.check("maph", Value(320)).asInt();
		ecols = rf.check("mapw", Value(240)).asInt();

		//get output map size
		Bottle arng = rf.findGroup("azrange");
		Bottle erng = rf.findGroup("elrange");
		if (arng.isNull() || arng.size() < 3 || erng.isNull() || erng.size() < 3) {
			return false;
		} else {
			azlo = arng.get(1).asDouble(); azhi = arng.get(2).asDouble();
			ello = erng.get(1).asDouble(); elhi = erng.get(2).asDouble();
		}

		//get camera parameters
		if (getCamPrj(rf,"CAMERA_CALIBRATION_LEFT",Pl))
		{
			fxl = Pl(0,0); fyl = Pl(1,1);
			cxl = Pl(0,2); cyl = Pl(1,2);
		}
		else {
			fprintf(stderr,"No intrinsic camera parameters found for left camera, exiting\n");
			return false;
		}
		if (getCamPrj(rf,"CAMERA_CALIBRATION_RIGHT",Pr))
		{
			fxr = Pr(0,0); fyr = Pr(1,1);
			cxr = Pr(0,2); cyr = Pr(1,2);
		}
		else {
			fprintf(stderr,"No intrinsic camera parameters found for right camera, exiting\n");
			return false;
		}

		//open up ports
		portImgL=new BufferedPort<ImageOf<PixelFloat> >;
		string portImlName="/"+name+"/img:l";
		portImgL->open(portImlName.c_str());

		portImgR=new BufferedPort<ImageOf<PixelFloat> >;
		string portImrName="/"+name+"/img:r";
		portImgR->open(portImrName.c_str());

		portImgLO=new BufferedPort<ImageOf<PixelFloat> >;
		string portImlOName="/"+name+"/img:lo";
		portImgLO->open(portImlOName.c_str());

		portImgRO=new BufferedPort<ImageOf<PixelFloat> >;
		string portImrOName="/"+name+"/img:ro";
		portImgRO->open(portImrOName.c_str());

		portHAngIn=new BufferedPort<yarp::sig::Vector>;
		string portHAngName="/"+name+"/pos:h";
		portHAngIn->open(portHAngName.c_str());

		//define the root frame for the egosphere as the midpoint b/w eyes
		eyeL = new iCubEye("left");
		eyeR = new iCubEye("right");
		eyeL->setAllConstraints(false);
		eyeR->setAllConstraints(false);
		eyeC = new iCubCFrame("right");
		eyeC->setAllConstraints(false);
		eyeL->releaseLink(0); eyeC->releaseLink(0); eyeR->releaseLink(0);
		eyeL->releaseLink(1); eyeC->releaseLink(1); eyeR->releaseLink(1);
		eyeL->releaseLink(2); eyeC->releaseLink(2); eyeR->releaseLink(2);

		//get origin of egosphere frame
		yarp::sig::Vector q(eyeC->getDOF()); q=0.0;
		egoToRoot=eyeC->getH(q);
		rootToEgo=SE3inv(egoToRoot);

		//calculate and store the root xyz locs of the az/el map values for later use
		double az, el;
		xrVals.resize(erows, ecols); xrVals.zero();
		yrVals.resize(erows, ecols); yrVals.zero();
		zrVals.resize(erows, ecols); zrVals.zero();
		yarp::sig::Vector xyzEgo(4);
		yarp::sig::Vector xyzRoot(4);
		for (int mi,mj,i = 0; i < erows; i++) {
			for (int j = 0; j < ecols; j++) {

				az = j*((azhi-azlo)/(float)ecols)+azlo;
				el = i*((elhi-ello)/(float)erows)+180+ello;
				xyzEgo(0) = cos(PI*az/180.0)*cos(PI*el/180.0);
				xyzEgo(1) = -sin(PI*el/180.0);
				xyzEgo(2) = sin(PI*az/180.0)*cos(PI*el/180.0);
				xyzEgo(3) = 1.0;
				xyzRoot = egoToRoot*xyzEgo;

				//output map should range from azlo/elhi in top left corner
				mi = erows-1-i; mj = j;
				xrVals(mi,mj) = xyzRoot(0); yrVals(mi,mj) = xyzRoot(1); zrVals(mi,mj) = xyzRoot(2);
			}
		}

		return true;

	}

	virtual void run()
	{

		// get both input images
		ImageOf<PixelFloat> *pImgL = portImgL->read(false);
		ImageOf<PixelFloat> *pImgR = portImgR->read(false);

		//if we have both images
		if (pImgL && pImgR)
		{

			ImageOf<PixelFloat> &loutImg = portImgLO->prepare();
			ImageOf<PixelFloat> &routImg = portImgRO->prepare();

			//get the current head coordinates, assuming fixed torso for now
			yarp::sig::Vector *torsoAng = new yarp::sig::Vector(3);
			torsoAng->zero();
			yarp::sig::Vector *headAng = portHAngIn->read(true);
			Matrix Hl, Hr;

			//find the world to eye root matrices
			yarp::sig::Vector angles(8);
			angles[0] = (*torsoAng)[0]; angles[1] = (*torsoAng)[1]; angles[2] = (*torsoAng)[2];
			angles[3] = (*headAng)[0]; angles[4] = (*headAng)[1];
			angles[5] = (*headAng)[2]; angles[6] = (*headAng)[3];
			angles[7] = (*headAng)[4] + (*headAng)[5]/2.0;
			angles = PI*angles/180.0;
			Hl = SE3inv(eyeL->getH(angles));
			angles[7] = PI*((*headAng)[4] - (*headAng)[5]/2.0)/180.0;
			Hr = SE3inv(eyeR->getH(angles));

			//apply spherical warping
			ImageOf<PixelFloat> mapx, mapy;
			ImageOf<PixelFloat> oimg;
			mapx.resize(ecols,erows); mapy.resize(ecols,erows);	oimg.resize(ecols,erows);
			Mat * Iiml = new Mat(pImgL->height(), pImgL->width(), CV_32F, (void *)pImgL->getRawImage());
			Mat * Iimr = new Mat(pImgR->height(), pImgR->width(), CV_32F, (void *)pImgR->getRawImage());
			Mat * Oim = new Mat(erows, ecols, CV_32F, (void *)oimg.getRawImage());
			Mat * Mx = new Mat(erows, ecols, CV_32FC1, (void *)mapx.getRawImage());
			Mat * My = new Mat(erows, ecols, CV_32FC1, (void *)mapy.getRawImage());

			//flip images u/d to keep them consistent with output map convention
			flip(*Iiml, *Iiml, 0);
			flip(*Iimr, *Iimr, 0);

			//warp left image
			yarp::sig::Vector xyz(4);
			yarp::sig::Vector xyzEye(4);
			double u, v;

			//transform to xyz of current eye root
			for (int i = 0; i < erows; i++) {
				for (int j = 0; j < ecols; j++) {
					xyz(0) = xrVals(i,j); xyz(1) = yrVals(i,j); xyz(2) = zrVals(i,j); xyz(3) = 1.0;
					xyzEye = Hl*xyz;
					xyzEye(2) = -xyzEye(2);
					if (xyzEye(2) > 0) {
						u = xyzEye(0)/xyzEye(2); v = xyzEye(1)/xyzEye(2);
						mapx.pixel(j,i) = u*fxl+cxl;
						mapy.pixel(j,i) = v*fyl+cyl;
					} else {
						mapx.pixel(j,i) = -1.0;
						mapy.pixel(j,i) = -1.0;
					}
				}
			}

			//apply map
			remap(*Iiml, *Oim, *Mx, *My, INTER_LINEAR);
			loutImg.copy(oimg);

			//do same for right image
			for (int i = 0; i < erows; i++) {
				for (int j = 0; j < ecols; j++) {
					xyz(0) = xrVals(i,j); xyz(1) = yrVals(i,j); xyz(2) = zrVals(i,j); xyz(3) = 1.0;
					xyzEye = Hr*xyz;
					xyzEye(2) = -xyzEye(2);
					if (xyzEye(2) > 0) {
						u = xyzEye(0)/xyzEye(2); v = xyzEye(1)/xyzEye(2);
						mapx.pixel(j,i) = u*fxr+cxr;
						mapy.pixel(j,i) = v*fyr+cyr;
					} else {
						mapx.pixel(j,i) = -1.0;
						mapy.pixel(j,i) = -1.0;
					}
				}
			}

			//apply map
			remap(*Iimr, *Oim, *Mx, *My, INTER_LINEAR);
			routImg.copy(oimg);

			//write images out
			portImgLO->write();
			portImgRO->write();

			delete Iiml, Iimr, Oim, Mx, My;

		}
	}

	virtual void threadRelease()
	{

		portImgL->interrupt();
		portImgR->interrupt();
		portImgLO->interrupt();
		portImgRO->interrupt();
		portHAngIn->interrupt();

		portImgL->close();
		portImgR->close();
		portImgLO->close();
		portImgRO->close();
		portHAngIn->close();

		delete portImgL;
		delete portImgR;
		delete portImgLO;
		delete portImgRO;
		delete portHAngIn;

	}

};

class egoRemapperModule: public RFModule
{
protected:
	egoRemapperThread *thr;

public:
	egoRemapperModule() { }

	virtual bool configure(ResourceFinder &rf)
	{
		Time::turboBoost();

		thr=new egoRemapperThread(rf);
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

	//YARP_REGISTER_DEVICES(icubmod)

	Network yarp;

	if (!yarp.checkNetwork())
		return -1;

	ResourceFinder rf;

	rf.configure("ICUB_ROOT",argc,argv);

	egoRemapperModule mod;

	return mod.runModule(rf);
}



