/*
 *  egoRemapper.cpp
 *
 * 	Logan Niehaus
 * 	7/12/11
 * 	module for remapping a provided calibrated image to an egosphere image.
 * 	head joint state port must be running for this module to function,
 * 	as it is used to get the current head pose estimates for remapping.
 *  input images should	be only pixelfloats (intended for salience maps).
 *  each map is placed in a mosaic, and only values in the current field
 *  of view are updated each time a new image is received. these mosaics are
 *  combined as a weighted sum into an aggregate attention map, which is also
 *  published.
 *
 *  inputs:
 *  	/egoRemapper/map<0-N>:l	-- left camera image stream (calibrated)
 *  	/egoRemapper/map<0-N>:r	-- right camera image stream (calibrated)
 *
 *
 *  params: ([R]equired/[D]efault <Value>/[O]ptional)
 *
 *		mapw, maph				-- egosphere image width/height (D 320 x 240)
 *		azrange					-- total range of azimuth angles in deg (R, ex: 0 180)
 *		elrange					-- total range of elevation angles in deg (R, ex: -90 90)
 *		<camera parameters>		-- intrinsic calibration parameters for each camera listed under [CAMERA_CALIBRATION_LEFT/RIGHT].
 *										see iKinGazeCtrl docs for example of how to do this
 *		weights					-- vector of weights applied to each input map, (ex: weights 0.1 1.0 0.5)
 *									length should be equal to nmaps (R)
 *		decays					-- vector of decay factors applied to each mosaic before updates are
 *									applied. decays should be near zero as (1-decay) will be premultiplied to the mosaic
 *		nmaps					-- number of stereo map ports to open. maps are labeled /egoRemapper/mapN:l, with
 *										N ranging from 0 to nmaps-1 (D 1)
 *  	name					-- module port basename (D /egoRemapper)
 *  	verbose					-- setting flag makes the module shoot debug info to stdout
 *
 *  outputs:
 *  	/egoRemapper/map<0-N>:lo  -- left egosphere remapped image
 *  	/egoRemapper/map<0-N>:ro  -- right egosphere remapped image
 *  	/egoRemapper/agg:lo		  -- left aggregate egosphere remapped image
 *  	/egoRemapper/agg:ro		  -- right aggregate egosphere remapped image
 *  	/egoRemapper/sal:lo		  -- left aggregate un-warped image
 *  	/egoRemapper/sal:ro		  -- right aggregate un-warped image
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
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::ctrl;

#define PI 3.14159

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

	BufferedPort<yarp::sig::Vector> *portHAngIn;
	BufferedPort<ImageOf<PixelFloat> > **portImgL;
	BufferedPort<ImageOf<PixelFloat> > **portImgR;
	BufferedPort<ImageOf<PixelFloat> > **portImgLO;
	BufferedPort<ImageOf<PixelFloat> > **portImgRO;
	BufferedPort<ImageOf<PixelFloat> > *portAggL;
	BufferedPort<ImageOf<PixelFloat> > *portAggR;
	BufferedPort<ImageOf<PixelFloat> > *portSalLO;
	BufferedPort<ImageOf<PixelFloat> > *portSalRO;
	int nmaps;

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
	Mat * Mxl, * Myl, * Mxr, * Myr;

	//mosaics and auxiliary objects
	Mat ** mosaicl, ** mosaicr;
	Mat * updMskL, * updMskR;
	Mat * whtBlk;
	Mat * maggL, * maggR;
	Mat * msagL, * msagR;

	//aggregate image parameters
	yarp::sig::Vector weights;
	yarp::sig::Vector decays;

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
		nmaps = rf.check("nmaps", Value(1)).asInt();
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

		//get aggregator weights, decays
		Bottle wvals = rf.findGroup("weights");
		Bottle dvals = rf.findGroup("decays");
		if (wvals.isNull())
			weights.resize(nmaps); weights = 1.0;
		if (dvals.isNull())
			decays.resize(nmaps); decays = 1.0;
		for (int i = 0; i < nmaps; i++) {
			if (wvals.size()-1 < i)
				wvals.addDouble(1.0);
			if (dvals.size()-1 < i)
				dvals.addDouble(0.0);
			weights.push_back(wvals.get(i+1).asDouble());
			decays.push_back(dvals.get(i+1).asDouble());
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
		string tmpName;
		char mpnStr[10];
		portImgL = new BufferedPort<ImageOf<PixelFloat> > * [nmaps];
		portImgR = new BufferedPort<ImageOf<PixelFloat> > * [nmaps];
		portImgLO = new BufferedPort<ImageOf<PixelFloat> > * [nmaps];
		portImgRO = new BufferedPort<ImageOf<PixelFloat> > * [nmaps];
		for (int i = 0; i < nmaps; i++) {

			sprintf(mpnStr, "map%d", i);

			portImgL[i] = new BufferedPort<ImageOf<PixelFloat> >;
			tmpName = "/" + name + "/" + mpnStr + ":l";
			portImgL[i]->open(tmpName.c_str());

			portImgR[i] = new BufferedPort<ImageOf<PixelFloat> >;
			tmpName = "/" + name + "/" + mpnStr + ":r";
			portImgR[i]->open(tmpName.c_str());

			portImgLO[i] = new BufferedPort<ImageOf<PixelFloat> >;
			tmpName = "/" + name + "/" + mpnStr + ":lo";
			portImgLO[i]->open(tmpName.c_str());

			portImgRO[i] = new BufferedPort<ImageOf<PixelFloat> >;
			tmpName = "/" + name + "/" + mpnStr + ":ro";
			portImgRO[i]->open(tmpName.c_str());

		}

		portAggL=new BufferedPort<ImageOf<PixelFloat> >;
		string portAggLName="/"+name+"/agg:l";
		portAggL->open(portAggLName.c_str());

		portAggR=new BufferedPort<ImageOf<PixelFloat> >;
		string portAggRName="/"+name+"/agg:r";
		portAggR->open(portAggRName.c_str());

		portSalLO=new BufferedPort<ImageOf<PixelFloat> >;
		string portSalLName="/"+name+"/sal:lo";
		portSalLO->open(portSalLName.c_str());

		portSalRO=new BufferedPort<ImageOf<PixelFloat> >;
		string portSalRName="/"+name+"/sal:ro";
		portSalRO->open(portSalRName.c_str());

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

				az = j*((azhi-azlo)/(float)ecols)+azlo+90;
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

		//initialize storage for maps
		Mxl = new Mat(erows, ecols, CV_32FC1);
		Myl = new Mat(erows, ecols, CV_32FC1);
		Mxr = new Mat(erows, ecols, CV_32FC1);
		Myr = new Mat(erows, ecols, CV_32FC1);

		//initialize mosaics
		mosaicl = new Mat * [nmaps];
		mosaicr = new Mat * [nmaps];
		for (int i = 0; i < nmaps; i++) {
			mosaicl[i] = new Mat(erows, ecols, CV_32F);
			mosaicl[i]->setTo(Scalar(0));
			mosaicr[i] = new Mat(erows, ecols, CV_32F);
			mosaicr[i]->setTo(Scalar(0));
		}
		updMskL = new Mat(erows, ecols, CV_32F);
		updMskR = new Mat(erows, ecols, CV_32F);
		whtBlk = new Mat(cyl*2, cxl*2, CV_32F);
		whtBlk->setTo(Scalar(255));

		return true;

	}

	virtual void run()
	{

		//get the current head coordinates, assuming fixed torso for now
		Matrix Hl, Hr;
		yarp::sig::Vector *torsoAng = new yarp::sig::Vector(3);
		yarp::sig::Vector *headAng = portHAngIn->read(true);
		torsoAng->zero();

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

		//create spherical warping map for current configuration
		Mxl->setTo(Scalar(-1)); Myl->setTo(Scalar(-1));
		Mxr->setTo(Scalar(-1)); Myr->setTo(Scalar(-1));

		//transform to xyz of current eye root
		yarp::sig::Vector xyz(4);
		yarp::sig::Vector xyzLE(4);
		yarp::sig::Vector xyzRE(4);
		double u, v;
		for (int i = 0; i < erows; i++) {
			for (int j = 0; j < ecols; j++) {
				xyz(0) = xrVals(i,j); xyz(1) = yrVals(i,j); xyz(2) = zrVals(i,j); xyz(3) = 1.0;
				xyzLE = Hl*xyz;
				xyzRE = Hr*xyz;
				xyzLE(2) = -xyzLE(2); xyzRE(2) = -xyzRE(2);
				if (xyzLE(2) > 0) {
					u = xyzLE(0)/xyzLE(2); v = xyzLE(1)/xyzLE(2);
					Mxl->at<float>(i,j) = u*fxl+cxl;
					Myl->at<float>(i,j) = v*fyl+cyl;
				}
				if (xyzRE(2) > 0) {
					u = xyzRE(0)/xyzRE(2); v = xyzRE(1)/xyzRE(2);
					Mxr->at<float>(i,j) = u*fxr+cxr;
					Myr->at<float>(i,j) = v*fyr+cyr;
				}
			}
		}

		//create the update mask for mosaics
		remap(*whtBlk, *updMskL, *Mxl, *Myl, INTER_LINEAR);
		remap(*whtBlk, *updMskR, *Mxr, *Myr, INTER_LINEAR);
		erode(*updMskL, *updMskL, Mat());
		erode(*updMskR, *updMskR, Mat());

		//prepare aggregator images
		ImageOf<PixelFloat> &laggImg = portAggL->prepare();
		ImageOf<PixelFloat> &raggImg = portAggR->prepare();
		laggImg.resize(ecols, erows); laggImg.zero();
		raggImg.resize(ecols,erows); raggImg.zero();
		maggL = new Mat(laggImg.height(), laggImg.width(), CV_32F, (void *)laggImg.getRawImage());
		maggR = new Mat(raggImg.height(), raggImg.width(), CV_32F, (void *)raggImg.getRawImage());

		ImageOf<PixelFloat> &sagL = portSalLO->prepare();
		ImageOf<PixelFloat> &sagR = portSalRO->prepare();
		sagL.resize(cxl*2, cyl*2); sagL.zero();
		sagR.resize(cxr*2 ,cyr*2); sagR.zero();
		msagL = new Mat(sagL.height(), sagL.width(), CV_32F, (void *)sagL.getRawImage());
		msagR = new Mat(sagR.height(), sagR.width(), CV_32F, (void *)sagR.getRawImage());

		//check for any input images that are available, and apply map
		ImageOf<PixelFloat> *pImgL;
		ImageOf<PixelFloat> *pImgR;
		Mat * Iiml, * Iimr, *Oiml, *Oimr;
		for (int i = 0; i < nmaps; i++) {

			pImgL = portImgL[i]->read(false);
			pImgR = portImgR[i]->read(false);
			(*mosaicl[i]) = decays[i]*(*mosaicl[i]);
			(*mosaicr[i]) = decays[i]*(*mosaicr[i]);

			if (pImgL && pImgR)
			{

				ImageOf<PixelFloat> &loutImg = portImgLO[i]->prepare();
				ImageOf<PixelFloat> &routImg = portImgRO[i]->prepare();
				loutImg.resize(ecols,erows); routImg.resize(ecols,erows);

				Iiml = new Mat(pImgL->height(), pImgL->width(), CV_32F, (void *)pImgL->getRawImage());
				Iimr = new Mat(pImgR->height(), pImgR->width(), CV_32F, (void *)pImgR->getRawImage());
				Oiml = new Mat(loutImg.height(), loutImg.width(), CV_32F, (void *)loutImg.getRawImage());
				Oimr = new Mat(routImg.height(), routImg.width(), CV_32F, (void *)routImg.getRawImage());

				//add unwarped maps straight to the normal aggregator
				(*msagL) = (*msagL) + weights[i]*(*Iiml);
				(*msagR) = (*msagR) + weights[i]*(*Iimr);

				//flip images to keep them consistent with output map convention
				flip(*Iiml, *Iiml, -1);
				flip(*Iimr, *Iimr, -1);

				//apply maps
				remap(*Iiml, *Oiml, *Mxl, *Myl, INTER_LINEAR);
				remap(*Iimr, *Oimr, *Mxr, *Myr, INTER_LINEAR);

				//update mosaic
				for (int j = 0; j < erows; j++) {
					for (int k = 0; k < ecols; k++) {
						if (updMskL->at<float>(j,k) > 0)
							mosaicl[i]->at<float>(j,k) = Oiml->at<float>(j,k);
						if (updMskR->at<float>(j,k) > 0)
							mosaicr[i]->at<float>(j,k) = Oimr->at<float>(j,k);
					}
				}
				flip(*Oiml, *Oiml, 1);
				flip(*Oimr, *Oimr, 1);

				//write to port and cleanup
				portImgLO[i]->write();
				portImgRO[i]->write();
				delete Iiml, Iimr, Oiml, Oimr;

			}

			(*maggL) = (*maggL) + weights[i]*(*mosaicl[i]);
			(*maggR) = (*maggR) + weights[i]*(*mosaicr[i]);

		}

		//flip final image lr again (not sure why?)
		flip(*maggL, *maggL, 1);
		flip(*maggR, *maggR, 1);

		//saturate saliences at 0
		threshold(*maggL, *maggL, 0, 0, CV_THRESH_TOZERO);
		threshold(*maggR, *maggR, 0, 0, CV_THRESH_TOZERO);
		threshold(*msagL, *msagL, 0, 0, CV_THRESH_TOZERO);
		threshold(*msagR, *msagR, 0, 0, CV_THRESH_TOZERO);

		//write aggregated maps, clean
		portSalLO->write();
		portSalRO->write();
		portAggL->write();
		portAggR->write();
		delete maggL, maggR, msagL, msagR;

	}

	virtual void threadRelease()
	{

		for (int i = 0; i < nmaps; i++) {

			portImgL[i]->interrupt();
			portImgR[i]->interrupt();
			portImgLO[i]->interrupt();
			portImgRO[i]->interrupt();

			portImgL[i]->close();
			portImgR[i]->close();
			portImgLO[i]->close();
			portImgRO[i]->close();

			delete portImgL[i];
			delete portImgR[i];
			delete portImgLO[i];
			delete portImgRO[i];

			delete mosaicl[i];
			delete mosaicr[i];

		}

		portHAngIn->interrupt();
		portHAngIn->close();
		portAggL->interrupt();
		portAggL->close();
		portAggR->interrupt();
		portAggR->close();
		portSalLO->interrupt();
		portSalLO->close();
		portSalRO->interrupt();
		portSalRO->close();

		delete portHAngIn;
		delete portImgL, portImgR;
		delete portImgLO, portImgRO;
		delete portAggL, portAggR;
		delete portSalLO, portSalRO;

		delete Mxl, Myl, Mxr, Myr;
		delete updMskL, updMskR, whtBlk, mosaicl, mosaicr;


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

	Network yarp;

	if (!yarp.checkNetwork())
		return -1;

	ResourceFinder rf;

	rf.configure("ICUB_ROOT",argc,argv);

	egoRemapperModule mod;

	return mod.runModule(rf);
}



