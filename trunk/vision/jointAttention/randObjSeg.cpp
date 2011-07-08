/*
 *  randObjSeg
 *
 * 	Logan Niehaus
 * 	6/12/11 (updated 7/7/11)
 * 	receives a binary image (PixelFloat) in, which is a mask for all regions to be segmented.
 * 	draws contours around all non-zero blobs and selects a random one to be sent to port.
 * 	module no longer handles segmentation directly, and selects only a random object
 *
 *  inputs:
 *  	/randObjSeg/img:i	-- pixelfloat image with all non-zero pixels to be segmented.
 *  	/randObjSeg/ref:i	-- rgb reference image, used to create debug output image
 *
 *  params:
 *  	name		-- module ports basename (D /randObjSeg)
 *  	rate		-- update rate in ms; objs segmented and published at this rate (D 50)
 *
 *  outputs:
 *  	/randObjSeg/obj:o	-- Nx2 Matrix containing list of contour pixels for segmented obj.
 *  	/randObjSeg/img:o	-- rgb output image with the selected contour outlined in red.
 *
 *  TODO:
 *  	need to make this a lot more configurable, and a lot more general. pretty hackish right now
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/math/Math.h>

#include <cv.h>

#include <string>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

class randObjSegThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelFloat> > *portImgIn;
	BufferedPort<ImageOf<PixelRgb> > *portRefIn;
	BufferedPort<ImageOf<PixelRgb> > *portImgOut;
	BufferedPort<yarp::sig::Matrix>	*portObjOut;

	int trate;

public:

	randObjSegThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	virtual bool threadInit()
	{

		name=rf.check("name",Value("randObjSeg")).asString().c_str();
		trate = rf.check("rate",Value(50)).asInt();
		this->setRate(trate);

		portImgIn=new BufferedPort<ImageOf<PixelFloat> >;
		string portInName="/"+name+"/img:i";
		portImgIn->open(portInName.c_str());

		portRefIn=new BufferedPort<ImageOf<PixelRgb> >;
		string portRefName="/"+name+"/ref:i";
		portRefIn->open(portRefName.c_str());

		portImgOut=new BufferedPort<ImageOf<PixelRgb> >;
		string portOutName="/"+name+"/img:o";
		portImgOut->open(portOutName.c_str());

		portObjOut=new BufferedPort<yarp::sig::Matrix>;
		string portObjName="/"+name+"/obj:o";
		portObjOut->open(portObjName.c_str());

		srand(time(NULL));

		return true;

	}

	virtual void run()
	{

		// get inputs
		ImageOf<PixelFloat> *pSegIn=portImgIn->read(false);

		//if there is a seg image, use that
		if (pSegIn)
		{

			ImageOf<PixelRgb> *pImgIn=portRefIn->read(false);
			ImageOf<PixelRgb> &imgOut= portImgOut->prepare();

			Mat * M = new Mat(pSegIn->height(), pSegIn->width(), CV_32F, (void *)pSegIn->getRawImage());
			Mat X;
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;

			M->convertTo(X,CV_8UC1);
			findContours(X, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

			//pick a random segmented object
			int iBlob = -1;
			if (contours.size() > 0) {
				iBlob = rand() % contours.size();
			}

			if (pImgIn) {
				imgOut.copy(*pImgIn);
			} else {
				imgOut.copy(*pSegIn);
			}

			//write the contour pixels of the object to port
			Matrix &object = portObjOut->prepare();
			yarp::sig::Vector xs, ys;
			vector<Point> cts = contours[iBlob];
			for (int i = 0; i < cts.size(); i++) {
				xs.push_back(cts.at(i).y);
				ys.push_back(cts.at(i).x);
				imgOut.pixel(cts.at(i).x,cts.at(i).y) = PixelRgb(255, 0, 0);
			}
			object.resize(xs.size(),2);
			object.setCol(0,xs);
			object.setCol(1,ys);

			portObjOut->write();
			portImgOut->write();

			delete M;

		}

	}

	virtual void threadRelease()
	{

		portImgIn->interrupt();
		portRefIn->interrupt();
		portImgOut->interrupt();
		portObjOut->interrupt();

		delete portImgIn;
		delete portRefIn;
		delete portImgOut;
		delete portObjOut;

	}

};

class randObjSegModule: public RFModule
{

protected:

	randObjSegThread *thr;

public:
	randObjSegModule() { }

	virtual bool configure(ResourceFinder &rf)
	{
		Time::turboBoost();

		thr=new randObjSegThread(rf);
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

	randObjSegModule mod;

	return mod.runModule(rf);
}
