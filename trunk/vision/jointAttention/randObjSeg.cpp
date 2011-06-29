/*
 *  randObjSeg
 *
 * 	Logan Niehaus
 * 	6/12/11
 * 	module that segments a particular kind of image, namely a color image taken
 *  from the robot with the expectation of a white background with dark or colorful
 *  objects sitting on a table. uses the watershed algorithm provided by opencv
 *  to do the segmentation. chooses a random segmented object and publishes its contour
 *  data.
 *
 *  segmentation desc: pixelwise OR taken between thresholded color and (inverted) intensity
 *  	salience maps, responding to colorful or dark objects. canny edge detection is applied
 *  	to these maps as well, and the pixelwise OR of the edge maps is inverted and used as a
 *  	mask for the combined thresholded image, in order to further separate occluded objects.
 *  	specks are filtered out and all proper object contours are given markers for the
 *  	watershed algorithm. all non-salient pixels in top half of image are marked as backdrop,
 *  	and all non-salient pixels in bottom half are marked as the table. the watershed algorithm
 *  	is run using these initial markings. basin markers corresponding to non-backdrop/table
 *  	pixels are then segmented. a random segmented object is chosen from these and its contour
 *  	data is published to port.
 *
 *
 *  inputs:
 *  	/randObjSeg/img:i	-- RGB input image as described above
 *
 *  params:
 *  	colthresh	-- color salience threshold (D 50.0)
 *  	inthresh	-- intensity salience threshold (D 130.0)
 *  	minobjsize	-- minimum segmented obj size; smaller objects not chosen (D 100.0)
 *  	name		-- module ports basename (D /randObjSeg)
 *  	rate		-- update rate in ms; objs segmented and published at this rate (D 50)
 *
 *  outputs:
 *  	/randObjSeg/img:o	-- same as input image, but with segmented object outlined
 *  	/randObjSeg/obj:o	-- Nx2 Matrix containing list of contour pixels for segmented obj.
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

#include <iCub/vis/Salience.h>
#include <iCub/vis/IntensitySalience.h>
#include "colorTransform.h"

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
using namespace iCub::vis;

#define PI 3.14159

class randObjSegThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *portImgIn;
	BufferedPort<ImageOf<PixelRgb> > *portImgOut;
	BufferedPort<yarp::sig::Matrix>	*portObjOut;

	IntensitySalience * ifilter;

	double colthresh, inthresh;
	int minobjsize;
	int trate;

public:

	randObjSegThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	virtual bool threadInit()
	{

		name=rf.check("name",Value("randObjSeg")).asString().c_str();
		colthresh = rf.check("colthresh",Value(50.0)).asDouble();
		inthresh = rf.check("inthresh",Value(130.0)).asDouble();
		minobjsize = rf.check("minobjsize",Value(100)).asInt();
		trate = rf.check("rate",Value(50)).asInt();
		this->setRate(trate);

		portImgIn=new BufferedPort<ImageOf<PixelRgb> >;
		string portInName="/"+name+"/img:i";
		portImgIn->open(portInName.c_str());

		portImgOut=new BufferedPort<ImageOf<PixelRgb> >;
		string portOutName="/"+name+"/img:o";
		portImgOut->open(portOutName.c_str());

		portObjOut=new BufferedPort<yarp::sig::Matrix>;
		string portObjName="/"+name+"/obj:o";
		portObjOut->open(portObjName.c_str());

		ifilter = new IntensitySalience;
		ifilter->open(rf);

		srand(time(NULL));

		return true;

	}

	virtual void run()
	{

		// get inputs
		ImageOf<PixelRgb> *pImgIn=portImgIn->read(false);
		ImageOf<PixelRgb> *cDest = NULL;
		ImageOf<PixelFloat> *cSal = NULL;
		ImageOf<PixelFloat> *iSal = NULL;


		//if there is a camera image, use that
		if (pImgIn)
		{

			cDest = new ImageOf<PixelRgb>;
			cSal = new ImageOf<PixelFloat>;
			iSal = new ImageOf<PixelFloat>;
			ifilter->apply(*pImgIn, *cDest, *iSal);

			//apply color normalization and invert to get color based salience
			normalizeColor(*pImgIn, *cDest, *cSal);

			ImageOf<PixelRgb> &imgOut= portImgOut->prepare();

			Mat Y,Z,Tmp;
			Mat * M = new Mat(cSal->height(), cSal->width(), CV_32F, (void *)cSal->getRawImage());
			Mat * N = new Mat(iSal->height(), iSal->width(), CV_32F, (void *)iSal->getRawImage());
			Mat * Mt = new Mat(cSal->height(), cSal->width(), CV_32F);
			Mat * Nt = new Mat(iSal->height(), iSal->width(), CV_32F);
			Mat * mark = new Mat(cSal->height(), cSal->width(), CV_32S);

			//do a rough canny edge detection
			M->convertTo(Tmp, CV_8UC1);
			Canny(Tmp, Y, 140, 150);
			N->convertTo(Tmp, CV_8UC1);
			Canny(Tmp, Z, 140, 150);
			max(Y,Z,Y);

			//get the binary image of salient areas (high color and dark areas)
			threshold(*M, *Mt, colthresh, 255.0, CV_THRESH_BINARY);
			threshold(*N, *Nt, inthresh, 255.0, CV_THRESH_BINARY_INV);
			max(*Mt,*Nt,*Mt);

			//clear away some of the flecks inside of the blobs
			dilate(*Mt, *M, Mat());
			dilate(*M, *M, Mat());
			erode(*M, *M, Mat());
			erode(*M, *M, Mat());
			erode(*M, *M, Mat());

			//use canny boundaries to cut some lines b/w different objects
			M->setTo(Scalar(0),Y);

			//get list of blobs, do rough marking for watershed algorithm
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			Mat X;
			int nblobs = 3;
			double mv;
			M->convertTo(X,CV_8UC1);
			findContours(X, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
			mark->setTo(Scalar(0));
			for (int i = 0; i < contours.size(); i++) {
				if (contourArea(Mat(contours[i])) > minobjsize) {

					//check here if shape is roughly compact
					mv = arcLength(Mat(contours[i]),true);
					if (4*PI*contourArea(Mat(contours[i]))/(mv*mv) < .3) {

						//for complex shapes, using contours as marker init works better
						drawContours(*mark, contours, i, Scalar(nblobs), -1);

					} else {

						//for very compact shapes, just use the central moment
						Moments m = moments(Mat(contours[i]));
						mark->at<int>(Point2f(m.m10/m.m00,m.m01/m.m00)) = nblobs;

					}
					nblobs++;
				}
			}

			//mark pixels which are definitely in the background
			//assume a bit of structure here - namely there is a table and background
			for (int i = 0; i < 10; i++) {
				dilate(*M,*M,Mat());
			}
			for (int i = 0; i < M->rows; i++) {
				for (int j = 0; j < M->cols; j++) {
					if (M->at<float>(i,j) == 0) {
						if (i > cSal->height()/2+50) {
							mark->at<float>(i,j) = 1; //table
						}
						if (i < cSal->height()/2-50) {
							mark->at<float>(i,j) = 2; //backdrop
						}
					}
				}
			}

			//run watershed algorithm
			imgOut.copy(*pImgIn);
			Mat I((IplImage *)imgOut.getIplImage(), false);
			watershed(I, *mark);

			//zero out all marked background pixels
			mark->convertTo(*M, CV_32F);
			for (int i = 0; i < M->rows; i++) {
				for (int j = 0; j < M->cols; j++) {
					if (M->at<float>(i,j) <= 2) {
						M->at<float>(i,j) = 0;
					} else {
						M->at<float>(i,j) *= 255.0/(float)nblobs;
					}
				}
			}
			contours.clear();
			M->convertTo(X,CV_8UC1);
			findContours(X, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

			//pick a random segmented object
			int iBlob = -1;
			vector<int> vBlobs(0);
			for (int i = 0; i < contours.size(); i++) {
				if (contourArea(Mat(contours[i])) > minobjsize) {
					vBlobs.push_back(i);
				}
			}
			if (vBlobs.size() > 0) {
				iBlob = vBlobs.at(rand() % vBlobs.size());
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


			delete Mt;
			delete Nt;
			delete mark;
			delete cDest;
			delete iSal;
			delete cSal;

		}

	}

	virtual void threadRelease()
	{

		portImgIn->interrupt();
		portImgOut->interrupt();
		portObjOut->interrupt();

		delete portImgIn;
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
