/*
 *  objectSegmentation.cpp
 *
 * 	Logan Niehaus
 * 	6/12/11 (overhaul 7/30/11)
 * 	module that segments best on color image taken from the robot with the
 *  expectation of a white background with dark or colorful objects sitting on a table.
 *  salience maps are used to pre-label watershed basins, after which opencv's watershed
 *  algorithm is used to segment. publishes a binary image from which contours
 *  of segmented objects can be easily extracted.
 *
 *  segmentation desc: pixelwise OR taken between thresholded the provided
 *  	salience maps. canny edge detection is applied to these maps as well,
 *  	and the pixelwise OR of the edge maps is inverted and used as a
 *  	mask for the combined thresholded image, in order to further separate occluded objects.
 *  	specks are filtered out and all proper object contours are given markers for the
 *  	watershed algorithm. all non-salient pixels in top half of image are marked as backdrop,
 *  	and all non-salient pixels in bottom half are marked as the table. the watershed algorithm
 *  	is run using these initial markings. basin markers corresponding to non-backdrop/table
 *  	pixels are then segmented. a binary image is produced, from which segmented object
 *  	contours can be found.
 *
 *
 *  inputs:
 *  	/objectSeg/img:i		-- RGB input image as described above
 *  	/objectSeg/map<0-N>:i 	-- saliency maps to use in labeling of basins
 *
 *  params:
 *  	nmaps		-- number of map ports to open. maps are labeled /objectSeg/mapN:l, with
 *							N ranging from 0 to nmaps-1 (D 1)
 *  	thresh		-- vector of threshold values to apply to salience maps. must be length = nmaps
 *  	minobjsize	-- minimum segmented obj size; smaller objects not chosen (D 100.0)
 *  	maxobjsize	-- maximum segmented obj size; larger objects not chosen (D 5000.0)
 *  	tableloc	-- number of pixels above the horizontal center line the workspace extends. only
 *  					objects with centroids below this line will be segmented (D 10)
 *  	cplow, cphi	-- canny thresholding parameters. see canny edge detection algorithm explanation (D 140, 150)
 *  	name		-- module ports basename (D /objectSeg)
 *  	rate		-- update rate in ms; objs segmented and published at this rate (D 50)
 *
 *  outputs:
 *  	/objectSeg/map:o	-- binary image with object regions labeled as 1
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

#define PI 3.14159

class objectSegThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *portImgIn;
	BufferedPort<ImageOf<PixelFloat> > **portMapIn;
	BufferedPort<ImageOf<PixelFloat> > *portImgOut;

	yarp::sig::Vector threshs;

	int nmaps;
	int minobjsize, maxobjsize, tableloc;
	double cplow, cphi;

public:

	objectSegThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	virtual bool threadInit()
	{

		name=rf.check("name",Value("objectSeg")).asString().c_str();
		nmaps = rf.check("nmaps",Value(1)).asInt();
		cplow = rf.check("cplow",Value(140.0)).asDouble();
		cphi = rf.check("cphi",Value(150.0)).asDouble();
		minobjsize = rf.check("minobjsize",Value(100)).asInt();
		maxobjsize = rf.check("maxobjsize",Value(5000)).asInt();
		tableloc = rf.check("tableloc",Value(10)).asInt();

		Bottle tvs = rf.findGroup("thresh");
		threshs.resize(nmaps);
		if (tvs.size() != nmaps+1) {
			fprintf(stderr,"please specify threshold vector with length equal to nmaps\n");
			return false;
		} else {
			 for (int i = 0; i < nmaps; i++) {
				 threshs[i] = tvs.get(i+1).asDouble();
			 }
		}

		//open up ports
		string tmpName;
		char mpnStr[10];
		portMapIn = new BufferedPort<ImageOf<PixelFloat> > * [nmaps];
		for (int i = 0; i < nmaps; i++) {
			sprintf(mpnStr, "map%d", i);
			portMapIn[i] = new BufferedPort<ImageOf<PixelFloat> >;
			tmpName = "/" + name + "/" + mpnStr + ":i";
			portMapIn[i]->open(tmpName.c_str());

		}

		portImgIn=new BufferedPort<ImageOf<PixelRgb> >;
		string portInName="/"+name+"/img:i";
		portImgIn->open(portInName.c_str());

		portImgOut=new BufferedPort<ImageOf<PixelFloat> >;
		string portOutName="/"+name+"/img:o";
		portImgOut->open(portOutName.c_str());

		return true;

	}

	virtual void run()
	{

		// get inputs
		ImageOf<PixelRgb> *pImgIn=portImgIn->read(false);

		//if there is a camera image, use that
		if (pImgIn)
		{

			ImageOf<PixelFloat> &imgOut= portImgOut->prepare();
			imgOut.resize(*pImgIn); imgOut.zero();

			Mat Y = Mat::zeros(pImgIn->height(), pImgIn->width(), CV_8UC1);
			Mat T = Mat(pImgIn->height(), pImgIn->width(), CV_32F, (void *)imgOut.getRawImage());
			Mat Z, Tmp;
			Mat * M, * N;
			Mat * mark = new Mat(pImgIn->height(), pImgIn->width(), CV_32S);

			for (int i = 0; i < nmaps; i++) {

				ImageOf<PixelFloat> *pMapIn=portMapIn[i]->read(true);

				M = new Mat(pMapIn->height(), pMapIn->width(), CV_32F, (void *)pMapIn->getRawImage());

				//do a rough canny edge detection
				M->convertTo(Tmp, CV_8UC1);
				Canny(Tmp, Z, cplow, cphi);
				max(Y,Z,Y);

				//get the binary image of salient areas (high color and dark areas)
				threshold(*M, Tmp, threshs[i], 255.0, CV_THRESH_BINARY);
				max(T,Tmp,T);

				delete M;

			}


			//clear away some of the flecks inside of the blobs
			dilate(T, T, Mat());
			erode(T, T, Mat());
			erode(T, T, Mat());

			//use canny boundaries to cut some lines b/w different objects
			T.setTo(Scalar(0),Y);

			//get list of blobs, do rough marking for watershed algorithm
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			Mat X;
			int nblobs = 3;
			double mv;
			T.convertTo(X,CV_8UC1);
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

			//mark->convertTo(Z, CV_32F);
			//Z = Z*(255.0/(float)nblobs);

			//mark pixels which are definitely in the background
			//assume a bit of structure here - namely there is a table and background
			for (int i = 0; i < 10; i++) {
				dilate(T,T,Mat());
			}
			for (int i = 0; i < T.rows; i++) {
				for (int j = 0; j < T.cols; j++) {
					if (T.at<float>(i,j) == 0) {
						if (i > imgOut.height()/2+50) {
							mark->at<float>(i,j) = 1; //table
						}
						if (i < imgOut.height()/2-50) {
							mark->at<float>(i,j) = 2; //backdrop
						}
					}
				}
			}

			//run watershed algorithm
			Mat I((IplImage *)pImgIn->getIplImage(), false);
			watershed(I, *mark);

			//zero out all marked background pixels
			mark->convertTo(T, CV_32F);
			for (int i = 0; i < T.rows; i++) {
				for (int j = 0; j < T.cols; j++) {
					if (T.at<float>(i,j) <= 2) {
						T.at<float>(i,j) = 0;
					} else {
						T.at<float>(i,j) *= 255.0/(float)nblobs;
					}
				}
			}

			contours.clear();
			T.convertTo(X,CV_8UC1);
			findContours(X, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

			//filter out certain unwanted objects and publish the binary image
			for (int i = 0; i < contours.size(); i++) {
				if (contourArea(Mat(contours[i])) > minobjsize &&
						contourArea(Mat(contours[i])) < maxobjsize) {
					//check to make sure the object is on the table as user has defined
					Moments m = moments(Mat(contours[i]));
					//check bounding dimensions to eliminate weird edge artifacts
					RotatedRect RR = minAreaRect(Mat(contours[i]));
					double arat;
					if (RR.size.width > RR.size.height) {
						arat = RR.size.width/RR.size.height;
					} else {
						arat = RR.size.height/RR.size.width;
					}
					if (m.m01/m.m00 > (pImgIn->height()/2 - tableloc) &&
							arat < 10.0) {
						drawContours(T, contours, i, Scalar(255.0), -1);
					}
				}
			}

			//Z.copyTo(T);
			portImgOut->write();

			delete mark;

		}

	}

	virtual void threadRelease()
	{

		for (int i = 0; i < nmaps; i++) {

			portMapIn[i]->interrupt();
			portMapIn[i]->close();
			delete portMapIn[i];

		}

		portImgIn->interrupt();
		portImgOut->interrupt();

		portImgIn->close();
		portImgOut->close();

		delete portImgIn;
		delete portImgOut;
		delete portMapIn;

	}

};

class objectSegModule: public RFModule
{

protected:

	objectSegThread *thr;

public:
	objectSegModule() { }

	virtual bool configure(ResourceFinder &rf)
	{
		Time::turboBoost();

		thr=new objectSegThread(rf);
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

	objectSegModule mod;

	return mod.runModule(rf);
}

