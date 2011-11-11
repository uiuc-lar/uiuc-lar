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
 *  salienceAggregator.cpp
 *
 * 	Logan Niehaus
 * 	5/7/11 (overhaul 6/21/11)
 * 	demo-ish module for testing the joint attention capabilities. updated to use
 * 		watershed-based object segmentation documented in randObjSeg.cpp. segmented
 * 		object with highest average salience value is written to port.
 *
 * inputs:
 * 		/objectSalience/img:i	-- input image from icub
 * 		/objectSalience/jas:i 	-- salience map from joint attention system
 *
 * params:
 *  	colthresh	-- color salience threshold (D 50.0)
 *  	inthresh	-- intensity salience threshold (D 130.0)
 *  	minobjsize	-- minimum segmented obj size; smaller objects not chosen (D 100.0)
 *  	name		-- module ports basename (D /objectSalience)
 *
 * outputs:
 * 		/objectSalience/img:o 	-- image with a contour placed around the attended to region
 * 		/objectSalience/obj:o	-- Nx2 Matrix containing list of contour pixels for segmented obj.
 *
 * TODOs: doesn't quite work right yet, a lot of mis-segmentations happening. need better object
 * 			filtering, probably using prior knowledge of experimental setup.
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
#include <time.h>
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


class salAggThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *portImgIn;
	BufferedPort<ImageOf<PixelRgb> > *portImgOut;
	BufferedPort<ImageOf<PixelFloat> > *portGazeIn;
	BufferedPort<yarp::sig::Matrix>	*portObjOut;

	IntensitySalience * ifilter;

	double colthresh, inthresh;
	int minobjsize;

public:

	salAggThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	virtual bool threadInit()
	{

		name=rf.check("name",Value("objectSalience")).asString().c_str();
		colthresh=rf.check("colthresh",Value(50.0)).asDouble();
		inthresh = rf.check("inthresh",Value(130.0)).asDouble();
		minobjsize=rf.check("minobjsize",Value(100)).asInt();

		portImgIn=new BufferedPort<ImageOf<PixelRgb> >;
		string portInName="/"+name+"/img:i";
		portImgIn->open(portInName.c_str());

		portImgOut=new BufferedPort<ImageOf<PixelRgb> >;
		string portOutName="/"+name+"/img:o";
		portImgOut->open(portOutName.c_str());

		portGazeIn=new BufferedPort<ImageOf<PixelFloat> >;
		string portGazeName="/"+name+"/jas:i";
		portGazeIn->open(portGazeName.c_str());

		portObjOut=new BufferedPort<yarp::sig::Matrix>;
		string portObjName="/"+name+"/obj:o";
		portObjOut->open(portObjName.c_str());

		ifilter = new IntensitySalience;
		ifilter->open(rf);

		return true;

	}

	virtual void run()
	{

		// get inputs
		ImageOf<PixelRgb> *pImgIn=portImgIn->read(false);
		ImageOf<PixelRgb> *cDest = NULL;
		ImageOf<PixelFloat> *cSal = NULL;
		ImageOf<PixelFloat> *iSal = NULL;


		if (pImgIn) {

			ImageOf<PixelRgb> &imgOut= portImgOut->prepare();
			ImageOf<PixelFloat> *pJAIn = portGazeIn->read(true);

			cDest = new ImageOf<PixelRgb>;
			cSal = new ImageOf<PixelFloat>;
			iSal = new ImageOf<PixelFloat>;
			ifilter->apply(*pImgIn, *cDest, *iSal);

			//apply color normalization and invert to get color based salience
			normalizeColor(*pImgIn, *cDest, *cSal);

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
			M->setTo(Scalar(0));
			Mat icubImg((IplImage *)imgOut.getIplImage(), false);
			for (int i = 0; i < contours.size(); i++) {
				if (contours[i].size() > minobjsize) {
					drawContours(*M, contours, i, Scalar(255), -1);
					drawContours(icubImg, contours, i, Scalar(255, 0, 0), 1);
				}
			}

			//use the color salience bw image to mask the joint attention map
			Mat * P = new Mat(pJAIn->height(), pJAIn->width(), CV_32F, (void *)pJAIn->getRawImage());
			multiply(*M, *P, *N);

			//find the blob with highest average salience
			Rect br;
			double maxSal = 0.0;
			int iBlob = -1;
			for (int i = 0; i < contours.size(); i++) {

				//weed out small ones ones first
				if (contours[i].size() > minobjsize) {

					br = boundingRect(Mat(contours[i]));
					if (sum(Mat(*N,br))[0]/abs(contourArea(Mat(contours[i]))) > maxSal) {
						maxSal = sum(Mat(*N,br))[0]/abs(contourArea(Mat(contours[i])));
						iBlob = i;
					}
				}

			}


			drawContours(icubImg, contours, iBlob, Scalar(0, 0, 255), 1);
			portImgOut->write();

			delete M;
			delete N;
			delete P;
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
		portGazeIn->interrupt();
		portObjOut->interrupt();


		delete portImgIn;
		delete portImgOut;
		delete portGazeIn;
		delete portObjOut;

	}

};

class salAggModule: public RFModule
{

protected:

	salAggThread *thr;

public:
	salAggModule() { }

	virtual bool configure(ResourceFinder &rf)
	{
		Time::turboBoost();

		thr=new salAggThread(rf);
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

	salAggModule mod;

	return mod.runModule(rf);
}
