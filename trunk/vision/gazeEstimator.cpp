/*
 *  gazeEstimator.cpp
 *
 * 	Logan Niehaus
 * 	5/2/11
 * 	module for estimating the gaze of the head. has some issues.
 *
 * inputs: input image from the icub in rgb
 * params:	threshold values for detecting eyes and face (see explanation) (20,20)
 * 			minimum eye size for eye detection (10)
 * 			maximum eye separation distance (30)
 * outputs: vector with gaze estimate representation, vector with head bounding box in image coords
 * 			image with markup showing what is being detected as head eyes and bridge of nose
 * desc: viola-jones doesnt really work well for this case as there is quite a range of pose variation.
 * 			this uses a ruddy (skin) filter, and assumes the head is the largest blob. also makes the
 * 			assumption about scene geometry that the head occurs near the top half of the screen, this
 * 			is needed because as the hand gets closer to the robot, perspective makes it look huge.
 * 			then we try to find some blobs on the inside of head blob, heuristics for expected size
 * 			and distance used to help find eyes. then we find x,y location of point b/w eyes. if only
 * 			one eye detected, set this as the midpoint b/w they eye and nearest left/right edge of head
 * 			bounding box. representation is basically the left/right pose (ranges from -1:1, with midpoint
 * 			at left edge of bounding box being -1, right 1, and zero dead middle), and distance of
 * 			midpoint to bottom of head BB (can't use more invariant features as for left/right b/c
 * 			foreshortening does some weird things).
 * TODOs: yarp more things. in general this algorithm is kind of hackish, which might be fixed.
 * 			midpoint estimation and even eye location estimation URGENTLY need to be put through a
 * 			kalman filter. it is already assuming the head exists so this should not be a huge leap.
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

#include <iCub/vis/Salience.h>
#include <iCub/vis/RuddySalience.h>
#include <iCub/vis/BlobResult.h>

#include <cv.h>

#include <string>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <deque>
#include <algorithm>

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::vis;


bool contComp(vector<Point> c1, vector<Point> c2) { return (contourArea(c1) < contourArea(c2)); }


class skinDetThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *portImgIn;
	BufferedPort<ImageOf<PixelRgb> > *portImgOut;
	BufferedPort<yarp::sig::Vector>			 *portFeatOut;
	BufferedPort<yarp::sig::Vector>			 *portHeadBox;

	RuddySalience * filter;

	//running params
	double faceThresh;
	double eyeThresh;
	int minEyeSize;
	double maxEyeDist;

public:

	skinDetThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	virtual bool threadInit()
	{

		name=rf.check("name",Value("myFaceDetector")).asString().c_str();
		faceThresh=rf.check("facethresh",Value(20.0)).asDouble();
		eyeThresh=rf.check("eyethresh",Value(20.0)).asDouble();
		minEyeSize=rf.check("mineyesize",Value(10)).asInt();
		maxEyeDist=rf.check("maxeyedist",Value(30.0)).asDouble();

		portImgIn=new BufferedPort<ImageOf<PixelRgb> >;
		string portInName="/"+name+"/img:i";
		portImgIn->open(portInName.c_str());

		portImgOut=new BufferedPort<ImageOf<PixelRgb> >;
		string portOutName="/"+name+"/img:o";
		portImgOut->open(portOutName.c_str());

		portFeatOut=new BufferedPort<yarp::sig::Vector>;
		string portFeatName="/"+name+"/feat:o";
		portFeatOut->open(portFeatName.c_str());

		portHeadBox=new BufferedPort<yarp::sig::Vector>;
		string portHeadName="/"+name+"/head:o";
		portHeadBox->open(portHeadName.c_str());

		filter = new RuddySalience;
		filter->open(rf);


		return true;
	}

	virtual void run()
	{

		// get inputs
		ImageOf<PixelRgb> *pImgIn=portImgIn->read(false);

		// process camera image
		if (pImgIn)
		{

			ImageOf<PixelRgb> *pDest = new ImageOf<PixelRgb>;
			ImageOf<PixelFloat> *pSal = new ImageOf<PixelFloat>;
			ImageOf<PixelRgb> &imgOut= portImgOut->prepare();
			yarp::sig::Vector &featureVec = portFeatOut->prepare();
			yarp::sig::Vector &headBoxVec = portHeadBox->prepare();
			imgOut.copy(*pImgIn);

			//apply ruddy salience filter
			filter->apply(*pImgIn, *pDest, *pSal);

			//threshold to get skin colored objects
			Mat * T = new Mat(pSal->height(), pSal->width(), CV_32F, (void *)pSal->getRawImage());
			Mat * C = new Mat(pSal->height(), pSal->width(), CV_32F);
			threshold(*T, *C, faceThresh, 255.0, CV_THRESH_BINARY);


			//find the list of skin colored blobs using connected components
			vector<vector<Point> > contours;
			vector<vector<Point> > cts;
			vector<Vec4i> hierarchy;
			Mat X;
			C->convertTo(X,CV_8UC1);
			findContours(X, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

			//will assume face is located on the largest such blob
			double maxSize = 0.0;
			int biggestblob = -1;
			for (int i = 0; i < contours.size(); i++) {
				//also assume the face is in the top half of the image
				Moments fc = moments(contours[i]);
				if ((abs(contourArea(contours[i])) > maxSize) && (fc.m01/fc.m00 < pSal->height()/2)) {
					maxSize = abs(contourArea(contours[i]));
					biggestblob = i;
				}
			}

			IplImage * mainImgIpl = (IplImage *)imgOut.getIplImage();
			Mat mainImg(mainImgIpl, true);

			//draw a box around the face zone
			Rect faceRect = boundingRect(contours[biggestblob]);

			//select the face part of the image
			Mat * subFace = new Mat(*T, faceRect);


			//with face bounding box, use diff thresholds if desired
			threshold(*subFace, *subFace, eyeThresh, 255.0, CV_THRESH_BINARY);
			drawContours(*T, contours, biggestblob, Scalar(255), 1, 8, hierarchy, 0);
			subFace->convertTo(X,CV_8UC1);
			hierarchy.clear();
			contours.clear();
			findContours(X, contours, hierarchy, CV_RETR_LIST, CHAIN_APPROX_NONE, Point(faceRect.x,faceRect.y));

			//find the main head blob (assuming largest inside the head box)
			maxSize = 0.0;
			biggestblob = -1;
			for (int i = 0; i < contours.size(); i++) {
				if (abs(contourArea(contours[i])) > maxSize) {
					maxSize = abs(contourArea(contours[i]));
					biggestblob = i;
				}
			}


			//for this contour, find its two largest internal contours above a min size, call these the eyes
			deque<int> eyeCan(2);
			deque<double> canVals(2);
			for (int i = 0; i < contours.size(); i++) {
				if (i != biggestblob) {
					Moments m = moments(contours[i]);
					if ((pointPolygonTest(contours[biggestblob], Point2f(m.m10/m.m00,m.m01/m.m00), false) > 0) &&
							abs(contourArea(contours[i])) > minEyeSize) {
						if (abs(contourArea(contours[i])) > canVals.front()) {
							canVals.push_front(abs(contourArea(contours[i])));
							canVals.pop_back();
							eyeCan.push_front(i);
							eyeCan.pop_back();
						}
						else if (abs(contourArea(contours[i])) > canVals.back()) {
							canVals.pop_back();
							canVals.push_back(abs(contourArea(contours[i])));
							eyeCan.pop_back();
							eyeCan.push_back(i);
						}
					}
				}
			}


			//calculate the center position of the eyes
			Point * eyeMidPoint = NULL;
			Moments m = moments(contours[biggestblob]);
			Moments n = moments(contours[biggestblob]);
			if (canVals.front() > 0) {
				m = moments(contours[eyeCan.front()]);
				n = moments(contours[eyeCan.front()]);
				drawContours(mainImg, contours, eyeCan.front(), Scalar(0, 0, 255), 1, 8, hierarchy, 0);
			}
			//if we have detected the second eye, find the midpoint
			if (canVals.back() > 0) {
				n = moments(contours[eyeCan.back()]);
				eyeMidPoint = new Point((m.m10/m.m00+n.m10/n.m00)/2, (m.m01/m.m00+n.m01/n.m00)/2);
				drawContours(mainImg, contours, eyeCan.back(), Scalar(0, 0, 255), 1, 8, hierarchy, 0);
			}
			//if we do not detect the second eye, assume its location to be at edge of bounding box (on the correct side)
			else {
				if (m.m10/m.m00 < (faceRect.x+faceRect.width/2)) {
					eyeMidPoint = new Point((m.m10/m.m00+faceRect.x)/2, (m.m01/m.m00+n.m01/n.m00)/2);
				}
				else {
					eyeMidPoint = new Point((m.m10/m.m00+faceRect.x+faceRect.width)/2, (m.m01/m.m00+n.m01/n.m00)/2);
				}
			}

			circle(mainImg, *eyeMidPoint, 1, Scalar(0, 255, 0), 2);

			//azimuth representation is the ratio of area in bb to left of center, to that right of center, minus one
			double ratio = (double)(eyeMidPoint->x-(faceRect.x+faceRect.width/2.0))/(double)(faceRect.width/2.0);
			//elevation representation is distance from eye midpt to bottom of bounding box
			double botdist = (double)(faceRect.y+faceRect.height-eyeMidPoint->y);

			//annotate the output image (debugging mostly)
			char labelc[100];
			sprintf(labelc,"r = %f", ratio);
			string label(labelc);
			rectangle(mainImg, Point(faceRect.x,faceRect.y), Point(faceRect.x+faceRect.width,faceRect.y+faceRect.height), Scalar(255,0,0));
			putText(mainImg, label, Point(faceRect.x,faceRect.y+faceRect.height), FONT_HERSHEY_SIMPLEX, 0.2, Scalar(0, 0, 255));

			//write out the image, features, and head position
			featureVec.clear();
			headBoxVec.clear();
			featureVec.push_back(ratio);
			featureVec.push_back(botdist);
			headBoxVec.push_back(faceRect.x);
			headBoxVec.push_back(faceRect.y);
			headBoxVec.push_back(faceRect.width);
			headBoxVec.push_back(faceRect.height);
			IplImage outImg = mainImg;
			imgOut.wrapIplImage(&outImg);


			portImgOut->write();
			portFeatOut->write();
			portHeadBox->write();

			delete T;
			delete C;
			delete pDest;
			delete pSal;

		}
	}

	virtual void threadRelease()
	{

		portImgIn->interrupt();
		portImgOut->interrupt();
		portFeatOut->interrupt();
		portHeadBox->interrupt();
		portImgIn->close();
		portImgOut->close();
		portFeatOut->close();
		portHeadBox->close();

		filter->close();

		delete portImgIn;
		delete portImgOut;
		delete portFeatOut;
		delete portHeadBox;
		delete filter;

	}

};

class skinDetModule: public RFModule
{
protected:
	skinDetThread *thr;

public:
	skinDetModule() { }

	virtual bool configure(ResourceFinder &rf)
	{
		Time::turboBoost();

		thr=new skinDetThread(rf);
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

	skinDetModule mod;

	return mod.runModule(rf);
}



