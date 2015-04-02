/*
 * Copyright (C) 2013 Logan Niehaus
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
 *  gazeEstimator2.cpp
 *
 * 	Logan Niehaus
 * 	9/22/13
 * 	better module for estimating the gaze of the head. has some issues.
 *
 * inputs: input image from the icub in rgb
 * params:	threshold values for detecting eyes and face (see explanation) (20,20)
 * 			minimum eye size for eye detection (10)
 * 			maximum eye separation distance (30)
 * outputs: vector with gaze estimate representation, vector with head bounding box in image coords
 * 			image with markup showing what is being detected as head eyes and bridge of nose
 * 			if connected to the stereoVision algorithm, will send XYZ location of eye midpoint
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


class gazeEstThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *portImgIn;
	BufferedPort<ImageOf<PixelRgb> > *portImgOut;
	BufferedPort<yarp::sig::Vector>			 *portFeatOut;
	BufferedPort<yarp::sig::Vector>			 *portEyeCent;


	//algorithm parameters
	double mincd; //minimum corner distance
	double harrisk; //harris corner detection parameter
	double faceThresh; //ruddiness threshold for head blob detection
	double eyeThresh; //parameter for detecting eye/mouth blobs
	double resetThresh; //parameter for deciding whether to reset tracking points
	int minEyeSize;
	double maxEyeDist;
	double minFaceSize;
	double maxTPSpread;

	//data containers
	Mat trackPoints; //tracking points
	Mat pointMem; //point membership. marks as belonging to eyeL, eyeR, mouth
	Mat prevImg;
	Point2f e1p, e2p, emp, mmp;

	//flags
	bool attemptReset;


public:

	gazeEstThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	virtual bool threadInit()
	{

		name=rf.check("name",Value("gazeEstimator")).asString().c_str();
		faceThresh=rf.check("facethresh",Value(135.0)).asDouble();
		eyeThresh=rf.check("eyethresh",Value(112.0)).asDouble();
		minEyeSize=rf.check("mineyesize",Value(10)).asInt();
		maxEyeDist=rf.check("maxeyedist",Value(20.0)).asDouble();
		minFaceSize=rf.check("minfacesize",Value(100.0)).asDouble();
		maxTPSpread=rf.check("maxtpspread",Value(10.0)).asDouble();

		mincd=rf.check("mincd",Value(3.0)).asDouble();
		harrisk=rf.check("harrisk",Value(0.01)).asDouble();
		resetThresh=rf.check("resetthresh",Value(0.5)).asDouble();


		portImgIn=new BufferedPort<ImageOf<PixelRgb> >;
		string portInName="/"+name+"/img:i";
		portImgIn->open(portInName.c_str());

		portImgOut=new BufferedPort<ImageOf<PixelRgb> >;
		string portOutName="/"+name+"/img:o";
		portImgOut->open(portOutName.c_str());

		portFeatOut=new BufferedPort<yarp::sig::Vector>;
		string portFeatName="/"+name+"/feat:o";
		portFeatOut->open(portFeatName.c_str());

		portEyeCent=new BufferedPort<yarp::sig::Vector>;
		string portHeadName="/"+name+"/eye:o";
		portEyeCent->open(portHeadName.c_str());

		attemptReset = true;


		return true;
	}

	/*
	 * Attempt to reinitialize the face tracking points
	 * Return false if unable to get an acceptable set
	 *
	 */
	virtual bool initTrackPoints(Mat G, Mat &tPoints, Mat mask) {

		bool rval = false;

		//get the head bounding rect
		vector<vector<Point> > HC;
		findContours(mask, HC, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		Rect hr = boundingRect(HC[0]);


		//first erode the head mask to keep potential eye/mouth blobs away from edges
		erode(mask, mask, Mat(), Point(-1,-1), 3);

		//try to threshold out eye and mouth blobs on face
		Mat T(G.rows,G.cols,CV_8UC1);
		threshold(G,T,(int)eyeThresh,255,CV_THRESH_BINARY_INV);
		min(T,mask,T);

		//dilate everything a little
		dilate(T,T,Mat());

		//get the blobs of the image, their size and center locations
		vector<vector<Point> > contours;
		findContours(T, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		vector<double> cAreas(contours.size());
		vector<Point> centers(contours.size());
		int nsuff = 0;
		for (int i = 0; i < contours.size(); i++) {

			cAreas[i] = contourArea(contours[i]);
			if (cAreas[i] > minEyeSize) {
				nsuff++;
			}

			Rect rr = boundingRect(contours[i]);
			centers[i] = Point(rr.x+rr.width/2.0, rr.y+rr.height/2.0);

		}

		//if there arent even 3 clear blobs, or two of sufficient size for eyes abort
		if (contours.size() < 3 | nsuff < 2) {

			return false;

		}
		else {

			//sort the blobs by size
			vector<int> szIndex(contours.size());
			sortIdx(cAreas, szIndex, CV_SORT_DESCENDING);

			//take the two largest to be the eyes
			int e1dx = szIndex[0];
			int e2dx = szIndex[1];

			//check if they are two far apart (they probably aren't eyes then
			if (sqrt((centers[e1dx].x-centers[e2dx].x)*(centers[e1dx].x-centers[e2dx].x) +
					(centers[e1dx].y-centers[e2dx].y)*(centers[e1dx].y-centers[e2dx].y)) > maxEyeDist) {
				return false;
			}

			//only do the reinitialization if we have a roughly head-on view of the face
			Point2f emt = Point((centers[e1dx].x+centers[e2dx].x)/2.0,(centers[e1dx].y+centers[e2dx].y)/2.0);
			double ratio = (double)(emt.x-(hr.x+hr.width/2.0))/(double)(hr.width/2.0);
			if (ratio > 0.2 || ratio < -0.2) {
				return false;
			}

			//look for largest blob below the eyes and between them to be mouth
			int mmdx = -1;
			for (int i = 2; i < contours.size(); i++) {
				if (centers[szIndex[i]].y > centers[e1dx].y+15 && centers[szIndex[i]].y > centers[e2dx].y+15
						&& cAreas[szIndex[i]] > 5) {
					mmdx = szIndex[i];
					break;
				}
			}

			//if no suitable blob found, return false
			if (mmdx < 0) {
				return false;
			}

			//get three corners to track for each eye and two for the mouth
			vector<Point2f> corners;
			vector<Point2f> e1c, e2c, mmc;

			Mat CM = Mat::zeros(G.rows,G.cols,CV_8UC1);
			drawContours(CM, contours, e1dx, Scalar(255), -1);
			goodFeaturesToTrack(G, e1c, 3, 1e-10, mincd, CM, 3, true, harrisk);
			if (e1c.size() != 3) {
				return false;
			}
			if (pointPolygonTest(contours[e1dx],e1c[0],false) < 0 ||
					pointPolygonTest(contours[e1dx],e1c[1],false) < 0 ||
					pointPolygonTest(contours[e1dx],e1c[2],false) < 0) {
				return false;
			}
			corners.push_back(e1c[0]); corners.push_back(e1c[1]); corners.push_back(e1c[2]);

			CM = Mat::zeros(G.rows,G.cols,CV_8UC1);
			drawContours(CM, contours, e2dx, Scalar(255), -1);
			goodFeaturesToTrack(G, e2c, 3, 1e-10, mincd, CM, 3, true, harrisk);
			if (e2c.size() != 3) {
				return false;
			}
			if (pointPolygonTest(contours[e2dx],e2c[0],false) < 0 ||
					pointPolygonTest(contours[e2dx],e2c[1],false) < 0 ||
					pointPolygonTest(contours[e2dx],e2c[2],false) < 0) {
				return false;
			}
			corners.push_back(e2c[0]); corners.push_back(e2c[1]); corners.push_back(e2c[2]);

			CM = Mat::zeros(G.rows,G.cols,CV_8UC1);
			drawContours(CM, contours, mmdx, Scalar(255), -1);
			goodFeaturesToTrack(G, mmc, 2, 1e-10, mincd, CM, 3, true, harrisk);
			if (mmc.size() != 2) {
				return false;
			}
			if (pointPolygonTest(contours[mmdx],mmc[0],false) < 0 ||
					pointPolygonTest(contours[mmdx],mmc[1],false) < 0) {
				return false;
			}
			corners.push_back(mmc[0]); corners.push_back(mmc[1]);

			rval = true;

			Mat ctmp(corners);
			ctmp.copyTo(tPoints);


		}

		return rval;

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
			yarp::sig::Vector &eyeCentVec = portEyeCent->prepare();
			imgOut.copy(*pImgIn);

			Mat I = (IplImage *) pImgIn->getIplImage();
			Mat * G = new Mat(I.rows, I.cols, CV_8UC1);

			cvtColor(I, *G, CV_RGB2GRAY);

			if (prevImg.empty()) {

				G->copyTo(prevImg);

			}

			//convert to LAB color space and threshold to get skin color blobs
			Mat ctmp(I.rows, I.cols, CV_8UC3);
			Mat rud(I.rows, I.cols, CV_8UC1);
			Mat * T = new Mat(I.rows, I.cols, CV_8UC1);

			int * frto = new int[2];
			frto[0] = 1; frto[1] = 0;

			cvtColor(I, ctmp, CV_RGB2Lab);
			mixChannels(&ctmp, 1, &rud, 1, frto, 1);
			threshold(rud,*T,(int)faceThresh,255,CV_THRESH_BINARY);

			//look for the largest such blob (it is the head)
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			findContours(*T, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

			//will assume face is located on the largest such blob
			double maxSize = 0.0;
			int biggestblob = -1;
			for (int i = 0; i < contours.size(); i++) {

				//also assume the face is in the top half of the image
				Mat CC = Mat(contours[i]);
				Moments fc = moments(CC);
				if ((abs(contourArea(CC)) > maxSize) && (fc.m01/fc.m00 < I.rows/2) &&
						abs(contourArea(CC)) > minFaceSize) {
					maxSize = abs(contourArea(CC));
					biggestblob = i;
				}
			}

			//if we need to (re)initialize the tracking points, do that
			Mat hmask = Mat::zeros(I.rows, I.cols, CV_8UC1);
			if (attemptReset) {

				Mat newTrackPoints(8,1,CV_32FC2);
				drawContours(hmask, contours, biggestblob, Scalar(255), -1);

				if (initTrackPoints(*G, newTrackPoints, hmask)) {

					trackPoints = newTrackPoints;
					attemptReset = 0;
					printf("Tracking points reinitialized\n");

					for (int i = 0; i < trackPoints.rows; i++) {

						printf("%f, %f\n", newTrackPoints.at<Point2f>(i,0).x, newTrackPoints.at<Point2f>(i,0).y);

					}

				}

			}

			//set up feature stuff
			double ratio, emdist;
			featureVec.resize(3);
			featureVec.zero();
			eyeCentVec.resize(2);
			eyeCentVec.zero();

			Mat O = (IplImage *) imgOut.getIplImage();
			cvtColor(*G,O,CV_GRAY2RGB);

			//if we currently have tracking points, calculate their optical flow into the next frame
			if (!trackPoints.empty()) {


				Mat nextPoints, status, ferr;
				calcOpticalFlowPyrLK(prevImg, *G, trackPoints, nextPoints, status, ferr, Size(11,11));
				trackPoints = nextPoints;

				//check to see if any of the points are just completely broken
				if (countNonZero(status) < 8) {

					attemptReset = true;
					trackPoints = Mat(0,0,CV_32FC2);

				}
				else {

					//calculate the eye/mouth points, midpoints, and view features
					Scalar e1t, e2t, mmt;
					e1t = mean(trackPoints.rowRange(0,3));
					e2t = mean(trackPoints.rowRange(3,6));
					mmt = mean(trackPoints.rowRange(6,8));
					e1p.x = e1t[0]; e1p.y = e1t[1];
					e2p.x = e2t[0]; e2p.y = e2t[1];
					mmp.x = mmt[0]; mmp.y = mmt[1];
					emp.x = (e1p.x+e2p.x)/2.0; emp.y = (e1p.y+e2p.y)/2.0;

					Rect hr = boundingRect(contours[biggestblob]);

					//azimuth representation is the ratio of area in bb to left of center, to that right of center, minus one
					ratio = (double)(emp.x-(hr.x+hr.width/2.0))/(double)(hr.width/2.0);
					//elevation representation is mouth center y position minus eye center y position
					emdist = (double)(mmp.y-emp.y);



					//determine if the tracking points have become inaccurate

					//check to see if any tracking points have come off the head BB entirely
					for (int i = 0; i < trackPoints.rows; i++) {
						if (trackPoints.at<Point2f>(i,0).x < hr.x || trackPoints.at<Point2f>(i,0).x > hr.x+hr.width ||
								trackPoints.at<Point2f>(i,0).y < hr.y || trackPoints.at<Point2f>(i,0).y > hr.y+hr.height) {
							attemptReset = true;
						}
					}

					//get distances between a facial features tracking points
					double e1dist, e2dist, mmdist;
					e1dist = (norm(Mat(trackPoints.at<Point2f>(0,0) - e1p)) + norm(Mat(trackPoints.at<Point2f>(1,0) - e1p)) +
							norm(Mat(trackPoints.at<Point2f>(2,0) - e1p)))/3.0;
					e2dist = (norm(Mat(trackPoints.at<Point2f>(3,0) - e2p)) + norm(Mat(trackPoints.at<Point2f>(4,0) - e2p)) +
							norm(Mat(trackPoints.at<Point2f>(5,0) - e2p)))/3.0;
					mmdist = (norm(Mat(trackPoints.at<Point2f>(6,0) - mmp)) + norm(Mat(trackPoints.at<Point2f>(7,0) - mmp)))/2.0;

					//check to see if any of the points have become too far apart
					if (e1dist > maxTPSpread || e2dist > maxTPSpread || mmdist > maxTPSpread) {
						attemptReset = true;
					}


					rectangle(O, hr, Scalar(255,0,0));


					//draw information to the output image for debugging
					for (int i = 0; i < trackPoints.rows; i++) {

						Point dp = trackPoints.at<Point2f>(i,0);
						circle(O, dp, 1, Scalar(0, 255, 0), 2);

					}

					circle(O, emp, 1, Scalar(255, 0, 0), 2);
					circle(O, mmp, 1, Scalar(0, 0, 255), 2);


					featureVec[0] = ratio;
					featureVec[1] = emdist;
					featureVec[2] = 1;
					if (attemptReset) {
						featureVec[2] = -1;
					}

					eyeCentVec[0] = emp.x;
					eyeCentVec[1] = emp.y;

				}
			}

			G->copyTo(prevImg);


			portImgOut->write();
			portFeatOut->write();
			portEyeCent->write();

		}
	}

	virtual void threadRelease()
	{

		portImgIn->interrupt();
		portImgOut->interrupt();
		portFeatOut->interrupt();
		portEyeCent->interrupt();
		portImgIn->close();
		portImgOut->close();
		portFeatOut->close();
		portEyeCent->close();

		delete portImgIn;
		delete portImgOut;
		delete portFeatOut;
		delete portEyeCent;

	}

	virtual bool setParam(string pname, Value pval) {

		bool success = true;

		if (pname == "facethresh") {
			faceThresh = pval.asDouble();
		}
		else if (pname == "eyethresh") {
			eyeThresh = pval.asDouble();
		}
		else if (pname == "mineyesize") {
			minEyeSize = pval.asInt();
		}
		else if (pname == "maxeyedist") {
			maxEyeDist = pval.asDouble();
		}
		else if (pname == "minfacesize") {
			minFaceSize = pval.asDouble();
		}
		else if (pname == "mincd") {
			mincd = pval.asDouble();
		}
		else {
			success = false;
		}

		return success;

	}

	virtual bool trackingReset() {

		attemptReset = 1;
		return true;

	}

};

class gazeEstModule: public RFModule
{
protected:

	gazeEstThread *thr;
	Port * rpcPort;
	string name;

public:
	gazeEstModule() { }

	bool respond(const Bottle& command, Bottle& reply) {

		string msg(command.get(0).asString().c_str());
		if (msg == "set") {
			if (command.size() < 3) {
				reply.add(-1);
			}
			else {
				string param(command.get(1).asString().c_str());
				reply.add(thr->setParam(param,command.get(2)));
			}
		}
		else if (msg == "reset") {
			reply.add(thr->trackingReset());
		}
		else {
			reply.add(-1);
		}

		return true;

	}

	virtual bool configure(ResourceFinder &rf)
	{
		Time::turboBoost();

		//set up the rpc port
		name=rf.check("name",Value("gazeEstimator")).asString().c_str();
		rpcPort = new Port;
		string portRpcName="/"+name+"/rpc";
		rpcPort->open(portRpcName.c_str());
		attach(*rpcPort);

		thr=new gazeEstThread(rf);
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

	gazeEstModule mod;

	return mod.runModule(rf);
}



