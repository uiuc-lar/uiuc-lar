/*
 *  stereoBlobTrack.cpp
 *
 * 	Logan Niehaus
 * 	6/28/11
 * 	stereo vision based blob tracker. finds the largest blob of a selected color and
 * 		focuses the robot's gaze on it. iKinGazeCtrl must be running for this module to
 * 		function. stereo blob locations will be continuously sent to the controller
 * 		until the average pixel distance of two blobs from the center of each image falls
 * 		below the tolerance level. once the distance passes under this threshold, a trigger
 * 		will be generated. position commands will begin to stream again once the
 * 		distance exceeds hysval*tol, the hysteresis tolerance.
 *
 *  inputs:
 *  	/stereoBlobTrack/img:l	-- robot's left camera image stream (calibrated)
 *  	/stereoBlobTrack/img:r	-- robot's right camera image stream (calibrated)
 *
 *  params: ([R]equired/[D]efault <Value>/[O]ptional)
 *  	red/green/blue/yellow	-- selected color channel to find blobs of (R)
 *  	thresh					-- color channel threshold for blob detection (D 15.0)
 *  	tol						-- stereo convergence tolerance (D 5.0)
 *  	hysval					-- tol hysteresis threshold multiplier (D 1.0, simple threshold)
 *  	nt, et					-- solver trajectory times; lower values -> faster head movement (D 1.0, 0.5 for icub)
 *  	name					-- module port basename (D /stereoBlobTrack)
 *  	robot					-- robot name, changes default traj time values (D icub)
 *  	mode					-- output data mode; arg should be either 'xyz' for cart or 'azelv' for azimuth/elevation/vergence (D xyz)
 *  	verbose					-- setting flag makes the position echo to stdout when triggered
 *
 *  outputs:
 *  	/stereoBlobTrack/img:o  -- debug image, shows right threshold image w/ target blob locations marked
 *  	/stereoBlobTrack/pos:o	-- triggered signal; publishes a timestamped vector containing current gaze position,
 *  								with convention specified by the 'mode' parameter. message is generated when
 *  								the stereo convergence value passes under the tol parameter (i.e. when
 *  								the robot is focused on the object). This is a buffered port, so the event may
 *  								be old by the time it is read. Check envelope to get event time.
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/sig/ImageDraw.h>

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

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;


YARP_DECLARE_DEVICES(icubmod)


class strBallLocThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *portImgL;
	BufferedPort<ImageOf<PixelRgb> > *portImgR;
	BufferedPort<ImageOf<PixelRgb> > *portImgD;
	BufferedPort<yarp::sig::Vector>  *portPos;

	PolyDriver clientGazeCtrl;
	IGazeControl *igaze;

	double tol;	//stopping tolerance
	int color; 	//which color blob to find
	double thresh; //color intensity threshold
	string robot; //robot name (just for adjusting default traj times)
	double neckTT, eyeTT; //neck and eye trajectory times (see ikingazectrl docs)
	double hval; //hysteresis tolerance multiplier
	bool mode; //output publishing mode
	bool verbose;

	bool stopped;

public:

	strBallLocThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	virtual bool threadInit()
	{


		name=rf.check("name",Value("stereoBlobTrack")).asString().c_str();
		tol=rf.check("tol",Value(5.0)).asDouble();
		thresh=rf.check("thresh",Value(15.0)).asDouble();
		hval=rf.check("hysval",Value(1.0)).asDouble();
		verbose=rf.check("verbose");

		//get the color channel from command line, parse for sanity
		color = -1;
		if (rf.check("red")) color = 0;
		if (rf.check("green")) color = 1;
		if (rf.check("blue")) color = 2;
		if (rf.check("yellow")) color = 3;
		if (color == -1) {
			printf("Please specify a blob color to track\n");
			return false;
		}
		if (((int)rf.check("red") + (int)rf.check("green") + (int)rf.check("blue") + (int)rf.check("yellow")) > 1) {
			printf("Please specify only one color blob to track\n");
			return false;
		}

		//get mode of position information expression
		string modestr = rf.check("mode",Value("xyz")).asString().c_str();
		if (modestr == "xyz") {
			mode = true;
		}
		else if (modestr == "azelv") {
			mode = false;
		} else {
			printf("Position mode incorrectly specified, setting to 'xyz'...\n");
			mode = true;
		}

		//get robot name and trajectory times. use diff default traj times for icub and sim
		robot = rf.check("robot",Value("nobot")).asString().c_str();
		if (robot == "icubSim") {
			neckTT = rf.check("nt",Value(0.6)).asDouble();
			eyeTT = rf.check("et",Value(0.1)).asDouble();
		}
		else if (robot == "icub") {
			neckTT = rf.check("nt",Value(1.0)).asDouble();
			eyeTT = rf.check("et",Value(0.5)).asDouble();
		}
		else {
			printf("No robot name specified, using real iCub default trajectory times\n");
			neckTT = rf.check("nt",Value(1.0)).asDouble();
			eyeTT = rf.check("et",Value(0.5)).asDouble();
		}

		//open up ports
		portImgL=new BufferedPort<ImageOf<PixelRgb> >;
		string portImlName="/"+name+"/img:l";
		portImgL->open(portImlName.c_str());

		portImgR=new BufferedPort<ImageOf<PixelRgb> >;
		string portImrName="/"+name+"/img:r";
		portImgR->open(portImrName.c_str());

		portImgD=new BufferedPort<ImageOf<PixelRgb> >;
		string portImdName="/"+name+"/img:o";
		portImgD->open(portImdName.c_str());

		portPos=new BufferedPort<yarp::sig::Vector>;
		string portPosName="/"+name+"/pos:o";
		portPos->open(portPosName.c_str());

		//start up gaze control client interface
		Property option("(device gazecontrollerclient)");
		option.put("remote","/iKinGazeCtrl");
		option.put("local","/client/gaze");
		clientGazeCtrl.open(option);
		igaze=NULL;
		if (clientGazeCtrl.isValid()) {
			clientGazeCtrl.view(igaze);
		} else {
			printf("could not initialize gaze control interface, failing...\n");
			return false;
		}

		//set gaze control interface params
		igaze->setNeckTrajTime(neckTT);
		igaze->setEyesTrajTime(eyeTT);
		igaze->bindNeckPitch(-30,30);
		igaze->bindNeckYaw(-25,25);
		igaze->bindNeckRoll(-10,10);

		stopped = false;

		return true;

	}

	virtual void run()
	{

		// get both input images
		ImageOf<PixelRgb> *pImgL=portImgL->read(false);
		ImageOf<PixelRgb> *pImgR=portImgR->read(false);
		ImageOf<PixelRgb> *tImg;

		ImageOf<PixelFloat> *pImgBRL;
		ImageOf<PixelFloat> *pImgBRR;
		ImageOf<PixelFloat> *oImg;

		//if we have both images
		if (pImgL && pImgR)
		{

			//set up processing
			yarp::sig::Vector loc;
			pImgBRL = new ImageOf<PixelFloat>;
			pImgBRR = new ImageOf<PixelFloat>;
			pImgBRL->resize(*pImgL);
			pImgBRR->resize(*pImgR);
			Mat * T, * X;
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			int biggestBlob;


			ImageOf<PixelRgb> &imgOut= portImgD->prepare();


			//pull out the individual color channels
			PixelRgb pxl;
			float lum, rn, bn, gn, val;
			for (int lr = 0; lr < 2; lr++) {

				if (!lr) {
					tImg = pImgL;
					oImg = pImgBRL;
				}
				else {
					tImg = pImgR;
					oImg = pImgBRR;
				}

				for (int x = 0; x < tImg->width(); x++) {
					for (int y = 0; y < tImg->height(); y++) {

						//normalize brightness (above a given threshold)
						pxl = tImg->pixel(x,y);
						lum = (float)(pxl.r+pxl.g+pxl.b);
						rn = 255.0F * pxl.r/lum;
						gn = 255.0F * pxl.g/lum;
						bn = 255.0F * pxl.b/lum;

						//get the selected color
						switch (color) {
						case 0:
							val = (rn - (gn+bn)/2);
							break;
						case 1:
							val = (gn - (rn+bn)/2);
							break;
						case 2:
							val = (bn - (rn+gn)/2);
							break;
						case 3:
							val = (rn+gn)/2.0 - bn;
							break;
						}
						if (val > 255.0) {
							val = 255.0;
						}
						if (val < 0.0) {
							val = 0.0;
						}
						oImg->pixel(x,y) = val;

					}
				}

				//threshold to find blue blobs
				T = new Mat(oImg->height(), oImg->width(), CV_32F, (void *)oImg->getRawImage());
				threshold(*T, *T, thresh, 255.0, CV_THRESH_BINARY);

				imgOut.copy(*oImg);

				X = new Mat(oImg->height(), oImg->width(), CV_8UC1);
				T->convertTo(*X,CV_8UC1);
				findContours(*X, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

				//find largest blob and its moment
				double maxSize = 0.0;
				biggestBlob = -1;
				double xloc, yloc;
				for (int i = 0; i < contours.size(); i++) {
					//do a heuristic check so that only 'compact' blobs are grabbed
					if (abs(contourArea(Mat(contours[i])))/arcLength(Mat(contours[i]), true) > maxSize &&
							abs(contourArea(Mat(contours[i])))/arcLength(Mat(contours[i]), true) > 2.0) {
						maxSize = abs(contourArea(Mat(contours[i])))/arcLength(Mat(contours[i]), true);
						biggestBlob = i;
					}
				}
				if (biggestBlob >= 0) {

					//if a valid object was found, add its location
					Moments m = moments(Mat(contours[biggestBlob]));
					xloc = m.m10/m.m00;
					yloc = m.m01/m.m00;
					loc.push_back(xloc);
					loc.push_back(yloc);

				}

				delete T;
				delete X;

			}

			//if a blob in both images was detected, go to it
			if (loc.size() == 4) {

				double du, dv;

				//check to see if within acceptable tolerance
				du = (loc[0] - 160 + loc[2] -160)/2.0;
				dv = (loc[1] - 120 + loc[3] -120)/2.0;
				printf("left/right average divergence: %f\n", sqrt(du*du+dv*dv));
				if (sqrt(du*du+dv*dv) < tol) {

					if (!stopped) {

						//stop tracking command
						igaze->stopControl();
						stopped = true;

						//generate a trigger signal indicating that the blob has been focused
						yarp::sig::Vector &cPos = portPos->prepare();
						if (mode) {
							igaze->getFixationPoint(cPos);
							if (verbose) {
								printf("fixated xyz: %f,\t%f,\t%f\n", cPos[0], cPos[1], cPos[2]);
							}
						} else {
							igaze->getAngles(cPos);
							if (verbose) {
								printf("fixed azelr: %f,\t%f,\t%f\n", cPos[0], cPos[1], cPos[2]);
							}
						}
						Bottle tStamp;
						tStamp.clear();
						tStamp.add(Time::now());
						portPos->setEnvelope(tStamp);
						portPos->write();

					}

				} else {

					if (!stopped || (sqrt(du*du+dv*dv) > hval*tol)) {
						//continue tracking the object
						yarp::sig::Vector pxl, pxr;
						pxl.push_back(loc[0]);
						pxl.push_back(loc[1]);
						pxr.push_back(loc[2]);
						pxr.push_back(loc[3]);
						igaze->lookAtStereoPixels(pxl,pxr);
						stopped = false;
					}

				}
				draw::addCrossHair(imgOut, PixelRgb(0, 255, 0), loc[0], loc[1], 10);
				draw::addCrossHair(imgOut, PixelRgb(0, 255, 0), loc[2], loc[3], 10);

			}

			//send out, cleanup
			portImgD->write();

			delete pImgBRL;
			delete pImgBRR;

		}
	}

	virtual void threadRelease()
	{

		clientGazeCtrl.close();

		portImgL->interrupt();
		portImgR->interrupt();
		portImgD->interrupt();
		portPos->interrupt();

		portImgL->close();
		portImgR->close();
		portImgD->close();
		portPos->close();

		delete portImgL;
		delete portImgR;
		delete portImgD;
		delete portPos;

	}

};

class strBallLocModule: public RFModule
{
protected:
	strBallLocThread *thr;

public:
	strBallLocModule() { }

	virtual bool configure(ResourceFinder &rf)
	{
		Time::turboBoost();

		thr=new strBallLocThread(rf);
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

	YARP_REGISTER_DEVICES(icubmod)

	Network yarp;

	if (!yarp.checkNetwork())
		return -1;

	ResourceFinder rf;

	rf.configure("ICUB_ROOT",argc,argv);

	strBallLocModule mod;

	return mod.runModule(rf);
}



