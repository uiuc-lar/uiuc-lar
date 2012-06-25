 /*
 * handOptFlow.cpp
 *
 * Lydia Majure
 *
 *
 * grabs from each camera, finds corners using cvGoodFeaturesToTrack
 * use features to calculate optical flow values for probable hand location
 * weigh the density of corners with the highest optical flow for hand loc
 * send out a salience map for use by fineMotorModule
 *
 * to do: increase threshold and somehow stabilize salience image
 *
 *
 */

//yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

//system
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

//gsl
#include <gsl/gsl_rng.h>

//opencv
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>


#include <math.h>
#include <iCub/ctrl/math.h>

const int MAX_CORNERS = 10;
const float DISP_THRESH = 55.0;
const int PEAK_VAL = 50;

using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;




class handLocThread : public RateThread {
protected:
	ResourceFinder &rf;

	BufferedPort<ImageOf<PixelRgb> > *eyeL;
	BufferedPort<ImageOf<PixelRgb> > *eyeR;

	BufferedPort<ImageOf<PixelRgb> > *salL;
	BufferedPort<ImageOf<PixelRgb> > *salR;

	//BufferedPort<Bottle> *salPeakL;
	//BufferedPort<Bottle> *salPeakR;

	ImageOf<PixelRgb> *imgLO;
	ImageOf<PixelRgb> *imgRO;
	
	Mat *oldSalL;
	Mat *oldSalR;

	//initial estimate from last calculation
	//vector<Point2f> *cornersNL;
	//vector<Point2f> *cornersNR;

	string robotName;
	string name;
	
	bool firstRun;

public:

	handLocThread(int period, ResourceFinder &_rf) : RateThread(period), rf(_rf) {}

	virtual bool handleParams(){
		if(rf.check("robot")){
			robotName = rf.find("robot").asString().c_str();
		}
		else{
			printf("Must specify robot name\n");
			return false;
		}

		name = rf.check("name",Value("handLoc")).asString().c_str();

		return true;
	}

	virtual bool threadInit(){
		if(!handleParams()){
			return false;
		}

		eyeL=new BufferedPort<ImageOf<PixelRgb> >;
		string eyeLName="/"+name+"/eye:l";
		eyeL->open(eyeLName.c_str());

		eyeR=new BufferedPort<ImageOf<PixelRgb> >;
		string eyeRName="/"+name+"/eye:r";
		eyeR->open(eyeRName.c_str());

		salL=new BufferedPort<ImageOf<PixelRgb> >;
		string salLName="/"+name+"/sal:l";
		salL->open(salLName.c_str());

		salR=new BufferedPort<ImageOf<PixelRgb> >;
		string salRName="/"+name+"/sal:r";
		salR->open(salRName.c_str());

		/*
		salPeakL = new BufferedPort<Bottle>;
		string salPeakLName = "/"+name+"/salPeak:l";
		salPeakL->open(salPeakLName.c_str());

		salPeakR = new BufferedPort<Bottle>;
		string salPeakRName = "/"+name+"/salPeak:r";
		salPeakR->open(salPeakRName.c_str());
		 */

		imgLO = eyeL->read(true);

		oldSalL = new Mat(imgLO->height(), imgLO->width(), CV_8UC3);
		oldSalR = new Mat(imgLO->height(), imgLO->width(), CV_8UC3);

		//cornersNL = new vector<Point2f>(MAX_CORNERS);
		//cornersNR = new vector<Point2f>(MAX_CORNERS);

		bool camCnct = 0;
		
		firstRun = true;

	}

	virtual void run(){

		//need somehow to get previous image and make sure it is recent enough?
		
		if(firstRun){
			imgLO = eyeL->read(true);
			imgRO = eyeR->read(true);
			*oldSalL = Mat::zeros(imgLO->height(), imgLO->width(), CV_8UC3);
			*oldSalR = Mat::zeros(imgLO->height(), imgLO->width(), CV_8UC3);
			firstRun = false;
		}
		else{

			ImageOf<PixelRgb> *imgL = eyeL->read(true);
			ImageOf<PixelRgb> *imgR = eyeR->read(true);
		
			ImageOf<PixelRgb> &salLOut = salL->prepare();
			ImageOf<PixelRgb> &salROut = salR->prepare();
			
			//Bottle &salPeakLOut = salPeakL->prepare();
			//Bottle &salPeakROut = salPeakR->prepare();

			//salPeakLOut.clear();
			//salPeakROut.clear();

			salLOut.copy(*imgL);
			salROut.copy(*imgR);
	
			Mat ImL = Mat((IplImage*)imgL->getIplImage(), true);
			Mat ImR = Mat((IplImage*)imgR->getIplImage(), true);
			
			Mat ImLO = Mat((IplImage*)imgLO->getIplImage(), true);
			Mat ImRO = Mat((IplImage*)imgRO->getIplImage(), true);
			
			Mat ImLG, ImRG;
			Mat ImLOG, ImROG;
			
			Mat salImL = Mat((IplImage*)salLOut.getIplImage(), false);
			Mat salImR = Mat((IplImage*)salROut.getIplImage(), false);
	
			//convert to grayscale
	
			cvtColor(ImL, ImLG, CV_RGB2GRAY);
			cvtColor(ImR, ImRG, CV_RGB2GRAY);
	
			cvtColor(ImLO, ImLOG, CV_RGB2GRAY);
			cvtColor(ImRO, ImROG, CV_RGB2GRAY);
			
			//calculate for both sides!
	
			Size imgSize = ImL.size();
	
	
			int corner_count = MAX_CORNERS;
			vector<Point2f> cornersOL(MAX_CORNERS);
	
			goodFeaturesToTrack(ImLOG,cornersOL,corner_count,0.01,5.0);

			//cornerSubPix(ImLOG,cornersOL,Size(10,10),Size(-1,-1),TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
			
			vector<uchar> featuresFoundL(MAX_CORNERS);
			vector<float> featuresErrorsL(MAX_CORNERS);
	
			vector<Point2f> cornersNL(MAX_CORNERS);
	
			calcOpticalFlowPyrLK(ImLOG,ImLG,cornersOL,cornersNL,featuresFoundL,featuresErrorsL, Size(20,20),3);

	
			//repeat for right
	
	
			vector<Point2f> cornersOR(MAX_CORNERS);
	
			goodFeaturesToTrack(ImROG,cornersOR,corner_count,0.01,5.0);
	
			//cornerSubPix(ImROG,cornersOR,Size(10,10),cvSize(-1,-1),TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
	
			vector<uchar> featuresFoundR(MAX_CORNERS);
			vector<float> featuresErrorsR(MAX_CORNERS);
	
			vector<Point2f> cornersNR(MAX_CORNERS);
	
			calcOpticalFlowPyrLK(ImROG,ImRG,cornersOR,cornersNR,featuresFoundR,featuresErrorsR, Size(20,20), 3);
	
			//calculate most salient points, in order
			Mat errorsL = Mat(featuresErrorsL, true);
			Mat errorsR = Mat(featuresErrorsR, true);
			
			Mat orderL = Mat(MAX_CORNERS,1,CV_8UC1);
			Mat orderR = Mat(MAX_CORNERS,1,CV_8UC1);
			
			sortIdx(errorsL,orderL,CV_SORT_DESCENDING + CV_SORT_EVERY_COLUMN);
			sortIdx(errorsR,orderR,CV_SORT_DESCENDING + CV_SORT_EVERY_COLUMN);
			
			
			//produce salience images
	
			//salImL = new Mat(imgL->height(), imgL->width(), CV_8UC3);
			//salImR = new Mat(imgR->height(), imgR->width(), CV_8UC3);
			
			salImL = Mat::zeros(imgL->height(), imgL->width(), CV_8UC3);
			salImR = Mat::zeros(imgR->height(), imgR->width(), CV_8UC3);
			
			//for now, all salient corners get equal weight
			for(int i = 0; i < MAX_CORNERS; i++){
				if(featuresFoundL[orderL.at<int>(i)]){
					if(errorsL.at<float>(i) > DISP_THRESH){
						Mat tmp = Mat::zeros(imgL->height(), imgL->width(), CV_8UC3);
						//Mat tmp2 = Mat::zeros(imgL->height(), imgL->width(), CV_8UC3);
						circle(tmp,cornersNL[orderL.at<int>(i)],20,Scalar(PEAK_VAL,PEAK_VAL,PEAK_VAL,0),-1);
						GaussianBlur(tmp,tmp,Size(29,29),0);
						add(tmp,salImL,salImL);
					}
				}
			}
			add(salImL,*oldSalL*0.5,salImL);
			//Mat gS = Mat::zeros(imgL->height(), imgL->width(), CV_8UC1);
			//cvtColor(salImL,gS,CV_RGB2GRAY);
			//double* minVal = new double(); double* maxVal = new double();
			//Point* minLoc = new Point(); Point* maxLoc = new Point();
			//minMaxLoc(gS,minVal,maxVal,minLoc,maxLoc);
			//salPeakLOut.add(maxLoc->x); salPeakLOut.add(maxLoc->y);
			
			for(int i = 0; i < MAX_CORNERS; i++){
				if(featuresFoundR[orderR.at<int>(i)]){
					if(errorsR.at<float>(i) > DISP_THRESH){
						Mat tmp = Mat::zeros(imgL->height(), imgL->width(), CV_8UC3);
						//Mat tmp2 = Mat::zeros(imgL->height(), imgL->width(), CV_8UC3);
						circle(tmp,cornersNR[orderR.at<int>(i)],20,Scalar(PEAK_VAL,PEAK_VAL,PEAK_VAL,0),-1);
						GaussianBlur(tmp,tmp,Size(29,29),0);
						add(tmp,salImR,salImR);
					}
				}
			}
			add(salImR,*oldSalR*0.5,salImR);
			//cvtColor(salImR,gS,CV_RGB2GRAY);
			//double* minVal; double* maxVal;
			//Point* minLoc; Point* maxLoc;
			//minMaxLoc(gS,minVal,maxVal,minLoc,maxLoc);
			//salPeakROut.add(maxLoc->x); salPeakROut.add(maxLoc->y);

			//convert back to iplImage
			
			//IplImage salLIpl = *salImL;
			//IplImage salRIpl = *salImR;
			
			//void* sL = &salLIpl;
			//void* sR = &salRIpl;
			
			//ImageOf<PixelRgb> salLOut = salL->prepare();
			//salLOut.wrapIplImage(sL);
			//ImageOf<PixelRgb> salROut = salR->prepare();
			//salROut.wrapIplImage(sR);
			
			//salLOut.resize(*imgL);
			//salROut.resize(*imgR);
			
			salL->write();
			salR->write();

			//salPeakL->write();
			//salPeakR->write();
		
			//delete ImL, ImR;
			//delete salImL, salImR;
			
			imgLO = imgL;
			imgRO = imgR;

			*oldSalL = salImL;
			*oldSalR = salImR;

			//delete minVal, maxVal;
			//delete minLoc, maxLoc;
			//cornersNL = cornersOL;
			//cornersNR = cornersOR;
		}

	}

	virtual void threadRelease(){
		eyeL->interrupt(); eyeR->interrupt();
		salL->interrupt(); salR->interrupt();
		//salPeakL->interrupt(); salPeakR->interrupt();
		eyeL->close(); eyeR->close();
		salL->close(); salR->close();
		//salPeakL->close(); salPeakR->close();
		delete eyeL; delete eyeR;
		delete salL; delete salR;
		//delete imgLO;
		//delete imgRO;
		delete oldSalL; delete oldSalR;
		//delete salPeakL; delete salPeakR;
		//delete cornersNL; delete cornersNR;
	}

};

class handLocModule : public RFModule {
protected:
	handLocThread *thr;
	Port *rpcPort;

public:
	handLocModule(){}

	bool respond(const Bottle& command, Bottle& reply){
		string msg(command.get(0).asString().c_str());
		reply.add(-1);
		return true;
	}

	virtual bool configure(ResourceFinder &rf){
		rpcPort = new Port;
		rpcPort->open("/handLoc");
		attach(*rpcPort);
		thr = new handLocThread(50,rf);
		bool ok = thr->start();
		if(!ok){
			delete thr;
			return false;
		}
		return true;
	}

	virtual bool interruptModule(){
		rpcPort->interrupt();
		return true;
	}

	virtual bool close(){
		rpcPort->close();
		thr->stop();
		delete rpcPort;
		delete thr;
		return true;
	}

	virtual double getPeriod() { return 1.0; }
	virtual bool updateModule() {return true;}
};

int main(int argc, char *argv[]){
	Network yarp;
	if(!yarp.checkNetwork()){
		return -1;
	}
	ResourceFinder rf;
	rf.configure("ICUB_ROOT",argc,argv);
	handLocModule mod;
	mod.runModule(rf);
	return 0;
}
