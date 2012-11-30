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
const float DISP_THRESH = 0.0;
const int PEAK_VAL = 50;
const float DECAY = 0.75;

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

	//BufferedPort<ImageOf<PixelRgb> > *salL;
	//BufferedPort<ImageOf<PixelRgb> > *salR;
	BufferedPort<ImageOf<PixelMono> > *salL;
	BufferedPort<ImageOf<PixelMono> > *salR;

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

		salL=new BufferedPort<ImageOf<PixelMono> >;
		string salLName="/"+name+"/sal:l";
		salL->open(salLName.c_str());

		salR=new BufferedPort<ImageOf<PixelMono> >;
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

		oldSalL = new Mat(imgLO->height(), imgLO->width(), CV_32F);
		oldSalR = new Mat(imgLO->height(), imgLO->width(), CV_32F);

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
			*oldSalL = Mat::zeros(imgLO->height(), imgLO->width(), CV_32F);
			*oldSalR = Mat::zeros(imgLO->height(), imgLO->width(), CV_32F);
			firstRun = false;
		}
		else{

			ImageOf<PixelRgb> *imgL = eyeL->read(true);
			ImageOf<PixelRgb> *imgR = eyeR->read(true);
		
			//ImageOf<PixelRgb> &salLOut = salL->prepare();
			//ImageOf<PixelRgb> &salROut = salR->prepare();

			ImageOf<PixelMono> &salLOut = salL->prepare();
			ImageOf<PixelMono> &salROut = salR->prepare();
			
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
			vector<Point2f> cornersOL(MAX_CORNERS, Point2f(0,0));
	
			goodFeaturesToTrack(ImLOG,cornersOL,corner_count,0.01,5.0);

			//cornerSubPix(ImLOG,cornersOL,Size(10,10),Size(-1,-1),TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
			
			vector<uchar> featuresFoundL(MAX_CORNERS,0);
			vector<float> featuresErrorsL(MAX_CORNERS,0);
	
			vector<Point2f> cornersNL(MAX_CORNERS, Point2f(0,0));
	
			calcOpticalFlowPyrLK(ImLOG,ImLG,cornersOL,cornersNL,featuresFoundL,featuresErrorsL, Size(20,20),3);

	
			//repeat for right
	
	
			vector<Point2f> cornersOR(MAX_CORNERS, Point2f(0,0));
	
			goodFeaturesToTrack(ImROG,cornersOR,corner_count,0.01,5.0);
	
			//cornerSubPix(ImROG,cornersOR,Size(10,10),cvSize(-1,-1),TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
	
			vector<uchar> featuresFoundR(MAX_CORNERS,0);
			vector<float> featuresErrorsR(MAX_CORNERS,0);
	
			vector<Point2f> cornersNR(MAX_CORNERS,Point2f(0,0));
	
			calcOpticalFlowPyrLK(ImROG,ImRG,cornersOR,cornersNR,featuresFoundR,featuresErrorsR, Size(20,20), 3);
	
			//calculate most salient points, in order
			Mat errorsL = Mat(featuresErrorsL,true);
			Mat errorsR = Mat(featuresErrorsR,true);
			
			Mat orderL = Mat::zeros(MAX_CORNERS,1,CV_8UC1);
			Mat orderR = Mat::zeros(MAX_CORNERS,1,CV_8UC1);
			
			sortIdx(errorsL,orderL,CV_SORT_DESCENDING + CV_SORT_EVERY_COLUMN);
			sortIdx(errorsR,orderR,CV_SORT_DESCENDING + CV_SORT_EVERY_COLUMN);
			
			
			//produce salience images
	


			Mat salValL = Mat::zeros(imgL->height(), imgL->width(), CV_32F);
			Mat salValR = Mat::zeros(imgR->height(), imgR->width(), CV_32F);
			//salImL = Mat::zeros(imgL->height(), imgL->width(), CV_32F);
			//salImR = Mat::zeros(imgR->height(), imgR->width(), CV_32F);
			//salImL = Mat::zeros(imgL->height(), imgL->width(), CV_8U);
			//salImR = Mat::zeros(imgR->height(), imgR->width(), CV_8U);

			//for now, all salient corners get equal weight
			for(int i = 0; i < MAX_CORNERS; i++){
				if(featuresFoundL[orderL.at<int>(i)]){
					if(errorsL.at<float>(i) > DISP_THRESH){
						//Mat tmp = Mat::zeros(imgL->height(), imgL->width(), CV_8UC3);
						//circle(tmp,cornersNL[orderL.at<int>(i)],20,Scalar(PEAK_VAL,PEAK_VAL,PEAK_VAL,0),-1);
						Mat tmp = Mat::zeros(imgL->height(), imgL->width(), CV_32F);
						//Mat tmp = Mat::zeros(imgL->height(), imgL->width(), CV_8U);
						circle(tmp,cornersNL[orderL.at<int>(i)],20,Scalar(PEAK_VAL),-1);
						GaussianBlur(tmp,tmp,Size(29,29),0);
						//add(tmp,salImL,salImL);
						add(tmp,salValL,salValL);
					}
				}
			}
			add(salValL,*oldSalL*DECAY,salValL);
			
			for(int i = 0; i < MAX_CORNERS; i++){
				if(featuresFoundR[orderR.at<int>(i)]){
					if(errorsR.at<float>(i) > DISP_THRESH){
						//Mat tmp = Mat::zeros(imgL->height(), imgL->width(), CV_8UC3);
						//circle(tmp,cornersNR[orderR.at<int>(i)],20,Scalar(PEAK_VAL,PEAK_VAL,PEAK_VAL,0),-1);
						//Mat tmp = Mat::zeros(imgL->height(), imgL->width(), CV_8U);
						Mat tmp = Mat::zeros(imgL->height(), imgL->width(), CV_32F);
						circle(tmp,cornersNR[orderR.at<int>(i)],20,Scalar(PEAK_VAL),-1);
						GaussianBlur(tmp,tmp,Size(29,29),0);
						add(tmp,salValR,salValR);
					}
				}
			}
			add(salValR,*oldSalR*DECAY,salValR);

			*oldSalL = salValL;
			*oldSalR = salValR;

			salValR.convertTo(salImR,CV_8U);
			salValL.convertTo(salImL,CV_8U);

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
			
			*imgLO = *imgL;
			*imgRO = *imgR;

			//*oldSalL = salImL;
			//*oldSalR = salImR;

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
