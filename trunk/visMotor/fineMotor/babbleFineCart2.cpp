/*
 * babbleFineCart2.cpp
 *
 * Lydia Majure
 * move hand by small amount,
 * locate in visual field, calculate disparity
 * train model with del-motors and <dx,dy,disparity>
 * fixate on hand again
 *
 * this version builds a model between cartesian location of hand and joint configuration
 * 
 * diff between this and babbleFineCart.cpp:
 * old version was creating too much bias due to sample distribution, this methodically sweeps
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

#include <math.h>
#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>

#include <boost/lexical_cast.hpp>

#include "SOM.h"


using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

YARP_DECLARE_DEVICES(icubmod)

class fineMotorThread : public Thread{
protected:
	ResourceFinder &rf;

	Port *armPlan;
	Port *armPred;

	BufferedPort<ImageOf<PixelFloat> > *portSalL;
	BufferedPort<ImageOf<PixelFloat> > *portSalR;

	double neckTT, eyeTT;

	string name;
	string robotName;
	string arm;

	//load initial map from file
	string mFile;
	string cFile;

    //camera projection matrix/params
    Mat Pl;
    Mat Pr;
    double fxl, fyl, cxl, cyl;
    double fxr, fyr, cxr, cyr;
    int voffset;

    //aux. data for stereo processing
    Mat R1, R2, P1, P2, Q;
    Mat R, Rl, Rr;
    Mat mpxL, mpyL, mpxR, mpyR;
    Point * mxDxL, * mxDxR;
    double * mxVal;
    Mat CF;

	const gsl_rng_type *Type;
	gsl_rng *r;

	PolyDriver *clientGazeCtrl;
	PolyDriver *robotDevice;

	IGazeControl *igaze;
	IPositionControl *pos;
	IEncoders *enc;

	int nj;
	yarp::sig::Vector *command;
	yarp::sig::Vector *tmp;

	int wsize;
	int nlags;

	//some SOM parameters

	double xMin; double xMax;
	double yMin; double yMax;
	double zMin; double zMax;

	//res = neurons/pixel
	double res;

	int mmapSize;
	int usedJoints;

	int X; int Y; int Z;

	int maxDiv;

	SOM ****egoMotMap;
	
	//training counts for units
	double ***numTimes;

	bool realRobot;

	int count;

public:
	fineMotorThread(ResourceFinder &_rf) : Thread(), rf(_rf){}

	virtual bool handleParams(){
		if(rf.check("robot")){
			robotName = rf.find("robot").asString().c_str();
		}
		else{
			printf("Must specify robot name\n");
			return false;
		}

		name = rf.check("name",Value("fineMotor2")).asString().c_str();
		arm = rf.check("arm", Value("left")).asString().c_str();

		mFile = rf.check("mFile",Value("none")).asString().c_str();
		cFile = rf.check("cFile",Value("none")).asString().c_str();

		//neckTT = rf.check("nt",Value(1.0)).asDouble();
		neckTT = rf.check("nt",Value(3.0)).asDouble();
		eyeTT = rf.check("et",Value(0.5)).asDouble();

		wsize = (int)rf.check("wsize",Value(25)).asInt()/2;
		nlags = rf.check("nlags",Value(50)).asInt();

		res = rf.check("res",Value(20)).asDouble();

		mmapSize = rf.check("mmapSize",Value(4)).asInt();
		usedJoints = rf.check("usedJoints",Value(4)).asInt();

		//i suppose this is just arbitrary
		maxDiv = rf.check("maxDiv",Value(10)).asInt();

		voffset = rf.check("voffset",Value(0)).asInt();

		return true;
	}

	virtual bool threadInit(){
		if(!handleParams()){
			return false;
		}

		armPlan = new Port;
		armPred = new Port;

		armPlan->open("/fineMotor2/plan:o");
		armPred->open("/fineMotor2/pred:i");

		gsl_rng_env_setup();
		Type = gsl_rng_default;
		r = gsl_rng_alloc(Type);
		srand(time(0));
		gsl_rng_default_seed = rand();

		igaze = NULL;

		Property options;
		options.put("device","gazecontrollerclient");
		options.put("remote","/iKinGazeCtrl");
		options.put("local","/client/gaze");
		clientGazeCtrl = new PolyDriver;
		clientGazeCtrl->open(options);

		options.clear();
		string localPorts = "/fineMotor2/cmd";
		string remotePorts = "/" + robotName + "/" + arm + "_arm";

		options.put("device", "remote_controlboard");
		options.put("local", localPorts.c_str());
		options.put("remote", remotePorts.c_str());

		robotDevice = new PolyDriver;
		robotDevice->open(options);

		if(clientGazeCtrl->isValid()){
			clientGazeCtrl->view(igaze);
		}
		else{
			return false;
		}

        igaze->setNeckTrajTime(neckTT);
        igaze->setEyesTrajTime(eyeTT);

		if (!robotDevice->isValid()){
			printf("Device not available. Here are known devices: \n");
			printf("%s", Drivers::factory().toString().c_str());
			Network::fini();
			return false;
		}

		bool ok;
		ok = robotDevice->view(pos);
		ok = ok && robotDevice->view(enc);

		if (!ok){
			printf("Problems acquiring interfaces\n");
			return false;
		}

		pos->getAxes(&nj);

		command = new yarp::sig::Vector;
		tmp = new yarp::sig::Vector;
		command->resize(nj);
		tmp->resize(nj);

		for (int i = 0; i < nj; i++) {
		         (*tmp)[i] = 25.0;
		}
		pos->setRefAccelerations(tmp->data());

		for (int i = 0; i < nj; i++) {
			(*tmp)[i] = 20.0;
			pos->setRefSpeed(i, (*tmp)[i]);
		}

		if(robotName == "icub"){
			realRobot = true;
		}
		else if(robotName == "icubSim"){
			realRobot = false;
		}
		else{
			printf("Not a valid robot\n");
			return 1;
		}

		//set arm and head s.t. hand is visible

		*command = 0;

		//set the arm joints to "middle" values
		(*command)[0] = -45;
		(*command)[1] = 45;
		(*command)[2] = 0;
		(*command)[3] = 45;

		//set the arm to a random value
		//(*command)[0] = -60 + 35*gsl_rng_uniform(r);
		//(*command)[1] = 10 + 90*gsl_rng_uniform(r);
		//(*command)[2] = 0;
		//(*command)[3] = 10 + 90*gsl_rng_uniform(r);


		//flex hand
		(*command)[4] = 60;
		(*command)[7] = 20;
		(*command)[10] = 15;
		(*command)[11] = 15;
		(*command)[12] = 15;
		(*command)[13] = 15;
		(*command)[14] = 15;
		(*command)[15] = 15;

		//pos->positionMove(command->data());

		//bool done = false;
		//while (!done){
			//pos->checkMotionDone(&done);
		//	Time::delay(0.1);
		//}


		bool fwCvOn = 0;
		fwCvOn = Network::connect("/fineMotor2/plan:o","/fwdConv:i");
		fwCvOn *= Network::connect("/fwdConv:o","/fineMotor2/pred:i");
		if (!fwCvOn){
			printf("Please run command:\n ./fwdConv --input /fwdConv:i --output /fwdConv:o\n");
			return false;
		}


		Bottle plan, pred;
		plan.clear();
		pred.clear();
		for (int i = 0; i < nj; i++){
			plan.add((*command)[i]);
		}
		armPlan->write(plan);
		armPred->read(pred);
		yarp::sig::Vector commandCart(3);
		for (int i = 0; i < 3; i++){
			commandCart[i] = pred.get(i).asDouble();
		}

		double rad = sqrt(commandCart[0]*commandCart[0]+commandCart[1]*commandCart[1]);

		if(rad < 0.3){
			(*command)[0] = -45;
			(*command)[1] = 45;
			(*command)[2] = 0;
			(*command)[3] = 45;
		}


		pos->positionMove(command->data());

		bool done = false;
		int j = 0;
		while (!done && j < 5){
			pos->checkMotionDone(&done);
			sleep(1);
			j++;
		}

		//fixate exactly on hand to start
		igaze->lookAtFixationPoint(commandCart);
		done = false;
		int i = 0;
		while(!done && i < 5){
			igaze->checkMotionDone(&done);
			//Time::delay(0.5);
			sleep(1);
			i++;
		}

		//camera calib params
		Pl = Mat::eye(3,3,CV_64F);
		Pr = Mat::eye(3,3,CV_64F);
		Bottle ipsL = rf.findGroup("CAMERA_CALIBRATION_LEFT");
		Bottle ipsR = rf.findGroup("CAMERA_CALIBRATION_RIGHT");
		if(ipsL.size() && ipsR.size()){
			Pl.at<double>(0,2) = cxl = ipsL.find("w").asDouble()/2.0;
            Pl.at<double>(1,2) = cyl = ipsL.find("h").asDouble()/2.0;
            Pl.at<double>(0,0) = fxl = ipsL.find("fx").asDouble();
            Pl.at<double>(1,1) = fyl = ipsL.find("fy").asDouble();

            Pr.at<double>(0,2) = cxr = ipsR.find("w").asDouble()/2.0;
            Pr.at<double>(1,2) = cyr = ipsR.find("h").asDouble()/2.0;
            Pr.at<double>(0,0) = fxr = ipsR.find("fx").asDouble();
            Pr.at<double>(1,1) = fyr = ipsR.find("fy").asDouble();
		}
		else{
			fprintf(stdout, "Could not find camera calibration parameters\n");
			return false;
		}

		portSalL=new BufferedPort<ImageOf<PixelFloat> >;
		string portSallName="/"+name+"/sal:l";
		portSalL->open(portSallName.c_str());

		portSalR=new BufferedPort<ImageOf<PixelFloat> >;
		string portSalrName="/"+name+"/sal:r";
		portSalR->open(portSalrName.c_str());


		//PLACEHOLDER LIMITS CHANGE ME
		xMin = -0.4; xMax = 0.0;
		yMin = -0.5; yMax = 0.0;
		zMin = -0.2; zMax = 0.5;



		//number of units along a dimension
		X = (xMax-xMin)*res;
		Y = (yMax-yMin)*res;
		Z = (zMax-zMin)*res;

		//initialize model

		egoMotMap = new SOM***[X];
		for (int x = 0; x < X; x ++){
			egoMotMap[x] = new SOM**[Y];
			for (int y = 0; y < Y; y++){
				egoMotMap[x][y] = new SOM*[Z];
				for (int z = 0; z < Z; z++){
					egoMotMap[x][y][z] = new SOM(mmapSize,usedJoints);
				}
			}
		}

		numTimes = new double**[X];
		for (int x = 0; x < X; x++){
			numTimes[x] = new double*[Y];
			for (int y = 0; y < Y; y++){
				numTimes[x][y] = new double[Z];
				for (int z = 0; z < Z; z++){
					numTimes[x][y][z] = 0;
				}
			}
		}
		
		count = 0;
		if(mFile != "none"){
			ifstream mapFile;
			mapFile.open(mFile.c_str());
			string s;
			//get training count
			getline(mapFile,s);
			istringstream cnt(s);
			cnt >> count;
			//discard lines
			getline(mapFile,s);
			getline(mapFile,s);
			for(int x = 0; x < X; x++){
				for(int y = 0; y < Y; y++){
					for(int z = 0; z < Z; z++){
						//discard
						getline(mapFile,s);
						for(int k = 0; k < mmapSize; k++){
							//discard
							getline(mapFile,s);
							//split the string
							getline(mapFile,s);
							istringstream iss(s);
							double val;
							for(int n = 0; n < usedJoints; n++){
								iss >> val;
								egoMotMap[x][y][z]->weights[k][n] = val;
							}
						}
					}
				}
			}
			mapFile.close();
		}

		//ADD HERE: read in counts from a file and update numTimes
		if(cFile != "none"){
			ifstream countFile;
			countFile.open(cFile.c_str());
			string s;
			//get training count
			getline(countFile,s);
			istringstream cnt(s);
			cnt >> count;
			//discard lines
			getline(countFile,s);
			getline(countFile,s);
			for(int x = 0; x < X; x++){
				for(int y = 0; y < Y; y++){
					for(int z = 0; z < Z; z++){
						//discard
						getline(countFile,s);
						getline(countFile,s);
						istringstream iss(s);
						double val;
						iss >> val;
						numTimes[x][y][z] = val;
					}
				}
			}
			countFile.close();
		}

		
		return true;
	}


	void mapWrite(string fName){
		ofstream mapFile;
		mapFile.open(fName.c_str());
		mapFile << count << endl;
		mapFile << X << " " << Y << " " << Z << endl;
		mapFile << mmapSize << " " << usedJoints << endl;
		for(int x = 0; x < X; x++){
			for(int y = 0; y < Y; y++){
				for(int z = 0; z < Z; z++){
					mapFile << x << " " << y << " " << z << endl;
					for(int k = 0; k < mmapSize; k++){
						mapFile << k << endl;
						for(int n = 0; n < usedJoints; n++){
							mapFile << egoMotMap[x][y][z]->weights[k][n];
							mapFile << " ";
						}
						mapFile << endl;
					}
				}
			}
		}
		mapFile.close();
		return;
	}
	
	void countWrite(string fName){
		ofstream countFile;
		countFile.open(fName.c_str());
		countFile << count << endl;
		countFile << X << " " << Y << " " << Z << endl;
		countFile << mmapSize << " " << usedJoints << endl;
		for(int x = 0; x < X; x++){
			for(int y = 0; y < Y; y++){
				for(int z = 0; z < Z; z++){
					countFile << x << " " << y << " " << z << endl;
					countFile << numTimes[x][y][z] << endl;
				}
			}
		}
		countFile.close();
		return;
	}


	virtual void run(){

		//double rxMin = 0; double rxMax = -100;
		//double ryMin = 0; double ryMax = -100;
		//double rzMin = 100; double rzMax = -100;
		int loopCnt = 0;
		while(isStopping() != true){
			loopCnt++;
			printf("Loop number: %i\n",loopCnt);
			double j0 = -60;
			while(j0 < -25){
				j0 = j0 + 20*gsl_rng_uniform(r);
				double j1 = 10;
				while(j1 < 100){
					j1 = j1 + 20*gsl_rng_uniform(r);
					double j2 = 0;
					while(j2 < 60){
						j2 = j2 + 20*gsl_rng_uniform(r);
						double j3 = 10;
						while(j3 < 100){
							j3 = j3 + 20*gsl_rng_uniform(r);
							(*command)[0]=j0; (*command)[1]=j1;
							(*command)[2]=j2; (*command)[3]=j3;
							//use fwd kin to find end effector position
							Bottle plan, pred;
							plan.clear();
							pred.clear();
							for (int i = 0; i < nj; i++){
								plan.add((*command)[i]);
							}
							armPlan->write(plan);
							armPred->read(pred);
							yarp::sig::Vector commandCart(3);
							for (int i = 0; i < 3; i++){
								commandCart[i] = pred.get(i).asDouble();
							}
							double rad = sqrt(commandCart[0]*commandCart[0]+commandCart[1]*commandCart[1]);

							if(rad > 0.3){
								Mat * Sl, * Sr, * Scl, * Scr;
								Mat * Sclm, * Scrm;

								printf("Moving to new position\n");
								pos->positionMove(command->data());
								printf("Actual hand position: %0.3fm, %0.3f, %0.3f\n",commandCart[0],commandCart[1],commandCart[2]);
								/*
								if(commandCart[0] < rxMin){
									rxMin = commandCart[0];
								}
								if(commandCart[0] > rxMax){
									rxMax = commandCart[0];
								}
								if(commandCart[1] < ryMin){
									ryMin = commandCart[1];
								}
								if(commandCart[1] > ryMax){
									ryMax = commandCart[1];
								}
								if(commandCart[2] < rzMin){
									rzMin = commandCart[2];
								}
								if(commandCart[2] > rzMax){
									rzMax = commandCart[2];
								}
								*/
								//printf("%.2f < x < %.2f, %.2f < y < %.2f, %.2f < z < %.2f\n",rxMin,rxMax,ryMin,ryMax,rzMin,rzMax);
								//Time::delay(1);
								bool done = false;
								int i = 0;
								while(!done && i < 2){
									pos->checkMotionDone(&done);
									i++;
									sleep(1);
								}
							//}

								

								//logan's code

								//get salience images
								ImageOf<PixelFloat> *pSalL = portSalL->read(true);
								ImageOf<PixelFloat> *pSalR = portSalR->read(true);
								Sl = new Mat(pSalL->height(), pSalL->width(), CV_32F, (void *)pSalL->getRawImage());
								Sr = new Mat(pSalR->height(), pSalR->width(), CV_32F, (void *)pSalR->getRawImage());
								Scl = new Mat(pSalL->height(), pSalL->width(), CV_32F);
								Scr = new Mat(pSalR->height(), pSalR->width(), CV_32F);
								Sclm = new Mat(pSalL->height(), pSalL->width(), CV_32F);
								Scrm = new Mat(pSalR->height(), pSalR->width(), CV_32F);

								//get head and l/r eye matrices, assuming fixed torso
								Matrix Hl, Hr, H;
								Mat R(3,3, CV_64F);
								vector<double> T(3);
								yarp::sig::Vector eo, ep;
								igaze->getLeftEyePose(eo, ep);
								Hl = axis2dcm(ep);
								Hl(0,3) = eo[0]; Hl(1,3) = eo[1]; Hl(2,3) = eo[2];
								igaze->getRightEyePose(eo, ep);
								Hr = axis2dcm(ep);
								Hr(0,3) = eo[0]; Hr(1,3) = eo[1]; Hr(2,3) = eo[2];

								//get the transform matrix from the left image to the right image
								H = SE3inv(Hr)*Hl;
								for (int i = 0; i < 3; i++) {
									for (int j = 0; j < 3; j++) {
										R.at<double>(i,j) = H(i,j);
									}
									T.at(i) = H(i,3);
								}



								//rectify the images
								stereoRectify(Pl, Mat::zeros(4,1,CV_64F), Pr, Mat::zeros(4,1,CV_64F), Size(cxl*2, cyl*2), R, Mat(T), R1, R2, P1, P2, Q);
								initUndistortRectifyMap(Pl, Mat(), R1, P1, Size(cxl*2, cyl*2), CV_32FC1, mpxL, mpyL);
								initUndistortRectifyMap(Pr, Mat(), R2, P2, Size(cxr*2, cyr*2), CV_32FC1, mpxR, mpyR);
								remap(*Sl, *Scl, mpxL, mpyL, INTER_LINEAR);
								remap(*Sr, *Scr, mpxR, mpyR, INTER_LINEAR);

								//end logan's image rectification


								//locate the most salient point again (with center focusing)
								mxDxL = new Point;
								mxDxR = new Point;
								bool winimg = false;
								mxVal = new double[2];
								Scl->copyTo(*Sclm); Scr->copyTo(*Scrm);
								minMaxLoc(*Sclm, NULL, mxVal, NULL, mxDxL);
								minMaxLoc(*Scrm, NULL, mxVal+1, NULL, mxDxR);
								if (mxVal[1] > mxVal[0]) {
									mxDxL->x = mxDxR->x; mxDxL->y = mxDxR->y;
									winimg = true;
								}

								//adjust for vertical offset from left to right image
								int vamin, vamax, vadj;
								if (voffset < 0) {
									vamin = -voffset; vamax = Scr->rows-1; vadj = 0;
								} else if (voffset > 0) {
									vamin = 0; vamax = Scr->rows-1-voffset; vadj = voffset;
								} else {
									vamin = 0; vamax = Scr->rows-1; vadj = 0;
								}


								//calc the xcorr b/w left/right for various shifts of window around object
								Mat salImg, srcImg, corr;
								Rect salWin, srcWin;
								if (!winimg) {
									copyMakeBorder(*Scl, salImg, wsize, wsize, wsize, wsize, BORDER_CONSTANT, Scalar(0));
									copyMakeBorder(Scr->rowRange(vamin,vamax), srcImg, wsize+vadj, wsize+vamin+1, wsize+nlags, wsize+nlags, BORDER_CONSTANT, Scalar(0));
								} else {
									copyMakeBorder(Scr->rowRange(vamin,vamax), salImg, wsize+vadj, wsize+vamin+1, wsize, wsize, BORDER_CONSTANT, Scalar(0));
									copyMakeBorder(*Scl, srcImg, wsize, wsize, wsize+nlags, wsize+nlags, BORDER_CONSTANT, Scalar(0));
									mxDxL->y = mxDxL->y + voffset;
								}
								salWin = Rect(mxDxL->x, mxDxL->y, 2*wsize+1, 2*wsize+1);
								srcWin = Rect(mxDxL->x, mxDxL->y, 2*nlags+1, 2*wsize+1);
								matchTemplate(srcImg(srcWin), salImg(salWin), corr, CV_TM_CCORR_NORMED);


								//find max of xcorr and calculate disparity at interest point
								int mCorr; double mxCrVal = 0;
								for (int i = 0; i < corr.cols; i++) {
									if (corr.at<float>(0,i) > mxCrVal) {
										mxCrVal = corr.at<float>(0,i);
										mCorr = i;
									}
								}

								printf("Max corr: %.2lf\n", mxCrVal);


								//remap corresponding point onto the unrectified image
								if (!winimg) {
									mxDxR->x = std::max(std::min(mxDxL->x+mCorr-nlags-1, (int)cxl*2), 0);
									mxDxR->y = mxDxL->y - voffset;
								} else {
									mxDxR->x = mxDxL->x; mxDxR->y = mxDxL->y - voffset;
									mxDxL->x = std::max(std::min(mxDxL->x+mCorr-nlags-1, (int)cxl*2), 0);
								}
								int ul = std::max(std::min((int)mpxL.at<float>(*mxDxL), (int)cxl*2),0);
								int vl = std::max(std::min((int)mpyL.at<float>(*mxDxL), (int)cyl*2),0);
								int ur = std::max(std::min((int)mpxR.at<float>(*mxDxR), (int)cxr*2),0);
								int vr = std::max(std::min((int)mpyR.at<float>(*mxDxR)+voffset, (int)cyr*2),0);

								//end logan's code (triangulation not needed)

								//use motor diff and retinal loc + disp to train
								//how best to represent retinal location?
								//<ur,vr,mxCrVal>?




								yarp::sig::Vector *armJ;
								armJ = new yarp::sig::Vector(usedJoints);
								armJ->zero();


								(*armJ)[0] = (*command)[0];
								(*armJ)[1] = (*command)[1];
								(*armJ)[2] = (*command)[2];
								(*armJ)[3] = (*command)[3];



								//fixate
								//fix the fixation on environmental residuals
								printf("%i, %i; %i, %i\n", ul, vl, ur, vr);
								//if (ul > 0 && ul < cxl*2 && ur > 0 && ur < cxl*2 && vl > 0 && vl < cyl*2 && vr > 0 && vr < cyl*2 && mxCrVal > 0.01 && mxCrVal < 0.99){
								if(ul >= 0 && ul <= cxl*2 && ur >= 0 && ur < cxl*2 && vl >= 0 && vl <= cyl*2 && vr >= 0 && vr < cyl*2){

									printf("Found hand? Fixating.\n");
									yarp::sig::Vector pxl(2), pxr(2);
									pxl[0] = ul; pxl[1] = vl;
									pxr[0] = ur; pxr[1] = vr;
									//igaze->lookAtStereoPixels(pxl, pxr);
									igaze->lookAtMono(0,pxl,0.4);
									done = false;
									int i = 0;
									while(!done && i < 2){
										igaze->checkMotionDone(&done);
										//Time::delay(0.5);
										sleep(1);
										i++;
									}

									yarp::sig::Vector headFix(3);
									igaze->getFixationPoint(headFix);
									printf("Fixation point: %.2lf, %.2lf, %.2lf\n", headFix[0], headFix[1], headFix[2]);

									if(headFix[0] > xMin && headFix[0] < xMax && headFix[1] > yMin && headFix[1] < yMax && headFix[2] >zMin && headFix[2] < zMax){
										printf("Training map\n");
										count++;
										int wX = floor((headFix[0]-xMin)*res);
										int wY = floor((headFix[1]-yMin)*res);
										int wZ = floor((headFix[2]-zMin)*res);
										double step = 0.5*exp(-count*1.0/(10*X*Y*Z*mmapSize));
										printf("step size %.3lf\n", step);
										if(!(wX < 0 || wX >= X || wY < 0 || wY >= Y || wZ < 0 || wZ >= Z)){
											egoMotMap[wX][wY][wZ]->update(armJ,step);
											//UPDATE COUNTS
											numTimes[wX][wY][wZ]++;
											if(wX - 1 >= 0){
												egoMotMap[wX-1][wY][wZ]->update(armJ,step*0.25);
												//UPDATE COUNTS (by .25)
												numTimes[wX-1][wY][wZ] += 0.25;
											}
											if(wX + 1 < X){
												egoMotMap[wX+1][wY][wZ]->update(armJ,step*0.25);
												numTimes[wX+1][wY][wZ] += 0.25;
											}
											if(wY - 1 >= 0){
												egoMotMap[wX][wY-1][wZ]->update(armJ,step*0.25);
												numTimes[wX][wY-1][wZ] += 0.25;
											}
											if(wY + 1 < Y){
												egoMotMap[wX][wY+1][wZ]->update(armJ,step*0.25);
												numTimes[wX][wY+1][wZ] += 0.25;
											}
											if(wZ - 1 >= 0){
												egoMotMap[wX][wY][wZ-1]->update(armJ,step*0.25);
												numTimes[wX][wY][wZ-1] += 0.25;
											}
											if(wZ + 1 < Z){
												egoMotMap[wX][wY][wZ+1]->update(armJ,step*0.25);
												numTimes[wX][wY][wZ+1] += 0.25;
											}
										}
										if(count%100 == 0){
											string fName = "cMap" + boost::lexical_cast<string>(count) + ".dat";
											mapWrite(fName);
											//WRITE COUNTS
											fName = "cCounts" + boost::lexical_cast<string>(count) + ".dat";
											countWrite(fName);
										}
										printf("Count: %i\n", count);
									}
									else{
										printf("No hand found, choosing random view\n");
										//double xR = xMin + (xMax-xMin)*gsl_rng_uniform(r);
										//double yR = yMin + (yMax-yMin)*gsl_rng_uniform(r);
										//double zR = zMin + (zMax-zMin)*gsl_rng_uniform(r);
										//yarp::sig::Vector fixR(3);
										//fixR[0] = xR; fixR[1] = yR; fixR[2] = zR;
										//igaze->lookAtFixationPoint(fixR);
										double dAz = -5 + 10*gsl_rng_uniform(r);
										double dEl = -5 + 10*gsl_rng_uniform(r);
										double dVer = -5 + 10*gsl_rng_uniform(r);
										yarp::sig::Vector dAng(3);
										dAng[0] = dAz; dAng[1] = dEl; dAng[2] = dVer;
										igaze->lookAtRelAngles(dAng);
										done = false;
										int i = 0;
										while(!done && i < 2){
											igaze->checkMotionDone(&done);
											//Time::delay(0.5);
											sleep(1);
											i++;
										}
									}
								}
								else{
									printf("No hand found, choosing random view\n");
									//double xR = xMin + (xMax-xMin)*gsl_rng_uniform(r);
									//double yR = yMin + (yMax-yMin)*gsl_rng_uniform(r);
									//double zR = zMin + (zMax-zMin)*gsl_rng_uniform(r);
									//yarp::sig::Vector fixR(3);
									//fixR[0] = xR; fixR[1] = yR; fixR[2] = zR;
									//igaze->lookAtFixationPoint(fixR);
									double dAz = -5 + 10*gsl_rng_uniform(r);
									double dEl = -5 + 10*gsl_rng_uniform(r);
									double dVer = -5 + 10*gsl_rng_uniform(r);
									yarp::sig::Vector dAng(3);
									dAng[0] = dAz; dAng[1] = dEl; dAng[2] = dVer;
									igaze->lookAtRelAngles(dAng);
									done = false;
									int i = 0;
									while(!done && i < 2){
										igaze->checkMotionDone(&done);
										//Time::delay(0.5);
										sleep(1);
										i++;
									}
								}
							}
						}
					}
				}

			
				delete [] mxVal;
				delete mxDxL;
				delete mxDxR;
				//delete Sl;
				//delete Sr;
				//delete Scl;
				//delete Scr;
				//delete Sclm;
				//delete Scrm;
				//delete armJ;
			}
		}
	}

	virtual void threadRelease(){
		portSalL->close();
		portSalR->close();
		armPlan->close();
		armPred->close();
		clientGazeCtrl->close();
		robotDevice->close();
	}
};

class fineMotorModule : public RFModule{
protected:
	fineMotorThread *thr;
	Port *rpcPort;

public:
	fineMotorModule(){}

	bool respond(const Bottle& command, Bottle& reply){
		string msg(command.get(0).asString().c_str());
		string mp(command.get(1).asString().c_str());
		if(msg == "write"){
			string fName(command.get(2).asString().c_str());
			if(mp == "ego"){
				thr->mapWrite(fName);
			}
			reply = command;
		}
		else{
			reply = "Unrecognized command\n";
		}
		return true;
	}

	virtual bool configure(ResourceFinder &rf){
		rpcPort = new Port;
		rpcPort->open("/fineMotor2");
		attach(*rpcPort);
		thr = new fineMotorThread(rf);
		bool ok = thr->start();
		if(!ok){
			delete thr;
			return false;
		}
		thr->mapWrite("initCartMap.dat");
		return true;
	}

	virtual bool interruptModule(){
		rpcPort->interrupt();
		//thr->stop();
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
	YARP_REGISTER_DEVICES(icubmod)
	Network yarp;
	if(!yarp.checkNetwork()){
		return -1;
	}
	ResourceFinder rf;
	rf.configure("ICUB_ROOT",argc,argv);
	fineMotorModule mod;
	mod.runModule(rf);
	return 0;
}




