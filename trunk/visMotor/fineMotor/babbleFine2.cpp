/*
 * babbleFine2.cpp
 *
 * Lydia Majure
 * move hand by small amount,
 * locate in visual field, calculate disparity
 * train model with del-motors and <dx,dy,disparity>
 * fixate on hand again
 *
 * difference in version 2:
 * sparse storage of map updates (to come)
 * multi-kohonen model
 * don't use joint 2 on arm
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
	string rmFile;
	string emFile;

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

	int uMin; int uMax;
	int vMin; int vMax;
	int dMin; int dMax;

	//res = neurons/pixel
	double eRes;
	double rRes;

	int mmapSize;
	int usedJoints;

	int U; int V; int D;

	int Y; int P; int G;

	int maxDiv;

	SOM ****retMotMap;
	SOM ****egoMotMap;

	bool realRobot;

	int count;

	//not really yaw and pitch
	int azMin; int azMax;
	int elMin; int elMax;
	int verMin; int verMax;

	//ImageOf<PixelFloat> *pSalL;
	//ImageOf<PixelFloat> *pSalR;

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

		rmFile = rf.check("rMFile",Value("none")).asString().c_str();
		emFile = rf.check("eMFile",Value("none")).asString().c_str();

		//neckTT = rf.check("nt",Value(1.0)).asDouble();
		neckTT = rf.check("nt",Value(3.0)).asDouble();
		eyeTT = rf.check("et",Value(0.5)).asDouble();

		wsize = (int)rf.check("wsize",Value(25)).asInt()/2;
		nlags = rf.check("nlags",Value(50)).asInt();

		eRes = rf.check("eRes",Value(0.2)).asDouble();

		rRes = rf.check("rRes",Value(0.2)).asDouble();

		mmapSize = rf.check("mmapSize",Value(4)).asInt();
		usedJoints = rf.check("usedJoints",Value(3)).asInt();

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

		//flex hand
		(*command)[4] = 60;
		(*command)[7] = 20;
		(*command)[10] = 15;
		(*command)[11] = 15;
		(*command)[12] = 15;
		(*command)[13] = 15;
		(*command)[14] = 15;
		(*command)[15] = 15;

		pos->positionMove(command->data());

		bool done = false;
		while (!done){
			pos->checkMotionDone(&done);
		//	Time::delay(0.1);
		}


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

		//pSalL = portSalL->read(true);
		//pSalR= portSalR->read(true);

		uMin = 0; uMax = cxl*2;
		vMin = 0; vMax = cyl*2;
		dMin = 0; dMax = maxDiv;


		//not really yaw and pitch
		azMin = -80; azMax = 0;
		elMin = -60; elMax = 0;
		verMin = 0; verMax = 20;

		//number of units along a dimension
		U = (uMax-uMin)*rRes;
		V = (vMax-vMin)*rRes;
		D = (dMax-dMin)*rRes;

		Y = (azMax-azMin)*eRes;
		P = (elMax-elMin)*eRes;
		G = (verMax-verMin)*eRes;

		//initialize model
		retMotMap = new SOM***[U];
		for (int u = 0; u < U; u ++){
			retMotMap[u] = new SOM**[V];
			for (int v = 0; v < V; v++){
				retMotMap[u][v] = new SOM*[D];
				for (int d = 0; d < D; d++){
					retMotMap[u][v][d] = new SOM(mmapSize,usedJoints);
				}
			}
		}

		egoMotMap = new SOM***[Y];
		for (int y = 0; y < Y; y ++){
			egoMotMap[y] = new SOM**[P];
			for (int p = 0; p < P; p++){
				egoMotMap[y][p] = new SOM*[G];
				for (int g = 0; g < G; g++){
					egoMotMap[y][p][g] = new SOM(mmapSize,usedJoints);
				}
			}
		}

		if(rmFile != "none"){
			ifstream mapFile;
			mapFile.open(rmFile.c_str());
			string s;
			//get training count
			getline(mapFile,s);
			istringstream cnt(s);
			cnt >> count;
			//discard lines
			getline(mapFile,s);
			getline(mapFile,s);
			for(int u = 0; u < U; u++){
				for(int v= 0; v < V; v++){
					for(int d = 0; d < D; d++){
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
								retMotMap[u][v][d]->weights[k][n] = val;
							}
						}
					}
				}
			}
			mapFile.close();
		}

		if(emFile != "none"){
			ifstream mapFile;
			mapFile.open(emFile.c_str());
			string s;
			//get training count
			getline(mapFile,s);
			istringstream cnt(s);
			cnt >> count;
			//discard lines
			getline(mapFile,s);
			getline(mapFile,s);
			for(int u = 0; u < U; u++){
				for(int v= 0; v < V; v++){
					for(int d = 0; d < D; d++){
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
								egoMotMap[u][v][d]->weights[k][n] = val;
							}
						}
					}
				}
			}
			mapFile.close();
		}

		count = 0;


		return true;
	}

	//ret=false when egomotor map, =true when retinomotor map

	void mapWrite(string fName, bool ret){
		ofstream mapFile;
		mapFile.open(fName.c_str());
		if(ret){
			mapFile << count << endl;
			mapFile << U << " " << V << " " << D << endl;
			mapFile << mmapSize << " " << usedJoints << endl;
			for(int u = 0; u < U; u++){
				for(int v = 0; v < V; v++){
					for(int d = 0; d < D; d++){
						mapFile << u << " " << v << " " << d << endl;
						for(int k = 0; k < mmapSize; k++){
							mapFile << k << endl;
							for(int n = 0; n < usedJoints; n++){
								//printf("%f\t", retMotMap[u][v][d]->weights[k][n]);
								mapFile << retMotMap[u][v][d]->weights[k][n];
								mapFile << " ";
							}
							//printf("\n");
							mapFile << endl;
						}
					}
				}
			}
		}
		else{
			mapFile << count << endl;
			mapFile << Y << " " << P << " " << G << endl;
			mapFile << mmapSize << " " << usedJoints << endl;
			for(int y = 0; y < Y; y++){
				for(int p = 0; p < P; p++){
					for(int g = 0; g < G; g++){
						mapFile << y << " " << p << " " << g << endl;
						for(int k = 0; k < mmapSize; k++){
							mapFile << k << endl;
							for(int n = 0; n < usedJoints; n++){
								mapFile << (*egoMotMap[y][p][g]).weights[k][n];
								mapFile << " ";
							}
							mapFile << endl;
						}
					}
				}
			}
		}
		mapFile.close();
		return;
	}


	virtual void run(){

		while(isStopping() != true){

			*tmp = *command;
			(*command)[0] = (*command)[0] + 10*(2*gsl_rng_uniform(r)-1);
			(*command)[1] = (*command)[1] + 10*(2*gsl_rng_uniform(r)-1);
			//(*command)[2] = (*command)[2] + 10*(2*gsl_rng_uniform(r)-1);
			(*command)[2] = (*command)[2];
			(*command)[3] = (*command)[3] + 10*(2*gsl_rng_uniform(r)-1);

			if ((*command)[0] > -25 || (*command)[0] < -60){
				(*command)[0] = (*tmp)[0];
			}
			if ((*command)[1] > 100 || (*command)[1] < 10){
				(*command)[1] = (*tmp)[1];
			}
			//if ((*command)[2] > 60 || (*command)[2] < 0){
			//	(*command)[2] = (*tmp)[2];
			//}
			if ((*command)[3] > 100 || (*command)[3] < 10){
				(*command)[3] = (*tmp)[3];
			}

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
				//Time::delay(1);
				bool done = false;
				while(!done){
					pos->checkMotionDone(&done);
					//Time::delay(1);
				}

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



				yarp::sig::Vector *dMotor;
				yarp::sig::Vector *armJ;
				dMotor = new yarp::sig::Vector(usedJoints);
				dMotor->zero();
				armJ = new yarp::sig::Vector(usedJoints);
				armJ->zero();
				//for(int i = 0; i < usedJoints; i++){
				//	(*dMotor)[i] = (*command)[i] - (*tmp)[i];
				//	(*armJ)[i] = (*command)[i];
				//}

				//since we are skipping joint 2
				(*dMotor)[0] = (*command)[0] - (*tmp)[0];
				(*dMotor)[1] = (*command)[1] - (*tmp)[1];
				(*dMotor)[2] = (*command)[3] - (*tmp)[3];

				(*armJ)[0] = (*command)[0];
				(*armJ)[1] = (*command)[1];
				(*armJ)[2] = (*command)[3];



				//fixate
				//fix the fixation on environmental residuals
				printf("%i, %i; %i, %i\n", ul, vl, ur, vr);
				if (ul > 0 && ul < uMax && ur > 0 && ur < uMax && vl > 0 && vl < vMax && vr > 0 && vr < vMax && mxCrVal > 0.01 && mxCrVal < 0.99){

					yarp::sig::Vector oldHeadAng(3);
					igaze->getAngles(oldHeadAng);
					printf("Found hand? Fixating.\n");
					yarp::sig::Vector pxl(2), pxr(2);
					pxl[0] = ul; pxl[1] = vl;
					pxr[0] = ur; pxr[1] = vr;
					igaze->lookAtStereoPixels(pxl, pxr);
					done = false;
					int i = 0;
					while(!done && i < 5){
						igaze->checkMotionDone(&done);
						//Time::delay(0.5);
						sleep(1);
						i++;
					}

					yarp::sig::Vector headAng(3);
					igaze->getAngles(headAng);

					if(headAng[0] > azMin && headAng[0] < azMax && headAng[1] > elMin && headAng[1] < elMax && headAng[2] >verMin && headAng[2] < verMax){
						printf("Training map\n");
						count++;

						//train retinomotor map
						int wU = floor((ul-uMin)*rRes);
						int wV = floor((vl-vMin)*rRes);
						//this is a hack
						int wD = floor((mxCrVal*10-dMin)*rRes);
						//printf("Disparity: %.3lf\n", mxCrVal);
						double step = 0.5*exp(-count*1.0/(5*U*V*D*mmapSize));
						printf("rmap step size %.3lf\n", step);
						if(!(wU < 0 || wU >= U || wV < 0 || wV >= V || wD < 0 || wD >= D)){
							retMotMap[wU][wV][wD]->update(dMotor,step);
							if(wU - 1 >= 0){
								retMotMap[wU-1][wV][wD]->update(dMotor,step*0.25);
							}
							if(wU + 1 < U){
								retMotMap[wU+1][wV][wD]->update(dMotor,step*0.25);
							}
							if(wV - 1 >= 0){
								retMotMap[wU][wV-1][wD]->update(dMotor,step*0.25);
							}
							if(wV + 1 < V){
								retMotMap[wU][wV+1][wD]->update(dMotor,step*0.25);
							}
							if(wD - 1 >= 0){
								retMotMap[wU][wV][wD-1]->update(dMotor,step*0.25);
							}
							if(wD + 1 < D){
								retMotMap[wU][wV][wD+1]->update(dMotor,step*0.25);
							}
						}

						int wY = floor((headAng(0)-azMin)*eRes);
						int wP = floor((headAng(1)-elMin)*eRes);
						int wG = floor((headAng(2)-verMin)*eRes);
						step = 0.5*exp(-count*1.0/(5*Y*P*G*mmapSize));
						printf("emap step size %.3lf\n", step);
						if(!(wY < 0 || wY >= Y || wP < 0 || wP >= P || wG < 0 || wG >= G)){
							egoMotMap[wY][wP][wG]->update(armJ,step);
							if(wY - 1 >= 0){
								egoMotMap[wY-1][wP][wG]->update(armJ,step*0.25);
							}
							if(wY + 1 < Y){
								egoMotMap[wY+1][wP][wG]->update(armJ,step*0.25);
							}
							if(wP - 1 >= 0){
								egoMotMap[wY][wP-1][wG]->update(armJ,step*0.25);
							}
							if(wP + 1 < P){
								egoMotMap[wY][wP+1][wG]->update(armJ,step*0.25);
							}
							if(wG - 1 >= 0){
								egoMotMap[wY][wP][wG-1]->update(armJ,step*0.25);
							}
							if(wD + 1 < D){
								egoMotMap[wY][wP][wG+1]->update(armJ,step*0.25);
							}
						}
						if(count%100 == 0){
							string rName = "rMap" + boost::lexical_cast<string>(count) + ".dat";
							string eName = "eMap" + boost::lexical_cast<string>(count) + ".dat";
							mapWrite(rName, true);
							mapWrite(eName, false);
						}
						printf("Count: %i\n", count);
					}
					else{
						printf("No hand found, choosing random view\n");
						int az = azMin + (azMax-azMin)*gsl_rng_uniform(r);
						int el = elMin + (elMax-elMin)*gsl_rng_uniform(r);
						int ver = verMin + (verMax-verMin)*gsl_rng_uniform(r);
						yarp::sig::Vector ang(3);
						ang[0] = az; ang[1] = el; ang[2] = ver;
						igaze->lookAtAbsAngles(ang);
						done = false;
						int i = 0;
						while(!done && i < 5){
							igaze->checkMotionDone(&done);
							//Time::delay(0.5);
							sleep(1);
							i++;
						}
					}
				}
				else{
					printf("No hand found, choosing random view\n");
					int az = azMin + (azMax-azMin)*gsl_rng_uniform(r);
					int el = elMin + (elMax-elMin)*gsl_rng_uniform(r);
					int ver = verMin + (verMax-verMin)*gsl_rng_uniform(r);
					yarp::sig::Vector ang(3);
					ang[0] = az; ang[1] = el; ang[2] = ver;
					igaze->lookAtAbsAngles(ang);
					done = false;
					int i = 0;
					while(!done && i < 5){
						igaze->checkMotionDone(&done);
						//Time::delay(0.5);
						sleep(1);
						i++;
					}
				}

				delete [] mxVal;
				delete mxDxL;
				delete mxDxR;
				delete Sl;
				delete Sr;
				delete Scl;
				delete Scr;
				delete Sclm;
				delete Scrm;
				delete dMotor;
				delete armJ;
				//delete pSalL;
				//delete pSalR;
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
				thr->mapWrite(fName,false);
			}
			if(mp == "ret"){
				thr->mapWrite(fName,true);
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
		thr->mapWrite("initRetMap.dat", true);
		thr->mapWrite("initEgoMap.dat", false);
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




