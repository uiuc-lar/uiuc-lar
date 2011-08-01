/*
 *  stereoAttentionSystem.cpp
 *
 * 	Logan Niehaus
 * 	7/19/11
 * 	rudimentary stereo attention system controller module. takes in a
 * 	composite (weighted sum) ego-sphere salience map for both left and right channels.
 *  the module switches b/w two basic behavioral modes: first, the module picks
 *  the most salient point from either l/r ego image. an 'estimate' focus is made
 *  by directing gaze at its az/el. after this, the unwarped salience maps are
 *  used to more accurately fixate on the salient region. to do this, the maps are
 *  rectified using the current kinematic configuration, and a windowed search is
 *  performed in the neighborhood of the salient region to register the correspondence.
 *  the gaze is then directed towards the xyz location triangulated from the match.
 *  a signal containing the object locations in each image [ul vl ur vr] is
 *  generated for the IOR module. fixation lasts some user-defined amount of time
 *  and then the algorithm returns to the global search state
 *
 *
 *  inputs:
 *  	/stereoAttention/ego:l	-- ego-mapped left weighted salience sum map
 *  	/stereoAttention/ego:r	-- ego-mapped right weighted salience sum map
 *  	/stereoAttention/sal:l	-- unwarped left weighted salience sum
 *  	/stereoAttention/sal:r	-- unwarped right weighted salience sum
 *  	/iKinGazeCtrl/head/		-- not connected, but must be running in order to function
 *
 *
 *  params: ([R]equired/[D]efault <Value>/[O]ptional)
 *
 *		azrange					-- total range of azimuth angles on incoming maps as first two args.
 *									optionally, may specify allowable search ranges as next args (R, ex: -60 60 -30 30)
 *		elrange					-- total range of elevation angles on incoming maps(R, ex: -90 90)
 *									allowable search ranges may also be specified
 *  	tol						-- stereo convergence tolerance (D 5.0)
 *  	nt, et					-- solver trajectory times; lower values -> faster head movement (D 1.0, 0.5 for icub)
 *  	wsize					-- scanning window size for stereo matching (D 25)
 *  	nlags					-- number of lags for stereo matching. will scan from -nlags to +nlags (D 50)
 *  	fixtime					-- object fixation duration, in seconds (D 5s)
 *  	<camera parameters>		-- intrinsic calibration params for each camera under [CAMERA_CALIBRATION_LEFT/RIGHT].
 *										see iKinGazeCtrl docs for example of how to do this
 *		lpf						-- apply lpf to maps before choosing max pt. arg is kernel size, w/ var size/5
 *		focus					-- apply gaussian focusing mask to images on secondary search. arg is var as fraction
 *									of image size
 *		voffset					-- vertical offset from left to right image in pixels; this adjusts for
 *									vertical camera misalignment on the icub (ex -20 shifts the right image up by 20)
 *		<x/y/zrange>			-- bounding box for allowable fixation points (in meters). ex: 'xrange 0.0 -10.0'
 *  	name					-- module port basename (D /stereoAttention)
 *  	debug					-- setting flag makes the module shoot debug info to stdout, and opens up debug image ports
 *
 *  outputs:
 *  	/stereoAttention/fxl:o	-- bottle containing fixation location in terms of image coordinates [ul vl ur vr].
 *  								generated when second-pass fixation point is determined
 *
 *  TODO:
 *  	complete system behavior needs testing
 *  	also need to basic stereo function using gaze control interface instead of joint port
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

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

#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::ctrl;

YARP_DECLARE_DEVICES(icubmod);

#define PI 3.14159


class stereoAttentionThread : public Thread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelFloat> > *portEgoL;
	BufferedPort<ImageOf<PixelFloat> > *portEgoR;
	BufferedPort<ImageOf<PixelFloat> > *portSalL;
	BufferedPort<ImageOf<PixelFloat> > *portSalR;
	BufferedPort<ImageOf<PixelFloat> > *portSalLO;
	BufferedPort<ImageOf<PixelFloat> > *portSalRO;
	Port *portFxlOut;

	//ikingaze objects and params
	PolyDriver clientGazeCtrl;
	IGazeControl *igaze;
	double neckTT, eyeTT;

	//ego map params
	double azlo, azhi, azalo, azahi;
	double ello, elhi, elalo, elahi;

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
	Mat CF;

	//misc running params
	int wsize, nlags;
	double fT;
	bool lpf, focus;
	int ksize;
	double fsize;
	bool debug;
	double xlo, xhi, ylo, yhi, zlo, zhi;

	int state;

public:

	stereoAttentionThread(ResourceFinder &_rf) : rf(_rf)
	{ }

	void topDownGazeCmd(yarp::sig::Vector &ang, bool wait = false) {

		igaze->lookAtAbsAngles(ang);
		if (wait)
			igaze->waitMotionDone(0.1, 10.0);

	}

	virtual bool threadInit()
	{

		name=rf.check("name",Value("stereoAttention")).asString().c_str();
		neckTT = rf.check("nt",Value(1.0)).asDouble();
		eyeTT = rf.check("et",Value(0.5)).asDouble();
		fT = rf.check("fixtime",Value(5.0)).asDouble();
		voffset = rf.check("voffset",Value(0)).asInt();
		wsize = (int)rf.check("wsize",Value(25)).asInt()/2;
		nlags = rf.check("nlags",Value(50)).asInt();
		debug = rf.check("debug");

		//get output map size
		Bottle arng = rf.findGroup("azrange");
		Bottle erng = rf.findGroup("elrange");
		if (arng.size() >= 3) {
			azlo = arng.get(1).asDouble(); azhi = arng.get(2).asDouble();
			azalo = azlo; azahi = azhi;
			if (arng.size() == 5) {
				azalo = arng.get(3).asDouble(); azahi = arng.get(4).asDouble();
			}
		} else {
			return false;
		}
		if (erng.size() >= 3) {
			ello = erng.get(1).asDouble(); elhi = erng.get(2).asDouble();
			elalo = ello; elahi = elhi;
			if (erng.size() == 5) {
				elalo = erng.get(3).asDouble(); elahi = erng.get(4).asDouble();
			}
		} else {
			return false;
		}

		//get camera calibration parameters
		Pl = Mat::eye(3, 3, CV_64F);
		Pr = Mat::eye(3, 3, CV_64F);
		Bottle ipsL = rf.findGroup("CAMERA_CALIBRATION_LEFT");
		Bottle ipsR = rf.findGroup("CAMERA_CALIBRATION_RIGHT");
		if (ipsL.size() && ipsR.size()) {

			Pl.at<double>(0,2) = cxl = ipsL.find("w").asDouble()/2.0;
			Pl.at<double>(1,2) = cyl = ipsL.find("h").asDouble()/2.0;
			Pl.at<double>(0,0) = fxl = ipsL.find("fx").asDouble();
			Pl.at<double>(1,1) = fyl = ipsL.find("fy").asDouble();

			Pr.at<double>(0,2) = cxr = ipsR.find("w").asDouble()/2.0;
			Pr.at<double>(1,2) = cyr = ipsR.find("h").asDouble()/2.0;
			Pr.at<double>(0,0) = fxr = ipsR.find("fx").asDouble();
			Pr.at<double>(1,1) = fyr = ipsR.find("fy").asDouble();

		}
		else {

			fprintf(stdout,"Could not find calibration parameters for one of the cameras\n");
			return false;

		}

		//get salience filtering options
		lpf = rf.check("lpf"); ksize = -1;
		if (lpf)
			ksize = rf.check("lpf", Value(25)).asInt();
		focus = rf.check("focus"); fsize = -1;
		if (focus)
			fsize = rf.check("focus", Value(0.5)).asDouble();


		//get allowable gaze locations if requested
		Bottle xrng = rf.findGroup("xrange"); xlo = -1e+10; xhi = 1e+10;
		Bottle yrng = rf.findGroup("yrange"); ylo = -1e+10; yhi = 1e+10;
		Bottle zrng = rf.findGroup("zrange"); zlo = -1e+10; zhi = 1e+10;
		if (xrng.size() == 3) {
			xlo = xrng.get(1).asDouble(); xhi = xrng.get(2).asDouble();
		}
		if (yrng.size() == 3) {
			ylo = yrng.get(1).asDouble(); yhi = yrng.get(2).asDouble();
		}
		if (zrng.size() == 3) {
			zlo = zrng.get(1).asDouble(); zhi = zrng.get(2).asDouble();
		}

		//open up ports
		portEgoL=new BufferedPort<ImageOf<PixelFloat> >;
		string portEgolName="/"+name+"/ego:l";
		portEgoL->open(portEgolName.c_str());

		portEgoR=new BufferedPort<ImageOf<PixelFloat> >;
		string portEgorName="/"+name+"/ego:r";
		portEgoR->open(portEgorName.c_str());

		portSalL=new BufferedPort<ImageOf<PixelFloat> >;
		string portSallName="/"+name+"/sal:l";
		portSalL->open(portSallName.c_str());

		portSalR=new BufferedPort<ImageOf<PixelFloat> >;
		string portSalrName="/"+name+"/sal:r";
		portSalR->open(portSalrName.c_str());

		portFxlOut=new Port;
		string portFxlName="/"+name+"/fxl:o";
		portFxlOut->open(portFxlName.c_str());
		portFxlOut->setTimeout(0.5);

		if (debug) {

			portSalLO=new BufferedPort<ImageOf<PixelFloat> >;
			string portSalloName="/"+name+"/sal:lo";
			portSalLO->open(portSalloName.c_str());

			portSalRO=new BufferedPort<ImageOf<PixelFloat> >;
			string portSalroName="/"+name+"/sal:ro";
			portSalRO->open(portSalroName.c_str());

		}

		//build the center focusing map
		if (focus) {
			CF = Mat::zeros(cyl*2, cxl*2, CV_32F);
			CF.at<float>(cyl, cxl) = 255.0;
			GaussianBlur(CF, CF, Size(cxl*2-1, cyl*2-1), cxl*fsize, cyl*fsize);
			CF = CF/(CF.at<float>(cyl,cxl));
		}

		//start up gaze control client interface
		Property option("(device gazecontrollerclient)");
		option.put("remote","/iKinGazeCtrl");
		option.put("local","/client/gaze");
		clientGazeCtrl.open(option);
		igaze=NULL;
		if (clientGazeCtrl.isValid()) {

			clientGazeCtrl.view(igaze);

			//set gaze control interface params
			igaze->setNeckTrajTime(neckTT);
			igaze->setEyesTrajTime(eyeTT);
			igaze->bindNeckPitch(-30,30);
			igaze->bindNeckYaw(-25,25);
			igaze->bindNeckRoll(-10,10);

		} else {

			printf("could not initialize gaze control interface, failing...\n");
			return false;

		}

		state = 0;

		return true;

	}

	virtual void run()
	{

		Mat * Esl, * Esr, * Sl, * Sr, * Scl, * Scr;
		Mat * Eslm, * Esrm, * Sclm, * Scrm;

		while (isStopping() != true) {


			//state 0: find most salient point, perform pre-focus
			if (state == 0) {

				ImageOf<PixelFloat> *pEgoL = portEgoL->read(true);
				ImageOf<PixelFloat> *pEgoR = portEgoR->read(true);
				Esl = new Mat(pEgoL->height(), pEgoL->width(), CV_32F, (void *)pEgoL->getRawImage());
				Esr = new Mat(pEgoR->height(), pEgoR->width(), CV_32F, (void *)pEgoR->getRawImage());
				Eslm = new Mat(pEgoL->height(), pEgoL->width(), CV_32F);
				Esrm = new Mat(pEgoR->height(), pEgoR->width(), CV_32F);

				//find point of highest salience across both pairs
				mxDxL = new Point;
				mxDxR = new Point;
				double * mxVal = new double[2];

				//filter if specified
				if (lpf) {
					GaussianBlur(*Esl, *Eslm, Size(ksize,ksize), ksize/5.0, ksize/5.0);
					GaussianBlur(*Esr, *Esrm, Size(ksize,ksize), ksize/5.0, ksize/5.0);
				} else {
					Esl->copyTo(*Eslm); Esr->copyTo(*Esrm);
				}

				//find the max pt locations in allowable region
				Rect alReg;
				alReg.x = std::max((int)((azalo-azlo)*(float)pEgoL->width()/(azhi-azlo)),0);
				alReg.width = std::min((int)((azahi-azlo)*(float)pEgoL->width()/(azhi-azlo)-alReg.x), pEgoL->width());
				alReg.y = std::max((int)((elhi-elahi)*(float)pEgoL->height()/(elhi-ello)),0);
				alReg.height = std::min((int)((elhi-elalo)*(float)pEgoL->height()/(elhi-ello)-alReg.y), pEgoL->height());
				minMaxLoc((*Eslm)(alReg), NULL, mxVal, NULL, mxDxL);
				minMaxLoc((*Esrm)(alReg), NULL, mxVal+1, NULL, mxDxR);
				if (mxVal[1] > mxVal[0]) {
					mxDxL->x = mxDxR->x+alReg.x; mxDxL->y = mxDxR->y+alReg.y;
				} else {
					mxDxL->x = mxDxL->x+alReg.x; mxDxL->y = mxDxL->y+alReg.y;
				}

				//publish annotated egomap if requested
				if (debug) {

					ImageOf<PixelFloat> &loImg = portSalLO->prepare();
					ImageOf<PixelFloat> &roImg = portSalRO->prepare();
					(*Esl) = (*Esl)*2.0; (*Esr) = (*Esr)*2.0;
					if (mxVal[1] > mxVal[0]) {
						circle(*Esr, *mxDxL, 3, Scalar(255), -1);
					} else {
						circle(*Esl, *mxDxL, 3, Scalar(255), -1);
					}
					loImg.copy(*pEgoL); roImg.copy(*pEgoR);
					portSalLO->write();
					portSalRO->write();

				}

				//find the az/el location of the point
				double az, el;
				az = azlo + ((azhi-azlo)/(float)pEgoL->width())*(mxDxL->x);
				el = elhi - ((elhi-ello)/(float)pEgoL->height())*(mxDxL->y);
				if (debug) {
					printf("directing gaze to az: %f, el: %f\n", az, el);
				}

				//move head to that az/el
				yarp::sig::Vector tang(3);
				tang[0] = az; tang[1] = el; tang[2] = 1;
				igaze->lookAtAbsAngles(tang);

				//wait for head move to complete, then transition to local search state
				igaze->waitMotionDone(0.1, 10.0); //wait a max of 10s for motion to finish
				state = 1;

				delete Esl, Esr;
				delete Eslm, Esrm;
				delete mxDxL, mxVal;


			}
			//state 1: lock in on most salient point in both images, track while verging
			else {


				//get the normal salience maps after pre-focusing
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

				//locate the most salient point again (with center focusing)
				mxDxL = new Point;
				mxDxR = new Point;
				bool winimg = false;
				double * mxVal = new double[2];
				if (lpf) {
					GaussianBlur(*Scl, *Sclm, Size(ksize,ksize), ksize/5.0, ksize/5.0);
					GaussianBlur(*Scr, *Scrm, Size(ksize,ksize), ksize/5.0, ksize/5.0);
				} else {
					Scl->copyTo(*Sclm); Scr->copyTo(*Scrm);
				}
				if (focus) {
					multiply(*Sclm, CF, *Sclm);
					multiply(*Scrm, CF, *Scrm);
				}
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

				//remap corresponding point onto the unrectified image
				if (!winimg) {
					mxDxR->x = std::max(std::min(mxDxL->x+mCorr-nlags-1, (int)cxl*2), 0);
					mxDxR->y = mxDxL->y - voffset;
				} else {
					mxDxR->x = mxDxL->x; mxDxR->y = mxDxL->y - voffset;
					printf("%d, %d, %d\n", mCorr, mCorr-nlags-1, (int)cxl*2);
					mxDxL->x = std::max(std::min(mxDxL->x+mCorr-nlags-1, (int)cxl*2), 0);
				}
				int ul = std::max(std::min((int)mpxL.at<float>(*mxDxL), (int)cxl*2),0);
				int vl = std::max(std::min((int)mpyL.at<float>(*mxDxL), (int)cyl*2),0);
				int ur = std::max(std::min((int)mpxR.at<float>(*mxDxR), (int)cxr*2),0);
				int vr = std::max(std::min((int)mpyR.at<float>(*mxDxR)+voffset, (int)cyr*2),0);

				//triangulate the point (least squares)
				Matrix A(4,3);
				yarp::sig::Vector b(4);
				yarp::sig::Vector Xp(3);
				Matrix iTmp(3,4); iTmp.zero(); iTmp(2,2) = 1.0;
				iTmp(0,0) = fxl; iTmp(1,1) = fyl;
				iTmp(0,2) = cxl-ul; iTmp(1,2) = cyl-vl;
				Hl = iTmp*SE3inv(Hl);
				iTmp(0,0) = fxr; iTmp(1,1) = fyr;
				iTmp(0,2) = cxr-ur; iTmp(1,2) = cyr-vr;
				Hr = iTmp*SE3inv(Hr);
				for (int i = 0; i < 2; i++) {
					for (int j = 0; j < 3; j++) {
						A(i,j) = Hl(i,j);
						A(i+2,j) = Hr(i,j);
					}
				}
				b[0] = -Hl(0,3); b[1] = -Hl(1,3);
				b[2] = -Hr(0,3); b[3] = -Hr(1,3);
				Xp = pinv(A)*b;

				//publish annotated salience map if requested
				if (debug) {

					ImageOf<PixelFloat> &loImg = portSalLO->prepare();
					ImageOf<PixelFloat> &roImg = portSalRO->prepare();
					(*Sl) = (*Sl)*2.0; (*Sr) = (*Sr)*2.0;
					if (!winimg) {
						circle(*Sl, Point(ul,vl), 10, Scalar(255), 2);
						circle(*Sr, Point(ur,vr-voffset), 5, Scalar(255), 2);
					} else {
						circle(*Sl, Point(ul,vl), 5, Scalar(255), 2);
						circle(*Sr, Point(ur,vr-voffset), 10, Scalar(255), 2);
					}
					loImg.copy(*pSalL); roImg.copy(*pSalR);
					portSalLO->write();
					portSalRO->write();
					printf("Fixating on point at x: %f, y: %f, z: %f\n", Xp[0], Xp[1], Xp[2]);
					printf("Img Coords ul: %d, vl: %d, ur: %d, vr: %d\n", ul,vl,ur,vr);

				}

				Bottle fxloc;
				fxloc.addInt(ul); fxloc.addInt(vl); fxloc.addInt(ur); fxloc.addInt(vr);

				//check if fixation point falls within allowable bounds
				if (Xp[0] < xhi && Xp[0] > xlo && Xp[1] < yhi && Xp[1] > xlo &&
						Xp[2] < zhi && Xp[2] > zlo) {

					//direct gaze to fixation point an issue an IOR request there
					igaze->lookAtFixationPoint(Xp);
					igaze->waitMotionDone(0.1, 10.0); //wait a max of 10s for motion to finish
					Time::delay(fT); //wait some additional time while looking at the object

					//generate signal for IOR
					fxloc.addInt(0); //final zero indicates that xyz projection is valid

				} else {

					//if outside, no gaze shift; publish only img coord of peak salience
					Bottle fxloc;
					if (!winimg) {
						fxloc.addInt(-1); //-1 signals xyz invalid, apply only in left image
					} else {
						fxloc.addInt(1); //1 signals xyz invalid, use only in right image
					}

				}
				portFxlOut->write(fxloc);

				state = 0;

				delete Sl, Sr, Scl, Scr;
				delete Sclm, Scrm;
				delete mxDxL, mxDxR, mxVal;


			}

		}

	}

	virtual void threadRelease()
	{

		if (clientGazeCtrl.isValid())
			clientGazeCtrl.close();

		portEgoL->interrupt();
		portEgoR->interrupt();
		portSalL->interrupt();
		portSalR->interrupt();
		portFxlOut->interrupt();

		portEgoL->close();
		portEgoR->close();
		portSalL->close();
		portSalR->close();
		portFxlOut->close();

		delete portEgoL, portEgoR, portSalL, portSalR;
		delete portFxlOut;

		if (debug) {
			portSalLO->interrupt();
			portSalRO->interrupt();
			portSalLO->close();
			portSalRO->close();
			delete portSalLO, portSalRO;
		}

	}

};

class stereoAttentionModule: public RFModule
{
protected:

	stereoAttentionThread *thr;
	Port * rpcPort;
	string name;

public:

	stereoAttentionModule() { }

	bool respond(const Bottle& command, Bottle& reply) {

		string msg(command.get(0).asString().c_str());
		if (msg == "ang") {
			if (command.size() < 2) {
				reply.add(-1);
			}
			else {
				yarp::sig::Vector azelr(3);
				azelr[0] = command.get(1).asDouble();
				azelr[1] = command.get(2).asDouble();
				azelr[2] = command.get(3).asDouble();
				thr->topDownGazeCmd(azelr,true);
				reply.add(1);
			}
		}
		else {
			reply.add(-1);
		}

		return true;

	}

	virtual bool configure(ResourceFinder &rf)
	{

		//set up the rpc port
		name=rf.check("name",Value("stereoAttention")).asString().c_str();
		rpcPort = new Port;
		string portRpcName="/"+name+"/rpc";
		rpcPort->open(portRpcName.c_str());
		attach(*rpcPort);


		thr=new stereoAttentionThread(rf);
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

	YARP_REGISTER_DEVICES(icubmod);

	Network yarp;

	if (!yarp.checkNetwork())
		return -1;

	ResourceFinder rf;

	rf.configure("ICUB_ROOT",argc,argv);

	stereoAttentionModule mod;

	return mod.runModule(rf);
}



