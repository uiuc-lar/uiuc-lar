// Threading headers
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
// Connection headers
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/sig/ImageDraw.h>
// Motor Control headers
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
// Gaze Control headers
#include <yarp/dev/GazeControl.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>
// Math headers
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
// OpenCV headers
#include <cv.h>

// Regular C headers
#include <string>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <fstream>
//#include <random>

#define ESMAX -3.0
#define WSMAX 7.0
#define SSMAX -15.0
#define NSMAX -5.0
#define EMAX -15.0
#define WMAX 20.0
#define SMAX -20.0
#define NMAX 15.0

#define OFFSET_X 2.0
#define OFFSET_Y -10.0

#define K_P 0.0015
#define K_D 0.001
#define K_I 0.0002
#define W_B 4.0

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

#define PI M_PI
#define ARMR_JNTS 16
#define HEAD_JNTS 6

bool greater_point(const std::vector<cv::Point2i>& a, const std::vector<cv::Point2i>& b) {
  return a.size() > b.size();
};

class mazeThread : public RateThread
{
	protected:

		ResourceFinder &rf;
		string name;
		string robot;

    // Ports (vision)
		BufferedPort<yarp::sig::Vector > *portHead;
		BufferedPort<ImageOf<PixelRgb> > *portImgL;
		BufferedPort<ImageOf<PixelRgb> > *portImgR;
	  BufferedPort<ImageOf<PixelBgr> > *portImgLO;
	  BufferedPort<ImageOf<PixelBgr> > *portImgRO;
		BufferedPort<ImageOf<PixelBgr> > *portImgO;

    // Driver (motor)
    PolyDriver driverArmR;
    PolyDriver driverHead;
    IPositionControl *ipos;
    IVelocityControl *ivel;
    IEncoders *iencs;
    yarp::sig::Vector encoders;
    yarp::sig::Vector setpoints;
    int jnts;

    //ikingaze objects and params
    PolyDriver clientGazeCtrl;
    IGazeControl *igaze;
    double neckTT, eyeTT;

		//forward kinematics for eyes (if no gazectrl)
		iCubEye * eyeL;
		iCubEye * eyeR;

		//camera projection matrix/params
		Mat Pl, Pr;
		double fxl, fyl, cxl, cyl;
		double fxr, fyr, cxr, cyr;
		int wl, hl, wr, hr;
		Mat mpxL, mpyL, mpxR, mpyR;
		Mat impxL, impyL, impxR, impyR;

    //storage objects for current disparity map and perspective transform matrix
	  yarp::sig::Vector QL,QR;
    Matrix H0, Hl0, Hr0;
    Mat R, R1, R2, P1, P2;

    //color thresholds (HSV)
    int threshold_H_min1;// initial minimum red threshold	(lower wrap)
    int threshold_H_max1;// initial maximum red threshold	(lower wrap)
    int threshold_H_min2;// initial minimum red threshold	(upper wrap)
    int threshold_H_max2;// initial maximum red threshold	(upper wrap)
    int max_H;
    int threshold_S_min;// initial minimum saturation threshold
    int threshold_S_max;// initial maximum saturation threshold
    int max_S;
    int threshold_V_min;// initial minimum intensity threshold
    int threshold_V_max;// initial maximum intensity threshold
    int max_V;
    bool ballfound;
    double ball_x, ball_y;
    
    //homography
    std::vector<Point2f> p_trans;
    std::vector<cv::Point2f> p_untrans;
    yarp::sig::Vector ball_uv;
    yarp::sig::Vector corner_sw;
    yarp::sig::Vector corner_nw;
    yarp::sig::Vector corner_ne;
    yarp::sig::Vector corner_se;

    //control stuff
    bool controlstarted;
    bool handclosed;
    // parameters
    double k_p, k_i, k_d, w_b;
    // state
    double b_dt, b_dto, tstamp;
    double b_x, b_y;
    double b_xo, b_yo;
    double b_dx, b_dy;
    double b_dxo, b_dyo;
    double b_xint, b_yint;
    // filtered
    double b_xf, b_yf;
    double b_dxf, b_dyf;
    int b_xm, b_ym;
    // control
    double b_ux, b_uy;
    double b_uxo, b_uyo;
    double b_uxf, b_uyf;
    double b_uxfo, b_uyfo;
    double b_uxsig, b_uysig;
    // hard maxes
    // 4: -11 E -7 == 8 W 12
    // 6: -18 S -15 == 1 N 5
    double ssoft, nsoft, esoft, wsoft;
    double shard, nhard, ehard, whard;
    double theta_max;
    // offsets
    double off_x, off_y;

		//module parameters
		bool strict;

		//semaphores for interaction with rpc port
		Semaphore * rpcMutex;

    // File output
    double omega_max;// maximum PE frequency
    double omega_max_threshold_upper;// = 2.0 * PI * 30.0 / 10.0;
    double omega_max_threshold_lower;// = 0.0;
    int b_m;
    int b_m_threshold_upper;// = 100;
    int b_m_threshold_lower;// = 5;
    ofstream myfile;
    double b_t;
    //std::random_device b_rd;

	public:

		mazeThread(ResourceFinder &_rf) : RateThread(5), rf(_rf) {}

		void merge(const ImageOf<PixelRgb> &imgR, const ImageOf<PixelRgb> &imgL, ImageOf<PixelBgr> &out)
		{
			if (out.width()!=imgR.width())
				out.resize(imgR.width(), imgR.height());

			int rr=0;
			for(int r=0; r<out.height(); r++)
			{
				const unsigned char *tmpR=imgR.getRow(rr);
				const unsigned char *tmpL=imgL.getRow(rr);
				unsigned char *tmpO=out.getRow(r);

				for(int c=0; c<out.width(); c++)
				{
					tmpO[2]=(unsigned char) (1/3.0*(tmpL[0]+tmpL[1]+tmpL[2]));
					tmpO[1]=(unsigned char) (1/3.0*(tmpR[0]+tmpR[1]+tmpR[2]));
          tmpO[0]=(unsigned char) 0;

					tmpO+=3;
					tmpL+=(3);
					tmpR+=(3);
				}

				rr++;
			}
		}

    void findBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs) {
      blobs.clear();
      // Fill the label_image with the blobs
      // 0  - background
      // 1  - unlabelled foreground
      // 2+ - labelled foreground
      cv::Mat label_image;
      binary.convertTo(label_image, CV_32SC1);
      int label_count = 2; // starts at 2 because 0,1 are used already
      for(int y=0; y < label_image.rows; y++) 
      {
        int *row = (int*)label_image.ptr(y);
        for(int x=0; x < label_image.cols; x++) 
        {
          if(row[x] != 1) 
          {
            continue;
          }
          cv::Rect rect;
          cv::floodFill(label_image, cv::Point(x,y), label_count, &rect, 0, 0, 4);
          std::vector <cv::Point2i> blob;
          for(int i=rect.y; i < (rect.y+rect.height); i++)
          {
            int *row2 = (int*)label_image.ptr(i);
            for(int j=rect.x; j < (rect.x+rect.width); j++) 
            {
              if(row2[j] != label_count)
              {
                continue;
              }
              blob.push_back(cv::Point2i(j,i));
            }
          }
          if (blob.size() > 10) { // min blob size
            blobs.push_back(blob);
            label_count++;
          }
        }
      }
    }

		virtual bool threadInit() {

			name=rf.check("name",Value("maze")).asString().c_str();
      robot=rf.check("robot",Value("icub")).asString().c_str();
      neckTT = rf.check("nt",Value(1.0)).asDouble();
      eyeTT = rf.check("et",Value(0.5)).asDouble();

			//get camera calibration parameters
			Pl = Mat::eye(3, 3, CV_64F);
			Pr = Mat::eye(3, 3, CV_64F);
			Bottle ipsL = rf.findGroup("CAMERA_CALIBRATION_LEFT");
			Bottle ipsR = rf.findGroup("CAMERA_CALIBRATION_RIGHT");
			if (ipsL.size() && ipsR.size()) {

				Pl.at<double>(0,2) = cxl = ipsL.find("cx").asDouble();
				Pl.at<double>(1,2) = cyl = ipsL.find("cy").asDouble();
				Pl.at<double>(0,0) = fxl = ipsL.find("fx").asDouble();
				Pl.at<double>(1,1) = fyl = ipsL.find("fy").asDouble();
				wl = ipsL.find("w").asInt();
				hl = ipsL.find("h").asInt();
				printf("Left cam parameters: %f, %f, %f, %f\n",cxl,cyl,fxl,fyl);

				Pr.at<double>(0,2) = cxr = ipsR.find("cx").asDouble();
				Pr.at<double>(1,2) = cyr = ipsR.find("cy").asDouble();
				Pr.at<double>(0,0) = fxr = ipsR.find("fx").asDouble();
				Pr.at<double>(1,1) = fyr = ipsR.find("fy").asDouble();
				wr = ipsR.find("w").asInt();
				hr = ipsR.find("h").asInt();
				printf("Right cam parameters: %f, %f, %f, %f\n",cxr,cyr,fxr,fyr);

			}
			else {

				fprintf(stdout,"Could not find calibration parameters for one of the cameras\n");
				return false;

			}

      printf("QL:\n");
      Bottle pars=rf.findGroup("STEREO_DISPARITY");
      if (Bottle *pXo=pars.find("QL").asList())
      {
        QL.resize(pXo->size());
        for (int i=0; i<(pXo->size()); i++) {
          QL[i]=pXo->get(i).asDouble();
          printf("%f,",QL(i));
        }

      }
      printf("\n");
      printf("QR:\n");

      if (Bottle *pXo=pars.find("QR").asList())
      {
        QR.resize(pXo->size());
        for (int i=0; i<(pXo->size()); i++) {
          QR[i]=pXo->get(i).asDouble();
          printf("%f,",QR(i));
        }
      }
      printf("\n");

      Bottle extrinsics=rf.findGroup("STEREO_DISPARITY");
      if (Bottle *pXo=extrinsics.find("HN").asList()) {
        H0.resize(4,4);
        for (int i=0; i<4; i++) {
          for (int j=0; j<4; j++) {
            H0(i,j) = pXo->get(i*4+j).asDouble();
          }
        }
      }

      printf("H0:\n");
      for (int i =0; i<4; i++) {
        for (int j=0; j<4; j++) {
          printf("%f,", H0(i,j));
        }
        printf("\n");
      }

      strict = rf.check("strict");

			//open up ports
			portHead=new BufferedPort<yarp::sig::Vector >;
			string portHeadName="/"+name+"/head:i";
			portHead->open(portHeadName.c_str());

			portImgL=new BufferedPort<ImageOf<PixelRgb> >;
			string portImgLName="/"+name+"/img:l";
			portImgL->open(portImgLName.c_str());

			portImgR=new BufferedPort<ImageOf<PixelRgb> >;
			string portImgRName="/"+name+"/img:r";
			portImgR->open(portImgRName.c_str());
		
      portImgLO=new BufferedPort<ImageOf<PixelBgr> >;
      string portImgLOName="/"+name+"/img:lo";
      portImgLO->open(portImgLOName.c_str());

      portImgRO=new BufferedPort<ImageOf<PixelBgr> >;
      string portImgROName="/"+name+"/img:ro";
      portImgRO->open(portImgROName.c_str());

			portImgO=new BufferedPort<ImageOf<PixelBgr> >;
			string portImgOName="/"+name+"/img:o";
			portImgO->open(portImgOName.c_str());
		
      //start up gaze control client interface
      Property optGaze("(device gazecontrollerclient)");
      optGaze.put("remote","/iKinGazeCtrl");
      optGaze.put("local","/client/gaze");
      clientGazeCtrl.open(optGaze);
      igaze=NULL;
      if (clientGazeCtrl.isValid()) {
        clientGazeCtrl.view(igaze);

        //set gaze control interface params
        igaze->setNeckTrajTime(neckTT);
        igaze->setEyesTrajTime(eyeTT);
        igaze->bindNeckPitch(-30,30);
        igaze->bindNeckYaw(-25,25);
        igaze->bindNeckRoll(-10,10);
      }
      else {
        printf("Could not initialize gaze control interface, please use head angle input port...\n");
      }

      //initialize head kinematics
      eyeL = new iCubEye("left");
      eyeR = new iCubEye("right");
      eyeL->setAllConstraints(false);
      eyeR->setAllConstraints(false);
			eyeL->releaseLink(0); eyeR->releaseLink(0);
			eyeL->releaseLink(1); eyeR->releaseLink(1);
			eyeL->releaseLink(2); eyeR->releaseLink(2);

			//set reader ports to do strict reads
			if (strict) {
				portImgL->setStrict(true);
				portImgR->setStrict(true);
			}
		  
      Hl0 = eyeL->getH(QL);
		  Hr0 = eyeR->getH(QR);


      //initialize head driver
      Property optHead("(device remote_controlboard)");
      string devremHeadName="/"+robot+"/head";
      optHead.put("remote", devremHeadName.c_str());
      string devlocHeadName="/"+name+"/head";
      optHead.put("local", devlocHeadName.c_str());

      //create device
      driverHead.open(optHead);
      if (!driverHead.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return false;
      }

      //initialize interface
      driverHead.view(ipos);
      driverHead.view(ivel);
      driverHead.view(iencs);
      if (ipos==NULL || ivel==NULL || iencs==NULL) {
        printf("Cannot get interface to robot head\n");
        driverHead.close();
        return false;
      }

      //get encoders
      jnts = 0;
      iencs->getAxes(&jnts);
      if (jnts != HEAD_JNTS) {
        printf("Joint number mismatch for head\n");
        driverHead.close();
        return false;
      }
      encoders.resize(jnts);
      setpoints.resize(jnts);

      //reference speed/accel
      yarp::sig::Vector tmp;
      tmp.resize(jnts);
      for (int i = 0; i < jnts; i++) {
        tmp[i] = 0.0;
      }
      ipos->setRefAccelerations(tmp.data());
      for (int i = 0; i < jnts; i++) {
        tmp[i] = 10.0;
        ipos->setRefSpeed(i, tmp[i]);
      }

      //initialize arm driver
      Property optArmR("(device remote_controlboard)");
      string devremArmRName="/"+robot+"/right_arm";
      optArmR.put("remote", devremArmRName.c_str());
      string devlocArmRName="/"+name+"/right_arm";
      optArmR.put("local", devlocArmRName.c_str());

      //create device
      driverArmR.open(optArmR);
      if (!driverArmR.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return false;
      }

      //initialize interface
      driverArmR.view(ipos);
      driverArmR.view(ivel);
      driverArmR.view(iencs);
      if (ipos==NULL || ivel==NULL || iencs==NULL) {
        printf("Cannot get interface to robot right_arm\n");
        driverArmR.close();
        return false;
      }

      //get encoders
      jnts = 0;
      iencs->getAxes(&jnts);
      if (jnts != ARMR_JNTS) {
        printf("Joint number mismatch for right_arm\n");
        driverArmR.close();
        return false;
      }
      encoders.resize(jnts);
      setpoints.resize(jnts);

      //reference speed/accel
      tmp.resize(jnts);
      for (int i = 0; i < jnts; i++) {
        tmp[i] = 0.0;
      }
      ipos->setRefAccelerations(tmp.data());
      for (int i = 0; i < jnts; i++) {
        tmp[i] = 10.0;
        ipos->setRefSpeed(i, tmp[i]);
      }
      /*
      for (int i = 0; i < jnts; i++) {
        tmp[i] = 30.0;
      }
      ipos->setRefSpeeds(tmp.data());
      */
      // Initialize thresholds;
      threshold_H_min1 = 0;// initial minimum red threshold	(lower wrap)
      threshold_H_max1 = 10;// initial maximum red threshold	(lower wrap)
      threshold_H_min2 = 160;// initial minimum red threshold	(upper wrap)
      threshold_H_max2 = 179;// initial maximum red threshold	(upper wrap)
      max_H = 179;
      threshold_S_min = 60;// initial minimum saturation threshold
      threshold_S_max = 200;// initial maximum saturation threshold
      max_S = 255;
      threshold_V_min = 50;// initial minimum intensity threshold
      threshold_V_max = 160;// initial maximum intensity threshold
      max_V = 255;

      ballfound = false;
      ball_x = 0;
      ball_y = 0;

      // Initialize abs corners
      //Order is SW, NW, NE, SE
      p_untrans.clear();
      p_untrans.push_back(Point2f(-64, 64));
      p_untrans.push_back(Point2f(-64, -64));
      p_untrans.push_back(Point2f(64, -64));
      p_untrans.push_back(Point2f(64, 64));

      ball_uv.resize(3);
      corner_sw.resize(3);
      corner_nw.resize(3);
      corner_ne.resize(3);
      corner_se.resize(3);

      // Initialize control stuff
      controlstarted = false;
      handclosed = false;
      // parameters
      k_p = K_P;
      k_d = K_D;
      k_i = K_I;
      w_b = W_B;
      // state
      b_dt = b_dto = 0.0;
      b_x = b_xo = 0.0;
      b_y = b_yo = 0.0;
      b_dx = b_dxo = 0.0;
      b_dy = b_dyo = 0.0;
      b_xf = b_yf = 0.0;
      b_dxf = b_dyf = 0.0;
      b_xint = b_yint = 0;
      //control
      b_ux = b_uxo = 0.0;
      b_uy = b_uyo = 0.0;
      b_uxf = b_uxfo = 0.0;
      b_uyf = b_uyfo = 0.0;
      theta_max = 0.1745; // 10 deg
      // 4: -11 E -7 == 8 W 12
      // 6: -18 S -15 == 1 N 5
      esoft = ESMAX;
      wsoft = WSMAX;
      ssoft = SSMAX;
      nsoft = NSMAX;
      ehard = EMAX;
      whard = WMAX;
      shard = SMAX;
      nhard = NMAX;
      // offsets
      off_x = OFFSET_X;
      off_y = OFFSET_Y;

			//initialize semaphores
			rpcMutex = new Semaphore;

      // open file for writing

      omega_max = 2.0 * PI * 30.0 / 15.0;// maximum PE frequency
      omega_max_threshold_upper = 2.0 * PI * 30.0 / 5.0;
      omega_max_threshold_lower = 0.0;
      b_m = 40.0;
      b_m_threshold_upper = 100;
      b_m_threshold_lower = 5;
      myfile.open("sysidentout.txt");
      myfile << "********* Omega_max: " << omega_max << "   #sinusoids (m): " << b_m << endl;
      b_t = 0.0;

			return true;
		}

		virtual void run() {

			//make sure there are two images available before grabbing one

			if (portImgL->getPendingReads() > 0 && portImgR->getPendingReads() >0) {

				//get the normal salience maps after pre-focusing
				ImageOf<PixelRgb> *pImgL = portImgL->read(false);
				ImageOf<PixelRgb> *pImgR = portImgR->read(false);

        //get envelope
        Stamp tStamp;
        portImgR->getEnvelope(tStamp);
        b_dto = b_dt;
        b_dt = tStamp.getTime() - tstamp;
        tstamp = tStamp.getTime();
        // filter
        b_dt = (b_dt + b_dto)/2;
        //printf("( t,dt) %f , %f\n", tstamp, b_dt);

        //set up (calibrated) color image matrices
				Mat Sl, Sr;
				Mat Scl, Scr;
        Mat Qtmp;

				Sl = (IplImage *) pImgL->getIplImage();
				Sr = (IplImage *) pImgR->getIplImage();

				//set up head and l/r eye matrices, assuming fixed torso
				Matrix H, Hlt, Hrt;
				Mat R(3,3, CV_64F);
				vector<double> T(3);
				yarp::sig::Vector eo, ep;

				bool havePose;

        //look for joint information
        yarp::sig::Vector *headAng = portHead->read(false);
        if (clientGazeCtrl.isValid()) {
          igaze->getLeftEyePose(eo, ep);
          Hlt = axis2dcm(ep);
          Hlt(0,3) = eo[0]; Hlt(1,3) = eo[1]; Hlt(2,3) = eo[2];

          igaze->getRightEyePose(eo, ep);
          Hrt = axis2dcm(ep);
          Hrt(0,3) = eo[0]; Hrt(1,3) = eo[1]; Hrt(2,3) = eo[2];

          havePose = true;
        }
        else if (headAng) {
          yarp::sig::Vector angles(8);

					//assuming torso fixed at home here
					angles[0] = 0.0; angles[1] = 0.0; angles[2] = 0.0;

					//get head angles (for left eye only at first)
					angles[3] = (*headAng)[0]; angles[4] = (*headAng)[1];
					angles[5] = (*headAng)[2]; angles[6] = (*headAng)[3];
					angles[7] = (*headAng)[4] + (*headAng)[5]/2.0;
					angles = PI*angles/180.0;

					Hlt = eyeL->getH(angles);
					angles[7] = PI*((*headAng)[4] - (*headAng)[5]/2.0)/180.0;
					Hrt = eyeR->getH(angles);

					havePose = true;
				}
				// if neither available, flag, don't do anything
				else {
					havePose = false;
				}

				//if we have a proper head/eye pose reading, do stuff
				if (havePose) {

					ImageOf<PixelBgr> &oImg = portImgO->prepare();
					oImg.resize(*pImgL);
					Mat om((IplImage *)oImg.getIplImage(), false);

          // some basic computation (remove for final)
		    	//merge(*pImgR, *pImgL, oImg);
          
          //get the transform matrix from the left image to the right image
          H = SE3inv(Hrt)*Hr0*H0*SE3inv(SE3inv(Hlt)*Hl0);
          for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
              R.at<double>(i,j) = H(i,j);
            }
            T.at(i) = H(i,3);
          }

          //rectify the images
          stereoRectify(Pl, Mat::zeros(4,1,CV_64F), Pr, Mat::zeros(4,1,CV_64F), Size(wl, hl), R, Mat(T), R1, R2, P1, P2, Qtmp, CALIB_ZERO_DISPARITY, -1);
          initUndistortRectifyMap(Pl, Mat(), R1, P1, Size(wl, hl), CV_32FC1, mpxL, mpyL);
          initUndistortRectifyMap(Pr, Mat(), R2, P2, Size(wr, hr), CV_32FC1, mpxR, mpyR);
          remap(Sl, Scl, mpxL, mpyL, INTER_LINEAR);
          remap(Sr, Scr, mpxR, mpyR, INTER_LINEAR);

          //convert to HSV color space and get the S channel (for colored objects basically)
          Mat scl1(Scl.rows, Scl.cols, CV_8UC3);
          Mat scr1(Scr.rows, Scr.cols, CV_8UC3);

					cvtColor(Scl,scl1,CV_RGB2HSV);
					cvtColor(Scr,scr1,CV_RGB2HSV);

          //find key points and ball
          //just use the image from the right camera
          Mat thr1, thr2;
          inRange(scr1, Scalar(threshold_H_min1, threshold_S_min,threshold_V_min), 
              Scalar(threshold_H_max1, threshold_S_max, threshold_V_max), thr1);
          inRange(scr1, Scalar(threshold_H_min2, threshold_S_min,threshold_V_min), 
              Scalar(threshold_H_max2, threshold_S_max, threshold_V_max), thr2);
          thr1 = (thr1 + thr2)/2;
		      cv::Mat output = cv::Mat::zeros(thr1.size(), CV_8UC3);
          cv::Mat binary;
          std::vector < std::vector<cv::Point2i > > blobs;
          threshold(thr1, binary, 0.0, 1.0, THRESH_BINARY);
          findBlobs(binary, blobs);
          // Sort blobs
          std::sort(blobs.begin(), blobs.end(), greater_point);

          std::vector<double> x_mean;
          std::vector<double> y_mean;
          int npt = blobs.size() > 5 ? 5 : blobs.size();
          Point mean_blobs[1][npt];
          x_mean.resize(npt,0.0);
          y_mean.resize(npt,0.0);
          //Select the 5 largest blobs
          for(int i=0; i<npt; i++)
          {
            for(int j=0; j < blobs[i].size(); j++)
            {
              x_mean[i]+=blobs[i][j].x;
              y_mean[i]+=blobs[i][j].y;
            }
            x_mean[i]/=blobs[i].size();
            y_mean[i]/=blobs[i].size();
            mean_blobs[0][i]=Point(x_mean[i],y_mean[i]);
          }
          // Find corners and ball
          int ncrn = npt > 4 ? 4 : npt;
          Point mean_corners[1][ncrn];
          std::vector<double> corner_error;
          corner_error.resize(npt,0.0);//Select the 4 largest blobs (5 with ball)
          int frame_width=thr1.size().width;
          int frame_height=thr1.size().height;

          //corner points defined going counter clockwise starting at top right corner
          double x_corner[]={0,50,frame_width-1-50,frame_width-1};
          double y_corner[]={frame_height-1,0,0,frame_height-1};

          std::vector<int> corner_map;
          corner_map.resize(ncrn,0);
          for(size_t i=0; i<ncrn; i++)//find the error to the 4 corners
          {
            for(size_t j=0; j<npt; j++)//Select the 4 largest blobs (5 with ball)
            {
              corner_error[j]=pow(x_mean[j]-x_corner[i],2)+pow(y_mean[j]-y_corner[i],2);
            }
            corner_map[i]=min_element(corner_error.begin(),corner_error.end())-corner_error.begin();
            mean_corners[0][i] = mean_blobs[0][corner_map[i]];
          }

          // missing is ball
          int ball_idx = 0;
          ballfound = false;
          if (npt == 5) {
            for (int i = 0; i < ncrn; i++) {
              ball_idx += corner_map[i];
            }
            ball_idx = 10-ball_idx;
            ball_x = x_mean[ball_idx];
            ball_y = y_mean[ball_idx];
            ballfound = true;
          }

          // plot polygon
		      const Point* ppts[1] = { mean_corners[0] };
		      int npts[] = { ncrn };
          int lineType = 8;
          fillPoly(thr1,ppts,npts,1,Scalar( 255, 255, 255 ),lineType);
          // Randomy color the blobs
          for(int i=0; i < blobs.size(); i++) {
            unsigned char r = 255 * (rand()/(1.0 + RAND_MAX));
            unsigned char g = 255 * (rand()/(1.0 + RAND_MAX));
            unsigned char b = 255 * (rand()/(1.0 + RAND_MAX));
            for(size_t j=0; j < blobs[i].size(); j++) {
              int x = blobs[i][j].x;
              int y = blobs[i][j].y;
              output.at<cv::Vec3b>(y,x)[0] = b;
              output.at<cv::Vec3b>(y,x)[1] = g;
              output.at<cv::Vec3b>(y,x)[2] = r;
            }
          }
          fillPoly(output,ppts,npts,1,Scalar( 255, 255, 255 ),lineType);


          for(size_t i=0; i<ncrn; i++)//label corners
          {
            if(i==0)
            {
              putText(output, "SW", cvPoint(mean_corners[0][i].x,mean_corners[0][i].y),
                  FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
            }
            if(i==1)
            {
              putText(output, "NW", cvPoint(mean_corners[0][i].x,mean_corners[0][i].y),
                  FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
            }
            if(i==2)
            {
              putText(output, "NE", cvPoint(mean_corners[0][i].x,mean_corners[0][i].y),
                  FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
            }
            if(i==3)
            {
              putText(output, "SE", cvPoint(mean_corners[0][i].x,mean_corners[0][i].y),
                  FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
            }
          }
          // label ball
          if (ballfound) {
            putText(output, "ball", cvPoint(ball_x,ball_y),
                FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,0,0), 1, CV_AA);
          }
          else {
            putText(output, "ball", cvPoint(ball_x,ball_y),
                FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
          }

          // Inverse homography
          //Order is SW, NW, NE, SE
          // x : 0 - width : W - E
          // y : 0 - height: N - S
          p_trans.clear();
          for (int i = 0; i < ncrn; i++) {
            p_trans.push_back(mean_corners[0][i]);
          }

          if (p_trans.size() == 4) { // found corners
            cv::Mat A_H = findHomography(p_trans, p_untrans);

            ball_uv[0] = ball_x;
            ball_uv[1] = ball_y;
            ball_uv[2] = 1;
            corner_sw[0] = mean_corners[0][0].x;
            corner_sw[1] = mean_corners[0][0].y;
            corner_sw[2] = 1;
            corner_nw[0] = mean_corners[0][1].x;
            corner_nw[1] = mean_corners[0][1].y;
            corner_nw[2] = 1;
            corner_ne[0] = mean_corners[0][2].x;
            corner_ne[1] = mean_corners[0][2].y;
            corner_ne[2] = 1;
            corner_se[0] = mean_corners[0][3].x;
            corner_se[1] = mean_corners[0][3].y;
            corner_se[2] = 1;
            yarp::sig::Matrix A_Htrans;
            A_Htrans.resize(3,3);
            for (int i = 0; i < 3; i++) {
              for (int j = 0; j < 3; j++) {
                A_Htrans[i][j] = A_H.at<double>(i,j);
                //printf("%f ", A_Htrans[i][j]);
              }
              //printf("\n");
            }
            ball_uv = A_Htrans*ball_uv;
            if (ball_uv[2] != 0.0) {
              ball_uv = ball_uv/ball_uv[2];
              // add an integrator to these?
              b_x = ball_uv[0];
              b_y = ball_uv[1];
              //b_dx = (b_x - b_xo)/b_dt;
              //b_dy = (b_y - b_yo)/b_dt;
            }
            corner_sw = A_Htrans*corner_sw;
            if (corner_sw[2] != 0.0)
              corner_sw = corner_sw/corner_sw[2];
            corner_nw = A_Htrans*corner_nw;
            if (corner_nw[2] != 0.0)
              corner_nw = corner_nw/corner_nw[2];
            corner_ne = A_Htrans*corner_ne;
            if (corner_ne[2] != 0.0)
              corner_ne = corner_ne/corner_ne[2];
            corner_se = A_Htrans*corner_se;
            if (corner_se[2] != 0.0)
              corner_se = corner_se/corner_se[2];
            //printf("%f %f %f\n", ball_uv[0], ball_uv[1], ball_uv[2]);
          }
          if (b_x > 64.0) { b_x = 64.0; }
          if (b_x < -64.0) { b_x = -64.0; }
          if (b_y > 64.0) { b_y = 64.0; }
          if (b_y < -64.0) { b_y = -64.0; }
          b_dx = (b_x - b_xo)/b_dt;
          b_dy = (b_y - b_yo)/b_dt;
          // filter
          b_xf = b_x;//(b_x + b_xf)/2;
          b_yf = b_y;//(b_y + b_yf)/2;
          b_dxf = b_dx;//(b_dx + b_dxf)/2;
          b_dyf = b_dy;//(b_dy + b_dyf)/2;
          b_xm = (int)floor((b_xf+64.0)/4);
          if (b_xm > 31) { b_xm = 31; }// bin from values 0 to 31
          b_ym = (int)floor((b_yf+64.0)/4);
          if (b_ym > 31) { b_ym = 31; }// bin from values 0 to 31
          //printf("( x, y): %f , %f\n", b_x, b_y);
          //printf("(dx,dy): %f , %f\n", b_dx, b_dy);
          
		      cv::Mat hgrph = cv::Mat::zeros(thr1.size(), CV_8UC3);
          Point abs_corners[1][ncrn];
          for (int i = 0; i < ncrn; i++) {
            abs_corners[0][i].x = p_untrans[i].x+100.0;
            abs_corners[0][i].y = p_untrans[i].y+100.0;
          }
		      const Point* hppts[1] = { abs_corners[0] };
          //transformed grid
          fillPoly(hgrph,hppts,npts,1,Scalar( 255, 255, 255 ),lineType);
          /*for(int i = 0; i < 33; i++) {
              cv::line(hgrph,Point(100.0,i*4.0+100.0),Point(128.0+100.0,i*4.0+100.0),cv::Scalar(0,255,0));
              cv::line(hgrph,Point(i*4.0+100.0,100.0),Point(i*4.0+100.0,100.0),cv::Scalar(0,255,0));
          }
          */
          cv::circle(hgrph,Point(ball_uv[0]+2.0+100.0,ball_uv[1]+2.0+100.0), 2, cv::Scalar(0,255,0), -1);

          if (ballfound) {
            putText(hgrph, "ball", cvPoint(ball_uv[0]+100,ball_uv[1]+100),
                FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,0,0), 1, CV_AA);
          }
          else {
            putText(hgrph, "ball", cvPoint(ball_uv[0]+100,ball_uv[1]+100),
                FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
          }
          putText(hgrph, "SW", cvPoint(corner_sw[0]+100,corner_sw[1]+100),
              FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA, false);
          putText(hgrph, "NW", cvPoint(corner_nw[0]+100,corner_nw[1]+100),
              FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
          putText(hgrph, "NE", cvPoint(corner_ne[0]+100,corner_ne[1]+100),
              FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
          putText(hgrph, "SE", cvPoint(corner_se[0]+100,corner_se[1]+100),
              FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA, false);

          // Perform Control
          if(controlstarted) {
            // do stuff
            iencs->getEncoders(encoders.data());
            // E-W control 4
            b_xint = b_x*b_dt+b_xint;
            //b_ux = k_p*b_x - k_d*(b_x-b_xo)/b_dt - k_i*b_xint;
            b_ux = k_p*b_xf + k_d*b_dxf + k_i*b_xint;
            b_uxf = (b_dt*w_b)/(2 + b_dt*w_b)*(b_ux + b_uxo)
              + (2 - b_dt*w_b)/(2 + b_dt*w_b)*b_uxfo;
            b_uxsig = off_x + b_uxf*(180/PI);
            // S-N control 6
            b_yint = b_y*b_dt+b_yint;
            //b_uy = k_p*b_y - k_d*(b_y-b_yo)/b_dt - k_i*b_yint;
            b_uy = k_p*b_yf + k_d*b_dyf + k_i*b_yint;
            b_uyf = (b_dt*w_b)/(2 + b_dt*w_b)*(b_uy + b_uyo)
              + (2 - b_dt*w_b)/(2 + b_dt*w_b)*b_uyfo;
            b_uysig = off_y + b_uyf*(180/PI);
            // Print stuff
            printf("x,y: %f %f\n", b_xf, b_yf);
            printf("dxy: %f %f\n", b_dxf, b_dyf);
            printf("non: %f %f\n", b_uxf, b_uyf);
            printf("raw: %f %f\n", b_uxsig, b_uysig);
            printf("enc: %f %f\n", encoders[4], encoders[6]);
            // Overwrite control input with persistent excitation
            // (i.e. no feedback from ball)
            // TODO

            b_t += b_dt;
		
		        //omega_max= 2.0 * PI * 30.0 / 100.0;// maximum PE frequency
		        //b_m = 20; //number of frequencies = integer greater than 0

            double w[2][100]={{
              0.683715572408358,
                0.132082955713563,
                0.722724539656766,
                0.110353480642349,
                0.117492852151833,
                0.640717922965926,
                0.328814214756803,
                0.653812022595774,
                0.749131463103519,
                0.583185731454876,
                0.740032327987778,
                0.234826914747904,
                0.734957541696052,
                0.970598525086615,
                0.866930291751916,
                0.0862345298634963,
                0.366436616319199,
                0.369198804330018,
                0.685028472661609,
                0.597941635383889,
                0.789363943641905,
                0.367652918437877,
                0.206027859505195,
                0.0866665473955323,
                0.771933917099107,
                0.205674521464760,
                0.388271631047802,
                0.551778531957228,
                0.228953252023100,
                0.641940620399187,
                0.484480372398180,
                0.151845525116267,
                0.781931966588002,
                0.100606322362422,
                0.294066333758628,
                0.237373019705579,
                0.530872257027928,
                0.0914987313394122,
                0.405315419880591,
                0.104846247115757,
                0.112283962156027,
                0.784427890743913,
                0.291570317906931,
                0.603533438750887,
                0.964422667215901,
                0.432484993970361,
                0.694752194617940,
                0.758099275289454,
                0.432642326147101,
                0.655498039803537,
                0.109755050723052,
                0.933759848385332,
                0.187460806421687,
                0.266178838907639,
                0.797830260211597,
                0.487603775433924,
                0.768958264058869,
                0.396006745217875,
                0.272938794123691,
                0.0372346340703280,
                0.673294914108653,
                0.429564459251853,
                0.451739234904736,
                0.609857169290216,
                0.0594032968582772,
                0.315811438338866,
                0.772722130862935,
                0.696432989006095,
                0.125332181109180,
                0.130151450389424,
                0.0923523387192016,
                0.00782029356933489,
                0.423109385164167,
                0.655573174937914,
                0.722922524692024,
                0.531209293582439,
                0.108817938273045,
                0.631766373528489,
                0.126499865329303,
                0.134303304313575,
                0.0985940927109977,
                0.142027248431928,
                0.168251298491528,
                0.196248922256955,
                0.317479775149435,
                0.316428999146291,
                0.217563309422821,
                0.251041846015736,
                0.892922405285977,
                0.703223224556291,
                0.555737942719387,
                0.184433667757653,
                0.212030842532321,
                0.0773468081126768,
                0.913800410779568,
                0.706715217696931,
                0.557788966754876,
                0.313428989936591,
                0.166203562902151,
                0.622497259279895
            },{
              0.987934734952495,
                0.170432023056883,
                0.257792250572013,
                0.396799318633144,
                0.0739947695769380,
                0.684096066962009,
                0.402388332696162,
                0.982835201393951,
                0.402183985222485,
                0.620671947199578,
                0.154369805479272,
                0.381345204444472,
                0.161133971849361,
                0.758112431327419,
                0.871111121915389,
                0.350776744885893,
                0.685535708747537,
                0.294148633767850,
                0.530629303856886,
                0.832423386285184,
                0.597490191872579,
                0.335311330705246,
                0.299225023333107,
                0.452592541569324,
                0.422645653220462,
                0.359606317972236,
                0.558319199869297,
                0.742545365701939,
                0.424334783625691,
                0.429355788576205,
                0.124872758719813,
                0.0244340160503740,
                0.290185265130727,
                0.317520582899226,
                0.653690133966475,
                0.956935924070684,
                0.935730872784880,
                0.457886333854367,
                0.240478396832085,
                0.763897944286478,
                0.759327383131096,
                0.740648064978614,
                0.743688341487326,
                0.105920416732765,
                0.681560430470316,
                0.463260578593719,
                0.212163205254934,
                0.0985187376881084,
                0.823574473927839,
                0.175009737382080,
                0.163569909784993,
                0.665987216411111,
                0.894389375354243,
                0.516558208351270,
                0.702702306950475,
                0.153590376619400,
                0.953457069886248,
                0.540884081241477,
                0.679733898210467,
                0.0365630180484529,
                0.809203851293793,
                0.748618871776197,
                0.120187017987081,
                0.525045164762609,
                0.325833628763249,
                0.546449439903069,
                0.398880752383199,
                0.415093386613047,
                0.180737760254794,
                0.255386740488051,
                0.0205357746581846,
                0.923675612620407,
                0.653699889008253,
                0.932613572048564,
                0.163512368527526,
                0.921097255892198,
                0.794657885388753,
                0.577394196706649,
                0.440035595760254,
                0.257613736712438,
                0.751946393867450,
                0.228669482105501,
                0.0641870873918986,
                0.767329510776574,
                0.671202185356536,
                0.715212514785840,
                0.642060828433852,
                0.419048293624883,
                0.390762082204175,
                0.816140102875323,
                0.317427863655850,
                0.814539772900651,
                0.789073514938958,
                0.852263890343846,
                0.505636617571756,
                0.635661388861377,
                0.950894415378135,
                0.443964155018810,
                0.0600188197794760,
                0.866749896999319,
            }};


            /*
            b_ux = 0.0;
            b_uy = 0.0;
            for (int j = 0; j < b_m; j++)
            {
              b_ux += sin(w[0][j] * b_t * omega_max);
              b_uy += sin(w[1][j] * b_t * omega_max);
            }
            b_ux = 1.0 * b_ux / b_m;
            b_uy = 1.0 * b_uy / b_m;

            int pe_zone = 1;

            if (b_x > 56.0 || b_x < -56.0) {
              //b_ux += 8.0 * ((b_x - (b_x > 0 ? 24.0 : -24.0))/64.0);
              pe_zone = 0;
            }
            if (b_y > 56.0 || b_y < -56.0) {
              //b_uy += 8.0 * ((b_y - (b_y > 0 ? 24.0 : -24.0))/64.0);
              pe_zone = 0;
            }
            if (b_x < -56.0 && b_y < -56.0) {
              //b_ux += -2.0;
              //b_uy += -2.0;
              pe_zone = 0;
            }
            if (b_x < -56.0 && b_y > 56.0) {
              //b_ux += -2.0;
              //b_uy += 2.0;
              pe_zone = 0;
            }
            if (b_x > 56.0 && b_y < -56.0) {
              //b_ux += 2.0;
              //b_uy += -2.0;
              pe_zone = 0;
            }
            if (b_x > 56.0 && b_y > 56.0) {
              //b_ux += 2.0;
              //b_uy += 2.0;
              pe_zone = 0;
            }
            //double randxy = ((double)(b_rd() % 100) - 50)/100.0;

            b_uxsig += b_ux;
            b_uysig += b_uy;
            */

            // Limit positions
            if (b_uxsig < esoft) { b_uxsig = esoft; }
            if (b_uxsig > wsoft) { b_uxsig = wsoft; }
            //if (b_uxsig < ehard) { b_uxsig = ehard; }
            //if (b_uxsig > whard) { b_uxsig = whard; }
            if (b_uysig < ssoft) { b_uysig = ssoft; }
            if (b_uysig > nsoft) { b_uysig = nsoft; }
            //if (b_uysig < shard) { b_uysig = shard; }
            //if (b_uysig > nhard) { b_uysig = nhard; }
            printf("cap: %f %f\n", b_uxsig, b_uysig);
            // Print to file
            // TODO: t, (optional dt), ball position (x,y), motor angles (x,y)
            //myfile << pe_zone << " " << b_t << " " << b_dt << " "
            //       << b_x << " " << b_y << " " << b_ux << " " << b_uy << " "
            //       << encoders[4] << " " << encoders[6] << " " << b_uxsig << " " << b_uysig << " " << (b_uxsig - off_x) << " " << (b_uysig - off_y) << endl;
            // send control out
            setpoints[0] = -55.0;
            setpoints[1] = 8.0;
            setpoints[2] = -20.0;
            setpoints[3] = 40.0;
            //setpoints[4] = off_x;
            setpoints[4] = b_uxsig;
            setpoints[5] = -5.0;
            //setpoints[6] = off_y;
            setpoints[6] = b_uysig;
            setpoints[7] = 0.0;
            setpoints[8] = 64.0;
            setpoints[9] = 26.0;
            setpoints[10] = 120.0;
            setpoints[11] = 73.0;
            setpoints[12] = 88.0;
            setpoints[13] = 90.0;
            setpoints[14] = 90.0;
            setpoints[15] = 145.0;
            ipos->positionMove(setpoints.data());
          }
          // update old values for next time
          b_xo = b_x;
          b_yo = b_y;
          b_dxo = b_dx;
          b_dyo = b_dy;
          b_uxo = b_ux;
          b_uyo = b_uy;
          b_uxfo = b_uxf;
          b_uyfo = b_uyf;

          // Write to output
          output.copyTo(om);

					if (strict) {
						portImgO->writeStrict();
					}
					else {
						portImgO->write();
          }

          //only write to these ports if they are actually being watched
          if (portImgLO->getOutputCount() > 0) {
            ImageOf<PixelBgr> &loImg = portImgLO->prepare();
            loImg.resize(*pImgL);
            Mat lm((IplImage *)loImg.getIplImage(), false);
            //cvtColor(hgrph,Scl,CV_GRAY2BGR);
            hgrph.copyTo(lm);
            portImgLO->write();
          }
          if (portImgRO->getOutputCount() > 0) {
            ImageOf<PixelBgr> &roImg = portImgRO->prepare();
            roImg.resize(*pImgR);
            Mat rm((IplImage *)roImg.getIplImage(), false);
            cvtColor(scr1,Scr,CV_HSV2BGR);
            Scr.copyTo(rm);
            portImgRO->write();
          }

				}

			}

		}

		virtual void threadRelease()
		{
		  if (clientGazeCtrl.isValid())
			  clientGazeCtrl.close();
      if (driverArmR.isValid())
        driverArmR.close();
      
			portHead->interrupt();
			portImgL->interrupt();
			portImgR->interrupt();
			portImgLO->interrupt();
			portImgRO->interrupt();
			portImgO->interrupt();

			portHead->close();
			portImgL->close();
			portImgR->close();
			portImgLO->close();
			portImgRO->close();
			portImgO->close();

			delete portHead, portImgL, portImgR, portImgLO, portImgRO, portImgO;
      delete eyeL, eyeR;
			delete rpcMutex;
		}

		virtual bool setParam(string pname, double pval) {
			bool success = true;

      if (pname == "k_p") {
        k_p = pval;
      }
      else if (pname == "k_d") {
        k_d = pval;
      }
      else if (pname == "k_i") {
        k_i = pval;
      }
      else if (pname == "w_b") {
        w_b = pval;
      }
      else if (pname == "h1_min") {
        threshold_H_min1 = pval;
      }
      else if (pname == "h1_max") {
        threshold_H_max1 = pval;
      }
      else if (pname == "h2_min") {
        threshold_H_min2 = pval;
      }
      else if (pname == "h2_max") {
        threshold_H_max2 = pval;
      }
      else if (pname == "s_min") {
        threshold_S_min = pval;
      }
      else if (pname == "s_max") {
        threshold_S_max = pval;
      }
      else if (pname == "v_min") {
        threshold_V_min = pval;
      }
      else if (pname == "v_max") {
        threshold_V_max = pval;
      }
      else if (pname == "w_max") {
        omega_max = 2 * PI * 30.0 / pval;
        if (omega_max > omega_max_threshold_upper) {
          omega_max = omega_max_threshold_upper;
        }
        if (omega_max < omega_max_threshold_lower) {
          omega_max = omega_max_threshold_lower;
        }
      }
      else if (pname == "m_num") {
        b_m = pval;
        if (b_m > b_m_threshold_upper) {
          b_m = b_m_threshold_upper;
        }
        if (b_m < b_m_threshold_lower) {
          b_m = b_m_threshold_lower;
        }
      }
			else {
				success = false;
			}

			return success;
		}

    virtual yarp::sig::Vector getParam(string pname) {
		  yarp::sig::Vector res;
      res.clear();

      if (pname == "offset") {
        printf("x(4): %f <%f to %f>, y(6): %f <%f to %f>\n", off_x, esoft, wsoft, off_y, ssoft, nsoft);
        res.push_back(off_x);
        res.push_back(esoft);
        res.push_back(wsoft);
        res.push_back(off_y);
        res.push_back(ssoft);
        res.push_back(nsoft);
      }
      else if (pname == "k_p") {
        res.push_back(k_p);
      }
      else if (pname == "k_d") {
        res.push_back(k_d);
      }
      else if (pname == "k_i") {
        res.push_back(k_i);
      }
      else if (pname == "w_b") {
        res.push_back(w_b);
      }
      else if (pname == "h1_min") {
        printf("h: %d\n", threshold_H_min1);
        res.push_back((double)threshold_H_min1);
      }
      else if (pname == "h1_max") {
        printf("h: %d\n", threshold_H_max1);
        res.push_back((double)threshold_H_max1);
      }
      else if (pname == "h2_min") {
        printf("h: %d\n", threshold_H_min2);
        res.push_back((double)threshold_H_min2);
      }
      else if (pname == "h2_max") {
        printf("h: %d\n", threshold_H_max2);
        res.push_back((double)threshold_H_max2);
      }
      else if (pname == "s_min") {
        printf("h: %d\n", threshold_S_min);
        res.push_back((double)threshold_S_min);
      }
      else if (pname == "s_max") {
        printf("h: %d\n", threshold_S_max);
        res.push_back((double)threshold_S_max);
      }
      else if (pname == "v_min") {
        printf("h: %d\n", threshold_V_min);
        res.push_back((double)threshold_V_min);
      }
      else if (pname == "v_max") {
        printf("h: %d\n", threshold_V_max);
        res.push_back((double)threshold_V_max);
      }
      else if (pname == "w_max") {
        printf("h: %f\n", omega_max);
        res.push_back((double)omega_max);
      }
      else if (pname == "m_num") {
        printf("h: %d\n", b_m);
        res.push_back((double)b_m);
      }
      else {
        res.push_back(-1);
      }

      return res;
    }

    virtual bool initJoint(string pname) {
      bool success = false;
      
      if (pname == "head" && !controlstarted) {
        IPositionControl *postmp;
        IVelocityControl *veltmp;
        IEncoders *encstmp;
        yarp::sig::Vector setpointstmp;
        int jntstmp;
        driverHead.view(postmp);
        driverHead.view(veltmp);
        driverHead.view(encstmp);
        if (postmp==NULL || veltmp==NULL || encstmp==NULL) {
          printf("Cannot get interface to robot head\n");
          //driverHead.close();
          return false;
        }

        //get encoders
        encstmp->getAxes(&jntstmp);
        if (jntstmp != HEAD_JNTS) {
          printf("Joint number mismatch for head\n");
          //driverHead.close();
          return false;
        }
        setpointstmp.resize(jntstmp);

        setpointstmp[0] = -22.0;
        setpointstmp[1] = -4.0;
        setpointstmp[2] = 0.0;
        setpointstmp[3] = -22.0;
        setpointstmp[4] = 12.0;
        setpointstmp[5] = 0.0;

        postmp->positionMove(setpointstmp.data());
        success = true;
      }
      else if (pname == "right_arm" && !controlstarted && !handclosed) {
        IPositionControl *postmp;
        IVelocityControl *veltmp;
        IEncoders *encstmp;
        yarp::sig::Vector setpointstmp;
        int jntstmp;
        driverArmR.view(postmp);
        driverArmR.view(veltmp);
        driverArmR.view(encstmp);
        if (postmp==NULL || veltmp==NULL || encstmp==NULL) {
          printf("Cannot get interface to robot right_arm\n");
          //driverArmR.close();
          return false;
        }

        //get encoders
        encstmp->getAxes(&jntstmp);
        if (jntstmp != ARMR_JNTS) {
          printf("Joint number mismatch for right_arm\n");
          //driverArmR.close();
          return false;
        }
        setpointstmp.resize(jntstmp);

        yarp::sig::Vector tmp;
        tmp.resize(jnts);
        for (int i = 0; i < jnts; i++) {
          tmp[i] = 10.0;
          postmp->setRefSpeed(i, tmp[i]);
        }
        postmp->setRefSpeeds(tmp.data());
        setpointstmp[0] = -55.0;
        setpointstmp[1] = 8.0;
        setpointstmp[2] = -20.0;
        setpointstmp[3] = 40.0;
        setpointstmp[4] = off_x;
        setpointstmp[5] = -5.0;
        setpointstmp[6] = off_y;
        setpointstmp[7] = 0.0;
        setpointstmp[8] = 64.0;
        setpointstmp[9] = 11.0;
        setpointstmp[10] = 0.0;
        setpointstmp[11] = 8.0;
        setpointstmp[12] = 11.0;
        setpointstmp[13] = 7.0;
        setpointstmp[14] = 0.0;
        setpointstmp[15] = 7.0;

        postmp->positionMove(setpointstmp.data());
        success = true;
      }
      else if (pname == "hand_close1" && !controlstarted && !handclosed) {
        IPositionControl *postmp;
        IVelocityControl *veltmp;
        IEncoders *encstmp;
        yarp::sig::Vector setpointstmp;
        int jntstmp;
        driverArmR.view(postmp);
        driverArmR.view(veltmp);
        driverArmR.view(encstmp);
        if (postmp==NULL || veltmp==NULL || encstmp==NULL) {
          printf("Cannot get interface to robot right_arm\n");
          //driverArmR.close();
          return false;
        }

        //get encoders
        encstmp->getAxes(&jntstmp);
        if (jntstmp != ARMR_JNTS) {
          printf("Joint number mismatch for right_arm\n");
          //driverArmR.close();
          return false;
        }
        setpointstmp.resize(jntstmp);
        
        // Close in several steps
        yarp::sig::Vector tmp;
        tmp.resize(jnts);
        for (int i = 0; i < jnts; i++) {
          tmp[i] = 10.0;
          postmp->setRefSpeed(i, tmp[i]);
        }
        postmp->setRefSpeeds(tmp.data());

        //setpoints
        setpointstmp[0] = -55.0;
        setpointstmp[1] = 8.0;
        setpointstmp[2] = -20.0;
        setpointstmp[3] = 40.0;
        setpointstmp[4] = off_x;
        setpointstmp[5] = -5.0;
        setpointstmp[6] = off_y;
        setpointstmp[7] = 0.0; //splay
        setpointstmp[8] = 64.0; //thumb
        setpointstmp[9] = 11.0;
        setpointstmp[10] = 0.0;
        setpointstmp[11] = 8.0;
        setpointstmp[12] = 11.0;
        setpointstmp[13] = 7.0;
        setpointstmp[14] = 0.0;
        setpointstmp[15] = 7.0;
        //move
        postmp->positionMove(setpointstmp.data());
        //delay
        yarp::os::Time::delay(1);
        //setpoints
        setpointstmp[9] = 26.0; //thumb proximal
        //move
        postmp->positionMove(setpointstmp.data());
        //delay
        yarp::os::Time::delay(1);
        //setpoints
        setpointstmp[10] = 120.0; //thumb distal
        //move
        postmp->positionMove(setpointstmp.data());
        success = true;
      }
      else if (pname == "hand_close2" && !controlstarted && !handclosed) {
        IPositionControl *postmp;
        IVelocityControl *veltmp;
        IEncoders *encstmp;
        yarp::sig::Vector setpointstmp;
        int jntstmp;
        driverArmR.view(postmp);
        driverArmR.view(veltmp);
        driverArmR.view(encstmp);
        if (postmp==NULL || veltmp==NULL || encstmp==NULL) {
          printf("Cannot get interface to robot right_arm\n");
          //driverArmR.close();
          return false;
        }

        //get encoders
        encstmp->getAxes(&jntstmp);
        if (jntstmp != ARMR_JNTS) {
          printf("Joint number mismatch for right_arm\n");
          //driverArmR.close();
          return false;
        }
        setpointstmp.resize(jntstmp);
        
        // Close in several steps
        yarp::sig::Vector tmp;
        tmp.resize(jnts);
        for (int i = 0; i < jnts; i++) {
          tmp[i] = 10.0;
          postmp->setRefSpeed(i, tmp[i]);
        }
        postmp->setRefSpeeds(tmp.data());

        //setpoints
        setpointstmp[0] = -55.0;
        setpointstmp[1] = 8.0;
        setpointstmp[2] = -20.0;
        setpointstmp[3] = 40.0;
        setpointstmp[4] = off_x;
        setpointstmp[5] = -5.0;
        setpointstmp[6] = off_y;
        setpointstmp[7] = 0.0; //splay
        setpointstmp[8] = 64.0; //thumb
        setpointstmp[9] = 26.0;
        setpointstmp[10] = 120.0;
        setpointstmp[11] = 8.0;
        setpointstmp[12] = 11.0;
        setpointstmp[13] = 7.0;
        setpointstmp[14] = 0.0;
        setpointstmp[15] = 7.0;
        //move
        postmp->positionMove(setpointstmp.data());
        //delay
        yarp::os::Time::delay(1);
        //setpoints
        setpointstmp[11] = 73.0; //index proximal
        setpointstmp[13] = 90.0; //middle proximal
        //move
        postmp->positionMove(setpointstmp.data());
        //delay
        yarp::os::Time::delay(1);
        //setpoints
        setpointstmp[12] = 88.0; //index distal
        setpointstmp[14] = 90.0; //middle distal
        //move
        postmp->positionMove(setpointstmp.data());
        success = true;
      }
      else if (pname == "hand_close3" && !controlstarted) {
        handclosed = true;
        IPositionControl *postmp;
        IVelocityControl *veltmp;
        IEncoders *encstmp;
        yarp::sig::Vector setpointstmp;
        int jntstmp;
        driverArmR.view(postmp);
        driverArmR.view(veltmp);
        driverArmR.view(encstmp);
        if (postmp==NULL || veltmp==NULL || encstmp==NULL) {
          printf("Cannot get interface to robot right_arm\n");
          //driverArmR.close();
          return false;
        }

        //get encoders
        encstmp->getAxes(&jntstmp);
        if (jntstmp != ARMR_JNTS) {
          printf("Joint number mismatch for right_arm\n");
          //driverArmR.close();
          return false;
        }
        setpointstmp.resize(jntstmp);
        
        // Close in several steps
        yarp::sig::Vector tmp;
        tmp.resize(jnts);
        for (int i = 0; i < jnts; i++) {
          tmp[i] = 10.0;
          postmp->setRefSpeed(i, tmp[i]);
        }
        postmp->setRefSpeeds(tmp.data());

        //setpoints
        setpointstmp[0] = -55.0;
        setpointstmp[1] = 8.0;
        setpointstmp[2] = -20.0;
        setpointstmp[3] = 40.0;
        setpointstmp[4] = off_x;
        setpointstmp[5] = -5.0;
        setpointstmp[6] = off_y;
        setpointstmp[7] = 0.0; //splay
        setpointstmp[8] = 64.0; //thumb
        setpointstmp[9] = 26.0;
        setpointstmp[10] = 120.0;
        setpointstmp[11] = 73.0;
        setpointstmp[12] = 88.0;
        setpointstmp[13] = 90.0;
        setpointstmp[14] = 90.0;
        setpointstmp[15] = 7.0;
        //move
        postmp->positionMove(setpointstmp.data());
        //delay
        yarp::os::Time::delay(1);
        //setpoints
        setpointstmp[15] = 145.0; //ring/pinky
        //move
        postmp->positionMove(setpointstmp.data());
        //delay
        yarp::os::Time::delay(1);
        success = true;
      }
      else if (pname == "offset" && !controlstarted) {
          driverArmR.view(iencs);
          if (iencs==NULL) {
            printf("Cannot get interface to robot right_arm\n");
            driverArmR.close();
            return false;
          }

          //get encoders
          jnts = 0;
          iencs->getAxes(&jnts);
          if (jnts != ARMR_JNTS) {
            printf("Joint number mismatch for right_arm\n");
            driverArmR.close();
            return false;
          }
          encoders.resize(jnts);
          iencs->getEncoders(encoders.data());
          // Set new offsets and limits
          off_x = encoders[4];
          off_y = encoders[6];
          if (off_x < ehard) { off_x = ehard; }
          if (off_x > whard) { off_x = whard; }
          if (off_y < shard) { off_y = shard; }
          if (off_y > nhard) { off_y = nhard; }
          // limits
          esoft = off_x - 5.0;
          wsoft = off_x + 5.0;
          ssoft = off_y - 5.0;
          nsoft = off_y + 5.0;
          if (esoft < ehard) { esoft = ehard; }
          if (wsoft > whard) { wsoft = whard; }
          if (ssoft < shard) { ssoft = shard; }
          if (nsoft > nhard) { nsoft = nhard; }
          success = true;
      }
      else if (pname == "hand_open" && !controlstarted) {
        handclosed = false;
        IPositionControl *postmp;
        IVelocityControl *veltmp;
        IEncoders *encstmp;
        yarp::sig::Vector setpointstmp;
        int jntstmp;
        driverArmR.view(postmp);
        driverArmR.view(veltmp);
        driverArmR.view(encstmp);
        if (postmp==NULL || veltmp==NULL || encstmp==NULL) {
          printf("Cannot get interface to robot right_arm\n");
          //driverArmR.close();
          return false;
        }

        //get encoders
        encstmp->getAxes(&jntstmp);
        if (jntstmp != ARMR_JNTS) {
          printf("Joint number mismatch for right_arm\n");
          //driverArmR.close();
          return false;
        }
        setpointstmp.resize(jntstmp);

        yarp::sig::Vector tmp;
        tmp.resize(jnts);
        for (int i = 0; i < jnts; i++) {
          tmp[i] = 10.0;
          postmp->setRefSpeed(i, tmp[i]);
        }
        postmp->setRefSpeeds(tmp.data());

        //setpoints
        setpointstmp[0] = -55.0;
        setpointstmp[1] = 8.0;
        setpointstmp[2] = -20.0;
        setpointstmp[3] = 28.0;
        setpointstmp[4] = off_x;
        setpointstmp[5] = -5.0;
        setpointstmp[6] = off_y;
        setpointstmp[7] = 0.0;
        setpointstmp[8] = 64.0;
        setpointstmp[9] = 11.0;
        setpointstmp[10] = 0.0;
        setpointstmp[11] = 8.0;
        setpointstmp[12] = 11.0;
        setpointstmp[13] = 7.0;
        setpointstmp[14] = 0.0;
        setpointstmp[15] = 7.0;
        //move
        postmp->positionMove(setpointstmp.data());
        //delay
        yarp::os::Time::delay(0.5);

        success = true;
      }
      else if (pname == "stop" && controlstarted) {
        controlstarted = false;
        //state
        b_xo = b_x;
        b_yo = b_y;
        b_dxo = b_dx;
        b_dyo = b_dy;
        //control
        b_xint = b_yint = 0.0;
        b_ux = b_uxo = 0.0;
        b_uy = b_uyo = 0.0;
        b_uxf = b_uxfo = 0.0;
        b_uyf = b_uyfo = 0.0;

        driverArmR.view(ipos);
        driverArmR.view(ivel);
        driverArmR.view(iencs);
        if (ipos==NULL || ivel==NULL || iencs==NULL) {
          printf("Cannot get interface to robot right_arm\n");
          driverArmR.close();
          return false;
        }

        //get encoders
        jnts = 0;
        iencs->getAxes(&jnts);
        if (jnts != ARMR_JNTS) {
          printf("Joint number mismatch for right_arm\n");
          driverArmR.close();
          return false;
        }
        encoders.resize(jnts);
        setpoints.resize(jnts);

        //reference speed/accel
        yarp::sig::Vector tmp;
        tmp.resize(jnts);
        for (int i = 0; i < jnts; i++) {
          tmp[i] = 0.0;
        }
        ipos->setRefAccelerations(tmp.data());
        for (int i = 0; i < jnts; i++) {
          tmp[i] = 10.0;
          ipos->setRefSpeed(i, tmp[i]);
        }
        ipos->setRefSpeeds(tmp.data());
        success = true;
      }
      else if (pname == "start" && !controlstarted && handclosed) {
        controlstarted = true;

        driverArmR.view(ipos);
        driverArmR.view(ivel);
        driverArmR.view(iencs);
        if (ipos==NULL || ivel==NULL || iencs==NULL) {
          printf("Cannot get interface to robot right_arm\n");
          driverArmR.close();
          return false;
        }

        //get encoders
        jnts = 0;
        iencs->getAxes(&jnts);
        if (jnts != ARMR_JNTS) {
          printf("Joint number mismatch for right_arm\n");
          driverArmR.close();
          return false;
        }
        encoders.resize(jnts);
        setpoints.resize(jnts);

        //reference speed/accel
        yarp::sig::Vector tmp;
        tmp.resize(jnts);
        for (int i = 0; i < jnts; i++) {
          tmp[i] = 0.0;
        }
        ipos->setRefAccelerations(tmp.data());
        for (int i = 0; i < jnts; i++) {
          tmp[i] = 30.0;
          ipos->setRefSpeed(i, tmp[i]);
        }
        ipos->setRefSpeeds(tmp.data());
        success = true;
      }
      else {
        success = false;
      }

      return success;
    }
    
};

class mazeModule: public RFModule
{
	protected:

		mazeThread *thr;
		RpcServer * rpcPort;
		string name;

	public:

		mazeModule() { }

		bool respond(const Bottle& command, Bottle& reply) {
			// Process Commands
			string msg(command.get(0).asString().c_str());
			if (msg == "loc") {
				// Return location of ball?
				if (command.size() == 3) {
					//yarp::sig::Vector mloc;
					//int u = command.get(1).asInt();
					//int v = command.get(2).asInt();
					//mloc = thr->getLocation(arguments?);

					//reply.add(mloc[0]);
					//reply.add(mloc[1]);
					//reply.add(mloc[2]);
					reply.add(-1);

				}
				else {
					reply.add(-1);
				}
			}
			else if (msg == "set") {
				// Param name and value needed
				if (command.size() < 3) {
					reply.add(-1);
				}
				else {
					string param(command.get(1).asString().c_str());
					double pval = command.get(2).asDouble(); // better as float?
					reply.add(thr->setParam(param,pval));
				}
			}
			else if (msg == "get") {
				// Param name and value needed
				if (command.size() < 2) {
					reply.add(-1);
				}
				else {
					string param(command.get(1).asString().c_str());
          yarp::sig::Vector res;
          res = thr->getParam(param);
          for (int i = 0; i < res.size(); i++) {
					  reply.add(res[i]);
          }
				}
			}
      else if (msg == "init") {
				if (command.size() < 2) {
					reply.add(-1);
				}
				else {
					string param(command.get(1).asString().c_str());
					reply.add(thr->initJoint(param));
				}
      }
      else if (msg == "stop") {
        string param = "stop";
        reply.add(thr->initJoint(param));
      }
      else if (msg == "start") {
        string param = "start";
        reply.add(thr->initJoint(param));
      }
			else {
				reply.add(-1);
			}

			return true;
		}

		virtual bool configure(ResourceFinder &rf)
		{

			//set up the rpc port
			name=rf.check("name",Value("maze")).asString().c_str();
			rpcPort = new RpcServer;
			string portRpcName="/"+name+"/rpc";
			rpcPort->open(portRpcName.c_str());
			attach(*rpcPort);


			thr=new mazeThread(rf);
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

	mazeModule mod;

	return mod.runModule(rf);
}




