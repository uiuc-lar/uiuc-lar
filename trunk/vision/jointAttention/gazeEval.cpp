/*
 *  gazeEval.cpp
 *
 * 	Logan Niehaus
 * 	5/2/11
 * 	joint attentional saliency mapper
 *
 * inputs: should receive gaze features from gaze estimation module
 * 			also takes in the current icub camera image (but why?)
 * params: tab delimted files for the neuron locations and weights
 * 			also needs some covariance matrices for blurring
 * 			relative sizes of the output space must also be defined unfortunately
 * 			tolerance on nonmax supression (0 chooses only max, 1 allows everything) default 0.5
 * 			takes alpha for temporal exponential filtering (optional)
 * outputs: rbg (should be float) salience map normalized to have inf_norm of 255
 * TODOs: Yarpify this much more, break some things out into classes, fix N1,N2 garbage
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
#include <iCub/vis/RuddySalience.h>
#include <iCub/vis/BlobResult.h>

#include <cv.h>

#include <string>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <deque>
#include <queue>

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::vis;

#define PI 3.14159
#define N1 31
#define N2 21

class gazeEvalThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;
	string weightsfile;
	string inputsfile;
	string outputsfile;

	ImageOf<PixelFloat> * Wimg;
	ImageOf<PixelFloat> * Isimg;
	ImageOf<PixelFloat> * Osimg;
	yarp::sig::Vector nmlzr;
	Matrix W;
	Matrix M;
	Matrix N;
	Matrix iSx;
	Matrix iSp;
	double sqDetSp;
	double sqDetSx;
	double tolerance;

	int ih,iw;

	bool filtering;
	double alpha;
	ImageOf<PixelFloat> * oMap;

	BufferedPort<ImageOf<PixelRgb> > *portImgIn;
	BufferedPort<ImageOf<PixelRgb> > *portImgOut;
	BufferedPort<ImageOf<PixelFloat> > *portMapOut;
	BufferedPort<yarp::sig::Vector>	 *portFeatVec;
	BufferedPort<yarp::sig::Vector>	 *portHeadVec;


public:

	gazeEvalThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	virtual bool threadInit()
	{

		name=rf.check("name",Value("myGazeEval")).asString().c_str();
		weightsfile = rf.check("weights",Value("./weights.dat")).asString().c_str();
		inputsfile = rf.check("inputspace",Value("./inputs.dat")).asString().c_str();
		outputsfile = rf.check("outputspace",Value("./outputs.dat")).asString().c_str();
		iw = rf.check("iw",Value(320)).asInt();
		ih = rf.check("ih",Value(240)).asInt();
		tolerance = rf.check("tol",Value(0.5)).asDouble();
		filtering = rf.check("alpha");
		if (filtering) {
			alpha = rf.find("alpha").asDouble();
			printf("filtering is ON, with alpha = %f\n",alpha);
		} else {
			printf("filtering is OFF\n");
		}

		portImgIn=new BufferedPort<ImageOf<PixelRgb> >;
		string portInName="/"+name+"/img:i";
		portImgIn->open(portInName.c_str());

		portImgOut=new BufferedPort<ImageOf<PixelRgb> >;
		string portOutName="/"+name+"/img:o";
		portImgOut->open(portOutName.c_str());

		portFeatVec=new BufferedPort<yarp::sig::Vector>;
		string portFeatName="/"+name+"/feat:i";
		portFeatVec->open(portFeatName.c_str());

		portHeadVec=new BufferedPort<yarp::sig::Vector>;
		string portHeadName="/"+name+"/head:i";
		portHeadVec->open(portHeadName.c_str());

		portMapOut=new BufferedPort<ImageOf<PixelFloat> >;
		string portMapName="/"+name+"/map:o";
		portMapOut->open(portMapName.c_str());


		//load neural net data
		Wimg = new ImageOf<PixelFloat>;
		file::read(*Wimg, weightsfile.c_str());
		W.resize(Wimg->height(),Wimg->width());
		for (int i = 0; i < Wimg->width(); i++) {
			for (int j = 0; j < Wimg->height(); j++) {
				W(j,i) = Wimg->pixel(i,j);
			}
		}

		Isimg = new ImageOf<PixelFloat>;
		file::read(*Isimg, inputsfile.c_str());
		M.resize(Isimg->height(),Isimg->width());
		for (int i = 0; i < Isimg->width(); i++) {
			for (int j = 0; j < Isimg->height(); j++) {
				M(j,i) = Isimg->pixel(i,j);
			}
		}

		Osimg = new ImageOf<PixelFloat>;
		file::read(*Osimg, outputsfile.c_str());
		N.resize(Osimg->height(),Osimg->width());
		nmlzr.resize(Osimg->height());
		for (int i = 0; i < Osimg->width(); i++) {
			for (int j = 0; j < Osimg->height(); j++) {
				N(j,i) = Osimg->pixel(i,j);
				nmlzr(j) = 1;
			}
		}

		//precision matrices hardcoded atm
		iSp.resize(2,2);
		iSx.resize(2,2);
		iSp.zero();
		iSp(0,0) = 20;
		iSp(1,1) = 0.1;
		iSx(0,0) = 0.0033;
		iSx(1,1) = 0.005;
		sqDetSp = sqrt(1/(iSp(0,0)*iSp(1,1)));
		sqDetSx = sqrt(1/(iSx(0,0)*iSx(1,1)));

		oMap = NULL;

		return true;
	}

	virtual void run()
	{

		yarp::sig::Vector Ap;
		yarp::sig::Vector Ax;
		yarp::sig::Vector xy(2);
		yarp::sig::Vector xym(2);
		double nconst = 1;
		double maxresp, pmaxv;
		int amaxr;

		//check for feature vector from head module
		yarp::sig::Vector *featVec = portFeatVec->read(false);


		// process head position
		if (featVec)
		{


			ImageOf<PixelFloat> &mapOut= portMapOut->prepare();
			ImageOf<PixelFloat> * sMap = new ImageOf<PixelFloat>;
			sMap->resize(iw,ih);
			sMap->zero();

			//get activations
			Ap.resize(M.rows());
			yarp::sig::Vector mc(2);
			for (int i = 0; i < M.rows(); i++) {

				mc = (*featVec)-M.getRow(i);
				Ap[i] = (1/(sqDetSp*2*PI))*exp(-0.5*dot(mc,(iSp*mc)));

			}

			//evaluate
			Ax.resize(N.rows());
			Ax = W*Ap;
			nconst = 1.0/dot(Ax,nmlzr);
			Ax = nconst*Ax;
			pmaxv = 0.0;


			//use this to create a saliency map output
			ImageOf<PixelFloat> * Xr = new ImageOf<PixelFloat>;
			Xr->resize(N1,N2);
			Xr->zero();
			for (int i = 0; i < N1; i++) {
				for (int j = 0; j < N2; j++) {
					Xr->pixel(i,j) = (255.0)*10*Ax[i*N2+j];
					if (Xr->pixel(i,j) > pmaxv) {
						pmaxv = Xr->pixel(i,j);
						xy(0) = i;
						xy(1) = j;
					}
				}
			}


			//set up image for opencv to do this
			Mat * IM = new Mat(sMap->height(), sMap->width(), CV_32F, (void *)sMap->getRawImage());
			Mat * C = new Mat(Xr->height(), Xr->width(), CV_32F, (void *)Xr->getRawImage());
			Mat * T = new Mat(Xr->height(), Xr->width(), CV_32F);
			threshold(*C, *T, (1-tolerance)*pmaxv, 255.0, CV_THRESH_BINARY);

			//find the blob with the maximum in it
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			Mat X;
			T->convertTo(X,CV_8UC1);
			findContours(X, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
			amaxr = 0;
			for (int i = 0; i < contours.size(); i++) {
				if (pointPolygonTest(contours[i], Point2f(xy(0),xy(1)), false) >= 0) {
					amaxr = i;
				}
			}

			//make a quick and dirty estimate of the gaussian that fits this blob
			Xr->zero();
			vector<Point> nimp(0);
			drawContours(*C, contours, amaxr, Scalar(1.0), -1, 8, hierarchy, 0);
			int npx = 0;
			for (int i = 0; i < N1; i++) {
				for (int j = 0; j < N2; j++) {
					if (Xr->pixel(i,j) > 0) {
						xym(0) = N[i*N2+j][0];
						xym(1) = N[i*N2+j][1];
						if (sMap->isPixel((int)xym(0)-1,(int)xym(1)-1)) {
							nimp.push_back(Point((int)xym(0)-1,(int)xym(1)-1));
						}
					}
				}
			}

			//fit an ellipse around the salient area and filter
			if (nimp.size() >= 6) {
				RotatedRect GP = fitEllipse(nimp);
				if (GP.size.width < 500.0 && GP.size.height < 500.0 &&
						GP.size.width > 0.0 && GP.size.height > 0.0) {
					ellipse(*IM, GP, Scalar(100.0), -1, 8);
					GaussianBlur(*IM, *IM, Size(55,55), 20.0, 20.0);
					normalize(*IM, *IM, 255.0, 0, NORM_INF);
				} else {
					printf("Tried to draw ellipse with bad axes: %f %f\n",GP.size.width,GP.size.height);
				}
			}

			//do some temporal filtering if desired
			if (filtering) {
				//check if its the first time
				if (oMap == NULL) {
					oMap = sMap;
				} else {
					for (int i = 0; i < sMap->width(); i++) {
						for (int j = 0; j < sMap->height(); j++) {
							sMap->pixel(i,j) = alpha*sMap->pixel(i,j) + (1-alpha)*oMap->pixel(i,j);
						}
					}
					delete oMap;
					oMap = sMap;
				}
			}

			//check for a camera image
			ImageOf<PixelRgb> *pImgIn=portImgIn->read(false);
			if (pImgIn) {

				ImageOf<PixelRgb> &imgOut= portImgOut->prepare();
				//imgOut.copy(*pImgIn);
				//draw::addCrossHair(imgOut, PixelRgb(0,255,0), xy(0), xy(1), 10);

				imgOut.copy(*sMap);
				portImgOut->write();

			}

			//write out the map
			mapOut.copy(*sMap);
			portMapOut->write();


			delete T;
			delete C;
			delete IM;
			delete Xr;


		}
	}

	virtual void threadRelease()
	{

		portImgIn->interrupt();
		portImgOut->interrupt();
		portFeatVec->interrupt();
		portHeadVec->interrupt();
		portMapOut->interrupt();
		delete portImgIn;
		delete portImgOut;
		delete portFeatVec;
		delete portHeadVec;
		delete portMapOut;

	}

};

class gazeEvalModule: public RFModule
{
protected:
	gazeEvalThread *thr;

public:
	gazeEvalModule() { }

	virtual bool configure(ResourceFinder &rf)
	{
		Time::turboBoost();

		thr=new gazeEvalThread(rf);
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

	gazeEvalModule mod;

	return mod.runModule(rf);
}



