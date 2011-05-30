/*
 *  jointAttentionMap.cpp
 *
 * 	Logan Niehaus
 * 	5/28/11
 * 	joint attentional saliency mapper (learner + evaluator).
 * 	this module is built off of the old gazeEval, now incorporates online learning
 * 	of the map, as well as evaluation of the map given inputs
 *
 * inputs: 	receives gaze estimate from gaze estimation module
 * 			checks for location of detected objects (and either learns or evaluates the previous,
 * 				based on if it is there or not)
 * 			rpc service to change runtime parameters
 *
 * params: 	<param name> - <desc> (Req/Def <val>/Opt: <example value>)
 * 			mr - input neuron center ranges (R: "mr m0_low m0_high m1_low m1_high")
 * 			nr - output neuron center ranges (R: same as above)
 * 			m, n - number of input/output neurons (R: "m m0 m1..." or "n n0 n1")
 * 			eta - neural network learning rate (hebb rule) (R: "eta 0.1")
 * 			leak - neural network leakage rate (R: "leak 0.9999")
 * 			sm, sn - gaussian RBF variances, only diagonal cov. used (R: "sm sm0 sm1")
 * 			name - module base name, ports take the form "/name/port:x" (D: "jointAttention")
 * 			w, h - output map dimensions (D: 320x240)
 * 			tol - tolerance on nonmax supression (0 chooses only max, 1 allows everything) (D: 0.5)
 * 			alpha - zero location for temporal exponential filtering (O)
 * 			weights, inputs, outputs - data files containing previously trained parameters, csv format (O)
 *
 * outputs: rbg (should be float) salience map normalized to have inf_norm of 255
 * 			upon request should also produce the neuron weights, params, etc...
 *
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

#define PI 3.14159

class jointAttentionThread : public RateThread
{
protected:

	//names
	ResourceFinder &rf;
	string name;

	//data ports
	BufferedPort<ImageOf<PixelFloat> > *portMapOut;
	BufferedPort<yarp::sig::Vector>	 *portFeatVec;
	BufferedPort<yarp::sig::Vector>	 *portHeadVec;
	BufferedPort<ImageOf<PixelRgb> > *portImgOut;

	//running params
	int m0, m1, n0, n1;
	yarp::sig::Vector mr;
	yarp::sig::Vector nr;
	double eta, leak;
	int ih,iw;
	bool filtering;
	double alpha;
	Matrix Sm, Sn, iSm, iSn;
	double tolerance;

	//runtime data/map parameters
	Matrix W,M,N;
	yarp::sig::Vector nmlzr;
	double sqDetSm;
	double sqDetSn;
	ImageOf<PixelFloat> * oMap;
	bool initialized;

	//optional loading of trained network data
	string weightsfile;
	string inputsfile;
	string outputsfile;
	ImageOf<PixelFloat> * Wimg;
	ImageOf<PixelFloat> * Isimg;
	ImageOf<PixelFloat> * Osimg;


public:

	jointAttentionThread(ResourceFinder &_rf) : RateThread(50), rf(_rf), initialized(false)
	{ }

	Matrix getW() { return W; }

	virtual bool loadTrainingData(string weightsfile, string inputsfile, string outputsfile) {

		//load neural net data
		Wimg = new ImageOf<PixelFloat>;
		if (file::read(*Wimg, weightsfile.c_str())) {
			W.resize(Wimg->height(),Wimg->width());
			for (int i = 0; i < Wimg->width(); i++) {
				for (int j = 0; j < Wimg->height(); j++) {
					W(j,i) = Wimg->pixel(i,j);
				}
			}
		} else {
			fprintf(stderr,"could not read weight file, skipping...\n");
			return false;
		}

		Isimg = new ImageOf<PixelFloat>;
		if (file::read(*Isimg, inputsfile.c_str())) {
			M.resize(Isimg->height(),Isimg->width());
			for (int i = 0; i < Isimg->width(); i++) {
				for (int j = 0; j < Isimg->height(); j++) {
					M(j,i) = Isimg->pixel(i,j);
				}
			}
		} else {
			fprintf(stderr,"could not read inputs file, skipping...\n");
			return false;
		}

		Osimg = new ImageOf<PixelFloat>;
		if (file::read(*Osimg, outputsfile.c_str())) {
			N.resize(Osimg->height(),Osimg->width());
			nmlzr.resize(Osimg->height());
			for (int i = 0; i < Osimg->width(); i++) {
				for (int j = 0; j < Osimg->height(); j++) {
					N(j,i) = Osimg->pixel(i,j);
					nmlzr(j) = 1;
				}
			}
		} else {
			fprintf(stderr,"could not read outputs file, skipping...\n");
			return false;
		}

		//get all of the other necessary parameters based on the provided data (assuming a particular structure)


		return true;

	}


	virtual bool handleParams()
	{

		//first check if previously trained data is given: if it is, use it
		if (rf.check("weights") && rf.check("inputs") && rf.check("outputs")) {

			weightsfile = rf.find("weights").asString().c_str();
			inputsfile = rf.find("inputs").asString().c_str();
			outputsfile = rf.find("outputs").asString().c_str();

			if (loadTrainingData(weightsfile,inputsfile,outputsfile)) {
				fprintf(stdout,"provided training data loaded successfully, proceeding...\n");
				initialized = true;
				return true;
			} else {
				fprintf(stdout,"tried to load provided training data but failed, continuing as normal...\n");
			}

		}

		//start by parsing all the input parameters, fail if necessary params not met
		Bottle mneurons = rf.findGroup("m");
		Bottle nneurons = rf.findGroup("n");
		Bottle mrange = rf.findGroup("mr");
		Bottle nrange = rf.findGroup("nr");
		Bottle sigm = rf.findGroup("sm");
		Bottle sign = rf.findGroup("sn");
		if (mneurons.isNull() || nneurons.isNull() || mneurons.size() != 3 || nneurons.size() != 3) {
			fprintf(stderr,"number of neurons not properly set (sizes %d, %d), exiting...\n", mneurons.size(), nneurons.size());
			return false;
		} else {
			m0 = mneurons.get(1).asInt();
			m1 = mneurons.get(2).asInt();
			n0 = nneurons.get(1).asInt();
			n1 = nneurons.get(2).asInt();
		}
		if (mrange.isNull() || nrange.isNull() || mrange.size() != 5 || nrange.size() != 5) {
			fprintf(stderr,"neuron center ranges not properly set, exiting...\n");
			return false;
		} else {
			for (int i = 1; i < 5; i++) {
				mr.push_back(mrange.get(i).asDouble());
				nr.push_back(nrange.get(i).asDouble());
			}
		}
		if (sigm.isNull() || sign.isNull() || sigm.size() != 3 || sign.size() != 3) {
			fprintf(stderr,"number of neurons not properly set, exiting...\n");
			return false;
		} else {
			Sm.resize(2,2);
			Sn.resize(2,2);
			Sm.zero();
			Sn.zero();
			Sm(0,0) = sigm.get(1).asDouble();
			Sm(1,1) = sigm.get(2).asDouble();
			Sn(0,0) = sign.get(1).asDouble();
			Sn(1,1) = sign.get(2).asDouble();
		}
		if (!rf.check("eta")) {
			fprintf(stderr,"no learning rate (eta) set, unable to proceed\n");
			return false;
		} else {
			eta = rf.find("eta").asDouble();
		}
		if (!rf.check("leak")) {
			fprintf(stderr,"no leakage rate (leak) set, unable to proceed\n");
			return false;
		} else {
			leak = rf.find("leak").asDouble();
		}

		//optional params
		name=rf.check("name",Value("jointAttention")).asString().c_str();
		iw = rf.check("w",Value(320)).asInt();
		ih = rf.check("h",Value(240)).asInt();
		tolerance = rf.check("tol",Value(0.5)).asDouble();
		filtering = rf.check("alpha");
		if (filtering) {
			alpha = rf.find("alpha").asDouble();
			printf("filtering is ON, with alpha = %f\n",alpha);
		} else {
			printf("filtering is OFF\n");
		}

		return true;

	}

	virtual bool threadInit()
	{

		//load all of the input parameters
		if (!handleParams()) {
			return false;
		}

		//initialize ports
		portFeatVec=new BufferedPort<yarp::sig::Vector>;
		string portFeatName="/"+name+"/loc:i";
		portFeatVec->open(portFeatName.c_str());

		portHeadVec=new BufferedPort<yarp::sig::Vector>;
		string portHeadName="/"+name+"/gaze:i";
		portHeadVec->open(portHeadName.c_str());

		portMapOut=new BufferedPort<ImageOf<PixelFloat> >;
		string portMapName="/"+name+"/map:o";
		portMapOut->open(portMapName.c_str());

		portImgOut=new BufferedPort<ImageOf<PixelRgb> >;
		string portOutName="/"+name+"/view";
		portImgOut->open(portOutName.c_str());

		//initialize all data structures (if not pre-loaded)
		if (!initialized) {

			//init weight matrix to zero
			W.resize(n0*n1,m0*m1);
			W.zero();

			//set input neuron center locations
			double tmp;
			M.resize(m0*m1,2);
			for (int i = 0; i < m0; i++) {
				tmp = i*((mr[1]-mr[0])/(double)(m0-1))+mr[0];
				for (int j = 0; j < m1; j++) {
					M(i*m1+j,0) = tmp;
					M(i*m1+j,1) = j*((mr[3]-mr[2])/(double)(m1-1))+mr[2];
				}
			}

			//set output neuron center locations
			N.resize(n0*n1,2);
			nmlzr.resize(n0*n1);
			for (int i = 0; i < n0; i++) {
				tmp = i*((nr[1]-nr[0])/(double)(n0-1))+nr[0];
				for (int j = 0; j < n1; j++) {
					N(i*n1+j,0) = tmp;
					N(i*n1+j,1) = j*((nr[3]-nr[2])/(double)(n1-1))+nr[2];
					nmlzr(i*n1+j) = 1.0;
				}
			}

			initialized = true;

		}

		//pre-calculate some things
		iSm = luinv(Sm);
		iSn = luinv(Sn);
		sqDetSm = sqrt(det(Sm));
		sqDetSn = sqrt(det(Sn));

		oMap = new ImageOf<PixelFloat>;
		oMap->resize(iw,ih);
		oMap->zero();

		return true;

	}

	yarp::sig::Vector forwardActivation(const yarp::sig::Vector &in) {

		yarp::sig::Vector Ap(0);

		//get activations
		Ap.resize(M.rows());
		yarp::sig::Vector mc(2);
		for (int i = 0; i < M.rows(); i++) {
			mc = in-M.getRow(i);
			Ap[i] = (1/(sqDetSm*2*PI))*exp(-0.5*dot(mc,(iSm*mc)));
		}

		return Ap;

	}

	yarp::sig::Vector backwardActivation(const yarp::sig::Vector &fb) {

		yarp::sig::Vector Ax(0);

		//get activations
		Ax.resize(N.rows());
		yarp::sig::Vector mc(2);
		for (int i = 0; i < M.rows(); i++) {
			mc = fb-N.getRow(i);
			Ax[i] = (1/(sqDetSn*2*PI))*exp(-0.5*dot(mc,(iSn*mc)));
		}

		return Ax;

	}

	void evaluateMap(const yarp::sig::Vector &in, ImageOf<PixelFloat> &map) {

		yarp::sig::Vector Ap;
		yarp::sig::Vector Ax;
		yarp::sig::Vector xy(2);
		yarp::sig::Vector xym(2);
		double nconst = 1;
		double maxresp, pmaxv;
		int amaxr;

		map.resize(iw,ih);
		map.zero();

		//get input activations
		Ap = forwardActivation(in);

		//evaluate feedforward activation (normalize)
		Ax.resize(N.rows());
		Ax = W*Ap;
		nconst = 1.0/dot(Ax,nmlzr);
		Ax = nconst*Ax;
		pmaxv = 0.0;

		//use this to create a saliency map output
		ImageOf<PixelFloat> * Xr = new ImageOf<PixelFloat>;
		Xr->resize(n0,n1);
		Xr->zero();
		for (int i = 0; i < n0; i++) {
			for (int j = 0; j < n1; j++) {
				Xr->pixel(i,j) = (255.0)*10*Ax[i*n1+j];
				if (Xr->pixel(i,j) > pmaxv) {
					pmaxv = Xr->pixel(i,j);
					xy(0) = i;
					xy(1) = j;
				}
			}
		}

		//set up image for opencv to do this
		Mat * IM = new Mat(map.height(), map.width(), CV_32F, (void *)map.getRawImage());
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
			if (pointPolygonTest(Mat(contours[i]), Point2f(xy(0),xy(1)), false) >= 0) {
				amaxr = i;
			}
		}

		//make a quick and dirty estimate of the gaussian that fits this blob
		Xr->zero();
		vector<Point> nimp(0);
		drawContours(*C, contours, amaxr, Scalar(1.0), -1, 8, hierarchy, 0);
		int npx = 0;
		for (int i = 0; i < n0; i++) {
			for (int j = 0; j < n1; j++) {
				if (Xr->pixel(i,j) > 0) {
					xym(0) = N[i*n1+j][0];
					xym(1) = N[i*n1+j][1];
					if (map.isPixel((int)xym(0)-1,(int)xym(1)-1)) {
						nimp.push_back(Point((int)xym(0)-1,(int)xym(1)-1));
					}
				}
			}
		}

		//fit an ellipse around the salient area and filter
		if (nimp.size() >= 6) {
			RotatedRect GP = fitEllipse(Mat(nimp));
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
			for (int i = 0; i < map.width(); i++) {
				for (int j = 0; j < map.height(); j++) {
					map.pixel(i,j) = alpha*map.pixel(i,j) + (1-alpha)*oMap->pixel(i,j);
				}
			}
			oMap->copy(map);
		}

		delete T;
		delete C;
		delete IM;
		delete Xr;

	}

	void trainMap(const yarp::sig::Vector &in, const yarp::sig::Vector &fb, ImageOf<PixelFloat> &map) {

		yarp::sig::Vector Ap;
		yarp::sig::Vector Ax;
		Matrix Apm, Axm;
		Matrix Wh;
		double wsum = 0.0;

		//get forward activation
		Ap = forwardActivation(in);
		Apm.resize(1,Ap.size());
		Apm.setRow(0, Ap);

		//get feedback activation
		Ax = backwardActivation(fb);
		Axm.resize(Ax.size(),1);
		Axm.setCol(0, Ax);

		//take outer product of forward and backward activations, normalize
		Wh = Axm*Apm;
		for (int i = 0; i < Wh.rows(); i++) {
			for (int j = 0; j < Wh.cols(); j++) {
				wsum += Wh(i,j);
			}
		}
		Wh = (1.0/wsum)*Wh;

		//update weight matrix with hebb rule
		W = eta*Wh + leak*W;

		//get the map for the updated neuron weights
		evaluateMap(in, map);

	}

	virtual void run()
	{

		//check for feature vector from head module
		yarp::sig::Vector *headVec = portHeadVec->read(false);
		yarp::sig::Vector *featVec = NULL;

		//if an input space estimate was received
		if (headVec)
		{

			ImageOf<PixelFloat> * sMap = new ImageOf<PixelFloat>;

			//check to see if a corresponding output space sample was also generated
			featVec = portFeatVec->read(false);

			//if one was, perform a training iteration
			if (featVec) {

				trainMap(*headVec, *featVec, *sMap);
				printf("received feed forward and feedback data, training\n");

			}

			//if only feed-forward, just evaluate map
			else {

				evaluateMap(*headVec, *sMap);

			}

			//write out the salience map and viewable map (if needed)
			ImageOf<PixelFloat> &mapOut= portMapOut->prepare();
			mapOut.copy(*sMap);
			portMapOut->write();
			if (portImgOut->getOutputCount() > 0) {
				ImageOf<PixelRgb> &imgOut= portImgOut->prepare();
				imgOut.copy(*sMap);
				portImgOut->write();
			}

		}

	}



	virtual void threadRelease()
	{

		portFeatVec->interrupt();
		portHeadVec->interrupt();
		portMapOut->interrupt();
		portImgOut->interrupt();

		delete portFeatVec;
		delete portHeadVec;
		delete portMapOut;
		delete portImgOut;

	}

};

class jointAttentionModule: public RFModule
{
protected:

	jointAttentionThread *thr;
	Port * rpcPort;
	string name;

public:

	jointAttentionModule() { }

	bool respond(const Bottle& command, Bottle& reply) {

		//handle information requests
		string msg(command.get(0).asString().c_str());

		//parameter gets
		if (msg == "get") {
			if (command.size() < 2) {
				reply.add(-1);
			}
			else {
				string arg(command.get(1).asString().c_str());
				if (arg == "W") {
					Matrix W = thr->getW();
					for (int i = 0; i < W.rows(); i++) {
						for (int j = 0; j < W.cols(); j++) {
							reply.add(W(i,j));
						}
					}
				}
			}
		}

		return true;

	}

	virtual bool configure(ResourceFinder &rf)
	{
		Time::turboBoost();

		//set up the rpc port
		name=rf.check("name",Value("jointAttention")).asString().c_str();
		rpcPort = new Port;
		string portRpcName="/"+name+"/rpc";
		rpcPort->open(portRpcName.c_str());
		attach(*rpcPort);

		thr=new jointAttentionThread(rf);
		if (!thr->start())
		{
			delete thr;
			return false;
		}

		return true;
	}

	virtual bool close()
	{
		rpcPort->close();
		thr->stop();

		delete rpcPort;
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

	jointAttentionModule mod;

	return mod.runModule(rf);
}



