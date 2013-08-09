/*
 * Copyright (C) 2011 Logan Niehaus
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
 * 			rpc service to interact at runtime (not fully functional). neuron
 * 				weights, input centers, and output centers can be saved to a file
 * 				by issuing the command "save <X> filename.tab" over the RPC port, where
 * 				<X> is either W, M, or N
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
 * 			weights, inputs, outputs - data files containing previously trained parameters, tab format (O)
 *
 * outputs: rbg and PixelFloat salience maps normalized to have inf_norm of 255
 * 			upon request should also produce the neuron weights, params, etc...
 *
 * TODOs:   improve functionality for rpc interface,
 * 			more configuration options for rbfnn output post-processing.
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

#include "RBFMap.h"

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

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
	yarp::sig::Vector m;
	yarp::sig::Vector n;
	yarp::sig::Vector mr;
	yarp::sig::Vector nr;
	yarp::sig::Vector sm;
	yarp::sig::Vector sn;
	double eta, leak;
	Matrix Sm, Sn, iSm, iSn;
	Rect bb;

	int ih,iw;
	bool filtering;
	double alpha;
	double tolerance;

	//runtime data/map parameters
	RBFMap * R;
	ImageOf<PixelFloat> * oMap;

	//optional loading of trained network data
	string weightsfile;
	string inputsfile;
	string outputsfile;


public:

	jointAttentionThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{

		R = new RBFMap();

	}

	Matrix getW() { return R->getW(); }
	Matrix getM() { return R->getM(); }
	Matrix getN() { return R->getN(); }

	virtual bool handleParams()
	{

		//first check if previously trained data is given: if it is, use it
		if (rf.check("weights") && rf.check("inputs") && rf.check("outputs")) {

			weightsfile = rf.find("weights").asString().c_str();
			inputsfile = rf.find("inputs").asString().c_str();
			outputsfile = rf.find("outputs").asString().c_str();

			if (R->initFromFiles(weightsfile,inputsfile,outputsfile)) {
				fprintf(stdout,"provided training data loaded successfully, proceeding...\n");
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
			m.push_back(mneurons.get(1).asInt());
			m.push_back(mneurons.get(2).asInt());
			n.push_back(nneurons.get(1).asInt());
			n.push_back(nneurons.get(2).asInt());
			R->setNetworkSize(m,n);
		}
		if (mrange.isNull() || nrange.isNull() || mrange.size() != 5 || nrange.size() != 5) {
			fprintf(stderr,"neuron center ranges not properly set, exiting...\n");
			return false;
		} else {
			for (int i = 1; i < 5; i++) {
				mr.push_back(mrange.get(i).asDouble());
				nr.push_back(nrange.get(i).asDouble());
			}
			R->setNeuronRanges(mr,nr);
		}
		if (sigm.isNull() || sign.isNull() || sigm.size() != 3 || sign.size() != 3) {
			fprintf(stderr,"number of neurons not properly set, exiting...\n");
			return false;
		} else {
			sm.push_back(sigm.get(1).asDouble());
			sm.push_back(sigm.get(2).asDouble());
			sn.push_back(sign.get(1).asDouble());
			sn.push_back(sign.get(2).asDouble());
			R->setActivationRadii(sm,sn);
		}
		if (!rf.check("eta")) {
			fprintf(stderr,"no learning rate (eta) set, unable to proceed\n");
			return false;
		} else {
			R->setEta(rf.find("eta").asDouble());
		}
		if (!rf.check("leak")) {
			fprintf(stderr,"no leakage rate (leak) set, unable to proceed\n");
			return false;
		} else {
			R->setLeak(rf.find("leak").asDouble());
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

		//printf("BB: x: %f, y: %f, w: %f, h: %f\n",nr[0], nr[2], (nr[1]-nr[0]), (nr[3]-nr[2]));
		bb = Rect(nr[0], nr[2], (nr[1]-nr[0]), (nr[3]-nr[2]));

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
		if (!R->initNetwork()) {

			fprintf(stderr,"error: failed to initialize neural network, quitting\n");
			return false;

		}

		oMap = new ImageOf<PixelFloat>;
		oMap->resize(iw,ih);
		oMap->zero();

		return true;

	}

	void evaluateMap(const yarp::sig::Vector &Ax, ImageOf<PixelFloat> &map) {

		yarp::sig::Vector xy(2);
		yarp::sig::Vector xym(2);
		Matrix N = getN();
		double pmaxv;
		int amaxr;

		map.resize(iw,ih);
		map.zero();

		//use this to create a saliency map output
		ImageOf<PixelFloat> * Xr = new ImageOf<PixelFloat>;
		Xr->resize(n[0],n[1]);
		Xr->zero();
		for (int i = 0; i < n[0]; i++) {
			for (int j = 0; j < n[1]; j++) {
				Xr->pixel(i,j) = (255.0)*10*Ax[i*n[1]+j];
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

		//threshold to cut out low activations
		threshold(*C, *T, (1-tolerance)*pmaxv, 255.0, THRESH_TOZERO);

		/*
		//upsample to normal imagesize and gaussian blur
		resize(*T, *IM, Size(iw,ih));
		GaussianBlur(*IM, *IM, Size(55,55), 20.0, 20.0);

		//normalize it so that the max value is equal to full scale (255)
		normalize(*IM, *IM, 255.0, 0, NORM_INF);
		*/

		//printf("x: %f, y: %f, w: %f, h: %f\n",nr[0], nr[2], (nr[1]-nr[0]), (nr[3]-nr[2]));
		Mat * IMr = new Mat(*IM, bb);
		resize(*T, *IMr, Size(IMr->cols,IMr->rows));
		GaussianBlur(*IM, *IM, Size(55,55), 20.0, 20.0);

		//normalize it so that the max value is equal to full scale (255)
		normalize(*IM, *IM, 255.0, 0, NORM_INF);

		//rectangle(*IM, bb, 255.0, 2);

		/*
		//threshold to get the blobb
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
		for (int i = 0; i < n[0]; i++) {
			for (int j = 0; j < n[1]; j++) {
				if (Xr->pixel(i,j) > 0) {
					xym(0) = N[i*n[1]+j][0];
					xym(1) = N[i*n[1]+j][1];
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
		*/

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
		delete IMr;

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
			yarp::sig::Vector Ax;

			//check to see if a corresponding output space sample was also generated
			featVec = portFeatVec->read(false);

			//if one was, perform a training iteration
			if (featVec) {

				printf("received feed forward and feedback data, training\n");
				R->trainHebb(*headVec, *featVec, Ax);
				evaluateMap(Ax, *sMap);

			}

			//if only feed-forward, just evaluate map
			else {

				R->evaluate(*headVec, Ax);
				evaluateMap(Ax, *sMap);

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

		delete R;

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
				//just a placeholder for now
				reply.add(-1);
			}
		}
		else if (msg == "save") {
			if (command.size() < 3) {
				reply.add(-1);
			}
			else {
				string arg(command.get(1).asString().c_str());
				string filename(command.get(2).asString().c_str());
				Matrix S;
				ImageOf<PixelFloat> imToWrite;
				if (arg == "W") {
					S = thr->getW();
				}
				else if (arg == "M") {
					S = thr->getM();
				}
				else if (arg == "N") {
					S = thr->getN();
				}
				else {
					reply.add(-1);
				}
				imToWrite.resize(S.cols(),S.rows());
				for (int i = 0; i < S.rows(); i++) {
					for (int j = 0; j < S.cols(); j++) {
						imToWrite.pixel(j,i) = S(i,j);
					}
				}
				file::write(imToWrite,filename.c_str());
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



