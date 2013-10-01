/*
 * Copyright (C) 2013 Logan Niehaus
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
 *  jointAttention3d.cpp
 *
 * 	Logan Niehaus
 * 	9/25/13
 * 	joint attentional saliency mapper (learner + evaluator) for 3d
 * 	this module is built off of the old gazeEval, now incorporates online learning
 * 	of the map, as well as evaluation of the map given inputs
 *
 * inputs: 	receives gaze estimate features from gaze estimation module, and image coordinates of
 * 				the eye midpoint from gaze estimator as well
 * 			communicates with stereoVision module to get the XYZ location of the eye midpoint
 * 			checks for location of detected objects (and either learns or evaluates the previous,
 * 				based on if it is there or not)
 * 			for evaluation, maps gaze features to difference between target XY and head XY. Z is fixed
 * 				at a certain depth, usually that of the table
 * 			learning not working yet
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
#include <yarp/os/RpcClient.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>

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
using namespace iCub::ctrl;

class jointAttention3dThread : public RateThread
{
protected:

	//names
	ResourceFinder &rf;
	string name;

	//data ports
	BufferedPort<ImageOf<PixelMono> > *portMapOut;
	BufferedPort<yarp::sig::Vector>	 *portFeatVec;
	BufferedPort<yarp::sig::Vector>	 *portEyeVec;
	BufferedPort<ImageOf<PixelRgb> > *portImgOut;

	RpcClient * portStereo;

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

	//3d stuff
	Matrix H0;
	Matrix K; //left camera matrix

	//optional loading of trained network data
	string weightsfile;
	string inputsfile;
	string outputsfile;


public:

	jointAttention3dThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
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
		name=rf.check("name",Value("jointAttention3d")).asString().c_str();
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

		K.resize(3,3);
		K.zero(); K(2,2) = 1.0;
		K(0,0) = rf.check("fx",Value(237.54)).asDouble();
		K(1,1) = rf.check("fy",Value(237.54)).asDouble();
		K(0,2) = rf.check("cx",Value(160)).asDouble();
		K(1,2) = rf.check("cy",Value(120)).asDouble();


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
		string portFeatName="/"+name+"/feat:i";
		portFeatVec->open(portFeatName.c_str());

		portEyeVec=new BufferedPort<yarp::sig::Vector>;
		string portEyeName="/"+name+"/eye:i";
		portEyeVec->open(portEyeName.c_str());

		portMapOut=new BufferedPort<ImageOf<PixelMono> >;
		string portMapName="/"+name+"/map:o";
		portMapOut->open(portMapName.c_str());

		portImgOut=new BufferedPort<ImageOf<PixelRgb> >;
		string portOutName="/"+name+"/view";
		portImgOut->open(portOutName.c_str());

		portStereo=new RpcClient;
		string portStereoName="/"+name+"/svrpc";
		portStereo->open(portStereoName.c_str());

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


	virtual void run()
	{

		//if an input space estimate was received
		if (portFeatVec->getPendingReads() > 0 && portEyeVec->getPendingReads() >0)
		{

			yarp::sig::Vector *eyeVec = portEyeVec->read(false);
			yarp::sig::Vector *featVec = portFeatVec->read(false);

			ImageOf<PixelFloat> * sMap = new ImageOf<PixelFloat>;
			yarp::sig::Vector Ax;


			//check to see if feature vector is valid
			if ((*featVec)[2] != 0) {

				featVec->pop_back();

				R->evaluate(*featVec, Ax);


				//retrieve the xyz position of the eye center, as well as the H matrix
				if (portStereo->getOutputCount() < 0) {

					printf("Not connected to stereoVision rpc port!\n");

				}
				else {


					Bottle eloc, Htmp;

					Bottle hmsg("geth");

					portStereo->write(hmsg, Htmp);
					H0.resize(4,4); H0.zero();
					for (int i = 0; i < 4; i++) {
						for (int j = 0; j < 4; j++) {
							H0(i,j) = Htmp.get(i*4+j).asDouble();
						}
					}
					H0 = SE3inv(H0);

					Bottle bmsg;
					bmsg.addString("loc");
					bmsg.addInt((*eyeVec)[0]);
					bmsg.addInt((*eyeVec)[1]);


					portStereo->write(bmsg, eloc);

					//reproject points that we care about into 3d
					double maxResp = findMax(Ax);
					Matrix XYZ, PC;
					Matrix N = R->getN();
					int np = 0;
					for (int i = 0; i < Ax.size(); i++) {

						if (Ax[i] >= tolerance*maxResp) {

							np++;
							XYZ.resize(4,np);
							XYZ(0,np-1) = N(i,0)+eloc.get(0).asDouble();
							XYZ(1,np-1) = N(i,1)+eloc.get(1).asDouble();
							XYZ(2,np-1) = -0.13; XYZ(3,np-1) = 1;
						}
					}

					if (XYZ.cols() != 0) {

						//transform into left eye reference frame
						XYZ = H0*XYZ;

						//use the left camera matrix to project into image coordinates
						PC = K*XYZ.submatrix(0,2,0,XYZ.cols()-1);

					}

					ImageOf<PixelMono> &mapOut= portMapOut->prepare();
					ImageOf<PixelRgb> &imgOut= portImgOut->prepare();
					mapOut.resize(iw,ih);
					imgOut.resize(iw,ih);
					mapOut.zero(); imgOut.zero();

					Mat * IM = new Mat(ih, iw, CV_8UC1, (void *)mapOut.getRawImage());
					Mat OM = (IplImage *) imgOut.getIplImage();


					//fit a convex contour around these points
					vector<Point> nimp(0);
					for (int i = 0; i < PC.cols(); i++) {

						nimp.push_back(Point(PC(0,i)/PC(2,i),PC(1,i)/PC(2,i)));
						circle(*IM, nimp[i], 1, Scalar(255), 2);
					}

					Mat hullpts;
					vector<Mat> hullwrap;

					if (XYZ.cols() != 0) {
						convexHull(nimp,hullpts);
						hullwrap.push_back(hullpts);
						drawContours(*IM, hullwrap, 0, Scalar(255), -1);
					}

					GaussianBlur(*IM, *IM, Size(55,55), 20.0, 20.0);

					cvtColor(*IM,OM,CV_GRAY2RGB);

					portMapOut->write();
					portImgOut->write();


				}

			}
		}

	}



	virtual void threadRelease()
	{

		portFeatVec->interrupt();
		portEyeVec->interrupt();
		portMapOut->interrupt();
		portImgOut->interrupt();
		portStereo->interrupt();

		delete portFeatVec;
		delete portEyeVec;
		delete portMapOut;
		delete portImgOut;
		delete portStereo;

		delete R;

	}

};

class jointAttention3dModule: public RFModule
{
protected:

	jointAttention3dThread *thr;
	Port * rpcPort;
	string name;

public:

	jointAttention3dModule() { }

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
		name=rf.check("name",Value("jointAttention3d")).asString().c_str();
		rpcPort = new Port;
		string portRpcName="/"+name+"/rpc";
		rpcPort->open(portRpcName.c_str());
		attach(*rpcPort);

		thr=new jointAttention3dThread(rf);
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

	jointAttention3dModule mod;

	return mod.runModule(rf);
}



