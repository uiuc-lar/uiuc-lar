/*
 *  topDownObjectMap.cpp
 *
 * 	Logan Niehaus
 * 	7/05/11
 * 	top-down object salience attention map generator. algorithm receives a
 * 	feature vector (possibly with a mask) as its input. the algorithm then
 * 	does a rough segmentation to locate interesting objects, and calculates
 * 	the feature vectors for these objects. the salience of an object is
 * 	proportional to its closeness in feature space to the provided feature vector.
 * 	object salience is set to this metric for each pixel of the object.
 *
 * inputs:
 * 		/topDownObjMap/feat:i	-- feature vector to match as described above
 * 		/topDownObjMap/mask:i	-- optional feature mask, specifying which features the
 * 									salience calculation should take into account
 * 		/topDownObjMap/img:i	-- reference image for segmentation and feat. extraction
 *		/topDownObjMap/seg:i	-- binary image (as PixelFloat) for segmenting object regions
 *
 * params:
 * 		decay	-- number of frames it takes for salience due to a single top-down input to
 * 					fade (D 60, ~4s@15fps)
 * 		wta		-- flag to enable winner-take-all behavior. for each top-down event, only the closest
 * 					object is made salient. it is set to full (255.0) salience. (O)
 *	 	name	-- module basename (D /topDownObjMap)
 *
 *
 * outputs:
 * 		/topDownObjMap/map:o	-- object matching salience map described above
 * 		/topDownObjMap/obj:o	-- Nx2 matrix containing contour pixels of most salience object
 *
 * TODO:   everything
 *
 */

//yarp includes
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

//external cv includes
#include <iCub/vis/Salience.h>
#include <iCub/vis/IntensitySalience.h>
#include <cv.h>

//misc inc
#include <string>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <deque>

//internal cv includes
#include "visFeatExtractor.h"
#include "colorTransform.h"

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::vis;

struct tdEvent {

	yarp::sig::Vector f;	//feature vector
	yarp::sig::Vector m;	//feature mask
	int timeToDeath;		//number of frames until salience from event disappears

};

class EventBuffer : public deque<tdEvent> {

private:

	Semaphore mutex;

public:

	void lock()   { mutex.wait(); }
	void unlock() { mutex.post(); }

};

class tdosPort : public BufferedPort<yarp::sig::Vector> {

protected:

	BufferedPort<yarp::sig::Vector> * mPort;
	EventBuffer & eBuf;
	int mdecay;

public:

	tdosPort(EventBuffer &_Eb, BufferedPort<yarp::sig::Vector> *& _mPort, int _mdecay)
	: eBuf(_Eb), mPort(_mPort), mdecay(_mdecay) {	}

	//callback for incoming objects
	virtual void onRead(yarp::sig::Vector& fv) {

		//when an feature vector arrives, check the mask port for input
		yarp::sig::Vector *mask = mPort->read(false);

		//if no mask provided, set mask to all ones
		if (!mask) {
			mask = new yarp::sig::Vector(fv.size());
			(*mask) = 1.0;
		}
		printf("received new top-down event\n");

		//create top-down event descriptor and pass to main thread
		tdEvent T;
		T.f = fv;
		T.m = yarp::sig::Vector(*mask);
		T.timeToDeath = mdecay;
		eBuf.lock();
		eBuf.push_back(T);
		eBuf.unlock();

	}

};

class topDownObjSalThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	//ports
	BufferedPort<ImageOf<PixelRgb> > 	*portImgIn;
	tdosPort							*portFeatIn;
	BufferedPort<yarp::sig::Vector>		*portMaskIn;
	BufferedPort<ImageOf<PixelFloat> > 	*portSegIn;
	BufferedPort<ImageOf<PixelFloat> > 	*portMapOut;
	BufferedPort<Matrix>				*portObjOut;
	BufferedPort<ImageOf<PixelRgb> > 	*portImgOut;

	//data objects
	visFeatures * V;
	EventBuffer buf;

	//running parameters
	int decaytime;
	bool wta;


public:

	topDownObjSalThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	virtual bool threadInit()
	{

		name=rf.check("name",Value("topDownObjMap")).asString().c_str();
		decaytime=rf.check("decay",Value(60)).asInt();
		wta = rf.check("wta");

		V = new visFeatures();

		portImgIn=new BufferedPort<ImageOf<PixelRgb> >;
		string portInName="/"+name+"/img:i";
		portImgIn->open(portInName.c_str());

		portMaskIn=new BufferedPort<yarp::sig::Vector>;
		string portMaskName="/"+name+"/mask:i";
		portMaskIn->open(portMaskName.c_str());

		portSegIn=new BufferedPort<ImageOf<PixelFloat> >;
		string portSegName="/"+name+"/seg:i";
		portSegIn->open(portSegName.c_str());

		portFeatIn=new tdosPort(buf, portMaskIn, decaytime);
		string portFeatName="/"+name+"/feat:i";
		portFeatIn->open(portFeatName.c_str());
		portFeatIn->useCallback();

		portMapOut=new BufferedPort<ImageOf<PixelFloat> >;
		string portMapName="/"+name+"/map:o";
		portMapOut->open(portMapName.c_str());

		portObjOut=new BufferedPort<yarp::sig::Matrix>;
		string portObjName="/"+name+"/obj:o";
		portObjOut->open(portObjName.c_str());

		portImgOut=new BufferedPort<ImageOf<PixelRgb> >;
		string portOutName="/"+name+"/img:o";
		portImgOut->open(portOutName.c_str());

		return true;

	}

	virtual void run()
	{

		//get latest image
		ImageOf<PixelRgb> *pImgIn=portImgIn->read(false);
		ImageOf<PixelFloat> &mapOut= portMapOut->prepare();
		ImageOf<PixelRgb> &imgOut= portImgOut->prepare();


		if (pImgIn) {

			mapOut.resize(*pImgIn);
			mapOut.zero();

			//if there are any recent events
			if (!buf.empty()) {

				vector<vector<Point> > contours;
				vector<Vec4i> hierarchy;
				vector<yarp::sig::Vector> objFeatures;
				ImageOf<PixelFloat> *segMask=portSegIn->read(false);

				//compute the list of segmented objects
				if (segMask) {

					Mat * M = new Mat(segMask->height(), segMask->width(), CV_32F, (void *)segMask->getRawImage());
					Mat X;

					M->convertTo(X,CV_8UC1);
					findContours(X, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

				}

				//for each object, get its feature descriptor
				for (int i = 0; i < contours.size(); i++) {

					Matrix cts(contours[i].size(), 2);
					for (int j = 0; j < contours[i].size(); j++) {
						cts[j][0] = contours[i].at(j).y;
						cts[j][1] = contours[i].at(j).x;
					}
					objFeatures.push_back(V->extractFeatures(*pImgIn, cts));

				}

				Mat * S = new Mat(pImgIn->height(), pImgIn->width(), CV_32F, (void *)mapOut.getRawImage());
				int nevents = buf.size();

				//compute feature-distance based salience
				for (int i = 0; i < buf.size(); i++) {

					Mat N(pImgIn->height(), pImgIn->width(), CV_32F);
					yarp::sig::Vector ef, tf, msk;
					double dist, sal, maxsal;
					int msaldx;
					tdEvent &te = buf.at(i);
					ef = te.f;
					msk = te.m;
					N = 0.0;
					maxsal = -1;

					for (int j = 0; j < objFeatures.size(); j++) {

						//calculate the similarity of object and provided features
						tf = objFeatures[j];
						dist = 0.0;
						for (int k = 0; k < ef.size(); k++) {
							dist += msk[k]*(ef[k]-tf[k])*(ef[k]-tf[k]);
						}
						dist = sqrt(dist);

						//scale by interest decay factor
						sal = (1.0)/(1.0 + dist);
						sal *= ((double)te.timeToDeath/(double)decaytime);

						if (wta) {
							if (sal > maxsal) {
								maxsal = sal;
								msaldx = j;
							}
						}
						else {
							//apply salience for that object
							drawContours(N, contours, i, Scalar(sal), -1);
						}

					}

					//if winner-take-all behavior enabled, only draw one
					if (wta) {

						sal = ((double)te.timeToDeath/(double)decaytime);
						drawContours(N, contours, msaldx, Scalar(sal), -1);

					}

					//increment event decay
					te.timeToDeath -= 1;
					if (te.timeToDeath <= 0) {
						i--;
						buf.lock();
						buf.pop_front();
						buf.unlock();
					}


					//add event maps together
					add(*S, N, *S);

				}

				//scale final map so that multiple events don't make a mess
				(*S) = (255.0/(double)nevents)*(*S);


			}

			imgOut.copy(mapOut);
			portImgOut->write();
			portMapOut->write();

		}

	}

	virtual void threadRelease()
	{

		portImgIn->interrupt();
		portFeatIn->interrupt();
		portMaskIn->interrupt();
		portSegIn->interrupt();
		portMapOut->interrupt();
		portObjOut->interrupt();
		portImgOut->interrupt();

		delete portImgIn;
		delete portFeatIn;
		delete portMaskIn;
		delete portSegIn;
		delete portMapOut;
		delete portObjOut;
		delete portImgOut;

		delete V;

	}

};

class topDownObjSalModule: public RFModule
{

protected:

	topDownObjSalThread *thr;

public:

	topDownObjSalModule() { }

	virtual bool configure(ResourceFinder &rf)
	{

		thr=new topDownObjSalThread(rf);
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

	topDownObjSalModule mod;

	return mod.runModule(rf);
}
