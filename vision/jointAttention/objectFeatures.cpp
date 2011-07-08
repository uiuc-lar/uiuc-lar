/*
 *  objectFeatures.cpp
 *
 * 	Logan Niehaus
 * 	6/12/11
 * 	module to interface with visual feature processor
 *
 * 	inputs:
 * 		/objectFeatures/obj:i	-- Nx2 matrix containing object contour points
 * 		/objectFeatures/img:i	-- reference image for object feat. extraction
 *
 *  params:
 *  	name	-- module basename (D /objectFeatures)
 *  	verbose -- flag to turn on/off dumping of features to screen (O)
 *
 *  outputs:
 *  	/objectFeatures/feat:o	-- Vector containing feature vales extracted for provided object
 *
 *  TODO:
 *  	make this module more customizable, after the object feature extractor class is more customizable
 *
 */


#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>

#include <cv.h>

#include <string>
#include <stdio.h>
#include <stdlib.h>

#include "visFeatExtractor.h"

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

class vfPort : public BufferedPort<Matrix> {

protected:

	//feature extractor
	visFeatures * V;

	//image and output ports
	BufferedPort<ImageOf<PixelRgb> > * iPort;
	BufferedPort<yarp::sig::Vector> * oPort;

	bool verbose;

public:

	vfPort(visFeatures *&_V, BufferedPort<ImageOf<PixelRgb> > *& _iPort,
			BufferedPort<yarp::sig::Vector> *& oPort_)
	: V(_V), iPort(_iPort), oPort(oPort_), verbose(false) {	}

	void setVerbosity(bool _verbose) { verbose = _verbose; }

	//callback for incoming objects
	virtual void onRead(Matrix& p) {

		//when an object arrives grab the current image (don't block)
		ImageOf<PixelRgb> *pImgIn = iPort->read(false);

		if (pImgIn) {

			yarp::sig::Vector &features = oPort->prepare();

			//pass them to the feature extractor
			features = V->extractFeatures(*pImgIn, p);

			if (verbose) {
				for (int i = 0; i < features.size()-1; i++) {
					printf("%f, ", features[i]);
				}
				printf("%f;\n",features[features.size()-1]);
			}

			//write the feature vector out
			oPort->write();

		}

	}

};

class visFeatThread : public Thread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > 	*portImgIn;
	vfPort								*portObjIn;
	BufferedPort<yarp::sig::Vector> 	*portFeatOut;

	visFeatures * V;


public:

	visFeatThread(ResourceFinder &_rf) : rf(_rf)
	{ }

	virtual bool threadInit()
	{

		name=rf.check("name",Value("objectFeatures")).asString().c_str();

		V = new visFeatures();

		portImgIn=new BufferedPort<ImageOf<PixelRgb> >;
		string portInName="/"+name+"/img:i";
		portImgIn->open(portInName.c_str());

		portFeatOut=new BufferedPort<yarp::sig::Vector>;
		string portFeatName="/"+name+"/feat:o";
		portFeatOut->open(portFeatName.c_str());

		portObjIn=new vfPort(V, portImgIn, portFeatOut);
		string portObjName="/"+name+"/obj:i";
		portObjIn->open(portObjName.c_str());
		portObjIn->useCallback();
		portObjIn->setVerbosity(rf.check("verbose"));

		return true;

	}

	virtual void run()
	{

		//sit here and look pretty
		while (isStopping() != true) {

			continue;

		}

	}

	virtual void threadRelease()
	{

		portImgIn->interrupt();
		portObjIn->interrupt();
		portFeatOut->interrupt();

		delete V;

		delete portImgIn;
		delete portObjIn;
		delete portFeatOut;

	}

};

class visFeatModule: public RFModule
{

protected:

	visFeatThread *thr;

public:

	visFeatModule() { }

	virtual bool configure(ResourceFinder &rf)
	{

		thr=new visFeatThread(rf);
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

	visFeatModule mod;

	return mod.runModule(rf);
}
