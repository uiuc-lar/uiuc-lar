#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

#include <iCub/vis/Salience.h>
#include <iCub/vis/MotionSalience.h>

#include <cv.h>

#include <string>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <deque>

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::vis;

#define ALPHA 0.05
#define SCALING 0.2
#define INTERESTTHRESH 40

class shakeDetThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *portImgIn;
	BufferedPort<ImageOf<PixelRgb> > *portImgOut;
	BufferedPort<Bottle>			 *portDetLoc;

	ImageOf<PixelFloat> *prevSal;

	int imx,imy;

	MotionSalience * filter;

public:

	shakeDetThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
    { }

    virtual bool threadInit()
    {

        name=rf.check("name",Value("myShakeDetector")).asString().c_str();

        portImgIn=new BufferedPort<ImageOf<PixelRgb> >;
        string portInName="/"+name+"/img:i";
        portImgIn->open(portInName.c_str());

        portImgOut=new BufferedPort<ImageOf<PixelRgb> >;
        string portOutName="/"+name+"/img:o";
        portImgOut->open(portOutName.c_str());

        portDetLoc=new BufferedPort<Bottle>;
        string portLocName="/"+name+"/loc:o";
        portDetLoc->open(portLocName.c_str());

        filter = new MotionSalience;
        filter->open(rf);

        prevSal = NULL;

    	return true;
    }

    virtual void run()
    {

        // get inputs
        ImageOf<PixelRgb> *pImgIn=portImgIn->read(false);

        // process camera image
        if (pImgIn)
        {

        	ImageOf<PixelRgb> *pDest = new ImageOf<PixelRgb>;
        	ImageOf<PixelFloat> *pSal = new ImageOf<PixelFloat>;
        	ImageOf<PixelRgb> &imgOut=portImgOut->prepare();
        	Bottle &detectedLoc=portDetLoc->prepare();

        	//apply motion salience filter
        	filter->apply(*pImgIn, *pDest, *pSal);

        	//set up dimensions if first sample
        	if (prevSal == NULL) {
        		imx = pSal->width();
        		imy = pSal->height();
        		prevSal = new ImageOf<PixelFloat>;
        		prevSal->resize(imx,imy);
        		prevSal->zero();
        	}

        	//temporal filtering
        	ImageOf<PixelFloat> * motionSal = new ImageOf<PixelFloat>;
        	motionSal->copy(*pSal);
        	int xmax, ymax;
        	double cmax = 0.0;
        	for (int i = 0; i < motionSal->width(); i++) {
        		for (int j = 0; j < motionSal->height(); j++) {

        			motionSal->pixel(i,j) = SCALING*ALPHA*motionSal->pixel(i,j) + (1-ALPHA)*prevSal->pixel(i,j);

        			//find current maximum and print
        			if (motionSal->pixel(i,j) > cmax) {
        				cmax = motionSal->pixel(i,j);
        				xmax = i;
        				ymax = j;
        			}

        		}
        	}

        	if (cmax > INTERESTTHRESH) {
        		detectedLoc.clear();
        		detectedLoc.addInt(xmax);
        		detectedLoc.addInt(ymax);
        		portDetLoc->write();
        	}
        	else {
        		portDetLoc->unprepare();
        	}


        	imgOut.copy(*motionSal);
        	delete prevSal;
        	prevSal = motionSal;

            portImgOut->write();

            delete pDest;
            delete pSal;

        }
    }

    virtual void threadRelease()
    {

    	portImgIn->interrupt();
    	portImgOut->interrupt();
    	portDetLoc->interrupt();
    	portImgIn->close();
    	portImgOut->close();
    	portDetLoc->close();

    	filter->close();

    	delete portImgIn;
    	delete portImgOut;
    	delete portDetLoc;
    	delete filter;

    }

};

class shakeDetModule: public RFModule
{
protected:
    shakeDetThread *thr;

public:
    shakeDetModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new shakeDetThread(rf);
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

    shakeDetModule mod;

    return mod.runModule(rf);
}



