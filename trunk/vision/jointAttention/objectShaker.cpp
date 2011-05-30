/*
 *  objectShaker.cpp
 *
 * 	Logan Niehaus
 * 	5/2/11
 * 	module for detecting the presence of an interesting visual stimulus
 *
 * inputs: input image from the icub in rgb
 * params:	integrator coeff and leak coeff values
 * 			alpha for temporal filtering of motion image (needs yarped)
 * 			threshold for color salience, saturation value for motion
 * 			detection threshold for an interesting object
 * outputs: upon detection it puts out a vector w/ location of object in image coordinates
 * 			also puts out an image showing the current combined salience
 * desc: uses two salience maps: color and motion. an interesting object is defined as
 * 			something which is colorful, and is active for a sustained amount of time.
 * 			color map gets thresholded. motion map first gets edges smoothed and then it gets
 * 			a saturation to make sure there isnt overwhelming response to any single event.
 * 			motion map is then temporally filtered with a leaky integrator, and the color salience
 * 			map is applied as a mask. detections are reported as the centroid of all points
 * 			above the threshold.
 * TODOs: yarp this. also maybe it should be possible to receive the salience maps so they only
 * 			get computed once? also it may be worthwhile to look at how this can be generalized
 *
 */

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
#include <iCub/vis/ColorSalience.h>

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


class shakeDetThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *portImgIn;
	BufferedPort<ImageOf<PixelRgb> > *portImgOut;
	BufferedPort<yarp::sig::Vector>	 *portDetLoc;

	ImageOf<PixelFloat> *prevSal;

	int imx,imy;

	MotionSalience * filter;
	ColorSalience * cfilter;

	double alpha;
	double leakage;
	double detthresh;
	double colthresh;
	double motionsat;
	double nmaxmindst;
	int lsx, lsy;

public:

	shakeDetThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
    { }

    virtual bool threadInit()
    {

        name=rf.check("name",Value("shakeDetector")).asString().c_str();
        alpha=rf.check("alpha",Value(0.05)).asDouble();
        leakage=rf.check("leak",Value(0.9999)).asDouble();
        detthresh=rf.check("threshold",Value(90.0)).asDouble();
        colthresh=rf.check("colthresh",Value(30.0)).asDouble();
        motionsat=rf.check("motionsat",Value(50.0)).asDouble();
        nmaxmindst=rf.check("nmaxmindst",Value(25.0)).asDouble();


        portImgIn=new BufferedPort<ImageOf<PixelRgb> >;
        string portInName="/"+name+"/img:i";
        portImgIn->open(portInName.c_str());

        portImgOut=new BufferedPort<ImageOf<PixelRgb> >;
        string portOutName="/"+name+"/img:o";
        portImgOut->open(portOutName.c_str());

        portDetLoc=new BufferedPort<yarp::sig::Vector>;
        string portLocName="/"+name+"/loc:o";
        portDetLoc->open(portLocName.c_str());

        filter = new MotionSalience;
        filter->open(rf);

        cfilter = new ColorSalience;
        cfilter->open(rf);

        prevSal = NULL;
        lsx = -1000;
        lsy = -1000;

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
        	ImageOf<PixelFloat> *cSal = new ImageOf<PixelFloat>;
        	ImageOf<PixelRgb> &imgOut=portImgOut->prepare();

        	//apply motion salience filter
        	filter->apply(*pImgIn, *pDest, *pSal);
        	cfilter->apply(*pImgIn, *pDest, *cSal);

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
        	ImageOf<PixelFloat> * motionBase = new ImageOf<PixelFloat>;
        	motionSal->copy(*pSal);
        	motionBase->copy(*pSal);

        	//color salience mask
        	Mat * M = new Mat(cSal->height(), cSal->width(), CV_32F, (void *)cSal->getRawImage());
        	Mat * msk = new Mat(cSal->height(), cSal->width(), CV_32F);
        	threshold(*M, *msk, colthresh, 1.0, CV_THRESH_BINARY);
        	msk->copyTo(*M);

        	//spatial filter first
        	Mat * T = new Mat(motionSal->height(), motionSal->width(), CV_32F, (void *)motionSal->getRawImage());
        	Mat * C = new Mat(motionSal->height(), motionSal->width(), CV_32F);
        	GaussianBlur(*T, *C, Size(25,25), 5.0, 5.0);
        	threshold(*C, *T, motionsat, 1.0, THRESH_TRUNC);

        	int xmax, ymax, mcnt;
        	double cmax = 0.0;

        	mcnt = 0;
        	xmax = 0;
        	ymax = 0;
        	for (int i = 0; i < motionSal->width(); i++) {
        		for (int j = 0; j < motionSal->height(); j++) {

        			motionSal->pixel(i,j) = cSal->pixel(i,j)*(alpha*motionSal->pixel(i,j) + leakage*prevSal->pixel(i,j));
        			if (motionSal->pixel(i,j) > 255) {
        				motionSal->pixel(i,j) = 255;
        			}

        			motionBase->pixel(i,j) = cSal->pixel(i,j)*motionSal->pixel(i,j);

        			//find current centroid of detected pixels
        			if (motionBase->pixel(i,j) > detthresh) {
        				mcnt++;
        				xmax += i;
        				ymax += j;
        			}

        		}
        	}

        	imgOut.copy(*motionBase);


        	if (mcnt > 0) {

        		if (sqrt((xmax/mcnt-lsx)*(xmax/mcnt-lsx)+(ymax/mcnt-lsy)*(ymax/mcnt-lsy)) > nmaxmindst) {

                	yarp::sig::Vector &detectedLoc=portDetLoc->prepare();
					detectedLoc.clear();
					detectedLoc.push_back(xmax/mcnt);
					detectedLoc.push_back(ymax/mcnt);
					portDetLoc->write();
					draw::addCrossHair(imgOut, PixelRgb(0,255,0), xmax/mcnt, ymax/mcnt, 10);
					lsx = xmax/mcnt;
					lsy = ymax/mcnt;

        		} else {

        			draw::addCrossHair(imgOut, PixelRgb(255,0,0), xmax/mcnt, ymax/mcnt, 10);

        		}


        	}
        	else {
        		portDetLoc->unprepare();
        	}

        	delete prevSal;
        	prevSal = motionSal;

            portImgOut->write();

            delete msk;
            delete C;
            delete motionBase;
            delete pDest;
            delete pSal;
            delete cSal;

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



