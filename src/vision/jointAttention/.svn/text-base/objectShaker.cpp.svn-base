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

	ImageOf<PixelFloat> *accu;
	ImageOf<PixelFloat> *deccu;
	Mat * A;
	Mat * D;

	int imx,imy;

	MotionSalience * filter;
	ColorSalience * cfilter;

	double alpha;
	double detthresh;
	double colthresh;
	double motionsat;
	double motionthresh;
	double nmaxmindst;
	int decaytime;
	int nmineventsize;
	int lsx, lsy;

public:

	shakeDetThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
    { }

    virtual bool threadInit()
    {

        name=rf.check("name",Value("shakeDetector")).asString().c_str();
        detthresh=rf.check("threshold",Value(90.0)).asDouble();
        colthresh=rf.check("colthresh",Value(30.0)).asDouble();
        motionsat=rf.check("motionsat",Value(50.0)).asDouble();
        nmaxmindst=rf.check("nmaxmindst",Value(25.0)).asDouble();
        motionthresh=rf.check("motionthresh",Value(10.0)).asDouble();
        nmineventsize=rf.check("mineventsize",Value(40)).asInt();
        decaytime=rf.check("decayt",Value(10)).asInt();

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

        accu = new ImageOf<PixelFloat>;
        accu->resize(0,0);
        deccu = new ImageOf<PixelFloat>;
        deccu->resize(0,0);

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
        	if (accu->height() == 0 || accu->width() == 0) {
        		imx = pSal->width();
        		imy = pSal->height();
        		accu->resize(imx,imy);
        		accu->zero();
        		deccu->resize(imx,imy);
        		deccu->zero();
           	    A = new Mat(accu->height(), accu->width(), CV_32F, (void *)accu->getRawImage());
           	    D = new Mat(deccu->height(), deccu->width(), CV_32F, (void *)deccu->getRawImage());
        	}


        	//threshold to get color salience mask
        	Mat * M = new Mat(cSal->height(), cSal->width(), CV_32F, (void *)cSal->getRawImage());
        	Mat * msk = new Mat(cSal->height(), cSal->width(), CV_32F);
        	threshold(*M, *msk, colthresh, 1.0, CV_THRESH_BINARY);

        	//smooth out and saturate the motion salience map
        	Mat * T = new Mat(pSal->height(), pSal->width(), CV_32F, (void *)pSal->getRawImage());
        	Mat * C = new Mat(pSal->height(), pSal->width(), CV_32F);
        	GaussianBlur(*T, *C, Size(25,25), 5.0, 5.0);
        	threshold(*C, *T, motionsat, 1.0, THRESH_TRUNC);

        	//combine the two maps and threshold to get 'colorful, moving' objects
        	multiply(*msk, *T, *M);
        	threshold(*M, *C, motionthresh, 1.0, CV_THRESH_BINARY);

        	//to detect a 'sustained' activity event, keep an integrator of the combined map
        	add(*A,*C,*A);

        	//also keep an inactivity accumulator (activity in a pixel resets its count)
        	add(*D,1.0,*D);
        	subtract(1.0, *C, *msk);
        	multiply(*D, *msk, *D);
        	min(*D, decaytime+1, *D);

        	//if a pixel is inactive long enough, reset its accumulator
        	threshold(*D, *msk, decaytime, 1.0, CV_THRESH_BINARY_INV);
        	multiply(*A, *msk, *A);
        	min(*A, 255.0, *A);
        	imgOut.copy(*accu);

        	//threshold the accumulator to find regions of sustained interest (such as a moving toy)
        	double * cmax = new double;
        	Point * mloc = new Point;
        	int xmax, ymax, mcnt;
        	minMaxLoc(*A, NULL, cmax, NULL, mloc);

        	//find the region of the most active point
        	if (*cmax > detthresh) {

        		Rect * bbox = new Rect;
				threshold(*A, *C, detthresh, 1.0, CV_THRESH_BINARY);
				mcnt = floodFill(*C, *mloc, Scalar(255.0), bbox);
				C->copyTo(*T);

				xmax = bbox->x+(bbox->width/2.0);
				ymax = bbox->y+(bbox->height/2.0);

				//if the region has enough active points to be considered an 'event'
				if (mcnt > nmineventsize) {

					//non-max supression of sequential detection
					if (sqrt((xmax-lsx)*(xmax-lsx)+(ymax-lsy)*(ymax-lsy)) > nmaxmindst) {

						yarp::sig::Vector &detectedLoc=portDetLoc->prepare();
						detectedLoc.clear();
						detectedLoc.push_back(xmax);
						detectedLoc.push_back(ymax);
						portDetLoc->write();
						draw::addCrossHair(imgOut, PixelRgb(0,255,0), xmax, ymax, 10);
						lsx = xmax;
						lsy = ymax;
						printf("event detected, generating point\n");

					} else {

						draw::addCrossHair(imgOut, PixelRgb(255,0,0), xmax, ymax, 10);

					}

				}

				delete bbox;

        	}

        	delete cmax;
        	delete mloc;

        	//imgOut.copy(*pDest);

            portImgOut->write();

            delete msk;
            delete C;
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



