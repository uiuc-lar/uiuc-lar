/*
 *  salienceAggregator.cpp
 *
 * 	Logan Niehaus
 * 	5/7/11
 * 	demo-ish module for testing the joint attention capabilities. basically uses color
 * 		and joint attention salience maps to do some rudimentary object segmentation
 *
 * inputs: input image from icub (maybe optional if we only care about maps)
 * 			salience maps from joint attention system
 * 			(color salience map from outside?)
 * params:	integrator coeff and leak coeff values
 * 			alpha for temporal filtering of motion image (needs yarped)
 * 			threshold for color salience, saturation value for motion
 * 			detection threshold for an interesting object
 * outputs: image with a contour placed around the attended to region
 * 			 also potentially a vector of points for the contour, or bounding box
 * desc: simple object salience/segmentation based on learned joint attention. attention map
 * 			gets generated from gazeEval module and sent here, and either icub image gets sent
 * 			and colormap gets generated here or straight colormap is sent in. colormap will be
 * 			thresholded and laid over with the joint attention map. find contours. blob w/
 * 			highest average salience gets chosen, drawn around, sent off, etc...
 * TODOs: kalman filter on blob location, or possibly extra temporal filter to control jitter
 * 			on joint attention. also maybe inhibition of return.
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
#include <iCub/vis/ColorSalience.h>

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


class salAggThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *portImgIn;
	BufferedPort<ImageOf<PixelRgb> > *portImgOut;
	BufferedPort<ImageOf<PixelFloat> > *portColorIn;
	BufferedPort<ImageOf<PixelFloat> > *portGazeIn;
	BufferedPort<yarp::sig::Matrix>	*portObjOut;

	ColorSalience * cfilter;

	double colthresh;
	double minobjsize;

public:

	salAggThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
    { }

    virtual bool threadInit()
    {

        name=rf.check("name",Value("mySalAgg")).asString().c_str();
        colthresh=rf.check("colthresh",Value(10.0)).asDouble();
        minobjsize=rf.check("minobjsize",Value(30.0)).asDouble();

        portImgIn=new BufferedPort<ImageOf<PixelRgb> >;
        string portInName="/"+name+"/img:i";
        portImgIn->open(portInName.c_str());

        portImgOut=new BufferedPort<ImageOf<PixelRgb> >;
        string portOutName="/"+name+"/img:o";
        portImgOut->open(portOutName.c_str());

        portColorIn=new BufferedPort<ImageOf<PixelFloat> >;
        string portColorName="/"+name+"/color:i";
        portColorIn->open(portColorName.c_str());

        portGazeIn=new BufferedPort<ImageOf<PixelFloat> >;
        string portGazeName="/"+name+"/gaze:i";
        portGazeIn->open(portGazeName.c_str());

        portObjOut=new BufferedPort<yarp::sig::Matrix>;
        string portObjName="/"+name+"/obj:o";
        portObjOut->open(portObjName.c_str());

        cfilter = new ColorSalience;
        cfilter->open(rf);

    	return true;

    }

    virtual void run()
    {

        // get inputs
    	ImageOf<PixelFloat> *pColIn=portColorIn->read(false);
        ImageOf<PixelRgb> *pImgIn=portImgIn->read(false);
    	ImageOf<PixelRgb> *cDest = NULL;
    	ImageOf<PixelFloat> *cSal = NULL;


        //if there is a color salience map coming in take it
        if (pColIn) {

        	portColorIn->acquire();
        	cSal = pColIn;

        }
        //if there is a camera image, use that
        else if (pImgIn)
        {


        	cDest = new ImageOf<PixelRgb>;
        	cSal = new ImageOf<PixelFloat>;
        	cfilter->apply(*pImgIn, *cDest, *cSal);

        	delete cDest;

        }

        //if we got the map one way or another go on
		if (cSal) {

			ImageOf<PixelRgb> &imgOut= portImgOut->prepare();
			ImageOf<PixelFloat> *pJAIn = portGazeIn->read(true);


			//start by thresholding the color salience map
        	Mat * M = new Mat(cSal->height(), cSal->width(), CV_32F, (void *)cSal->getRawImage());
        	Mat * N = new Mat(cSal->height(), cSal->width(), CV_32F);
        	threshold(*M, *N, colthresh, 1.0, CV_THRESH_BINARY);
        	N->copyTo(*M);

        	//find the blobs of the color salience map
        	vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
        	Mat X;
        	M->convertTo(X,CV_8UC1);
        	findContours(X, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

        	//use the color salience bw image to mask the joint attention map
        	Mat * P = new Mat(pJAIn->height(), pJAIn->width(), CV_32F, (void *)pJAIn->getRawImage());
        	multiply(*M, *P, *N);

        	//find the blob with highest average salience
        	Rect br;
        	double maxSal = 0.0;
        	int iBlob = -1;
        	for (int i = 0; i < contours.size(); i++) {

        		//weed out small ones ones first
        		if (contours[i].size() > minobjsize) {

        			br = boundingRect(contours[i]);
        			if (sum(Mat(*N,br))[0]/abs(contourArea(contours[i])) > maxSal) {
        				maxSal = sum(Mat(*N,br))[0]/abs(contourArea(contours[i]));
        				iBlob = i;
        			}

        		}
        	}

        	//if we had an icub cam image, draw on it and send off
        	if (pImgIn) {

        		imgOut.copy(*pImgIn);
        		Mat icubImg((IplImage *)imgOut.getIplImage(), false);
        		if (iBlob >= 0) {
        			drawContours(icubImg, contours, iBlob, Scalar(255,0,0), -1, 8, hierarchy, 0);
        		}
        		//drawContours(icubImg, contours, iBlob, Scalar(255, 0, 0), -1, 8);
        		portImgOut->write();
        		delete cSal;

        	}
        	//if we just got a color map, return all the things to yarp
        	else {

        		portColorIn->release(cSal);
        		portImgOut->unprepare();

        	}

        	//publish the most salient blobs as nx2 matrix of pts
        	if (iBlob >= 0) {
				yarp::sig::Matrix &salCont = portObjOut->prepare();
				salCont.resize(contours[iBlob].size(),2);
				salCont.zero();
				for (int i = salCont.rows(); i > 0; i--) {
					salCont(i-1,0) = contours[iBlob].back().x;
					salCont(i-1,1) = contours[iBlob].back().y;
					contours[iBlob].pop_back();
				}
				portObjOut->write();
        	}

        	delete M;
        	delete N;
        	delete P;

		}



    }

    virtual void threadRelease()
    {

    	portImgIn->interrupt();
    	portImgOut->interrupt();
    	portColorIn->interrupt();
    	portGazeIn->interrupt();
    	portObjOut->interrupt();

    	cfilter->close();

    	delete cfilter;
    	delete portImgIn;
    	delete portImgOut;
    	delete portColorIn;
    	delete portGazeIn;
    	delete portObjOut;

    }

};

class salAggModule: public RFModule
{

protected:

    salAggThread *thr;

public:
    salAggModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new salAggThread(rf);
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

    salAggModule mod;

    return mod.runModule(rf);
}
