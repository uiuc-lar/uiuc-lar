/*
 *  stereoBallLoc.cpp
 *
 * 	Logan Niehaus
 * 	5/12/11
 * 	module for detecting the image coordinate location of a ball for use w/ ikingazectrl module
 * 		first order goal here is just to find the blue ball
 *
 *
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


class strBallLocThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *portImgL;
	BufferedPort<ImageOf<PixelRgb> > *portImgR;
	BufferedPort<ImageOf<PixelRgb> > *portImgD;
	BufferedPort<Bottle>			 *portDetLoc;

	RpcClient						 *portGazeRpc;

	double tol;	//stopping tolerance
	bool headstopped;

public:

	strBallLocThread(ResourceFinder &_rf) : RateThread(20), rf(_rf)
    { }

    virtual bool threadInit()
    {

        name=rf.check("name",Value("stereoBallLoc")).asString().c_str();
        tol=rf.check("tol",Value(5.0)).asDouble();

        portImgL=new BufferedPort<ImageOf<PixelRgb> >;
        string portImlName="/"+name+"/img:l";
        portImgL->open(portImlName.c_str());

        portImgR=new BufferedPort<ImageOf<PixelRgb> >;
        string portImrName="/"+name+"/img:r";
        portImgR->open(portImrName.c_str());

        portImgD=new BufferedPort<ImageOf<PixelRgb> >;
        string portImdName="/"+name+"/img:o";
        portImgD->open(portImdName.c_str());

        portDetLoc=new BufferedPort<Bottle>;
        string portLocName="/"+name+"/loc:o";
        portDetLoc->open(portLocName.c_str());

        portGazeRpc=new RpcClient;
        string portRpcName="/"+name+"/rpc:o";
        portGazeRpc->open(portRpcName.c_str());
        if (!portGazeRpc->addOutput("/iKinGazeCtrl/head/rpc")) {

        	printf("could not connect to gaze controller rpc, not good\n");

        } else {

        	printf("connected to gaze controller rpc\n");
        	Bottle neckLimits, reply;
        	neckLimits.addString("bind");
        	neckLimits.addString("pitch");
        	neckLimits.add(-30);
        	neckLimits.add(30);
        	portGazeRpc->write(neckLimits,reply);
        	neckLimits.clear();
        	neckLimits.addString("bind");
        	neckLimits.addString("yaw");
        	neckLimits.add(-25);
        	neckLimits.add(25);
        	portGazeRpc->write(neckLimits,reply);
        	neckLimits.clear();
        	neckLimits.addString("bind");
        	neckLimits.addString("roll");
        	neckLimits.add(-10);
        	neckLimits.add(10);
        	portGazeRpc->write(neckLimits,reply);

        }

        headstopped = true;

    	return true;

    }

    virtual void run()
    {

        // get both input images
        ImageOf<PixelRgb> *pImgL=portImgL->read(false);
        ImageOf<PixelRgb> *pImgR=portImgR->read(false);
        ImageOf<PixelRgb> *tImg;

        ImageOf<PixelFloat> *pImgBRL;
        ImageOf<PixelFloat> *pImgBRR;
        ImageOf<PixelFloat> *oImg;

        //if we have both images
        if (pImgL && pImgR)
        {

        	//set up processing
    		Bottle &loc = portDetLoc->prepare();
            pImgBRL = new ImageOf<PixelFloat>;
            pImgBRR = new ImageOf<PixelFloat>;
            pImgBRL->resize(*pImgL);
            pImgBRR->resize(*pImgR);
            Mat * T, * X;
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			int biggestBlob;
			loc.clear();


    		ImageOf<PixelRgb> &imgOut= portImgD->prepare();

        	//pull out the blueness channel
        	PixelRgb pxl;
        	float lum, rn, bn, gn, val;
    		for (int lr = 0; lr < 2; lr++) {

    			if (!lr) {
    				tImg = pImgL;
    				oImg = pImgBRL;
    			}
    			else {
    				tImg = pImgR;
    				oImg = pImgBRR;
    			}

                for (int x = 0; x < tImg->width(); x++) {
                	for (int y = 0; y < tImg->height(); y++) {

                		//normalize brightness (above a given threshold)
                		pxl = tImg->pixel(x,y);
                		lum = (float)(pxl.r+pxl.g+pxl.b);
                		rn = 255.0F * pxl.r/lum;
                		gn = 255.0F * pxl.g/lum;
                		bn = 255.0F * pxl.b/lum;

                		//get the blueness
                		//val = (bn - (rn+gn)/2);
                		
				val = (rn+gn)/2.0 - bn;
				if (val > 255.0) {
                			val = 255.0;
                		}
                		if (val < 0.0) {
                			val = 0.0;
                		}
                		oImg->pixel(x,y) = val;

                	}
                }

                //threshold to find blue blobs
    			T = new Mat(oImg->height(), oImg->width(), CV_32F, (void *)oImg->getRawImage());
    			threshold(*T, *T, 15.0, 255.0, CV_THRESH_BINARY);

        		imgOut.copy(*oImg);

    			X = new Mat(oImg->height(), oImg->width(), CV_8UC1);
    			T->convertTo(*X,CV_8UC1);
    			findContours(*X, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    			//find largest blob and its moment
    			double maxSize = 0.0;
    			biggestBlob = -1;
    			double xloc, yloc;
    			for (int i = 0; i < contours.size(); i++) {
    				if (abs(contourArea(Mat(contours[i])))/arcLength(Mat(contours[i]), true) > maxSize &&
    						abs(contourArea(Mat(contours[i])))/arcLength(Mat(contours[i]), true) > 2.0) {
    					maxSize = abs(contourArea(Mat(contours[i])))/arcLength(Mat(contours[i]), true);
    					biggestBlob = i;
    				}
    			}
    			if (biggestBlob >= 0) {

    				//if a valid object was found, add its location
    				Moments m = moments(Mat(contours[biggestBlob]));
    				xloc = m.m10/m.m00;
    				yloc = m.m01/m.m00;
    				loc.add(xloc);
    				loc.add(yloc);

    			}

    			delete T;
    			delete X;

    		}

    		//if a blob in both images was detected, go to it
    		if (loc.size() == 4) {

    			double du, dv;

    			//check to see if within acceptable tolerance
    			du = (loc.get(0).asDouble() - 160 + loc.get(2).asDouble() -160)/2.0;
    			dv = (loc.get(1).asDouble() - 120 + loc.get(3).asDouble() -120)/2.0;
    			if (sqrt(du*du+dv*dv) < tol && !headstopped) {

    				Bottle stopCmd, reply;
    				stopCmd.addString("stop");
    				if (portGazeRpc->getOutputCount() > 0) {

        				printf("target reached, stopping\n");
    					portGazeRpc->write(stopCmd,reply);
    					headstopped = true;

    				} else {

        				printf("target reached, but could not contact gaze controller\n");

    				}
    				portDetLoc->unprepare();

    			} else {
        			printf("current mean error: %f\n", sqrt(du*du+dv*dv));
        			headstopped = false;
					portDetLoc->write();
    			}
				draw::addCrossHair(imgOut, PixelRgb(0, 255, 0), loc.get(0).asInt(), loc.get(1).asInt(), 10);
				draw::addCrossHair(imgOut, PixelRgb(0, 255, 0), loc.get(2).asInt(), loc.get(3).asInt(), 10);

    		} else {

    			portDetLoc->unprepare();

    		}

    		//send out, cleanup
    		portImgD->write();


    		delete pImgBRL;
    		delete pImgBRR;


        }
    }

    virtual void threadRelease()
    {

    	portImgL->interrupt();
    	portImgR->interrupt();
    	portImgD->interrupt();
    	portDetLoc->interrupt();
    	portGazeRpc->interrupt();

    	portImgL->close();
    	portImgR->close();
    	portImgD->close();
    	portDetLoc->close();
    	portGazeRpc->close();

    	delete portImgL;
    	delete portImgR;
    	delete portImgD;
    	delete portDetLoc;
    	delete portGazeRpc;

    }

};

class strBallLocModule: public RFModule
{
protected:
    strBallLocThread *thr;

public:
    strBallLocModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new strBallLocThread(rf);
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

    strBallLocModule mod;

    return mod.runModule(rf);
}



