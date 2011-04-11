/*
 * ruddy filter based blob detector
 * -only works under assumption that faces are pretty big blobs
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
#include <iCub/vis/RuddySalience.h>
#include <iCub/vis/BlobResult.h>

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

#define MINEYESIZE 10

class skinDetThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *portImgIn;
	BufferedPort<ImageOf<PixelRgb> > *portImgOut;
	BufferedPort<yarp::sig::Vector>			 *portFeatOut;
	BufferedPort<yarp::sig::Vector>			 *portHeadBox;

	RuddySalience * filter;

public:

	skinDetThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
    { }

    virtual bool threadInit()
    {

        name=rf.check("name",Value("myFaceDetector")).asString().c_str();

        portImgIn=new BufferedPort<ImageOf<PixelRgb> >;
        string portInName="/"+name+"/img:i";
        portImgIn->open(portInName.c_str());

        portImgOut=new BufferedPort<ImageOf<PixelRgb> >;
        string portOutName="/"+name+"/img:o";
        portImgOut->open(portOutName.c_str());

        portFeatOut=new BufferedPort<yarp::sig::Vector>;
        string portFeatName="/"+name+"/feat:o";
        portFeatOut->open(portFeatName.c_str());

        portHeadBox=new BufferedPort<yarp::sig::Vector>;
        string portHeadName="/"+name+"/head:o";
        portHeadBox->open(portHeadName.c_str());

        filter = new RuddySalience;
        filter->open(rf);


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
        	ImageOf<PixelRgb> &imgOut= portImgOut->prepare();
        	yarp::sig::Vector &featureVec = portFeatOut->prepare();
        	yarp::sig::Vector &headBoxVec = portHeadBox->prepare();
        	imgOut.copy(*pImgIn);

        	//apply ruddy salience filter
        	filter->apply(*pImgIn, *pDest, *pSal);

        	//threshold to get skin colored objects
        	Mat * T = new Mat(pSal->height(), pSal->width(), CV_32F, (void *)pSal->getRawImage());
        	Mat * C = new Mat(pSal->height(), pSal->width(), CV_32F);
        	threshold(*T, *C, 30.0, 255.0, CV_THRESH_BINARY);

        	//find the list of skill colored blobs using connected components
        	vector<vector<Point> > contours;
        	vector<Vec4i> hierarchy;
        	Mat X;
            C->convertTo(X,CV_8UC1);
            findContours(X, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

            //will assume face is located on the largest such blob
            double maxSize = 0.0;
            int biggestblob = -1;
            for (int i = 0; i < contours.size(); i++) {
            	if (abs(contourArea(contours[i])) > maxSize) {
            		maxSize = abs(contourArea(contours[i]));
            		biggestblob = i;
            	}
            }

            IplImage * mainImgIpl = (IplImage *)imgOut.getIplImage();
            Mat mainImg(mainImgIpl, true);

            //draw a box around the face zone
            Rect faceRect = boundingRect(contours[biggestblob]);

        	//select the face part of the image
            Mat * subFace = new Mat(*T, faceRect);


        	//with face bounding box, use diff thresholds
            threshold(*subFace, *subFace, 20.0, 255.0, CV_THRESH_BINARY);
            subFace->convertTo(X,CV_8UC1);
            hierarchy.clear();
            contours.clear();
            findContours(X, contours, hierarchy, CV_RETR_LIST, CHAIN_APPROX_NONE, Point(faceRect.x,faceRect.y));

            //again look for the largest contour
            maxSize = 0.0;
            biggestblob = -1;
            for (int i = 0; i < contours.size(); i++) {
            	if (abs(contourArea(contours[i])) > maxSize) {
            		maxSize = abs(contourArea(contours[i]));
            		biggestblob = i;
            	}
            }

            //for this contour, find its two largest internal contours above a min size, call these the eyes
            //drawContours(mainImg, contours, biggestblob, Scalar(255, 0, 0), 1, 8, hierarchy, 0);
            deque<int> eyeCan(2);
            deque<double> canVals(2);
            for (int i = 0; i < contours.size(); i++) {
            	if (i != biggestblob) {
            		Moments m = moments(contours[i]);
            		if ((pointPolygonTest(contours[biggestblob], Point2f(m.m10/m.m00,m.m01/m.m00), false) > 0) &&
            				abs(contourArea(contours[i])) > MINEYESIZE) {
            			if (abs(contourArea(contours[i])) > canVals.front()) {
            				canVals.push_front(abs(contourArea(contours[i])));
            				canVals.pop_back();
            				eyeCan.push_front(i);
            				eyeCan.pop_back();
            			}
            			else if (abs(contourArea(contours[i])) > canVals.back()) {
            				canVals.pop_back();
            				canVals.push_back(abs(contourArea(contours[i])));
            				eyeCan.pop_back();
            				eyeCan.push_back(i);
            			}
            		}
            	}
            }

            //calculate the center position of the eyes
            Point * eyeMidPoint = NULL;
            Moments m = moments(contours[biggestblob]);
            Moments n = moments(contours[biggestblob]);
            if (canVals.front() > 0) {
            	m = moments(contours[eyeCan.front()]);
            	n = moments(contours[eyeCan.front()]);
            }
            if (canVals.back() > 0) {
            	n = moments(contours[eyeCan.back()]);
            }
            eyeMidPoint = new Point((m.m10/m.m00+n.m10/n.m00)/2, (m.m01/m.m00+n.m01/n.m00)/2);
            circle(mainImg, *eyeMidPoint, 1, Scalar(0, 255, 0), 2);

            //representation is the ratio of area in bb to left of center, to that right of center
            double ratio = (double)(eyeMidPoint->x-(faceRect.x+faceRect.width/2.0))/(double)(faceRect.width/2.0);

            //annotate the output image (debugging mostly)
			char labelc[100];
            sprintf(labelc,"r = %f", ratio);
            string label(labelc);
            rectangle(mainImg, Point(faceRect.x,faceRect.y), Point(faceRect.x+faceRect.width,faceRect.y+faceRect.height), Scalar(255,0,0));
            putText(mainImg, label, Point(faceRect.x,faceRect.y+faceRect.height), FONT_HERSHEY_SIMPLEX, 0.2, Scalar(0, 0, 255));

            //write out the image, features, and head position
            featureVec.clear();
            headBoxVec.clear();
            featureVec.push_back(ratio);
            headBoxVec.push_back(faceRect.x);
            headBoxVec.push_back(faceRect.y);
            headBoxVec.push_back(faceRect.width);
            headBoxVec.push_back(faceRect.height);
            IplImage outImg = mainImg;
            imgOut.wrapIplImage(&outImg);

            portImgOut->write();
        	portFeatOut->write();
        	portHeadBox->write();

            delete T;
            delete C;
            delete pDest;
            delete pSal;

        }
    }

    virtual void threadRelease()
    {

    	portImgIn->interrupt();
    	portImgOut->interrupt();
    	portFeatOut->interrupt();
    	portHeadBox->interrupt();
    	portImgIn->close();
    	portImgOut->close();
    	portFeatOut->close();
    	portHeadBox->close();

    	filter->close();

    	delete portImgIn;
    	delete portImgOut;
    	delete portFeatOut;
    	delete portHeadBox;
    	delete filter;

    }

};

class skinDetModule: public RFModule
{
protected:
    skinDetThread *thr;

public:
    skinDetModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new skinDetThread(rf);
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

    skinDetModule mod;

    return mod.runModule(rf);
}



