/*
 *  gazeLearner.cpp
 *
 * 	Logan Niehaus
 * 	5/2/11
 * 	learning module for head position to world-coordinates map using nn (unfinished)
 *
 * inputs: takes vectors of the current gaze estimate, the head position, and location of detected objects
 * params: neuron running params: range of inputs, range of outputs, number of neurons
 * 			covariance matrices for input/output smearing
 * 			learning rate, leakage rate, etc...
 * outputs: should probably set up an RPC port so that modules can ping it for various settings
 * TODOs: Basically does nothing right now, just gathers the relevant data and dumps it to stdout
 * 		In the future, this will be able to train online, and do things like negative reinforcement?
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


class gazeLearnerThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<Bottle>			 	*portActLoc;
	BufferedPort<yarp::sig::Vector>		*portFeatVec;
	BufferedPort<yarp::sig::Vector>		*portHeadBox;


public:

	gazeLearnerThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
    { }

    virtual bool threadInit()
    {

        name=rf.check("name",Value("myGazeLearner")).asString().c_str();

        portActLoc=new BufferedPort<Bottle>;
        string portActName="/"+name+"/loc:i";
        portActLoc->open(portActName.c_str());

        portFeatVec=new BufferedPort<yarp::sig::Vector>;
        string portFeatName="/"+name+"/feat:i";
        portFeatVec->open(portFeatName.c_str());

        portHeadBox=new BufferedPort<yarp::sig::Vector>;
        string portHeadName="/"+name+"/head:i";
        portHeadBox->open(portHeadName.c_str());


    	return true;
    }

    virtual void run()
    {

        // get inputs
        Bottle *motionDetect = portActLoc->read(false);

        //if motion has been detected, look for the corresponding gaze info
        if (motionDetect)
        {

        	yarp::sig::Vector *featVec = portFeatVec->read(true);
        	yarp::sig::Vector *headBox = portHeadBox->read(true);

        	if (featVec && headBox) {

        		printf("%d %d %d, %d, %f, %f\n", (int)((*headBox)[0]+(*headBox)[2]/2), (int)((*headBox)[1]+(*headBox)[3]/2),
        				motionDetect->get(0).asInt(), motionDetect->get(1).asInt(), (*featVec)[0], (*featVec)[1]);

        	}


        }
    }

    virtual void threadRelease()
    {


    	portActLoc->interrupt();
    	portFeatVec->interrupt();
    	portHeadBox->interrupt();
    	portActLoc->close();
    	portFeatVec->close();
    	portHeadBox->close();


    	delete portActLoc;
    	delete portFeatVec;
    	delete portHeadBox;

    }

};

class gazeLearnerModule: public RFModule
{
protected:
    gazeLearnerThread *thr;

public:
    gazeLearnerModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new gazeLearnerThread(rf);
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

    gazeLearnerModule mod;

    return mod.runModule(rf);
}



