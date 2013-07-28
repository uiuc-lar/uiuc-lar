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
 *  actionPerformer.cpp
 *
 * 	Logan Niehaus
 * 	2/13/11
 * 	network front-end to the cartesian interface. takes a bottled 'action'
 * 	from the input port and makes the robot (simulator only at time being) perform it.
 * 	actions are all for a given arm, expressed in cartesian coordinates.
 *
 * 	(note: this module is was built from the icub example code written by ugo pattacini.
 * 		credit for the structure and low level stuff goes to him.)
 *
 * Module Args:
 * 	input 		-- input port name (D: /perform:i)
 * 	arm			-- which arm to use ('left' or 'right')
 *	threadrate  -- thread update rate (D: 0.05 s)
 *	maxpitch    -- maximum allowable torso pitch for solver (D: 30.0 deg)
 *	decimate 	-- decimation of incoming data (D: 2)
 *	trajtime	-- trajectory time for cart. controller (see yarp doc.) (D: 0.4s)
 *
 *	note about args: threadrate,decimate,and trajtime all interact with the solver, and will
 *		all affect solver performance. read docs before changing
 *
 * PORTS:
 *	Inputs: /perform:i	(bottle of bottles. first three elements of each bottle are considered xyz targets)
 *		TODO: make this not a bottle of bottles
 */

//yarp network
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

//yarp devices
#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>

//misc
#include <gsl/gsl_math.h>
#include <stdio.h>
#include <string>
#include <deque>


YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


class CtrlThread: public RateThread
{
protected:


	//control variables/structures
    PolyDriver        *client;
    ICartesianControl *arm;

    string fname;
    FILE	* jointsFile;
    deque<double *> action;
    int		genCount;

    Vector xd;
    Vector od;

    double t;
    double t0;
    double t1;

    bool actionDone;

    //input params
    double maxPitch;
    double trajTime;
    int decimate;
    bool armInUse;

public:

    CtrlThread(bool arm_, double period, double maxPitch_, int decimate_, double trajTime_)
    : armInUse(arm_), maxPitch(maxPitch_), decimate(decimate_), trajTime(trajTime_),  RateThread(int(period*1000.0)) { }

    //this is the way to pass an action in
    //warning: this will clear any old action data
    bool loadAction(Bottle * b) {

    	int count = 0;

    	action.clear();

    	for (int i = 0; i < b->size(); i++) {

    		Bottle * c = b->get(i).asList();
    		//assume cartesian XYZ
    		double * pos = new double[3];
    		for (int j = 0; j < 3; j++) {
    			pos[j] = c->get(j).asDouble();
    		}
    		if (count % decimate == 0) {
    			action.push_back(pos);
    		}
    		count++;

    	}

    	actionDone = false;
    	return true;

    }

    virtual bool threadInit()
    {
        //open up client to connect to the cartesian server
    	//assume simulator and necessary solvers are running
        Property option("(device cartesiancontrollerclient)");
        if (armInUse) {
			option.put("remote","/icubSim/cartesianController/right_arm");
			option.put("local","/cartesian_client/right_arm");
        }
        else {
			option.put("remote","/icubSim/cartesianController/left_arm");
			option.put("local","/cartesian_client/left_arm");
        }

    	actionDone = true;

        client=new PolyDriver;
        if (!client->open(option))
        {
            delete client;
            return false;
        }

        // open the view
        client->view(arm);

        // set trajectory time
        arm->setTrajTime(trajTime);	//slow for safety

        // get the torso dofs
        Vector newDof, curDof;
        arm->getDOF(curDof);
        newDof=curDof;

        //disable all torso movement
        newDof[0]=0;
        newDof[1]=0;
        newDof[2]=0;
        newDof[7]=0;
        newDof[8]=0;
        newDof[9]=0;

        // impose some restriction on the torso pitch
        limitTorsoPitch();

        // send the request for dofs reconfiguration
        arm->setDOF(newDof,curDof);

        //set up position and orientation vectors
        xd.resize(3);
        od.resize(4);

        return true;

    }

    virtual void afterStart(bool s)
    {
        if (s)
            fprintf(stdout,"Thread started successfully\n");
        else
            fprintf(stdout,"Thread did not start\n");

        t=t0=t1=Time::now();
    }

    virtual void run()
    {
        t=Time::now();

        if (getNextTarget()) {

            //drive to the target
            arm->goToPosition(xd);

        }

    }

    virtual void threadRelease()
    {

        arm->stopControl();
        delete client;

    }

    bool getNextTarget()
    {

    	//give the next sample
    	if (!action.empty()) {

    		double * temp = action.front();
    		xd[0] = temp[0];
    		xd[1] = temp[1];
    		xd[2] = temp[2];
    		action.pop_front();
    		delete temp;
    		return true;

    	}

    	//if we are finished with new positions, wait until robot catches up
    	else {

    		bool f;
    		//after the action is finished, wait until the motion is done
    		arm->checkMotionDone(&f);
    		if (f) {
    			actionDone = true;
    		}
    		return false;
    	}

    }

    double norm(const Vector &v)
    {
        return sqrt(dot(v,v));
    }

    void limitTorsoPitch()
    {
        int axis=0; // pitch joint
        double min, max;

		//set some limits on torso pitch
        arm->getLimits(axis,&min,&max);
        arm->setLimits(axis,min,maxPitch);
    }

    bool isDone() {

    	return actionDone;

    }

};



class CtrlModule: public RFModule
{
protected:

	//control
    CtrlThread *thr;
    ConstString fname;
    BufferedPort<Bottle> * inPort;

    //parameters/arguments
    string portName;
    string armName;
    bool whichArm;
    double threadRate;
    double maxPitch;
    double trajTime;
    int decimate;

public:

    void handleArgs(ResourceFinder &rf) {

    	portName = rf.check("input",Value("/perform:i"),"input port name").asString();
    	threadRate = rf.check("threadrate",Value(0.05),"thread rate").asDouble();
    	maxPitch = rf.check("maxpitch",Value(30.0),"max torso pitch").asDouble();
    	decimate = rf.check("decimate",Value(2),"input decimation factor").asInt();
    	trajTime = rf.check("trajtime",Value(0.4),"solver trajectory time").asDouble();

    	//get which arm to use. default to left if they didnt pass in left or right
    	armName = rf.check("arm", Value("left"),"arm name").asString();
    	if (armName == "right") {
    		whichArm = true;
    	}
    	else {
    		whichArm = false;
    	}

    }

    virtual bool configure(ResourceFinder &rf)
    {

    	handleArgs(rf);

    	inPort = new BufferedPort<Bottle>;
    	inPort->open(portName.c_str());
    	inPort->setStrict();

        Time::turboBoost();

        thr=new CtrlThread(whichArm, threadRate, maxPitch, decimate, trajTime);

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
        inPort->close();
        delete thr;
        delete inPort;

        return true;
    }

    virtual double getPeriod()    { return 0.5;  }
    virtual bool   updateModule() {

    	//while the thread is processing actions, lock out
    	if (thr->isDone()) {

    		//if no action is currently queued, ask port for a new one
    		Bottle *b = inPort->read(false);
    		if (b != NULL) {
        		printf("action length %d received\n",b->size());
    			thr->loadAction(b);
    		}

    	}

    	return true;

    }

};



int main(int argc, char *argv[])
{
    // we need to initialize the drivers list
    YARP_REGISTER_DEVICES(icubmod)

    Network yarp;
    if (!yarp.checkNetwork())
        return -1;

    CtrlModule mod;

    ResourceFinder rf;

    rf.configure("ICUB_ROOT", argc, argv);

    return mod.runModule(rf);
}




