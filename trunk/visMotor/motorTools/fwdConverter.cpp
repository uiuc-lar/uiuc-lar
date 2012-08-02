/*
 *  fwdConverter.cpp
 *
 * 	Logan Niehaus
 * 	12/17/10
 * 	Wrapper module for the forward kinematics of the icub's arms
 *
 * Module Args:
 * 	input 		-- input port name
 * 	output	  	-- output port name
 * 	arm			-- which arm
 *
 * PORTS: (w/ example ports)
 *	Inputs: /arm/state:o   		(Bottle provided by iCubInterface or iCub_SIM)
 *	Outputs: /kin/cartesian		(Bottle of x,y,z and euler angles XYZ)
 */

//yarp network
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/os/Stamp.h>

//icub includes
#include <iCub/iKin/iKinFwd.h>

//misc
#include <string.h>

//defines
#define PI 3.14159

//namespaces
using namespace iCub::iKin;
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class SamplingPort : public BufferedPort<Bottle>
{

protected:

	//data
	iCubArm			  *kinArm;

	//publishing port
	Port *oPort;

public:

	//small bit of initializing
	SamplingPort(iCubArm *& arm_, const char * oName):BufferedPort<Bottle>(){

		kinArm = arm_;

		//oPort = new BufferedPort<Bottle>;
		oPort = new Port;
		oPort->open(oName);

	}

	//callback for incoming joint data
	virtual void onRead(Bottle& b) {

		Bottle converted;
		Stamp tStamp;
		Vector angles(7);
		Vector endEffPos;

		//get the timestamp
		BufferedPort<Bottle>::getEnvelope(tStamp);

		//unpack
		for (int i = 0; i < 7; i++) {
			angles[i] = b.get(i).asDouble()*PI/180.0;
		}

		//convert
		endEffPos = kinArm->EndEffPose(angles,false);

		//repack
		for (int i = 0; i < endEffPos.size(); i++) {
			converted.add(endEffPos[i]);
		}

		//publish
		oPort->setEnvelope(tStamp);
		oPort->write(converted);

	}

	virtual void close() {

		oPort->close();
		delete oPort;

	}

};


class SamplingModule : public RFModule
{

protected:

	//running params
	string armName;		//which arm of the robot's are we looking at
	string recvPort;	//which robot are we looking at
	string sendPort;

	//ports
	SamplingPort * iPort;

	//tools
	iCubArm			  *kinArm;

	//safety
	bool safeInit;

public:

	virtual bool configure(ResourceFinder &rf)
	{
		Time::turboBoost();

		//get options
		recvPort = rf.check("input",Value("/jointsIn"),"input port").asString();
		armName = rf.check("arm",Value("right"),"arm to process").asString();
		sendPort = rf.check("output",Value("/cartesianOut"),"output port").asString();

		//make arm
		kinArm = new iCubArm(armName);

		//create and open ports
		iPort = new SamplingPort(kinArm,sendPort.c_str());
		iPort->useCallback();  // register callback
		iPort->open(recvPort.c_str());

		return true;
	}

	virtual bool close()
	{

		iPort->close();

		delete kinArm;
		delete iPort;
		//delete oPort;

		return true;
	}

	virtual double getPeriod()    { return 1.0;  }
	virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
	// we need to initialize the drivers list
	Network yarp;
	if (!yarp.checkNetwork())
		return -1;

	SamplingModule mod;

	ResourceFinder rf;

	rf.configure("ICUB_ROOT", argc, argv);

	return mod.runModule(rf);
}

