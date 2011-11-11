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
 *  lexiconLearner.cpp
 *
 * 	Logan Niehaus
 * 	3/9/11
 *
 *  Module just acts as a gate between the input and output based on key presses
 *
 * Module Args:
 * 	input - receiving port name
 *	output - output port name
 */

//yarp network
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

//misc
#include <string>
#include <math.h>
#include <deque>
#include <iostream>

//namespaces
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

class DataBuffer : public deque<Bottle> {

private:

	Semaphore mutex;

public:

	void lock()   { mutex.wait(); }
	void unlock() { mutex.post(); }

};

class GatingPort : public BufferedPort<Bottle> {

protected:

	//data
	DataBuffer &buffer;		//buffer shared with main thread

public:

	GatingPort(DataBuffer &buf) : buffer(buf) { }

	//callback for incoming position data
	virtual void onRead(Bottle& b) {

		//behavior at this point is just to trash old data every time a new sample comes in
		buffer.lock();
		buffer.clear();
		buffer.push_back(b);
		buffer.unlock();

		printf("data received, waiting on keypress...\n");

	}

};

class GatingModule : public RFModule {

protected:

	string recvPort;
	string sendPort;

	DataBuffer buf;				//main data buffer
	GatingPort *inPort;
	BufferedPort<Bottle> *outPort;

public:

	virtual bool configure(ResourceFinder &rf)
	{

		recvPort = rf.check("input",Value("/gate:i"),"input port").asString();
		sendPort = rf.check("output",Value("/gate:o"),"output port").asString();

		//set up ports
		inPort = new GatingPort(buf);
		outPort = new BufferedPort<Bottle>;
		outPort->open(sendPort.c_str());

		//set callback
		inPort->useCallback();
		inPort->open(recvPort.c_str());

		return true;

	}

	virtual bool close() {

		inPort->close();
		outPort->close();

		return true;

	}

	virtual double getPeriod() { return 0.5; }
	virtual bool updateModule() {

		char tmp;
		Bottle tbot;

		//keep blocking for a keypress
		printf("waiting for keypress ...\n");
		tmp = getchar();
		if (buf.size() > 0) {

			Bottle &reflect = outPort->prepare();
			reflect.clear();

			printf("data was present on port, passing...\n");
			buf.lock();
			tbot = buf.front();
			buf.clear();
			buf.unlock();

			reflect.append(tbot);
			outPort->write();

			printf("bottle contained: %s\n", tbot.toString().c_str());


		} else {

			printf("no data available, no action ... \n");

		}


		return true;

	}

};



int main(int argc, char *argv[])
{
	// we need to initialize the drivers list
	Network yarp;
	if (!yarp.checkNetwork())
		return -1;

	GatingModule mod;

	ResourceFinder rf;

	rf.configure("ICUB_ROOT", argc, argv);

	return mod.runModule(rf);
}


