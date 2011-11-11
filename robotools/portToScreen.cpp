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
 *  portToScreen.cpp
 *
 * 	Logan Niehaus
 * 	7/19/11
 * 	catches vectors coming in and echos them to the screen
 *
 * Module Args:
 *
 *	downsample -- number of samples to skip before writing each time (D 1, no downsampling)
 *	scale	-- number to scale all outputs by (D 1.0)
 *	name	-- receiver port name for module (D /pts)
 *
 * PORTS:
 *	inputs: /portToScreen/data:in -- yarp vectors of any length to dump to stdout
 */

//yarp network
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

//misc
#include <stdio.h>
#include <stdlib.h>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

class VLogPort : public BufferedPort<yarp::sig::Vector> {

protected:

	int count;
	int ds;
	float scale;

public:

	VLogPort(int _ds, float _scale) : count(0), ds(_ds), scale(_scale) { }

	//callback for incoming sequences
	virtual void onRead(yarp::sig::Vector& v) {

		if ((count % ds) == 0) {
			for (int i = 0; i < v.size()-1; i++) {
				printf("%f,\t", v[i]*scale);
			}
			printf("%f\n", v[v.size()-1]*scale);
		}
		count++;

	}

};

class DummyModule : public RFModule {

protected:

	VLogPort * vlp;
	string name;
	int ds;
	float scale;

public:

	virtual bool configure(ResourceFinder &rf) { 


		name = rf.check("name",Value("/pts")).asString().c_str();
		ds = rf.check("ds", Value(1)).asInt();
		scale = rf.check("scale", Value(1.0)).asDouble();

		vlp=new VLogPort(ds, scale);
		vlp->open(name.c_str());
		vlp->useCallback();

		return true; 

	}	
	virtual double getPeriod() { return 10.0; }
	virtual bool updateModule() { return true; }
	virtual bool close() { 
	
		vlp->interrupt();
		vlp->close();
		
		delete vlp;
		
		return true; 
	}

};


int main(int argc, char *argv[])
{
	// we need to initialize the drivers list
	Network yarp;
	if (!yarp.checkNetwork())
		return -1;

	ResourceFinder rf;

	DummyModule mod;

	rf.configure("ICUB_ROOT", argc, argv);

	return mod.runModule(rf);

}

