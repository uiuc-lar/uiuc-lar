/*
 *  dataPumper.cpp
 *
 * 	Logan Niehaus
 * 	2/13/11 (older)
 * 	super simple program that reads in a CSV file, writes it to a target port as a
 * 	bottle of bottles, and then quits.
 *
 * Module Args:
 *
 *	target	-- target port name, writes the data out to here
 *	file	-- csv file that the data will be read from
 *	type	-- 'vector', 'bottle' or 'sequence'. sequence reads the whole file in and
 *				writes to the port as a nested bottle of bottles. vector default
 *	rate	-- rate at which to write out samples (if type set to vector or bottle). def 10ms
 *
 *
 * PORTS:
 *	Outputs: /pumpOut (just an auxiliary port for making the connection to the target
 */

//yarp network
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

//misc
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

#define LINE_MAX_LEN 2048

int main(int argc, char *argv[])
{
    // we need to initialize the drivers list
    Network yarp;
    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;

    rf.configure("ICUB_ROOT", argc, argv);

    if (!rf.check("target")) {
    	printf("please specify target port name to send data to\n");
    	return false;
    }
    if (!rf.check("file")) {
    	printf("please specify a data file to use\n");
    	return false;
    }

    //open the output port and connect it to the target port
    string pname = "/pumpOut";
    string tname = rf.find("target").asString().c_str();
    double rate = rf.check("rate", Value(0.01)).asDouble();

    int type = -1;
    string typestr = rf.check("type",Value("vector")).asString().c_str();
    if (typestr == "vector")
    	type = -1;
    else if (typestr == "bottle")
    	type = 0;
    else if (typestr == "sequence")
    	type = 1;
    else
    	type = -1;

    string dlmchar = ",";
    if (rf.check("tab")) {
    	dlmchar = "\t";
    }
    if (rf.check("csv")) {
    	dlmchar = ",";
    }
    Port oPort;
    oPort.open(pname.c_str());
    Network::connect(pname.c_str(),tname.c_str());

    //read in the file
    string fname = rf.find("file").asString().c_str();
	FILE *fp = fopen(fname.c_str(),"r");
	char line[LINE_MAX_LEN];
	Bottle seq;

	//read the file into the bottle
	while (!feof(fp)) {

		if (type != 1) {
			Time::delay(rate);
		}

		fscanf(fp,"%s",line);

		char * entry;
		entry = strtok(line,dlmchar.c_str());

		if (type == 1) {
			Bottle &sqbot = seq.addList();
			while (entry != NULL) {
				sqbot.add(atof(entry));
				entry = strtok(NULL,dlmchar.c_str());
			}
		} else {
			Vector sample(0);
			Bottle sbot;
			while (entry != NULL) {
				if (type == -1)
					sample.push_back(atof(entry));
				else if (type == 0)
					sbot.add(atof(entry));
				entry = strtok(NULL,dlmchar.c_str());
			}
			if (type == -1)
				oPort.write(sample);
			else if (type == 0)
				oPort.write(sbot);
		}

	}
	fclose(fp);

	if (type == 1) {
		seq.pop();	//last sample in sequence is usually a garbage one
		oPort.write(seq);
	}

	oPort.close();

    return true;

}
