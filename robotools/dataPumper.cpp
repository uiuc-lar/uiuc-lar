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

//misc
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;

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
    ConstString pname("/pumpOut");
    ConstString tname = rf.find("target").asString();
    Port oPort;
    oPort.open(pname.c_str());
    Network::connect(pname.c_str(),tname.c_str());

    //read in the file
    ConstString fname = rf.find("file").asString();
	FILE *fp = fopen(fname.c_str(),"r");
	char line[LINE_MAX_LEN];
	Bottle action;

	//read the file into the bottle
	while (!feof(fp)) {
		fscanf(fp,"%s",line);
		Bottle & sample = action.addList();
		char * entry;
		entry = strtok(line,",");
		while (entry != NULL) {
			sample.addDouble(atof(entry));
			entry = strtok(NULL,",");
		}
	}
	fclose(fp);

	//last point could be potentially garbage?
	action.pop();

	//send it out to the port
	oPort.write(action);
	oPort.close();

    return true;

}
