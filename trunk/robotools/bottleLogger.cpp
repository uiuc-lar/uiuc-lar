/*
 *  dataPumper.cpp
 *
 * 	Logan Niehaus
 * 	3/10/11 (older)
 * 	simple program that logs sequences that come in, in the bottle of bottles format
 *
 * Module Args:
 *
 *	target	-- target port to attach on to
 *	basename -- basename of csv file that the data will be written to (basenameXX.csv)
 *
 * PORTS:
 *	inputs: /pumpOut (just an auxiliary port for making the connection to the target
 */

//yarp network
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/RFModule.h>

//misc
#include <stdio.h>
#include <stdlib.h>
#include <string>

//internal
#include "../imatlib/IMatVecOps.hh"

using namespace std;
using namespace yarp;
using namespace yarp::os;

void dumbCSVWriter(const char *, IMat &);

class LogPort : public BufferedPort<Bottle> {

protected:
	
	int count;
	string basename;
	
public:

	LogPort(const char * bname) : count(0), basename(bname) { }
	
	//callback for incoming sequences
	virtual void onRead(Bottle& b) {
		
		char bnum[10];
		string newname = basename;
		sprintf(bnum,"%02d",count);
		newname += bnum;
		newname += ".csv";
		
		IMat data;
		Bottle *tmp = b.get(0).asList();
		data.resize(b.size(),tmp->size());
		
		//unpack the sequence
		for (int i = 0; i < b.size(); i++) {
			Bottle *item = b.get(i).asList();
			for (int j = 0; j < item->size(); j++) {
				data(i,j) = item->get(j).asDouble();
			}
		}
		
		//write
		printf("trying to write to %s ... \n", newname.c_str());
		dumbCSVWriter(newname.c_str(), data);
		count++;
		
	}

};

class DummyModule : public RFModule {
	
protected:
	
public:
	
	virtual bool configure(ResourceFinder &rf) { return true; }	
	virtual double getPeriod() { return 10.0; }
	virtual bool updateModule() { return true; }
	virtual bool close() { return true; }
	
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

	if (!rf.check("target")) {
		printf("please specify target port name to log\n");
		return false;
	}
	if (!rf.check("basename")) {
		printf("please specify a basename to use\n");
		return false;
	}

	//open the output port and connect it to the target port
	ConstString pname("/loggerIn");
	ConstString tname = rf.find("target").asString();
	ConstString bname = rf.find("basename").asString();
	LogPort inPort(bname.c_str());
	inPort.open(pname.c_str());
	inPort.useCallback(); 
	Network::connect(tname.c_str(),pname.c_str());

	return mod.runModule(rf);

}


void dumbCSVWriter(const char * fileName, IMat &tM) {

	FILE *fp = fopen(fileName,"w");

	for (int i = 0; i < tM.m; i++) {
		for (int j = 0; j < tM.n-1; j++) {
			fprintf(fp,"%f,",tM(i,j));
		}
		fprintf(fp,"%f\n",tM(i,tM.n-1));
	}
	fclose(fp);

}

