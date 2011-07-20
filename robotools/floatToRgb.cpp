/*
 *  floatToRgb
 *
 * 	Logan Niehaus
 * 	takes a PixelFloat image in and converts it to a rgb image for viewing with yarpview
 *
 *  inputs:
 *  	/floatToRgb/img:i	-- pixelfloat image
 *
 *  params:
 *  	name		-- module ports basename (D /floatToRgb)
 *  	rate		-- update rate in ms; objs segmented and published at this rate (D 50)
 *
 *  outputs:
 *  	/floatToRgb/img:o	-- rgb output image
 *
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

#include <string>
#include <stdio.h>
#include <stdlib.h>

//namespaces
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

class floatToRgbThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelFloat> > *portImgIn;
	BufferedPort<ImageOf<PixelRgb> > *portImgOut;

	int trate;

public:

	floatToRgbThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	virtual bool threadInit()
	{

		name=rf.check("name",Value("floatToRgb")).asString().c_str();
		trate = rf.check("rate",Value(50)).asInt();
		this->setRate(trate);

		portImgIn=new BufferedPort<ImageOf<PixelFloat> >;
		string portInName="/"+name+"/img:i";
		portImgIn->open(portInName.c_str());

		portImgOut=new BufferedPort<ImageOf<PixelRgb> >;
		string portOutName="/"+name+"/img:o";
		portImgOut->open(portOutName.c_str());

		return true;

	}

	virtual void run()
	{

		// get inputs
		ImageOf<PixelFloat> *pImgIn=portImgIn->read(false);

		if (pImgIn)
		{

			ImageOf<PixelRgb> &imgOut= portImgOut->prepare();
			imgOut.copy(*pImgIn);
			portImgOut->write();

		}

	}

	virtual void threadRelease()
	{

		portImgIn->interrupt();
		portImgOut->interrupt();

		delete portImgIn;
		delete portImgOut;

	}

};

class floatToRgbModule: public RFModule
{

protected:

	floatToRgbThread *thr;
	Port * rpcPort;
	string name;

public:
	floatToRgbModule() { }

	bool respond(const Bottle& command, Bottle& reply) {

		//handle information requests
		string msg(command.get(0).asString().c_str());
		if (msg == "rate") {
			if (command.size() < 2) {
				reply.add(-1);
			}
			else {
				int newrate = command.get(1).asInt();
				bool rssuc;
				if (newrate > 0) {
					rssuc = thr->setRate(newrate);
					if (rssuc)
						reply.add(1);
					else
						reply.add(-1);
				}
			}
		}
		else {
			reply.add(-1);
		}


		return true;

	}


	virtual bool configure(ResourceFinder &rf)
	{
		Time::turboBoost();

		//set up the rpc port
		name=rf.check("name",Value("floatToRgb")).asString().c_str();
		rpcPort = new Port;
		string portRpcName="/"+name+"/rpc";
		rpcPort->open(portRpcName.c_str());
		attach(*rpcPort);

		thr=new floatToRgbThread(rf);
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

		rpcPort->interrupt();
		delete rpcPort;

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

	floatToRgbModule mod;

	return mod.runModule(rf);
}
