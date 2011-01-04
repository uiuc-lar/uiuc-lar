/*
 *  actionVad.cpp
 *
 * 	Logan Niehaus
 * 	12/17/10
 * 	activity detection algorithm for recognition system. also does much of the
 * 	major signal conditioning work
 *
 * Module Args:
 * 	input 		-- input port name
 * 	output	  	-- output port name
 *	decimate	-- decimate by X, after applying a filter (handle the filter here automatically)
 *	order		-- order of vad signal filter
 *	cutoff		-- cutoff frequency of the vad filter (given as [0,1] of nyquist)
 *  threshold	-- activity threshold
 *
 *
 * PORTS:
 *	Inputs: /kin/cartesian   	(Bottle of x,y,z and euler angles XYZ)
 *	Outputs: /vad/actions		(Bottle containing sequence of bottles, each w/ t,x,y,z. timestamp included)
 *
 */

//yarp network
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Semaphore.h>

//icub includes
#include <iCub/iKin/iKinFwd.h>

//misc
#include <string.h>
#include <math.h>
#include <deque>

//defines
#define PI 3.14159
#define FSIZE 20 		//decimation filter order

//namespaces
using namespace iCub::iKin;
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


//window method LP FIR filter
Vector fir1(int N, double fc) {

	double sum = 0.0;
	Vector window(N+1);	//hamming window
	Vector filter(N+1);

	for (int i = 0; i < N+1; i++) {

		window[i] = 0.54 - 0.46*cos((double)(2.0*PI*((double)i/(double)N)));
		if (i-N/2 == 0) {
			filter[i] = window[i]*1.0;
		} else {
			filter[i] = window[i]*sin(PI*fc*(i-(N/2)))/(PI*fc*(i-(N/2)));
		}
		sum += filter[i];
	}

	for (int i = 0; i < filter.size(); i++) {
		filter[i] = filter[i]/sum;
	}

	return filter;

}


class StatusChecker : public PortReader {

protected:

	int * active;

public:

	StatusChecker(int * active_) : active(active_) {}

	virtual bool read(ConnectionReader& connection) {
		Bottle in, out;
		bool ok = in.read(connection);
		if (!ok)
			return false;
		out.add(*active);
		ConnectionWriter *returnToSender = connection.getWriter();
		if (returnToSender!=NULL) {
			out.write(*returnToSender);
		}
		return true;
	}
};

class DataBuffer : public deque<Bottle> {

private:

	Semaphore mutex;

public:

    void lock()   { mutex.wait(); }
    void unlock() { mutex.post(); }

public:

};

/*
 * 	ADPort: callback port for handling incoming data.
 *		This port can decimate the data for the thread handling it.
 *		NB: because of the filtering for decimation, there will be phase delay,
 *
 */
class ADPort : public BufferedPort<Bottle> {

protected:

	//data
	DataBuffer &buffer;		//buffer shared with main thread
	Vector taps;			//filter taps
	deque<Bottle> * rSamp;	//previous samples

	//params
	int N;					//decimation factor

	//misc
	int counter;

public:

	ADPort(DataBuffer &buf, int decimate) : buffer(buf), N(decimate) {

		taps = fir1(FSIZE,1.0/(double)N);
		rSamp = new deque<Bottle>;

		counter = 1;

	}

	//callback for incoming position data
	virtual void onRead(Bottle& b) {

		Vector sums(b.size());
		Bottle item;
		Stamp tStamp;
		int m = rSamp->size();

		if (counter == N) {

			//apply filter
			for (int j = 0; j < b.size(); j++) {
				sums[j] = taps[0]*b.get(j).asDouble();
				for (int i = 0; i < m; i++) {
					sums[j] += taps[i+1]*rSamp->at(m-i-1).get(j).asDouble();
				}
				item.add(sums[j]);
			}

			//add the timestamp
			BufferedPort<Bottle>::getEnvelope(tStamp);
			item.add(tStamp.getTime());

			//push this onto the shared deque
			buffer.lock();
			buffer.push_back(item);
			buffer.unlock();

			//reset decimation counter
			counter = 1;

		} else {

			//don't calculate if we don't need to (using fir filter)

			//inc decimation counter
			counter++;

		}

		//push back
		rSamp->push_back(b);
		while (rSamp->size() > FSIZE) {
			rSamp->pop_front();
		}

	}

	virtual void close() {

		//delete taps;
		delete rSamp;

	}

};

//handler module
class ADModule : public RFModule {

protected:

	//communication objects
	string recvPort;
	string sendPort;
	ADPort * inPort;
	Port   * outPort;
	Port   * statPort;
	StatusChecker * checker;

	//params
	int filtOrder;
	double filtCutoff;
	double adThreshold;
	int adJump;
	int decimate;

	//data
	DataBuffer buf;			//main data buffer
	deque<double> motionSig;	//energy signal to be filtered
	Vector B;					//filter
	deque<Bottle> activeSig;	//current active signal
	Vector lastSample;

	//flags
	int status;
	int lastSize;				//checks to see if new data has arrived
	int hysT;					//time for gap jumping heuristic

public:

	virtual void handleArgs(ResourceFinder &rf) {

		recvPort = rf.check("input",Value("/madIn"),"input port").asString();
		sendPort = rf.check("output",Value("/madOut"),"output port").asString();
		filtOrder = rf.check("order",Value(20),"ad filter order").asInt();
		filtCutoff = rf.check("cutoff",Value(0.1),"ad filter cutoff").asDouble();
		adThreshold = rf.check("threshold",Value(0.0001),"activity threshold").asDouble();
		adJump = rf.check("jump",Value(50),"activity hysteresis").asInt();
		decimate = rf.check("decimate",Value(2),"signal decimation").asInt();

	}

	virtual bool configure(ResourceFinder &rf)
	{

		handleArgs(rf);

		//set up ports
		inPort = new ADPort(buf, decimate);
		outPort = new Port;
		outPort->open(sendPort.c_str());

		//set callback
		inPort->useCallback();
		inPort->open(recvPort.c_str());

		//set up status checking port
		statPort = new Port;
		checker = new StatusChecker(&status);
		string statName = sendPort + "/status";
		statPort->open(statName.c_str());
		statPort->setReader(*checker);

		//get filter taps
		B = fir1(filtOrder,filtCutoff);

		lastSize = 0;
		status = 0;
		lastSample.resize(3);

		return true;

	}

	virtual bool close() {

		inPort->close();
		outPort->close();

		return true;

	}

	virtual double getPeriod()    {
		return 0.0;
	}

	virtual bool   updateModule() {

		//if we picked up a new sample, process it
		if (buf.size() > 0) {

			//calculate motion
			double motion = 0.0;
			for (int i = 0; i < 3; i++) {
				//bump this up to keep it from numerical underflow
				double dxyz = 10*(buf.front().get(i).asDouble()-lastSample[i]);
				lastSample[i] = buf.front().get(i).asDouble();
				motion+= dxyz*dxyz;
			}
			motionSig.push_back(motion);

			//filter
			double fMotion = 0.0;
			for (int i = 0; i < motionSig.size(); i++) {
				fMotion += B[i]*motionSig.at(motionSig.size()-i-1);
			}

			//if we are above threshold
			if (fMotion > adThreshold) {

				//put this sample onto the current active sequence, reset timer
				activeSig.push_back(buf.front());
				hysT = adJump;
				status = 1;

			}
			//if we are below the threshold
			else {

				//see if the 'hysteresis' timer has run out
				if (hysT > 0) {

					//if it hasn't save the sample as possible, tick off timer
					activeSig.push_back(buf.front());
					hysT--;

				} else {

					//if it has, backtrack and publish the action (if available)
					if (activeSig.size() > 0) {

						//first prune off unwanted samples
						for (int i = 0; i < adJump; i++) {
							activeSig.pop_back();
						}

						//put the good ones in a bottle
						Bottle sequence;
						double initTime = activeSig.front().get(6).asDouble();
						double endTime = activeSig.back().get(6).asDouble();
						while (activeSig.size() > 0) {
							Bottle & sample = sequence.addList();
							sample = activeSig.front();
							double t = sample.pop().asDouble()-initTime;
							sample.add(t);
							activeSig.pop_front();
						}

						//put a front/back timestamps on it for synchrony detection
						Bottle tStamp;
						tStamp.addDouble(initTime);
						tStamp.addDouble(endTime);
						outPort->setEnvelope(tStamp);
						outPort->write(sequence);
					}
					status = 0;
				}
			}

			//remove processed samples
			buf.lock();
			buf.pop_front();
			buf.unlock();
			while (motionSig.size() > filtOrder+1) {
				motionSig.pop_front();
			}

		} else {

			//no new samples available, do nothing

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

	ADModule mod;

	ResourceFinder rf;

	rf.configure("ICUB_ROOT", argc, argv);

	return mod.runModule(rf);
}

