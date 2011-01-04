/*
 *  audioProcessing.cpp
 *
 * 	Logan Niehaus
 * 	12/18/10
 * 	activity detection and feature extraction algorithm for recognition system.
 * 	also does much of the major signal conditioning work. structure based
 * 	off of motion detection module. puts out MFCCs.
 *
 * Module Args: (activity detection)
 * 	input 		-- input port name
 *  output		-- output port name (features)
 *	decimate	-- decimate by X, after applying a filter (handle the filter here automatically)
 *  alpha		-- exponential filter coefficient for the energy signal
 *  threshold	-- activity threshold
 *  jump 		-- gap jumping heuristic parameter
 *
 *	(sound processing)
 *	nwindows	-- number of mel windows used
 *	ncoeffs		-- number of actual MFCCs returned
 *	frameSize	-- processing framesize. for icub applications, usually 512 ~= 20ms
 *	overlap		-- frame overlap for feature extraction
 *	samplerate	-- audio sample rate (original). default 48k is usual for portaudio
 *
 * PORTS:
 *	Inputs: /icub/microphone   	(YARP Sound structure. Contains most important information)
 *	Outputs: /vad/words			(Bottle of bottles, each with however many MFCCs for the sequence)
 */

//yarp network
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Sound.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>

//mfcc library include
#include "../speech/mfcc.h"

//misc
#include <string>
#include <math.h>
#include <deque>

//defines
#define PI 3.14159
#define FSIZE 20 		//decimation filter order
#define INT_MAX 32767
#define PFRAMES	10
#define FECMAX 7

//namespaces
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

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


class DataBuffer : public deque<double> {

private:

	Semaphore mutex;

public:

	void lock()   { mutex.wait(); }
	void unlock() { mutex.post(); }

};

/*
 * 	VADPort: callback port for handling incoming data.
 *		This port can decimate the data for the thread handling it.
 *
 */
class VADPort : public BufferedPort<Sound> {

protected:

	//data
	DataBuffer &buffer;		//buffer shared with main thread

	//params
	int N;					//decimation factor


public:

	VADPort(DataBuffer &buf, int decimate) : buffer(buf), N(decimate) { }

	//callback for incoming position data
	virtual void onRead(Sound& s) {

		int blockSize = s.getSamples();
		Stamp tStamp;

		//apply filter

		/*
		 * 	for the voice signal, going to assume a reasonable amount of decimation
		 * 	and that aliasing will not be an issue (has not been on samples so far)
		 */

		//lock the data buffer for the whole transfer
		buffer.lock();
		for (int i = 0; i < blockSize/N; i++) {
			buffer.push_back((double)s.getSafe(i*N,0)/(double)INT_MAX);
		}
		buffer.unlock();

	}

};

//handler module
class ADModule : public RFModule {

protected:

	//communication objects
	string recvPort;
	string sendPort;
	VADPort * inPort;
	Port   * outPort;
	Port   * statPort;
	StatusChecker * checker;

	//params (AD)
	double adThreshold;
	int adJump;
	int decimate;

	//params (audio)
	double sampleRate;		//expected sample rate of original signal
	int frameSize;		//frame size for processing (512 @ 24k ~= 20ms)
	int overlap;		//overlap -- mfcc increment will be frameSize-2*overlap
	int d;				//number of mfcc features to get
	int w;				//total # of mel windows over fl-fh

	//data
	DataBuffer buf;				//main data buffer
	deque<Bottle> activeSig;	//current active signal
	deque<Bottle> fecSig;		//keeps a small amount of old data for front-end clipping protection
	int status;

	//processing
	MFCCProcessor * M;			//mfcc processor
	double alpha;				//exp filter coeff
	double energy;				//energy value
	double baseline;			//noise/silence baseline value
	int	bCounter;				//counts the number of frames for getting the baseline

	FILE *fp;

public:

	virtual void handleArgs(ResourceFinder &rf) {

		//activity detection args
		recvPort = rf.check("input",Value("/vadIn"),"input port").asString();
		sendPort = rf.check("output",Value("/vadOut"),"output port").asString();
		alpha = rf.check("alpha",Value(0.1),"exp filter coeff").asDouble();
		adThreshold = rf.check("threshold",Value(0.0001),"activity threshold").asDouble();
		adJump = rf.check("jump",Value(30),"activity hysteresis").asInt();
		decimate = rf.check("decimate",Value(2),"signal decimation").asInt();

		//audio processing args
		frameSize = rf.check("jump",Value(512),"processing frame size").asInt();
		overlap = rf.check("decimate",Value(128),"frame overlap").asInt();
		w = rf.check("nwindows",Value(30),"number of mel windows").asInt();
		d = rf.check("ncoeffs",Value(15),"number of first n mel windows to return").asInt();
		sampleRate = rf.check("samplerate",Value(48000),"original sample rate").asDouble();
		sampleRate = sampleRate/decimate;

	}

	virtual bool configure(ResourceFinder &rf)
	{

		handleArgs(rf);

		//set up ports
		inPort = new VADPort(buf, decimate);
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


		//set up mfcc processor
		M = new MFCCProcessor(d+1, w, frameSize, overlap, sampleRate, 0.0, (double)(sampleRate/2.0), true);

		//vad stuff
		energy = 0.0;
		baseline = 0.0;
		bCounter = PFRAMES; 	//use PFRAMES frames to get noise/silence baseline
		status = 0;

		fp = fopen("/home/logan/workspace/scripts/phonetictest/livefeatures.dat","w");

		return true;

	}

	virtual bool close() {

		fclose(fp);

		inPort->close();
		outPort->close();

		return true;

	}

	virtual double getPeriod()    {
		return 0.0;
	}

	virtual bool   updateModule() {

		//every time we update, load in all new samples
		if (buf.size() > 0) {
			buf.lock();
			double * signal = new double[buf.size()];
			int inc = 0;
			while (buf.size() > 0) {
				signal[inc] = buf.front();
				buf.pop_front();
				inc++;
			}
			buf.unlock();
			M->pushData(signal,inc);
		}

		//process as many samples as are available
		double * mTemp; 	//temp mfcc holder

		while(M->getMFCCs(mTemp)) {

			//grab 0th mfcc, pad/get baseline if first sample
			if (bCounter > 0) {
				baseline += mTemp[0]/PFRAMES;
				bCounter--;
				delete mTemp;
				break;
			}

			//filter energy (use exp filter here)
			// b = [alpha]; a = [1, alpha-1]; alpha in [0,1]
			energy = (1-alpha)*energy + alpha*(mTemp[0]-baseline);

			//bottle the data
			Bottle f;
			for (int i = 0; i < d; i++) {
				f.add(mTemp[i+1]);
			}

			//hold some for front-end clipping
			fecSig.push_back(f);
			while (fecSig.size() > FECMAX) {
				fecSig.pop_front();
			}

			//check for activity
			if (energy > adThreshold) {

				//save features
				status = 1;
				while (fecSig.size() > 0) {
					activeSig.push_back(fecSig.front());
					fecSig.pop_front();
				}

			} else {

				//check to see if sound is big enough
				if (activeSig.size() > adJump) {

					//put it in a bottle, chopping off the vad lag samples
					Bottle sequence;
					while (activeSig.size() > FECMAX) {
						Bottle & sample = sequence.addList();
						sample = activeSig.front();
						activeSig.pop_front();
						for (int i = 0; i < sample.size(); i++) {
							fprintf(fp,"%f,",sample.get(i).asDouble());
						}
						fprintf(fp,"\n");
					}
					activeSig.clear();
					status = 0;

					//assuming roughly RT processing, backcalculate beginning and end timestamps
					Bottle timeStamps;
					double cTime = Time::now();
					timeStamps.addDouble(cTime - frameSize*(1.0/(double)sampleRate)*sequence.size());
					timeStamps.addDouble(cTime);

					//write it out
					printf("Activity detected, writing sequence of length %d\n", sequence.size());
					outPort->setEnvelope(timeStamps);
					outPort->write(sequence);


				} else {

					//clear it out
					activeSig.clear();
					status = 0;

				}
			}
			delete mTemp;
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

