/*
 *  robotMicConditioning.cpp
 *
 * 	Logan Niehaus
 * 	01/30/11
 * 	small module that does some conditioning on the robot's microphones, which are quite noisy.
 *  after getting a noise profile (at the beginning or by rpc request), it then uses
 *  a spectral subtraction method to clean some of the fan noise.
 *
 *
 * Module Args: (activity detection)
 * 	input 		-- input port name (sound)
 *  output		-- output port name (cleaned sound)
 *	decimate	-- decimate by X, (also can be called by audioProcessing. watch out)
 *
 *	(sound processing)
 *	n 			-- window length (D 2048)
 *	thresh  	-- activity gating thresh. as factor*baseline (D 1.3)
 *	atf 	 	-- attenuation factor in dB (D 20)
 *
 * PORTS:
 *	Inputs: /icub/microphone   	(YARP Sound structure. Contains most important information)
 *	Outputs: /cleaned/mic		(YARP Sound structure. This module should be easily removable)
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
#include <fftw3.h>
#include <algorithm>
#include <stdio.h>
#include <sndfile.h>

//defines
#define PI 3.14159
#define INT_MAX 32767

//namespaces
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


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
	int lastSize;
	int frequency;


public:

	VADPort(DataBuffer &buf, int decimate) : buffer(buf), N(decimate), lastSize(0), frequency(0) { }

	//callback for incoming position data
	virtual void onRead(Sound& s) {

		int blockSize = s.getSamples();
		lastSize = blockSize/N;
		frequency = s.getFrequency()/2;
		Stamp tStamp;

		//lock the data buffer for the whole transfer
		buffer.lock();
		for (int i = 0; i < blockSize/N; i++) {
			buffer.push_back((double)s.getSafe(i*N,0)/(double)INT_MAX);
		}
		buffer.unlock();

	}

	int getSize() { return lastSize; }
	int getFreq() { return frequency; }

};

//work thread
class CleanerThread : public Thread
{

private:

	//communication objects
	string recvPort;
	string sendPort;
	VADPort * inPort;
	Port   * outPort;

	//data
	DataBuffer buf;				//main data buffer
	deque<double> outgoing;		//outgoing data buffer

	//params (audio)
	int n;				//window size for spectrum (512 @ 24k ~= 20ms)
	int decimate;
	double atf;			//number of mfcc features to get
	double thresh;		//threshold for gating as percent baseline

	//processing objects
	double * window;	//hamming window
	double * iFrame;	//input frame
	fftw_complex *spec;	//spectral step
	double * baseLine;	//baseline spectral data
	double * oFrame;	//actual output frame
    fftw_plan p;		//fft
    fftw_plan q;		//ifft

    //SNDFILE * sf;
    //SF_INFO * sfinfo;

public:

	CleanerThread(string recvPort_, string sendPort_, int n_, int decimate_, double atf_, double thresh_) :
		recvPort(recvPort_), sendPort(sendPort_), n(n_), atf(atf_), thresh(thresh_), decimate(decimate_) {


		window = new double[n];	//hamming window
		iFrame = (double*) fftw_malloc(sizeof(double) * n);	//input frame
		spec = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (n/2)+1);	//spectral step
		baseLine = new double[(n/2)+1];	//baseline spectral data
		oFrame = (double*) fftw_malloc(sizeof(double) * n);	//actual output frame

	}

	virtual bool threadInit()
	{

		//set up processing
		window = new double[n];
		for (int i = 0; i < n; i++) {
			window[i] = 0.54 + 0.46*cos((double)(2*PI*(-(n/2)+i)/n));
		}
		for (int i = 0; i < (n/2); i++) {
			outgoing.push_back(0.0);	//zero pad the output buffer
		}

		//set up plans
		p = fftw_plan_dft_r2c_1d(n, iFrame, spec, NULL);
		q = fftw_plan_dft_c2r_1d(n, spec, oFrame, NULL);

		//set up ports
		inPort = new VADPort(buf, decimate);
		outPort = new Port;
		outPort->open(sendPort.c_str());

		//set callback
		inPort->useCallback();
		inPort->open(recvPort.c_str());
/*
		//soundfile
		sfinfo = new SF_INFO;
		sfinfo->channels = 1;
		sfinfo->samplerate = 24000;
		sfinfo->format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;
		sf = sf_open("/home/logan/workspace/scripts/grunk.wav",SFM_WRITE,sfinfo);
		printf("sf opened with code %d\n", sf_error(sf));
*/

		return true;


	}

	virtual void afterStart(bool s)
	{
		if (s)
			printf("Thread started successfully\n");
		else
			printf("Thread did not start\n");
	}

	bool setBaseline() {

		int bf = 10;
		double temp;
		int cbsize = 0;
		deque<double>::iterator mit;

		//wait until we have enough frames to gather a decent baseline
		while (cbsize < bf*(n/2)) {
			if (isStopping()) {
				//sit and spin
				return false;
			}
			cbsize = buf.size();
			sleep(1);
		}

		//when we have enough, process them
		printf("Gathering noise profile, out of %d init samples...\n", buf.size());
		for (int j = 0; j < bf-1; j++) {

			//unload a frame
			buf.lock();
			printf("copying data, buffer size %d\n",buf.size());
			int k = 0;
			for (mit = buf.begin(), k = 0; mit != buf.begin()+n; mit++, k++) {
				iFrame[k] = *mit;
			}
			printf("data copied\n");
			for (int i = 0; i < n/2; i++) {
				buf.pop_front();
			}
			buf.unlock();

			//process
			printf("processing frame %d\n", j);
			for (int i = 0; i < n; i++) {
				iFrame[i] *= window[i];
			}
			fftw_execute(p);
			for (int i = 0; i < (n/2)+1; i++) {
				temp = sqrt(spec[i][0]*spec[i][0]+spec[i][1]*spec[i][1]);
				if (temp > baseLine[i]) {
					baseLine[i] = temp;
				}
			}

		}
		printf("Baseline noise profile initialized...\n");
		return true;
	}

	virtual void run()
	{

		deque<double>::iterator mit;
		double * comp = new double[(n/2)+1];
		double * fcomp = new double[(n/2)+1];
		double temp;

		//get a baseline noise spectrum before moving on
		setBaseline();

		//stay in loop until given command to quit
		while (isStopping() != true) {

			//check to see if there is a full frame to process
			if (buf.size() >= n) {

				//unload a frame
				buf.lock();
				copy(buf.begin(),buf.begin()+n,iFrame);
				for (int i = 0; i < n/2; i++) {
					buf.pop_front();
				}
				buf.unlock();

				//process this frame
				for (int i = 0; i < n; i++) {
					iFrame[i] *= window[i];
				}
				fftw_execute(p);

				//compare it baseline, smooth it
				for (int i = 0; i < (n/2)+1; i++) {
					temp = sqrt(spec[i][0]*spec[i][0]+spec[i][1]*spec[i][1]);
					comp[i] = -atf;
					if (temp > baseLine[i]*thresh) {
						comp[i] = 0.0;
					}
				}
				fcomp[0] = 0.85*comp[0] + 0.15*comp[1];	//hardcoded, TODO
				fcomp[(n/2)] = 0.85*comp[(n/2)] + 0.15*comp[(n/2)-1];
				for (int i = 1; i < (n/2); i++) {
					fcomp[i] = 0.15*comp[i-1] + 0.7*comp[i] + 0.15*comp[i+1];
				}

				//apply spectral supression, convert back into time domain
				for (int i = 0; i < (n/2)+1; i++) {
					temp = pow(10.0, fcomp[i]/20.0);
					spec[i][0] *= temp;
					spec[i][1] *= temp;
				}
				fftw_execute(q);

				//put it on the queue to be sent away
				if (outgoing.size() < (n/2)) {
					printf("outgoing buffer got broken, quitting\n");
					this->stop();
				}
				int i;
				for (mit = outgoing.end()-(n/2), i = 0; mit != outgoing.end(); mit++, i++) {
					*mit += oFrame[i];
				}
				for (i = (n/2); i < n; i++) {
					outgoing.push_back(oFrame[i]);
				}
			}

			//check the outgoing queue to see if it is full enough
			if (outgoing.size() >  (inPort->getSize()+n/2)) {

				Sound processed;
				processed.resize(inPort->getSize());
				for (int i = 0; i < inPort->getSize(); i++) {
					//temp = outgoing.front()/(1e+3);  //
					//sf_write_double(sf, &temp, 1);
					processed.set((int)(outgoing.front()*INT_MAX),i);	//rescale
					outgoing.pop_front();
				}
				processed.setFrequency(inPort->getFreq());
				outPort->write(processed);

			}
		}

		delete comp;
		delete fcomp;

	}

	virtual void onStop() {

		inPort->interrupt();
		outPort->interrupt();

	}

	virtual void threadRelease()
	{

		//close down and cleanup
		//sf_write_sync(sf);
		//sf_close(sf);
		fftw_destroy_plan(p);
		fftw_destroy_plan(q);
		fftw_free(iFrame);
		fftw_free(spec);
		fftw_free(oFrame);
		delete window;
		delete baseLine;

	}
};



//handler module
class CleanerModule : public RFModule {

protected:

	string recvPort;
	string sendPort;

	//params (audio)
	int n;				//window size for spectrum (512 @ 24k ~= 20ms)
	int decimate;
	double atf;			//number of mfcc features to get
	double thresh;		//threshold for gating as percent baseline

	CleanerThread * thread;


public:

	virtual void handleArgs(ResourceFinder &rf) {

		//args
		recvPort = rf.check("input",Value("/earCleaner:i"),"input port").asString();
		sendPort = rf.check("output",Value("/earCleaner:o"),"output port").asString();
		atf = rf.check("atf",Value(20.0),"noise attenuation factor").asDouble();
		thresh = rf.check("thresh",Value(1.3),"activity threshold").asDouble();
		n = rf.check("n",Value(2048),"window size").asInt();
		decimate = rf.check("decimate",Value(2),"decimation factor").asInt();

	}

	virtual bool configure(ResourceFinder &rf)
	{

		handleArgs(rf);

		//hand off processing duties
		thread = new CleanerThread(recvPort,sendPort,n,decimate,atf,thresh);
		if (!thread->start())
		{
			printf("Thread init failed, quitting\n");
			delete thread;
			return false;
		}

		return true;

	}

	virtual bool close() {

		printf("Stopping thread...\n");
		thread->stop();
		delete thread;

		return true;

	}

	virtual double getPeriod()    {

		return 1.0;

	}

	virtual bool   updateModule() {

		return true;

	}

};


int main(int argc, char *argv[])
{
	// we need to initialize the drivers list
	Network yarp;
	if (!yarp.checkNetwork())
		return -1;

	CleanerModule mod;

	ResourceFinder rf;

	rf.configure("ICUB_ROOT", argc, argv);

	return mod.runModule(rf);
}

