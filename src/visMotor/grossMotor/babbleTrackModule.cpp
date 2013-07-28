/*
 * babbleTrackModule.cpp
 *
 * Lydia Majure
 * a threaded module that ensures that hand tracking and arm exploration are synced up
 */

//yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

//system
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

//gsl
#include <gsl/gsl_rng.h>

using namespace std;
using namespace yarp::dev;
YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

class babbleTrackThread : public RateThread{
protected:
	ResourceFinder &rf;

	Port *armPlan;
	Port *armPred;
	BufferedPort<Vector> *armLocJ;
	BufferedPort<Vector> *headLoc;

	string robotName;
	string name;
	string arm;

	//rng
	const gsl_rng_type *T;
	gsl_rng *r;

	PolyDriver *clientGazeCtrl;
	PolyDriver *robotDevice;

	IGazeControl *igaze;

	IPositionControl *pos;
	IEncoders *enc;

	int nj;
	Vector *command;
	Vector *tmp;

public:
	babbleTrackThread(int period, ResourceFinder &_rf) : RateThread(period), rf(_rf){}

	virtual bool handleParams(){
			if(rf.check("robot")){
				robotName = rf.find("robot").asString().c_str();
			}
			else{
				printf("Must specify robot name\n");
				return false;
			}

			name = rf.check("name",Value("babbleTrack")).asString().c_str();
			arm = rf.check("arm", Value("left")).asString().c_str();
			return true;
	}

	virtual bool threadInit(){
		if(!handleParams()){
			return false;
		}

		armPlan = new Port;
		armPred = new Port;
		armLocJ = new BufferedPort<Vector>;
		headLoc = new BufferedPort<Vector>;

		armPlan->open("/babbleTrack/plan:o");
		armPred->open("/babbleTrack/pred:i");
		armLocJ->open("/babbleTrack/arm:o");
		headLoc->open("/babbleTrack/head:o");

		gsl_rng_env_setup();
		T = gsl_rng_default;
		r = gsl_rng_alloc(T);

		igaze = NULL;

		Property options;
		options.put("device","gazecontrollerclient");
		options.put("remote","/iKinGazeCtrl");
		options.put("local","/client/gaze");

		clientGazeCtrl = new PolyDriver;
		clientGazeCtrl->open(options);

		options.clear();
		string localPorts = "/babbleTrack/cmd";
		string remotePorts = "/" + robotName + "/" + arm + "_arm";

		options.put("device", "remote_controlboard");
		options.put("local", localPorts.c_str());
		options.put("remote", remotePorts.c_str());

		robotDevice = new PolyDriver;
		robotDevice->open(options);

		if(clientGazeCtrl->isValid()){
			clientGazeCtrl->view(igaze);
		}
		else{
			return false;
		}


		if (!robotDevice->isValid()){
			printf("Device not available. Here are known devices: \n");
			printf("%s", Drivers::factory().toString().c_str());
			Network::fini();
			return false;
		}

		bool ok;
		ok = robotDevice->view(pos);
		ok = ok && robotDevice->view(enc);

		if (!ok){
			printf("Problems acquiring interfaces\n");
			return false;
		}

		pos->getAxes(&nj);

		command = new Vector;
		tmp = new Vector;
		command->resize(nj);
		tmp->resize(nj);

		for (int i = 0; i < nj; i++) {
		         (*tmp)[i] = 25.0;
		}
		pos->setRefAccelerations(tmp->data());

		for (int i = 0; i < nj; i++) {
			(*tmp)[i] = 20.0;
			pos->setRefSpeed(i, (*tmp)[i]);
		}

		*command = 0;

		//set the arm joints to "middle" values
		(*command)[0] = -45;
		(*command)[1] = 45;
		(*command)[2] = 0;
		(*command)[3] = 45;

		//flex hand
		(*command)[4] = 60;
		(*command)[7] = 20;
		(*command)[10] = 15;
		(*command)[11] = 15;
		(*command)[12] = 15;
		(*command)[13] = 15;
		(*command)[14] = 15;
		(*command)[15] = 15;
		pos->positionMove(command->data());

		bool done = false;
		while (!done){
			pos->checkMotionDone(&done);
			Time::delay(0.1);
		}

		bool fwCvOn = 0;
		fwCvOn = Network::connect("/babbleTrack/plan:o","/fwdConv:i");
		fwCvOn *= Network::connect("/fwdConv:o","/babbleTrack/pred:i");
		if (!fwCvOn){
			printf("Please run command:\n ./fwdConv --input /fwdConv:i --output /fwdConv:o\n");
			return false;
		}

		return true;
	}

	virtual void run(){
		tmp = command;
		(*command)[0] = -60*(gsl_rng_uniform(r));
		(*command)[1] = 100*(gsl_rng_uniform(r));
		(*command)[2] = -35 + 95*(gsl_rng_uniform(r));
		(*command)[3] = 10 + 90*(gsl_rng_uniform(r));
		printf("%.1lf %.1lf %.1lf %.1lf\n", (*command)[0], (*command)[1], (*command)[2], (*command)[3]);
		//above 0 doesn't seem to be safe for joint 0
		if ((*command)[0] > 0 || (*command)[0] < -60){
			(*command)[0] = (*tmp)[0];
		}
		if ((*command)[1] > 100 || (*command)[1] < -0){
			(*command)[1] = (*tmp)[1];
		}
		if ((*command)[2] > 60 || (*command)[2] < -35){
			(*command)[2] = (*tmp)[2];
		}
		if ((*command)[3] > 100 || (*command)[3] < 10){
			(*command)[3] = (*tmp)[3];
		}
		//use fwd kin to find end effector position
		Bottle plan, pred;
		for (int i = 0; i < nj; i++){
			plan.add((*command)[i]);
		}
		armPlan->write(plan);
		armPred->read(pred);
		Vector commandCart(3);
		for (int i = 0; i < 3; i++){
			commandCart[i] = pred.get(i).asDouble();
		}
		printf("Cartesian safety check\n");
		double rad = sqrt(commandCart[0]*commandCart[0]+commandCart[1]*commandCart[1]);
		// safety radius back to 30 cm
		if (rad > 0.3){
			pos->positionMove(command->data());
			bool done = false;
			while(!done){
				pos->checkMotionDone(&done);
				Time::delay(0.1);
			}
			printf("Moved arm to new location\n");
			Vector &armJ = armLocJ->prepare();
			Vector encoders(nj);
			enc->getEncoders(encoders.data());
			armJ = encoders;
			Vector noisyArm(3);
			for(int i = 0; i < 3; i++){
				//noisyArm[i] = commandCart[i] + 0.01*(2*gsl_rng_uniform(r)-1);
				//sanity check
				noisyArm[i] = commandCart[i] + 0.005*(2*gsl_rng_uniform(r)-1);
			}
			//insert here:
			//read off peak saliences
			//fixate there
			//calculate cartesian value, compare to real cart. value of arm


			printf("Looking at arm\n");
			igaze->lookAtFixationPoint(noisyArm);
			done = false;
			while(!done){
				igaze->checkMotionDone(&done);
				Time::delay(0.5);
			}
			//igaze->waitMotionDone(0.1,30);

			printf("Saw arm\n");

			Vector &headAng = headLoc->prepare();
			igaze->getAngles(headAng);
			Bottle tStamp;
			tStamp.clear();
			tStamp.add(Time::now());
			headLoc->setEnvelope(tStamp);


			headLoc->write();
			armLocJ->write();
			headLoc->unprepare();
			armLocJ->unprepare();

		}
		else{
			printf("Self collision detected!\n");
		}
	}

	virtual void threadRelease(){
		armPlan->interrupt(); armPred->interrupt();
		armPlan->close(); armPred->close();
		delete armPlan; delete armPred;
		armLocJ->interrupt(); headLoc->interrupt();
		armLocJ->close(); headLoc->close();
		delete armLocJ; delete headLoc;
		gsl_rng_free(r);
		//delete r;
		//clientGazeCtrl->close();
		//robotDevice->close();
		//delete igaze;
		//delete pos;
		//delete enc;
		//delete command;
		//delete tmp;
	}
};

class babbleTrackModule : public RFModule{
protected:
	babbleTrackThread *thr;
	Port *rpcPort;
public:
	babbleTrackModule() {}

	virtual bool configure(ResourceFinder &rf){
		rpcPort = new Port;
		rpcPort->open("/babbleTrack");
		attach(*rpcPort);
		thr = new babbleTrackThread(50,rf);
		bool ok = thr->start();
		if(!ok){
			delete thr;
			return false;
		}
		return true;
	}

	virtual double getPeriod() { return 1.0; }
	virtual bool updateModule() {return true;}


	virtual bool interruptModule(){
		rpcPort->interrupt();
		return true;
	}

	virtual bool close(){
		rpcPort->close();
		thr->stop();
		delete rpcPort;
		delete thr;
		return true;
	}
};

int main(int argc, char *argv[]){
	YARP_REGISTER_DEVICES(icubmod)
	Network yarp;
	if(!yarp.checkNetwork()){
		return -1;
	}
	ResourceFinder rf;
	rf.configure("ICUB_ROOT",argc,argv);
	babbleTrackModule mod;
	mod.runModule(rf);
	return 0;
}
