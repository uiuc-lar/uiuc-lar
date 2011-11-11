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
 *  stereoIOR.cpp
 *
 * 	Logan Niehaus
 * 	7/22/11
 * 	module implementing some basic IOR functionality
 *
 * inputs:
 *  	/stereoIOR/sal:l	-- unwarped left weighted salience sum (same as stereoAttn)
 *  	/stereoIOR/sal:r	-- unwarped right weighted salience sum (same as stereoAttn)
 *  	/stereoIOR/ior:i	-- ior event location as bottle of image coords [ul vl ur vr lbr].
 *  							if lbr is 0, coords can be used to reconstruct and xyz location.
 *  							if -1/1 only left/right coordinates are viable, and az/el approx is used
 *  	/stereoIOR/pos:h	-- streaming head position, for remapping of old events
 *
 * params:
 * 		decay	-- decay factor on event inhibition level (0-1, 0.01 is about 30s at 15fps)
 * 		maxtime	-- maximum lifespan for an IOR event. once exceeded, event will be removed.
 *		gauss	-- gaussian-style inhibition. arg specifies pixel width gaussian window (must be odd).
 *		thresh	-- threshold style inhibition. arg specifies thresh level as % of peak
 *					(nb gaussian and threshold inhibition behaviors are mutually exclusive)
 *		name	-- module basename for ports (D /stereoIOR)
 *		<cam parameters>		-- intrinsic calibration params see iKinGazeCtrl docs.
 *
 *
 * outputs:
 * 		/stereoIOR/ior:l	-- left IOR map for current image
 * 		/stereoIOR/ior:r	-- right IOR map for current image
 *
 * TODO:   fix threshold-behavior for IOR
 *
 */

//yarp includes
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>

//external cv includes
#include <cv.h>

//misc inc
#include <string>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <deque>

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::ctrl;

#define PI 3.14159

class iCubCFrame : public iCubEye {

public:

	iCubCFrame(const string &_type)	: iCubEye(_type) {

		(*this)[getN()-2].setD(0.0);
		this->blockLink(getN()-2,0.0);
		this->blockLink(getN()-1,0.0);

	}

};

struct iorEvent {

	int nframes;
	double x, y, z;
	double val;
	int ul, vl, ur, vr;
	int lbr;
	double az, el;
	bool unregistered;

};

class EventBuffer : public deque<iorEvent> {

private:

	Semaphore mutex;

public:

	void lock()   { mutex.wait(); }
	void unlock() { mutex.post(); }

};

class iorPort : public BufferedPort<Bottle> {

protected:

	EventBuffer &b;

public:

	iorPort(EventBuffer & _b) : b(_b) {	}

	//callback for incoming ior events
	virtual void onRead(Bottle& iore) {

		int ul = iore.get(0).asInt();
		int vl = iore.get(1).asInt();
		int ur = iore.get(2).asInt();
		int vr = iore.get(3).asInt();
		int lorr = 0;
		if (iore.size() > 4) {
			lorr = iore.get(4).asInt();
		}

		iorEvent E;
		E.unregistered = true;
		E.ul = ul; E.vl = vl; E.ur = ur; E.vr = vr;
		E.lbr = lorr;
		b.lock();
		b.push_back(E);
		b.unlock();

	}

};

class stereoIORThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	//ports
	BufferedPort<ImageOf<PixelFloat> > *portSalL;
	BufferedPort<ImageOf<PixelFloat> > *portSalR;
	BufferedPort<ImageOf<PixelFloat> > *portSalLO;
	BufferedPort<ImageOf<PixelFloat> > *portSalRO;
	BufferedPort<yarp::sig::Vector> *portHAngIn;
	iorPort * portEventIn;

	//data objects
	EventBuffer buf;
	ImageOf<PixelFloat> lImgL;
	ImageOf<PixelFloat> lImgR;

	//kinematics data
	iCubEye * eyeL, * eyeR;
	iCubCFrame * eyeC;
	Matrix Pl, Pr;
	Matrix rtToL, rtToR;
	Matrix egToRt, rtToEg;

	//running parameters
	double decay;
	int maxtime;
	int mode;
	int grad;
	double pthr;
	double gscale;

public:

	stereoIORThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	virtual bool threadInit()
	{

		name=rf.check("name",Value("stereoIOR")).asString().c_str();
		decay = rf.check("decay", Value(0.01)).asDouble();
		maxtime = rf.check("maxtime", Value(300)). asDouble();

		mode = -1;
		if (rf.check("gauss")) {
			mode = 0;
			grad = rf.find("gauss").asDouble();
			Mat G = getGaussianKernel(grad, (float)grad/5.0, CV_32F);
			minMaxLoc(G, NULL, &gscale, NULL, NULL);
			gscale = 1.0/(gscale*gscale);
		}
		if (rf.check("thresh")) {
			pthr = rf.find("thresh").asDouble();
			mode = 1;
		}
		if (mode < 0) {
			printf("please specify an inhibition function type (gauss or thresh)\n");
			return false;
		}

		//get camera calibration parameters
		Pl = Matrix(3, 4); Pl.zero(); Pl(2,2) = 1.0;
		Pr = Matrix(3, 4); Pr.zero(); Pr(2,2) = 1.0;
		Bottle ipsL = rf.findGroup("CAMERA_CALIBRATION_LEFT");
		Bottle ipsR = rf.findGroup("CAMERA_CALIBRATION_RIGHT");
		if (ipsL.size() && ipsR.size()) {

			Pl(0,2) = ipsL.find("w").asDouble()/2.0;
			Pl(1,2) = ipsL.find("h").asDouble()/2.0;
			Pl(0,0) = ipsL.find("fx").asDouble();
			Pl(1,1) = ipsL.find("fy").asDouble();

			Pr(0,2) = ipsR.find("w").asDouble()/2.0;
			Pr(1,2) = ipsR.find("h").asDouble()/2.0;
			Pr(0,0) = ipsR.find("fx").asDouble();
			Pr(1,1) = ipsR.find("fy").asDouble();

		}
		else {

			fprintf(stdout,"Could not find calibration parameters for one of the cameras\n");
			return false;

		}

		//open up ports
		portSalL=new BufferedPort<ImageOf<PixelFloat> >;
		string portSallName="/"+name+"/sal:l";
		portSalL->open(portSallName.c_str());

		portSalR=new BufferedPort<ImageOf<PixelFloat> >;
		string portSalrName="/"+name+"/sal:r";
		portSalR->open(portSalrName.c_str());

		portSalLO=new BufferedPort<ImageOf<PixelFloat> >;
		string portSalloName="/"+name+"/ior:l";
		portSalLO->open(portSalloName.c_str());

		portSalRO=new BufferedPort<ImageOf<PixelFloat> >;
		string portSalroName="/"+name+"/ior:r";
		portSalRO->open(portSalroName.c_str());

		portHAngIn=new BufferedPort<yarp::sig::Vector>;
		string portHAngName="/"+name+"/pos:h";
		portHAngIn->open(portHAngName.c_str());

		portEventIn=new iorPort(buf);
		string portIORName="/"+name+"/ior:i";
		portEventIn->open(portIORName.c_str());
		portEventIn->useCallback();

		//set up kinematic chains
		eyeL = new iCubEye("left");
		eyeR = new iCubEye("right");
		eyeL->setAllConstraints(false);
		eyeR->setAllConstraints(false);
		eyeC = new iCubCFrame("right");
		eyeC->setAllConstraints(false);
		eyeL->releaseLink(0); eyeC->releaseLink(0); eyeR->releaseLink(0);
		eyeL->releaseLink(1); eyeC->releaseLink(1); eyeR->releaseLink(1);
		eyeL->releaseLink(2); eyeC->releaseLink(2); eyeR->releaseLink(2);

		//get the location in root coordinates of ego center
		yarp::sig::Vector q(eyeC->getDOF()); q=0.0;
		egToRt=eyeC->getH(q);
		rtToEg=SE3inv(egToRt);

		return true;

	}

	virtual void registerEvent(iorEvent &E) {

		//register the new iorEvent
		E.nframes = 0;
		E.val = max(lImgL.pixel(E.ul,E.vl),lImgR.pixel(E.ur,E.vr));

		//check to see if xyz projection would be valid
		if (E.lbr == 0) {

			//get the event location
			yarp::sig::Vector b(4);
			yarp::sig::Vector Xp(3);
			Matrix A(4,3);
			Matrix Al = Pl;
			Matrix Ar = Pr;
			Al(0,2) = Al(0,2)-E.ul; Al(1,2) = Al(1,2)-E.vl;
			Ar(0,2) = Ar(0,2)-E.ur; Ar(1,2) = Ar(1,2)-E.vr;
			Al = Al*rtToL; Ar = Ar*rtToR;
			for (int i = 0; i < 2; i++) {
				b[i] = -Al(i,3); b[i+2] = -Ar(i,3);
				for (int j = 0; j < 3; j++) {
					A(i,j) = Al(i,j);
					A(i+2,j) = Ar(i,j);
				}
			}
			Xp = pinv(A)*b;
			E.x = Xp[0]; E.y = Xp[1]; E.z = Xp[2];

		} else {

			//get an ego-sphere based approximation
			yarp::sig::Vector pt(3);
			if (E.lbr < 0) {
				pt[0] = E.ul; pt[1] = E.vl; pt[2] = 1; pt = 10*pt;
				pt = pinv(Pl.transposed()).transposed()*pt;
				pt = rtToEg*SE3inv(rtToL)*pt; //get point location in ego frame coords
			} else {
				pt[0] = E.ur; pt[1] = E.vr; pt[2] = 1; pt = 10*pt;
				pt = pinv(Pr.transposed()).transposed()*pt;
				pt = rtToEg*SE3inv(rtToL)*pt; //get point location in ego frame coords
			}
			E.az = atan2(pt[0],pt[2]);
			E.el = -atan2(pt[1],pt[2]);

		}

		E.unregistered = false;

	}

	virtual void run()
	{

		//first check to see if any new events need to be registered
		for (int i = 0; i < buf.size(); i++) {

			iorEvent &event = buf.at(i);
			if (event.unregistered) {
				registerEvent(event);
			}

		}


		//get current head configuration
		yarp::sig::Vector *headAng = portHAngIn->read(true);
		yarp::sig::Vector angles(8); angles.zero();
		angles[3] = (*headAng)[0]; angles[4] = (*headAng)[1];
		angles[5] = (*headAng)[2]; angles[6] = (*headAng)[3];
		angles[7] = (*headAng)[4] + (*headAng)[5]/2.0;
		angles = PI*angles/180.0;
		rtToL = SE3inv(eyeL->getH(angles));
		angles[7] = PI*((*headAng)[4] - (*headAng)[5]/2.0)/180.0;
		rtToR = SE3inv(eyeR->getH(angles));

		//get current salience maps
		ImageOf<PixelFloat> *pSalL = portSalL->read(false);
		ImageOf<PixelFloat> *pSalR = portSalR->read(false);

		if (pSalL && pSalR) {

			//save the current maps
			lImgL.copy(*pSalL); lImgR.copy(*pSalR);

			ImageOf<PixelFloat> &loImg = portSalLO->prepare();
			ImageOf<PixelFloat> &roImg = portSalRO->prepare();
			loImg.copy(*pSalL); roImg.copy(*pSalR);
			Mat Sl(loImg.height(), loImg.width(), CV_32F, (void *)loImg.getRawImage());
			Mat Sr(roImg.height(), roImg.width(), CV_32F, (void *)roImg.getRawImage());
			Mat ffMskL(loImg.height()+2, loImg.width()+2, CV_8UC1); ffMskL.setTo(Scalar(0));
			Mat ffMskR(roImg.height()+2, roImg.width()+2, CV_8UC1); ffMskR.setTo(Scalar(0));
			Sl.setTo(Scalar(0));
			Sr.setTo(Scalar(0));

			yarp::sig::Vector xyz(4), uvl(3), uvr(3);
			int dcount = 0;
			for (int i = 0; i < buf.size(); i++) {

				//for each event, check if it is of single sided (ul/vl or ur/vr)
				iorEvent &tev = buf.at(i);

				if (tev.lbr != 0) {

					//for single sided events, use an az/el/r projection
					xyz[0] = 10.0*sin(tev.az)*cos(tev.el); xyz[1] = 10.0*sin(-tev.el);
					xyz[2] = 10.0*cos(tev.az)*cos(tev.el); xyz[3] = 1.0;
					xyz = egToRt*xyz;
					uvl = Pl*rtToL*xyz;
					uvr = Pr*rtToR*xyz;
					uvl[0] = uvl[0]/uvl[2]; uvl[1] = uvl[1]/uvl[2];
					uvr[0] = uvr[0]/uvr[2]; uvr[1] = uvr[1]/uvr[2];
					if (tev.lbr < 0) {
						uvr[0] = -1; uvr[1] = -1;
					} else {
						uvl[0] = -1; uvl[1] = -1;
					}


				} else {

					//if xyz-style event, reproject onto the image plane
					xyz[0] = tev.x; xyz[1] = tev.y; xyz[2] = tev.z; xyz[3] = 1;
					uvl = Pl*rtToL*xyz;
					uvr = Pr*rtToR*xyz;
					uvl[0] = uvl[0]/uvl[2]; uvl[1] = uvl[1]/uvl[2];
					uvr[0] = uvr[0]/uvr[2]; uvr[1] = uvr[1]/uvr[2];

				}

				//provide inhibition at the location
				if (mode) {

					//threshold-style inhibition (not yet functioning -- TODO)
					if (uvl[0] > 0 && uvl[0] < pSalL->width() &&
							uvl[1] > 0 && uvl[1] < pSalL->height()) {
						//floodFill(Sl, ffMskL, Point((int)uvl[0], (int)uvl[1]), Scalar(255*exp(-tev.nframes*decay)),
						//		NULL, Scalar((1-pthr)*tev.val), Scalar((1-pthr)*tev.val));
						circle(Sl, Point((int)uvl[0], (int)uvl[1]), pthr, Scalar(tev.val*exp(-tev.nframes*decay)), -1);

					}
					if (uvr[0] > 0 && uvr[0] < pSalR->width() &&
							uvr[1] > 0 && uvr[1] < pSalR->height()) {
						//floodFill(Sr, ffMskR, Point((int)uvr[0], (int)uvr[1]), Scalar(255*exp(-tev.nframes*decay)),
						//		NULL, Scalar((1-pthr)*tev.val), Scalar((1-pthr)*tev.val));
						circle(Sr, Point((int)uvr[0], (int)uvr[1]), pthr, Scalar(tev.val*exp(-tev.nframes*decay)), -1);
					}
				}
				else {

					//gaussian style inhibition -- place a delta at each projected point
					if (uvl[0] > 0 && uvl[0] < pSalL->width() &&
							uvl[1] > 0 && uvl[1] < pSalL->height()) {
						Sl.at<float>((int)uvl[1],(int)uvl[0]) = 255*exp(-tev.nframes*decay);
					}
					if (uvr[0] > 0 && uvr[0] < pSalR->width() &&
							uvr[1] > 0 && uvr[1] < pSalR->height()) {
						Sr.at<float>((int)uvr[1],(int)uvr[0]) = 255*exp(-tev.nframes*decay);
					}
				}

				//check to see if the event should expire after this frame
				if (tev.nframes > maxtime) {
					dcount++;
				} else {
					tev.nframes++;
				}

			}

			//do final processing on inhibition maps
			if (mode) {
				Sl.copyTo(Sl, ffMskL);
				Sr.copyTo(Sr, ffMskR);
			}
			else {
				GaussianBlur(Sl, Sl, Size(grad,grad), grad/5.0, grad/5.0);
				GaussianBlur(Sr, Sr, Size(grad,grad), grad/5.0, grad/5.0);
				Sl = Sl*gscale;
				Sr = Sr*gscale;
			}

			//retire any ior events that are too old
			buf.lock();
			for (int i = 0; i < dcount; i++) {
				buf.pop_front();
			}
			buf.unlock();

			//write out inhibition to port
			portSalLO->write();
			portSalRO->write();

		}

	}

	virtual void threadRelease()
	{


		portSalL->interrupt();
		portSalR->interrupt();
		portSalLO->interrupt();
		portSalRO->interrupt();
		portHAngIn->interrupt();
		portEventIn->interrupt();

		portSalL->close();
		portSalR->close();
		portSalLO->close();
		portSalRO->close();
		portHAngIn->close();
		portEventIn->close();

		delete portSalL, portSalR;
		delete portSalLO, portSalRO;
		delete portHAngIn;
		delete portEventIn;

	}

};

class stereoIORModule: public RFModule
{

protected:

	stereoIORThread *thr;

public:

	stereoIORModule() { }

	virtual bool configure(ResourceFinder &rf)
	{

		thr=new stereoIORThread(rf);
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

	stereoIORModule mod;

	return mod.runModule(rf);
}
