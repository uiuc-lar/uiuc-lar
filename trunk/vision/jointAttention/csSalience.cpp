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
 *  csSalience.cpp
 *
 * 	Logan Niehaus
 * 	7/30/11
 * 	center surround based salience algorithm implementation. based on:
 *
 *   Random Center Surround Bottom up Visual Attention Model useful for Salient Region Detection
 *   Vikram et al., 2011
 *
 *  see paper for details. a number of parameters have been made controllable here
 *  in order to improve algorithm performance
 *
 *  inputs:
 *		/csSalience/img:i	-- rgb input image
 *
 *
 *	params:
 *		sratio		-- number of pixels to sample, as a ratio of image w*h (D 0.03)
 *		scale		-- scale factor to apply to final image (D 1.0)
 *		mfsize		-- window size of median filter (D 3)
 *		ds			-- downsampling image by ds before processing (D 1 -- no resizing)
 *		name		-- module port basename (D /csSalience)
 *
 *	outputs:
 *		/csSalience/map:o	-- output salience image (pixelfloat)
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

#include <cv.h>

#include <string>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <deque>

#include <gsl/gsl_randist.h>
#include <gsl/gsl_rng.h>

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


class csSalThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *portImgIn;
	BufferedPort<ImageOf<PixelFloat> > *portImgOut;

	double sratio, scale;
	int ds;
	int mfsize;
	int d1, d2;

	//rng
	const gsl_rng_type * T;
	gsl_rng * r;

public:

	csSalThread(ResourceFinder &_rf) : RateThread(100), rf(_rf)
	{ }

	virtual bool threadInit()
	{

		name=rf.check("name",Value("csSalience")).asString().c_str();
		sratio = rf.check("sratio", Value(0.03)).asDouble();
		ds = rf.check("ds",Value(1)).asInt();
		mfsize = rf.check("mfsize",Value(3)).asInt();
		scale = rf.check("scale", Value(1.0)).asDouble();

		portImgIn=new BufferedPort<ImageOf<PixelRgb> >;
		string portInName="/"+name+"/img:i";
		portImgIn->open(portInName.c_str());

		portImgOut=new BufferedPort<ImageOf<PixelFloat> >;
		string portOutName="/"+name+"/map:o";
		portImgOut->open(portOutName.c_str());

		//setup rng
		gsl_rng_default_seed = time(NULL);
		gsl_rng_env_setup();
		T = gsl_rng_default;
		r = gsl_rng_alloc (T);

		return true;

	}

	void centerSurroundSal(const Mat *M, Mat *N, int d1, int d2) {

		int x1, y1, x2, y2, dst;
		double * fv = new double[3];
		N[0] = Mat::zeros(M[0].rows, M[0].cols, CV_32F);
		N[1] = Mat::zeros(M[1].rows, M[1].cols, CV_32F);
		N[2] = Mat::zeros(M[2].rows, M[2].cols, CV_32F);
		for (int i = 0; i < d1; i++) {
			x1 = (int)(N[0].cols*gsl_rng_uniform(r));
			y1 = (int)(N[0].rows*gsl_rng_uniform(r));
			for (int k = 0; k < 3; k++) {
				fv[k] = M[k].at<float>(y1,x1);
			}
			for (int j = 0; j < d2; j++) {
				x2 = (int)(N[0].cols*gsl_rng_uniform(r));
				y2 = (int)(N[0].rows*gsl_rng_uniform(r));
				if (!(x1 == x2 && y1 == y2)) {
					dst = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
					for (int k = 0; k < 3; k++) {
						N[k].at<float>(y2,x2) = N[k].at<float>(y2,x2) +
								abs(fv[k]-M[k].at<float>(y2,x2))/dst;
					}
				}
			}
		}

	}

	virtual void run()
	{

		// get inputs
		ImageOf<PixelRgb> *pImgIn=portImgIn->read(false);

		// process camera image
		if (pImgIn)
		{

			ImageOf<PixelFloat> &imgOut=portImgOut->prepare();

			double timein = Time::now();

			//convert to L*a*b space
			Mat I((IplImage *)pImgIn->getIplImage(), true);
			Mat J, K;
			cvtColor(I,I,CV_RGB2Lab);
			I.convertTo(J, CV_32FC3);


			//downsample for processing if so desired
			if (ds > 1) {
				resize(J, K, Size(), 1.0/(float)ds, 1.0/(float)ds);
			}

			//split the image up into channels
			Mat * C = new Mat[3];
			split(K, C);

			//undo the scaling done by cvtColor
			C[0] = C[0]*100.0/255.0;
			C[1] = C[1]-128.0;
			C[2] = C[2]-128.0;

			//get single channel center-surround salience values
			Mat * S = new Mat[3];
			Mat L, A, B, Ls, As, Bs;
			d1 = d2 = (int)(sratio*K.rows*K.cols);
			centerSurroundSal(C, S, d1, d2);

			//median filter each channel
			if (mfsize == 1) {
				S[0].copyTo(Ls);
				S[1].copyTo(As);
				S[2].copyTo(Bs);
			} else {
				medianBlur(S[0], Ls, mfsize);
				medianBlur(S[1], As, mfsize);
				medianBlur(S[2], Bs, mfsize);
			}

			//resample to original image size
			if (ds > 1) {
				resize(Ls, L, Size(J.cols, J.rows));
				resize(As, A, Size(J.cols, J.rows));
				resize(Bs, B, Size(J.cols, J.rows));
			}

			//create the combined map
			imgOut.resize(*pImgIn);
			double val;
			for (int i = 0; i < imgOut.width(); i++) {
				for (int j = 0; j < imgOut.height(); j++) {
					val = scale*sqrt(L.at<float>(j,i)*L.at<float>(j,i) +
							A.at<float>(j,i)*A.at<float>(j,i) +
							B.at<float>(j,i)*B.at<float>(j,i));
					imgOut.pixel(i,j) = val;
				}
			}
			portImgOut->write();

		}
	}

	virtual void threadRelease()
	{

		portImgIn->interrupt();
		portImgOut->interrupt();
		portImgIn->close();
		portImgOut->close();

		delete portImgIn;
		delete portImgOut;

		gsl_rng_free (r);

	}

};

class csSalModule: public RFModule
{
protected:
	csSalThread *thr;

public:
	csSalModule() { }

	virtual bool configure(ResourceFinder &rf)
	{
		Time::turboBoost();

		thr=new csSalThread(rf);
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

	csSalModule mod;

	return mod.runModule(rf);
}



