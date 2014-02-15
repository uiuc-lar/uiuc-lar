/*
 * Copyright (C) 2013 Logan Niehaus
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
 *  objSegTrackTrack.cpp
 *
 * 	Logan Niehaus
 * 	9/26/13
 * 	module for 3d object segmentation and tracking, based on the watershed and meanshift algorithms
 *
 *  segmentation desc: looks for colorful or dark blobs on a white table/background
 *  performs an initial watershed based segmentation to get object blobs
 *  for each blob, index it and calculate a basic color histogram model for it in L*a*b space
 *  to track to next image, backproject the histograms for each blob onto the image and apply meanshift algorithm
 *  use meanshift estimates to track blobs, refining with watershed segmentation
 *  output a segmentation map and try to keep track blobs marked with same index across maps
 *
 *
 *  inputs:
 *  	/objSegTrack/img:i		-- RGB input image as described above
 *
 *  params:
 *  	gthresh		-- inverted rgb threshold for watershed segmentation
 *  	sthresh		-- saturation channel threshold for rgb segmentation
 *  	minobjsize	-- minimum segmented obj size; smaller objects not chosen (D 100.0)
 *  	maxobjsize	-- maximum segmented obj size; larger objects not chosen (D 5000.0)
 *  	tableloc	-- number of pixels above the horizontal center line the workspace extends. only
 *  					objects with centroids below this line will be segmented (D 10)
 *  	cplow, cphi	-- canny thresholding parameters. see canny edge detection algorithm explanation (D 140, 150)
 *  	name		-- module ports basename (D /objSegTrack)
 *  	rate		-- update rate in ms; objs segmented and published at this rate (D 50)
 *
 *  outputs:
 *  	/objSegTrack/map:o	-- binary image with object regions labeled as 1
 *
 *  TODO:
 *  	need to make this a lot more configurable, and a lot more general. pretty hackish right now
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/math/Math.h>

#include <cv.h>

#include <string>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

#define PI 3.14159

//void smoothHist(Mat &, int *, int, double);


class Blob {

private:

	//current bounding rectangle for the blob
	Rect br;
	Rect bro;

	//normalized color histogram of the blob
	Mat H;

	//index label for segmentation
	int idx;


public:

	//constructor
	Blob(Mat Hi, Rect bri, int idxi) : br(bri), idx(idxi)
	{

		Hi.copyTo(H);

	}

	//deconstructor
	~Blob() {

		//H->release();
		//delete H;
	}

	Mat * getH() { return &H; }
	Rect getR() { return br; }
	void setR(Rect _br) {
		bro = br;
		br = _br;
	}
	void revR() { br = bro; }
	int getIdx() { return idx; }

};



class objSegTrackThread : public RateThread
{
protected:

	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *portImgIn;
	BufferedPort<ImageOf<PixelMono> > *portImgOut;


	//algorithm parameters
	double gthresh, sthresh;
	int minobjsize, maxobjsize, tableloc;
	double cplow, cphi;
	int inactthresh;
	int hlim, wlim, alim;
	double hsmooth;

	//data containers
	int nobs;
	vector<int> * isActive;
	vector<int> * cntInAct;
	vector<Blob> * objects;

	Mat * BH; //background histogram
	double bhScale;



public:

	objSegTrackThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
	{ }

	virtual bool threadInit()
	{

		name=rf.check("name",Value("objSegTrack")).asString().c_str();
		cplow = rf.check("cplow",Value(80.0)).asDouble();
		cphi = rf.check("cphi",Value(150.0)).asDouble();
		gthresh = rf.check("gthresh",Value(150.0)).asDouble();
		sthresh = rf.check("sthresh",Value(30.0)).asDouble();
		minobjsize = rf.check("minobjsize",Value(50)).asInt();
		maxobjsize = rf.check("maxobjsize",Value(5000)).asInt();
		inactthresh = rf.check("inactthresh",Value(50)).asInt();
		hlim = rf.check("hlim",Value(150)).asInt();
		wlim = rf.check("wlim",Value(200)).asInt();
		alim = rf.check("alim",Value(15000)).asInt();
		hsmooth = rf.check("hsmooth",Value(0.5)).asDouble();



		portImgIn=new BufferedPort<ImageOf<PixelRgb> >;
		string portInName="/"+name+"/img:i";
		portImgIn->open(portInName.c_str());

		portImgOut=new BufferedPort<ImageOf<PixelMono> >;
		string portOutName="/"+name+"/img:o";
		portImgOut->open(portOutName.c_str());

		//initialize data objects
		objects = new vector<Blob>;
		isActive = new vector<int>;
		cntInAct = new vector<int>;
		nobs = 0;
		BH = NULL;


		return true;

	}

	/* run a watershed segmentation on an image, or refine an initial segmentation using
	 * the watershed algorithm
	 */

	virtual void wsSegmentObjects(Mat * IM, Mat &OM, Mat * preLabel, int loff = -1, Mat * bLabel = NULL) {

		int clab = 1;

		Mat T(IM->rows, IM->cols, CV_8UC1);
		Mat C(IM->rows, IM->cols, CV_8UC1);
		Mat Tmp(IM->rows, IM->cols, CV_8UC1);
		Mat L = Mat::zeros(IM->rows, IM->cols, CV_32S);


		//get the pre-initialized labels if they exist
		if (preLabel) {

			preLabel->convertTo(L, CV_32S);

		}

		//get offset to start labeling at
		if (loff < 0) {

			double moff;
			minMaxIdx(L, NULL, &moff);
			loff = (int) moff;

		}


		//convert to gray and HSV (S channel)
		Mat G(IM->rows, IM->cols, CV_8UC1);
		Mat HSVt(IM->rows, IM->cols, CV_8UC3);
		Mat S(IM->rows, IM->cols, CV_8UC1);

		int * frto = new int[2];
		frto[0] = 1; frto[1] = 0;

		cvtColor(*IM, HSVt, CV_RGB2HSV);
		mixChannels(&HSVt, 1, &S, 1, frto, 1);
		cvtColor(*IM, G, CV_RGB2GRAY);

		//threshold the gray and sat images and combine them
		T = Mat::zeros(IM->rows, IM->cols, CV_8UC1);
		threshold(G, Tmp, 255-gthresh, 255.0, CV_THRESH_BINARY_INV);
		max(T,Tmp,T);
		threshold(S, Tmp, sthresh, 255.0, CV_THRESH_BINARY);
		max(T,Tmp,T);

		//pre-clear some of the really small flecks for later efficiency
		erode(T, T, Mat());

		//perform the canny edge detection on gray and sat images, and combine them
		C = Mat::zeros(IM->rows, IM->cols, CV_8UC1);
		Canny(G, Tmp, cplow, cphi, 3, true);
		max(C,Tmp,C);
		Canny(S, Tmp, cplow, cphi, 3, true);
		max(C,Tmp,C);

		//widen the canny edges
		dilate(C, C, Mat());

		//use the edge map to break apart touching objects
		T.setTo(Scalar(0),C);

		//find the major blobs
		vector<vector<Point> > contours;
		Mat X;
		T.copyTo(X);
		findContours(X, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		for (int i = 0; i < contours.size(); i++) {

			double carea = contourArea(Mat(contours[i]));

			if (carea > minobjsize && carea < maxobjsize) {

				//check to see if the area within the contour has already been labeled
				double labeled, mv;
				Tmp.setTo(Scalar(0));
				drawContours(Tmp, contours, i, Scalar(255), -1);
				minMaxIdx(L, NULL, &labeled, NULL, NULL, Tmp);

				//if not
				if (labeled <= 0.5) {

					//check here if shape is roughly compact
					mv = arcLength(Mat(contours[i]),true);
					if (4*PI*contourArea(Mat(contours[i]))/(mv*mv) < .3) {

						//for complex shapes, using contours as marker init works better
						drawContours(L, contours, i, Scalar(loff+clab), -1);

					} else {

						//if it is more compact, erode it first
						erode(Tmp, Tmp, Mat());
						L.setTo(Scalar(loff+clab), Tmp);

					}

					clab++;

				}
			}
		}

		//get some general points that we consider background to mark for the watershed algorithm
		L.convertTo(Tmp, CV_8UC1);
		max(Tmp, T, Tmp);
		threshold(Tmp, T, 1, 255.0, CV_THRESH_BINARY);
		dilate(T, T, Mat(), Point(-1,-1), 10);

		if (bLabel) {
			T = T | *bLabel;
		}

		threshold(T, Tmp, 1, 255.0, CV_THRESH_BINARY_INV);

		L.setTo(loff+clab, Tmp);

		//run the watershed algorithm
		watershed(*IM, L);

		//set the background pixels to 0
		L.convertTo(Tmp, CV_8UC1);

		threshold(Tmp, OM, loff+clab-1, 0.0, CV_THRESH_TOZERO_INV);

		//exp: try to get rid of some of the strange watershed artifacts
		erode(OM, OM, Mat());
		dilate(OM, OM, Mat());

	}

	virtual void run()
	{

		float lranges[] = { 3, 66, 129, 192, 255 };
		float uranges[] = { 60,   75,   90,  105,  120,  135,  150,  165,  180 };
		float vranges[] = { 90,  100,  110,  120,  130,  140, 150,  160,  170,  180 };

		/*
		int chn[] = {0 ,1, 2};
		int hsz[] = {4, 8, 9};
		const float * hranges[] = { lranges, uranges, vranges };
		*/

		int chn[] = {1, 2};
		int hsz[] = {8, 9};
		const float * hranges[] = { uranges, vranges };

		// get inputs
		ImageOf<PixelRgb> *pImgIn=portImgIn->read(false);

		//if there is a camera image, use that
		if (pImgIn)
		{

			ImageOf<PixelMono> &imgOut= portImgOut->prepare();
			imgOut.resize(*pImgIn);
			imgOut.zero();

			Mat IM = (IplImage *) pImgIn->getIplImage();
			Mat OM = (IplImage *) imgOut.getIplImage();

			Mat S = Mat(IM.rows, IM.cols, CV_8UC1);

			//convert image to desired color space
			Mat IL(IM.rows, IM.cols, CV_8UC3);
			cvtColor(IM, IL, CV_RGB2Luv);

			//check and see if we currently have any objects
			if (nobs == 0) {

				//if no active obj (likely init) do vanilla watershed seg
				wsSegmentObjects(&IM, S, NULL);

			}

			//if we have objects, try to locate them in the image
			else {


				Mat SI = Mat::zeros(IM.rows, IM.cols, CV_8UC1);
				Mat Cmax = Mat::zeros(IM.rows, IM.cols, CV_8UC1);
				Mat BI = Mat::ones(IM.rows, IM.cols, CV_8UC1);

				//for each active object, calculate the back projection of its histogram on the image
				Mat ** BP;
				Mat * tmpH;
				Mat * tmpI;
				BP = new Mat *[nobs+1];
				Rect tmpR;

				//calculate back projection of background model
				//printf("grenk, nobs: %d\n", nobs);
				BP[nobs] = new Mat(IM.rows, IM.cols, CV_8UC1);
				calcBackProject(&IL, 1, chn, *BH, *(BP[nobs]), hranges, bhScale, false);
				BP[nobs]->copyTo(Cmax);
				max(Cmax, 2, Cmax);

				for (int i = 0; i < nobs; i++) {

					//only do this for active blobs
					if ((*isActive)[i]) {

						int tidx = (*objects)[i].getIdx();

						BP[i] = new Mat(IM.rows, IM.cols, CV_8UC1);
						//Mat bpMsk = Mat::ones(IM.rows, IM.cols, CV_8UC1);
						tmpI = BP[i];

						tmpH = (*objects)[i].getH();
						tmpR = (*objects)[i].getR();

						double tScale = 255.0/norm(*tmpH, NORM_L1);

						calcBackProject(&IL, 1, chn, *tmpH, *tmpI, hranges, tScale, false);

						/*
						//assume that the object is not moving very fast, blank out all parts of the map not
						//within some region of the old bounding box
						for (int j = 0; j < tmpI->rows; j++) {
							for (int k = 0; k < tmpI->cols; k++) {
								if (!(j > tmpR.x-35 && j < tmpR.x+tmpR.width+35 &&
										k > tmpR.y-35 && k < tmpR.y+tmpR.height+35)) {
									tmpI->at<uchar>(j,k) = 0;
								}
							}
						}
						*/

						medianBlur(*tmpI, *tmpI, 3);


						//perform meanshift for each object to get location in this image
						//only do this if the object was active in the last frame
						if ((*cntInAct)[i] < 1) {
							meanShift(*tmpI, tmpR, TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
							(*objects)[i].setR(tmpR);
						}
						//otherwise, use last known location as potential bounding box
						else {
							tmpR = (*objects)[i].getR();
						}

						//if the bounding box for an object has grown too large, it probably indicates object is
						//marking background pixels
						if (tmpR.height > hlim || tmpR.width > wlim || tmpR.height*tmpR.width > alim) {

							(*isActive)[i] = 0;

						}
						else {

							//use this to paint the initial segmentation map
							Mat ti = Mat(*tmpI, tmpR);
							Mat ci = Mat(Cmax, tmpR);
							Mat msk = Mat::zeros(ti.rows, ti.cols, CV_8UC1);

							msk = ti > ci;
							ci = max(ti, ci);
							//erode(msk, msk, Mat());

							for (int j = 0; j < ti.rows; j++) {
								for (int k = 0; k < ti.cols; k++) {
									if (msk.at<uchar>(j,k) > 0) {
										SI.at<uchar>(tmpR.y+j,tmpR.x+k) = tidx;
										Cmax.at<uchar>(tmpR.y+j,tmpR.x+k) = ti.at<uchar>(j,k);
									}
									BI.at<uchar>(tmpR.y+j,tmpR.x+k) = 0;
								}
							}
						}
					}

				}


				dilate(SI,SI,Mat());
				erode(SI, SI, Mat(), Point(-1,-1), 2);

				BI = BI & (Cmax > 2);
				erode(BI, BI, Mat(), Point(-1,-1), 3);

				//refine with watershed algorithm
				wsSegmentObjects(&IM, S, &SI, nobs, &BI);
				S.copyTo(OM);
				//SI.copyTo(OM);

			}

			//after watershed, check to see if any objects have been added
			double mxlab;
			minMaxIdx(S, NULL, &mxlab, NULL, NULL);

			//update the bounding rectangle for all objects
			Mat pmsk(IM.rows, IM.cols, CV_8UC1);
			vector<vector<Point> > contours;
			vector<Point> mcontour;
			for (int i = 0; i < nobs; i++) {

				if ((*isActive)[i]) {

					pmsk = S == (*objects)[i].getIdx();

					if (countNonZero(pmsk) > minobjsize) {
						findContours(pmsk, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
						mcontour = contours[0];
						for (int j = 1; j < contours.size(); j++) {
							mcontour.insert(mcontour.end(), contours[j].begin(), contours[j].end());
						}
						(*objects)[i].setR(boundingRect(mcontour));
						(*cntInAct)[i] = 0;
					}

					//if an object did not appear after the WS seg, mark it as being inactive for this frame
					else {

						(*cntInAct)[i] += 1;

						//revert its bb, as we consider the update invalid
						(*objects)[i].revR();

						//if the object has been inactive for enough time, mark it as dead
						if ((*cntInAct)[i] > inactthresh) {
							printf("OBJ %d marked as inactive\n",i);
							(*isActive)[i] = 0;
						}

						if ((*cntInAct)[i] == 1) {
							printf("OBJ %d has just gone inactive\n",i);
						}
					}
				}
			}

			//calculate the histograms for new objects
			for (int i = nobs; i < (int)mxlab; i++) {

				printf("New OBJ %d added\n",i+1);

				Mat tHist;

				//get the mask for pixels belonging to that blob
				pmsk = S == (i+1);

				//calculate the bounding box
				findContours(pmsk, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
				mcontour = contours[0];
				for (int j = 1; j < contours.size(); j++) {
					mcontour.insert(mcontour.end(), contours[j].begin(), contours[j].end());
				}
				Rect br = boundingRect(mcontour);

				//calculate the histogram and normalize
				calcHist(&IL, 1, chn, pmsk, tHist, 2, hsz, hranges, false);
				GaussianBlur(tHist, tHist, Size(3,3), hsmooth);


				//create a descriptor for the blob and add it to the list
				Blob b(tHist, br, i+1);
				objects->push_back(b);
				isActive->push_back(1);
				cntInAct->push_back(0);

			}

			nobs = objects->size();

			//if background model has not been initialized yet
			if (!BH && nobs > 0) {

				Mat tHist;
				BH = new Mat;

				//get the mask for pixels belonging to background
				pmsk = S == 0;

				//calculate the histogram and normalize
				calcHist(&IL, 1, chn, pmsk, tHist, 2, hsz, hranges, false);
				GaussianBlur(tHist, tHist, Size(3,3), hsmooth);

				bhScale = 255.0/norm(tHist,NORM_L1);

				tHist.copyTo(*BH);

			}


			//S.convertTo(OM, CV_8UC1);
			normalize(OM, OM, 255, 0, NORM_INF);

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

	}

};

class objSegTrackModule: public RFModule
{

protected:

	objSegTrackThread *thr;

public:
	objSegTrackModule() { }

	virtual bool configure(ResourceFinder &rf)
	{
		Time::turboBoost();

		thr=new objSegTrackThread(rf);
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

	objSegTrackModule mod;

	return mod.runModule(rf);
}

/*
void smoothHist(Mat &H, int *hsize, int ksz, double sig) {

	//get gaussian kernel first
	Mat G = getGaussianKernel(ksz, sig);
	Mat Ht;

	//printf("crent: %d\n",H.dims);

	H.copyTo(Ht);

	//printf("forf: %d %d %d\n",hsize[0],hsize[1],hsize[2]);

	for (int i = 0; i < hsize[1]; i++) {
		for (int j = 0; j < hsize[2]; j++) {

			for (int k = 0; k < hsize[0]; k++) {
				//printf("blif: %d %d %d\n",k,i,j);
				H.at<double>(k,i,j) = 0;
				for (int l = -ksz/2; l < ksz/2+1; l++) {
					if (k+l>=0 && k+l<hsize[0]) {
						H.at<double>(k,i,j) += G.at<double>(ksz/2+l)*Ht.at<double>(k+l,i,j);
					}
				}
			}
		}
	}

	for (int i = 0; i < hsize[0]; i++) {
		for (int j = 0; j < hsize[2]; j++) {

			for (int k = 0; k < hsize[1]; k++) {
				H.at<double>(i,k,j) = 0;
				for (int l = -ksz/2; l < ksz/2+1; l++) {
					if (k+l>=0 && k+l<hsize[1]) {
						H.at<double>(i,k,j) += G.at<double>(ksz/2+l)*Ht.at<double>(i,k+l,j);
					}
				}
			}
		}
	}

	for (int i = 0; i < hsize[0]; i++) {
		for (int j = 0; j < hsize[1]; j++) {

			for (int k = 0; k < hsize[2]; k++) {
				H.at<double>(i,j,k) = 0;
				for (int l = -ksz/2; l < ksz/2+1; l++) {
					if (k+l>=0 && k+l<hsize[2]) {
						H.at<double>(i,j,k) += G.at<double>(ksz/2+l)*Ht.at<double>(i,j,k+l);
					}
				}
			}
		}
	}


}
*/
