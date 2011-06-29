/*
 *  visFeatExtractor.h
 *
 * 	Logan Niehaus
 * 	6/12/11
 * 	class for extraction of visual features from objects
 *
 * 	these features are:
 * 		-avg object color represented as a,b components of L*a*b color space
 * 		-waviness - ratio of perimeters b/w obj and its convex hull
 * 		-circularity - 4pi*Area/Perim.^2, a measure of object circularity
 * 		-squareness	-  ratio of convex hull area to bounding box area
 * 		-aspect ratio - ratio of largest dim to smallest orth. dim
 *
 */

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <cv.h>

#include <string>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

//namespaces
using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;

#define PI 3.14159

class visFeatures {


public:

	//constructors/destructors
	visFeatures() {

	}
	~visFeatures() {

	}

	//extraction functions
	yarp::sig::Vector extractFeatures(ImageOf<PixelRgb> &_IM, Matrix &_msk) {

		vector<Point> vp,hp;
		vector<vector<Point> > objCont;
		yarp::sig::Vector f(0);
		Mat retImg((IplImage *)_IM.getIplImage(), false);
		double cArea, hArea;

		//load in image and create the object mask
		Mat I((IplImage *)_IM.getIplImage(), true);
		cvtColor(I,I,CV_RGB2Lab);
		Mat M = Mat::zeros(_IM.height(), _IM.width(), CV_8UC1);
		for (int i = 0; i < _msk.rows(); i++) {

			M.at<unsigned char>((int)_msk[i][0],(int)_msk[i][1]) = 1;
			vp.push_back(Point2f(_msk[i][1],_msk[i][0]));
			retImg.at<Vec3b>(Point2f((int)_msk[i][1],(int)_msk[i][0])) = Vec3b(255,0,0);

		}
		objCont.push_back(vp);
		drawContours(M, objCont, 0, Scalar(1), -1);

		//calculate the avg pixel value of the blob
		int bSize = countNonZero(M);
		M = Mat::ones(_IM.height(), _IM.width(), CV_8UC1) - M;
		I.setTo(Scalar(0,0,0),M);
		Scalar aVal = sum(I);
		f.push_back(aVal[1]/(double)bSize);
		f.push_back(aVal[2]/(double)bSize);

		//find convex hull and its area
		cArea = contourArea(Mat(vp));
		convexHull(Mat(vp), hp);
		hArea = contourArea(Mat(hp));

		//calculate 'waviness'
		f.push_back(arcLength(Mat(hp),true)/arcLength(Mat(vp),true));

		//calculate 'circularity'
		f.push_back(4*PI*cArea/(arcLength(Mat(vp),true)*arcLength(Mat(vp),true)));

		//calculate 'squareness'
		RotatedRect RR = minAreaRect(Mat(vp));
		f.push_back(hArea/(RR.size.width*RR.size.height));

		//calculate aspect ratio
		if (RR.size.width/RR.size.height > 1) {
			f.push_back(RR.size.width/RR.size.height);
		} else {
			f.push_back(RR.size.height/RR.size.width);
		}



		return f;

	}


protected:


};
