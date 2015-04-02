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


protected:

	//feature vector masks
	bool avcolor;
	bool waviness;
	bool circularity;
	bool squareness;
	bool aspratio;


public:

	//constructors/destructors
	visFeatures()
	: avcolor(true), waviness(true), circularity(true), squareness(true), aspratio(true) {	}
	visFeatures(bool all)
	: avcolor(all), waviness(all), circularity(all), squareness(all), aspratio(all) {  }
	visFeatures(bool avc, bool wav, bool circ, bool squar, bool aspr) :
		avcolor(avc), waviness(wav), circularity(circ), squareness(squar), aspratio(aspr) { }

	void featAvcolor(bool flag) { avcolor = flag; }
	void featWaviness(bool flag) { waviness = flag; }
	void featCircularity(bool flag) { circularity = flag; }
	void featSquareness(bool flag) { squareness = flag; }
	void featAspratio(bool flag) { aspratio = flag; }
	void setAllFeatures(bool flag) {
		avcolor = flag;
		waviness = flag;
		circularity = flag;
		squareness = flag;
		aspratio = flag;
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
		if (avcolor) {
			int bSize = countNonZero(M);
			M = Mat::ones(_IM.height(), _IM.width(), CV_8UC1) - M;
			I.setTo(Scalar(0,0,0),M);
			Scalar aVal = sum(I);
			f.push_back((aVal[1]/(double)bSize-128.0)/64.0);
			f.push_back((aVal[2]/(double)bSize-128.0)/64.0);
		}

		//find convex hull and its area
		cArea = contourArea(Mat(vp));
		convexHull(Mat(vp), hp);
		hArea = contourArea(Mat(hp));

		//calculate 'waviness'
		if (waviness) {
			f.push_back(arcLength(Mat(hp),true)/arcLength(Mat(vp),true));
		}

		//calculate 'circularity'
		if (circularity) {
			f.push_back(4*PI*cArea/(arcLength(Mat(vp),true)*arcLength(Mat(vp),true)));
		}

		//calculate 'squareness'
		RotatedRect RR = minAreaRect(Mat(vp));
		if (squareness) {
			f.push_back(hArea/(RR.size.width*RR.size.height));
		}

		//calculate aspect ratio
		if (aspratio) {
			if (RR.size.width/RR.size.height > 1) {
				f.push_back(RR.size.width/RR.size.height);
			} else {
				f.push_back(RR.size.height/RR.size.width);
			}
		}

		return f;

	}



};
