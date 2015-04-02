/*************************************************************
***                                                        ***
***  THIS FUNCTION: Header File for Balancing	           *** 
***  USED WITH: Balance a Ball on a Beam w/ PID Control    ***
***  USED WITH FILENAME: balance.cpp & all function codes  ***
***  AUTHOR: Aaron F. Silver                               ***
***  VERSION: 2.0                                          ***
***  DATE: 3/14/2012                                       ***
***  DESCRIPTION:                                          ***
***     This header file loades appropriate libraries for  ***
***  all mathematics needed for control, as well as        ***
***  libraries for loading necessary equipment onboard     ***
***  the iCub robot.                                       ***
***                                                        ***
**************************************************************/

//#include libraries
#include <stdio.h>
#include <math.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Vector.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <sys/timeb.h>

//namespaces
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;
using namespace std;


//function declarations
void initialize(IPositionControl*, int, Vector);
void vision(int, double *, double *, double *, double *, double *, double *, ImageOf<PixelRgb> *);
double calibration(int *, int *, double, double, int, double);
void dataWrite(int, int, double, double, double, double, double, double, double, double);
double control(double, double, double, double, double, double, double, double, int, double, double, int, int);


