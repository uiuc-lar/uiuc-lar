/*
 * dispMap.cpp
 *
 */

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <cv.h>

using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

class dispMapThread : public RateThread{
protected:
	ResourceFinder &rf;
	string name;

	BufferedPort<ImageOf<PixelRgb> > *camLeft;
	BufferedPort<ImageOf<PixelRgb> > *camRight;
	BufferedPort<ImageOf<PixelMono16> > *dispImg;

public:
	dispMapThread(int period, ResourceFinder &_rf) : RateThread(period), rf(_rf) {}

	virtual bool threadInit(){

	}

};

int main(int argc, char *argv[]){

	return 0;
}
