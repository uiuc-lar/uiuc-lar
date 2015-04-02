yarp connect /myFaceDetector/feat:o /myGazeLearner/feat:i
yarp connect /myShakeDetector/loc:o /myGazeLearner/loc:i
yarp connect /myFaceDetector/head:o /myGazeLearner/head:i
sleep 2
yarp connect /imageloader/out /myShakeDetector/img:i
yarp connect /imageloader/out /myFaceDetector/img:i
