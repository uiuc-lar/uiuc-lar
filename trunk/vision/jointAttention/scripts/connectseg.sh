yarp connect /myFaceDetector/feat:o /myGazeEval/feat:i
yarp connect /myFaceDetector/head:o /myGazeEval/head:i
yarp connect /myGazeEval/map:o /mySalAgg/gaze:i
sleep 2
yarp connect /imageloader/out /myFaceDetector/img:i
yarp connect /imageloader/out /mySalAgg/img:i
yarp connect /mySalAgg/img:o /yarpview/img:i
