yarp connect /myFaceDetector/feat:o /myGazeEval/feat:i
yarp connect /myFaceDetector/head:o /myGazeEval/head:i
sleep 2
yarp connect /imageloader/out /myFaceDetector/img:i
yarp connect /imageloader/out /myGazeEval/img:i
yarp connect /myGazeEval/img:o /yarpview/img:i
