# Software configurations for which the apps are setup
############################################################
The iCub configuration for which all these apps are designed and requires
yarp.2.3.20 and icub.1.1.11.

# Before starting the apps
############################################################
A yarp network must exist in order to run any of these apps and yarp nodes must be initialized on the pc104 and marvin
and optionally on cluster-bert or cluster-ernie. To do this you must run the following commands from two different 
terminal windows on marvin.

$ yarpserver

and 

$ yarprun --server /marvin

Next you'll need to ssh into the iCub and run the following (assuming the robot is already running and gone through it's calibration sequence).

$ yarprun --server /pc104

# Running the apps
############################################################

To access the apps in this folder and run them on the iCub, run the following command from the same directory as this file.

$ gyarpmanager

Once gyarpmanager is up and running, select the applications folder from the menu one the left hand side of the gui 
and choose the app you desire. From there you should be able to hit "Run" and then "Connect". 

Your app should now be running. 
