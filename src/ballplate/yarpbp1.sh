#!/bin/sh

yarp connect /icub/cam/left /maze/img:l
yarp connect /icub/cam/right /maze/img:r
#yarp connect /icub/head/state:o /maze/head:i
