#!/bin/sh

yarp connect /maze/img:o /maze/view:o
yarp connect /maze/img:lo /maze/view:lo
yarp connect /maze/img:ro /maze/view:ro
