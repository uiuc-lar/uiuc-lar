#!/bin/sh

# make some windows
#gnome-terminal -e 'yarpview --name /maze/view:o --x 640 --y 0 --synch'
#gnome-terminal -e 'yarpview --name /maze/view:lo --x 960 --y 0 --synch'
#gnome-terminal -e 'yarpview --name /maze/view:ro --x 1280 --y 0 --synch'

# connect them
yarp connect /maze/img:o /maze/view:o
yarp connect /maze/img:lo /maze/view:lo
yarp connect /maze/img:ro /maze/view:ro
