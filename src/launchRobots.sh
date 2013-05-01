#!/bin/bash

robotIPs=(61 159 211 56 216 224 52)

sudo ifconfig eth0 192.168.52.100

prefix=''

for i in {0..6}
do
   delay=`echo sleep $i`
   # argument=`echo $argument --tab -e \"bash -c \'sleep $i\'; echo test\" -t ${robotIPs[$i]}`
    argument=`echo $argument --tab -e \"bash -c \'$delay\'\;\'/home/wliu/self-repair/src/connectRobot.sh ${robotIPs[$i]}\'\" -t ${robotIPs[$i]}`
done
    echo $argument | xargs gnome-terminal


