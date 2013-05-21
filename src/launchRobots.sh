#!/bin/bash

if [ $# = 0 ]; then
    robotIPs=(61 211 56 216 224 52)
    cmd=killRobot.sh
elif [ $1 = 0 ]; then
    robotIPs=(224 52 216 56 211 61)
    cmd=connectRobot.sh
elif [ $1 = 1 ]; then
    robotIPs=(61 211 56 216 52 224)
    cmd=enablePowersharing.sh
fi


sudo ifconfig eth0 192.168.52.100

prefix=''

counter=0

for i in ${robotIPs[@]} 
do
   counter=`expr ${counter} + 1`
   delay=`echo sleep $counter`
   # argument=`echo $argument --tab -e \"bash -c \'sleep $i\'; echo test\" -t ${robotIPs[$i]}`
    argument=`echo $argument --tab -e \"bash -c \'$delay\'\;\'/home/wliu/self-repair/src/${cmd} ${i}\'\" -t ${i}`
done
    echo $argument | xargs gnome-terminal


