#!/bin/bash

if [ $# = 0 ]; then
    robotIPs=(61 159 211 56 216 224 52)
    cmd=killRobot.sh
elif [ $1 = 0 ]; then
    robotIPs=(61 159 211 56 216 224 52)
    cmd=connectRobot.sh
elif [ $1 = 1 ]; then
    robotIPs=(211 61 159 56 216 52 224)
    cmd=enablePowersharing.sh
fi


sudo ifconfig eth0 192.168.52.100

prefix=''

for i in {0..6}
do
   delay=`echo sleep $i`
   # argument=`echo $argument --tab -e \"bash -c \'sleep $i\'; echo test\" -t ${robotIPs[$i]}`
    argument=`echo $argument --tab -e \"bash -c \'$delay\'\;\'/home/wliu/self-repair/src/${cmd} ${robotIPs[$i]}\'\" -t ${robotIPs[$i]}`
done
    echo $argument | xargs gnome-terminal


