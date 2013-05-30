#!/bin/sh

if [ $# -ne 1 ] ; then
	echo "usage: rcp2robot.sh NR"
	exit 1
fi

FILES="aw_option.cfg \
       scout_option.cfg\
       kit_option.cfg"
counter=0
for f in $FILES
do
    rcp root@192.168.52.${1}:/flash/morph/$f $f.${1}
done
