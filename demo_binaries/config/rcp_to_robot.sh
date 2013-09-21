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
    if [ -f $f.${1} ]; then
        echo "  " $f.${1} " --> 192.168.52.${1}/flash/morph/$f"
        rcp  $f.${1} root@192.168.52.${1}:/flash/morph/demo3/$f
    #else
    #    echo $f.${1} "is not found"
    fi
done
