#!/bin/sh

if [ $# -ne 2 ] ; then
	echo "usage: rcp2robot.sh NR xx"
	exit 1
fi

FILES="aw_option.cfg \
       scout_option.cfg"
counter=0
for f in $FILES
do
    counter=`expr ${counter} + 1`
    if [ ${counter} -le ${2} ]; then
        echo "  copying" $f 
        rcp root@192.168.52.${1}:/flash/morph/$f $f.${1}
    else
        echo "  skip" $f
    fi
done
