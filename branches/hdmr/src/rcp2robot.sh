#!/bin/sh

if [ $# -ne 2 ] ; then
	echo "usage: rcp2robot.sh NR xx"
	exit 1
fi

FILES="bin/morph \
    base/lib/libmorph_base.so \
    ../irobot/lib/libirobot.so"
counter=0
for f in $FILES
do
    counter=`expr ${counter} + 1`
    if [ ${counter} -le ${2} ]; then
        echo "  copying" $f 
        rcp $f root@192.168.52.${1}:/flash/morph
        #    else
        #        echo "  skip" $f
    fi
done
