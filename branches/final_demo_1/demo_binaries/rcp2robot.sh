#!/bin/sh

if [ $# -ne 2 ] ; then
	echo "usage: rcp2robot.sh NR xx"
	exit 1
fi

SRC_DIR=./
#SRC_DIR=/home/wliu/workspace/symbrion-morph/branches/self_repair/src 

FILES="${SRC_DIR}/morph \
    ${SRC_DIR}/libmorph_base.so \
    ${SRC_DIR}/libirobot.so"
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
