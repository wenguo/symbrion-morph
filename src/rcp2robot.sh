#!/bin/sh

if [ $# -ne 1 ] ; then
	echo "usage: rcp2robot.sh NR"
	exit 1
fi

FILES="bin/morph \
	base/lib/libmorph_base.so \
	../irobot/lib/libirobot.so"

for f in $FILES
do
	rcp $f root@192.168.52.${1}:/flash/morph
done
