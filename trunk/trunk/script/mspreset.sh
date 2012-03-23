#!/bin/sh

DIR=/sys/class/gpio/gpio35

if [ -d $DIR ];
then echo GPIO already set
else echo Set GPIO for Reset
echo 35 > /sys/class/gpio/export
fi
echo out > /sys/class/gpio/gpio35/direction
echo 0 > /sys/class/gpio/gpio35/value
sleep 1
echo 1 > /sys/class/gpio/gpio35/value
echo MSPs reset
