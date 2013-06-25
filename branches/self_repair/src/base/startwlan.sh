#!/bin/sh

echo 7 > /sys/class/gpio/export

echo out > /sys/class/gpio/gpio7/direction

echo 1 > /sys/class/gpio/gpio7/value
