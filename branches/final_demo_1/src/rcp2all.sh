#!/bin/bash    
robotIPs=(216 56 224 61 211)

for i in ${robotIPs[@]}
do
    echo copy files to 192.168.52.$i
    ./rcp2robot.sh $i ${1}

done    

#./rcp2robot.sh 52
