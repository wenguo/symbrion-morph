#!/bin/bash    
robotIPs=(61 211 56 216 224 52 155)

for i in ${robotIPs[@]}
do
    echo copy files to 192.168.52.$i
    ./rcp_from_robot.sh $i ${1}

done    

#./rcp2robot.sh 52
