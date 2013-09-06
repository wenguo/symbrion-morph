#!/bin/bash    
robotIPs=(211 56 216 61)
#robotIPs=(136 134 155 224)

for i in ${robotIPs[@]}
do
    echo copy files to 192.168.52.$i
    ./rcp_from_robot.sh $i ${1}

done    

#./rcp2robot.sh 52
