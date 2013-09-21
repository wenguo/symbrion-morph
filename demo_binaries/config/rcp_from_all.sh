#!/bin/bash    
robotIPs=(211 56 216 61 224)
#robotIPs=(134 56 155 61 136)

for i in ${robotIPs[@]}
do
    echo copy files to 192.168.52.$i
    ./rcp_from_robot.sh $i ${1}

done    

#./rcp2robot.sh 52
