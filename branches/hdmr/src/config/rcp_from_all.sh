#!/bin/bash    
#robotIPs=(61 211 56 216 224 52 155)
robotIPs=(69 220 72 200)

for i in ${robotIPs[@]}
do
    echo copy files to 192.168.52.$i
    ./rcp_from_robot.sh $i ${1}

done    

#./rcp2robot.sh 52
