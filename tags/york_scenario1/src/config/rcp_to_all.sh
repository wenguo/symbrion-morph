#!/bin/bash    
#robotIPs=(61 211 56 216 224 52 155)
robotIPs=(136 134 155 224)

for i in ${robotIPs[@]}
do
    echo copy files to 192.168.52.$i
    ./rcp_to_robot.sh $i

done    

#./rcp2robot.sh 52
