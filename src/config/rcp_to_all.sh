#!/bin/bash    
robotIPs=(136 56 134 61 155)

for i in ${robotIPs[@]}
do
    echo copy files to 192.168.52.$i
    ./rcp_to_robot.sh $i

done    

#./rcp2robot.sh 52
