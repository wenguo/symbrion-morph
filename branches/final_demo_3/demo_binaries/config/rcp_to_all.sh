#!/bin/bash    
#robotIPs=(211 56 216 61)
robotIPs=(134 56 155 61 136)

for i in ${robotIPs[@]}
do
    echo copy files to 192.168.52.$i
    ./rcp_to_robot.sh $i

done    

#./rcp2robot.sh 52
