#!/bin/bash    
robotIPs=(211 56 216 61 224)
#robotIPs=(136 56 134 61 155)

for i in ${robotIPs[@]}
do
    echo copy files to 192.168.52.$i
    ./rcp2robot.sh $i ${1}

done    

#./rcp2robot.sh 52
