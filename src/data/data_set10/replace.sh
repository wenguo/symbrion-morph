#!/bin/bash
FILES=./*.log
for f in $FILES
do
    echo "\"$f\","
#    sed -i -e "s/Debugging/99999/" $f
done
