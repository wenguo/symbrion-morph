#!/bin/python

import sys

if( len(sys.argv) > 1):
    ir_file = open(sys.argv[1],'r')
else:
    print "No file name provided. Exiting"
    exit

data = []

for line in ir_file:
    new_row = [ int(x) if x.lstrip('-').isdigit() else x for x in line.split() ]
    data.append( new_row )


print "Analysing the last 100 timesteps from a possible "+str(len(data))


last_100 = []
for x in range( int(sys.argv[2]), len(data) ):
   last_100.append( data[x] )

reflective_hist0 = [ x[2] for x in last_100 ]
reflective_hist1 = [ x[3] for x in last_100 ]
beacon0 = [ x[4] for x in last_100 ]
beacon1 = [ x[5] for x in last_100 ]
ambient_hist0 = [ x[6] for x in last_100 ]
ambient_hist1 = [ x[7] for x in last_100 ]
proximity0 = [ x[8] for x in last_100 ]
proximity1 = [ x[9] for x in last_100 ]

print "---------------------------------"
print "reflective_hist0: "+str(sum(reflective_hist0)/len(reflective_hist0))+" "+str(max(reflective_hist0))+" - "+str(min(reflective_hist0))
print "reflective_hist1: "+str(sum(reflective_hist1)/len(reflective_hist1))+" "+str(max(reflective_hist1))+" - "+str(min(reflective_hist1))
print "beacon0: "+str(sum(beacon0)/len(beacon0))+" "+str(max(beacon0))+" - "+str(min(beacon0))
print "beacon1: "+str(sum(beacon1)/len(beacon1))+" "+str(max(beacon1))+" - "+str(min(beacon1))
print "ambient_hist0: "+str(sum(ambient_hist0)/len(ambient_hist0))+" "+str(max(ambient_hist0))+" - "+str(min(ambient_hist0))
print "ambient_hist1: "+str(sum(ambient_hist1)/len(ambient_hist1))+" "+str(max(ambient_hist1))+" - "+str(min(ambient_hist1))
print "proximity0: "+str(sum(proximity0)/len(proximity0))+" "+str(max(proximity0))+" - "+str(min(proximity0))
print "proximity1: "+str(sum(proximity1)/len(proximity1))+" "+str(max(proximity1))+" - "+str(min(proximity1))
print "---------------------------------"



