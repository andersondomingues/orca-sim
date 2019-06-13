#!/bin/sh

#for any noc, use -s to split the window properly according to
#the width of the noc (e.g., in 2x3, use '-s 3')
#multitail ./logs/*debug.log -s 4 -D 
multitail ./logs/*debug.log -s 4
