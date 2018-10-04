# generates all double fault scenarios for a X by Y network
# example: python 5-faults.py 3 3 0 0.5'
# python ../../../../scripts/fault/5-faults.py 4 4 0 0.5 "0.5 ms" -1 1000
# it generates 50% of the possible faults for a 4x4 network where the master is located at node 0 - 1000 scenarios is created

import sys
import random

### change the seed to control the generated scenarios
#random.seed(1234)


num = int(sys.argv[7]) 
max_x = int(sys.argv[1])
max_y = int(sys.argv[2])
master = int(sys.argv[3])
fault_prob = float(sys.argv[4])
str_fault_time = sys.argv[5]
ports = 8
wires = 20
i=0
k=int(num)
for i in range(num):
	x1 = random.randrange(max_x)
	x2 = random.randrange(max_x)
	x3 = random.randrange(max_x)
	x4 = random.randrange(max_x)
	x5 = random.randrange(max_x)
	y1 = random.randrange(max_y)
	y2 = random.randrange(max_y)
	y3 = random.randrange(max_y)
	y4 = random.randrange(max_y)
	y5 = random.randrange(max_y)
	p1 = random.randrange(ports)
	p2 = random.randrange(ports)
	p3 = random.randrange(ports)
	p4 = random.randrange(ports)
	p5 = random.randrange(ports)
	w1 = random.randrange(wires)
	w2 = random.randrange(wires)
	w3 = random.randrange(wires)
	w4 = random.randrange(wires)
	w5 = random.randrange(wires)	
	idx1 = y1*max_x+x1	
	while idx1 == 0 :
		x1 = random.randrange(max_x)
		y1 = random.randrange(max_y)
		idx1 = y1*max_x+x1		
	
	idx2 = y2*max_x+x2
	while idx2 == 0 :
		x2 = random.randrange(max_x)
		y2 = random.randrange(max_y)
		idx2 = y2*max_x+x2
	
	idx3 = y3*max_x+x3
	while idx3 == 0 :
		x3 = random.randrange(max_x)
		y3 = random.randrange(max_y)
		idx3 = y3*max_x+x3
		
	idx4 = y4*max_x+x4
	while idx4 == 0 :
		x4 = random.randrange(max_x)
		y4 = random.randrange(max_y)
		idx4 = y4*max_x+x4
		
	idx5 = y5*max_x+x5
	while idx5 == 0 :
		x5 = random.randrange(max_x)
		y5 = random.randrange(max_y)
		idx5 = y5*max_x+x5
		
	f = open('scenario-'+str(i), 'w')		
	f.write('/test_bench/HeMPS/proc('+str(idx1)+')/slav/slave/PE/Router/data_in('+str(p1)+')('+str(w1)+') 1 '+str_fault_time+'\n')
	f.write('/test_bench/HeMPS/proc('+str(idx2)+')/slav/slave/PE/Router/data_in('+str(p2)+')('+str(w2)+') 1 '+str_fault_time+'\n')
	f.write('/test_bench/HeMPS/proc('+str(idx3)+')/slav/slave/PE/Router/data_in('+str(p3)+')('+str(w3)+') 1 '+str_fault_time+'\n')
	f.write('/test_bench/HeMPS/proc('+str(idx4)+')/slav/slave/PE/Router/data_in('+str(p4)+')('+str(w4)+') 1 '+str_fault_time+'\n')
	f.write('/test_bench/HeMPS/proc('+str(idx5)+')/slav/slave/PE/Router/data_in('+str(p5)+')('+str(w5)+') 1 '+str_fault_time+'\n')
	f.close()
	i = i+1

print num, 'scenarios were generated'
