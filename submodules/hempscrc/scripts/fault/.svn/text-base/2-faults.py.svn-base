# generates all double fault scenarios for a X by Y network
# example: python 2-faults.py 3 3 0 0.5'
# it generates 50% of the possible faults for a 3x3 network where the master is located at node 0

import sys
import random

### change the seed to control the generated scenarios
#random.seed(1234)

if (len(sys.argv) != 7 and len(sys.argv) != 8):
	print 'ERROR: missing parameters'
	print 'USAGE: python 2-faults.py <max-x> <max-y> <master-idx> <fault-selection-percentage> <fault-time> <seed>'
	sys.exit(1)
elif(len(sys.argv) == 8 and sys.argv[7] == 'random-time'):
	random_time = 1
else:
	random_time = 0

if (not sys.argv[1].isdigit()):
	print 'ERROR: 1st parameter must be an interger'
	print 'USAGE: python 2-faults.py <max-x> <max-y> <master-idx> <fault-selection-percentage> <fault-time> <seed>'
	sys.exit(1)
if (not sys.argv[2].isdigit()):
	print 'ERROR: 2nd parameter must be an integer'
	print 'USAGE: python 2-faults.py <max-x> <max-y> <master-idx> <fault-selection-percentage> <fault-time> <seed>'
	sys.exit(1)
if (not sys.argv[3].isdigit()):
	print 'ERROR: 3rd parameter must be an integer'
	print 'USAGE: python 2-faults.py <max-x> <max-y> <master-idx> <fault-selection-percentage> <fault-time> <seed>'
	sys.exit(1)

try:
    if (float(sys.argv[4]) > 1.0):
		print 'ERROR: 4th parameter must be <= 1.0. 100% will generate 1024 scenarios'
		print 'USAGE: python 2-faults.py <max-x> <max-y> <master-idx> <fault-selection-percentage> <fault-time> <seed>'
		sys.exit(1)
except ValueError:
	print 'ERROR: 4th parameter must be an float'
	print 'USAGE: python 2-faults.py <max-x> <max-y> <master-idx> <fault-selection-percentage> <fault-time> <seed>'
	sys.exit(1)
    
max_x = int(sys.argv[1])
max_y = int(sys.argv[2])
master = int(sys.argv[3])
fault_prob = float(sys.argv[4])
str_fault_time = sys.argv[5]
### change the seed to control the generated scenarios. use random seed if -1
if (int(sys.argv[6]) != -1):
	random.seed(int(sys.argv[6]))

ports = 8
wires = 20
i = 1

# scenario 0 is always a faulty-free scenario. this is used as reference app time
f = open('scenario-0', 'w')
f.close()

for x1 in range(max_x):
	for y1 in range(max_y):
		for p1 in range(ports):
			for x2 in range(max_x):
				for y2 in range(max_y):
					for p2 in range(ports):
						for w1 in range(wires):
							for w2 in range(wires):
								idx1 = y1*max_x+x1
								idx2 = y2*max_x+x2
								if ((idx1 == master) or (idx2 == master)): # skip the master router
									continue
								if (random.random() < fault_prob):
									f = open('scenario-'+str(i), 'w')
									if(random_time == 1):
										list_fault_time = str_fault_time.split()
										fault_time_module = list_fault_time[0];
										fault_time_unit = list_fault_time[1];
										# print "mod:" + fault_time_module + " unit:" + fault_time_unit + "\n"
										fault_time_unit_plus_random = random.random()%0.2 + float(fault_time_module)
										# print "passei1!\n"
										# f.write('/test_bench/HeMPS/proc('+str(idx)+')/slav/slave/faulty_port('+str(p)+') 1 '+ str(fault_time_unit_plus_random) + " " + fault_time_unit + '\n')
										
										f.write('/test_bench/HeMPS/proc('+str(idx1)+')/slav/slave/faulty_port('+str(p1)+') 1 '+ str(fault_time_unit_plus_random) + " " + fault_time_unit +'\n')
										f.write('/test_bench/HeMPS/proc('+str(idx2)+')/slav/slave/faulty_port('+str(p2)+') 1 '+ str(fault_time_unit_plus_random) + " " + fault_time_unit +'\n')

									else:
										f.write('/test_bench/HeMPS/proc('+str(idx1)+')/slav/slave/PE/Router/data_in('+str(p1)+')('+str(w1)+') 1 '+str_fault_time+'\n')
										f.write('/test_bench/HeMPS/proc('+str(idx2)+')/slav/slave/PE/Router/data_in('+str(p2)+')('+str(w2)+') 1 '+str_fault_time+'\n')
									f.close()
									
									i = i + 1

print i, 'scenarios were generated'
