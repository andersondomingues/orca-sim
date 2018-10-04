#!/bin/python

import sys,os
import re                         # reg exp
from statlib import stats
import matplotlib.pyplot as plt
from termcolor import colored     # terminal color
import glob
import operator


#GRID basic example
#python ../../scripts/fault/check-log.py mega ./ results 

# if you dont have any of these packages, just type
# sudo easy_install <pack-name>

if (os.environ['HEMPS_PATH'] == ''):
	print 'HEMPS_PATH  not defined'
	sys.exit(1)
hemps_path = os.environ['HEMPS_PATH']


if (len(sys.argv) < 2):
	print 'ERROR: 1st parameter must be check type. valid strings are: itc13, mega, results, task-time, packet-time, seek-cnt, faults, regression '
	sys.exit(1)

# type of test
type_test = sys.argv[1]

if ((type_test != 'task-time') and (type_test != 'packet-time') and (type_test != 'seek-cnt') and (type_test != 'faults') 
	and (type_test != 'mega') and (type_test != 'regression') and (type_test != 'itc13') and (type_test != 'results')):
	print 'ERROR: 1st parameter must be check type. valid strings are: itc13, mega, results, task-time, packet-time, seek-cnt, faults, regression'
	sys.exit(1)


def results(simul_path,report_file):
	# check if it passed or not
	passed = False
	task_alloc_list = os.popen('grep TASK_ALLOCATION '+simul_path+'/log/*').read().splitlines()
	task_term_list = os.popen('grep TASK_TERMINATED '+simul_path+'/log/*').read().splitlines()
	seeks = os.popen('grep \"SEEK to\" '+simul_path+'/log/* | wc -l').read()
	
	sucessfull_task_counter = 0
	#print task_alloc_list
	#print task_term_list
	task_time_list = []
	for task_alloc in task_alloc_list:
		#print 'alloc', task_alloc
		task_id = task_alloc.split('\t')[1]
		task_time = task_alloc.split('\t')[2]
		# search for the corresponding task_terminated
		for task_term in task_term_list:
			#print 'term', task_term
			if (task_id == task_term.split('\t')[1]):
				#print 'task:', task_id, 'exec_time:', int(task_term.split('\t')[2])-int(task_time)
				task_time_list.append((int(task_time), int(task_term.split('\t')[2])))
				sucessfull_task_counter = sucessfull_task_counter + 1
	
	# generating report file
	results = []
	if ((sucessfull_task_counter == len(task_alloc_list)) and (sucessfull_task_counter == len(task_term_list))):
		print simul_path + ' is ' + colored(' OK', 'green')
		report_file.write(simul_path + ' is OK\n')
		passed = True
	else:
		print simul_path + ' is ' + colored('NOK', 'red')
		report_file.write(simul_path + ' is NOK\n')
		passed = False

	report_file.write('number_of_seeks: ' + str(int(seeks)) + '\n')
	scenario_file = open(os.path.join(simul_path,'signals'), 'r')
	scenario = scenario_file.readlines()
	report_file.write('number_of_faults: ' + str(len(scenario)) + '\n')
	for line in scenario:
		# the signal name is always the 1st word of the line
		sig_name = line.split(' ')[0]
		# extract the router id and the port id from the signal name
		router_id = int(sig_name.split('(')[1].split(')')[0])
		port_id = int(sig_name.split('(')[2].split(')')[0])
		if port_id == 0:
			port = 'E0'
		elif port_id == 1:
			port = 'E1'
		elif port_id == 2:
			port = 'W0'
		elif port_id == 3:
			port = 'W1'
		elif port_id == 4:
			port = 'N0'
		elif port_id == 5:
			port = 'N1'
		elif port_id == 6:
			port = 'S0'
		elif port_id == 7:
			port = 'S1'
		else:
			print "ERROR: invalid port", port_id
			sys.exit(1)
		report_file.write('fault at router ' + str(router_id) + ' port ' + port	 + '\n')
	if (passed == True):
		# show app termination time
		# get the maximal task termination time
		report_file.write('app_end_time: ' + str(max([tup[1] for tup in task_time_list])) + ' \n')
		return  max([tup[1] for tup in task_time_list])
	else:
		# show log file such as terminated tasks, 
		report_file.write('terminated tasks:\n')
		for task_alloc in task_alloc_list:
			#print 'alloc', task_alloc
			task_id = task_alloc.split('\t')[1]
			task_time = task_alloc.split('\t')[2]
			# search for the corresponding task_terminated
			for task_term in task_term_list:
				#print 'term', task_term
				if (task_id == task_term.split('\t')[1]):
					report_file.write('task: ' + task_id  + ' exec_time: '  + str(int(task_term.split('\t')[2])-int(task_time)) + '\n')
		return -1


if (type_test == 'results'): # tested
	### generates results for the testcases that passed and show log for the ones that do not passed
	if (len(sys.argv) != 3):
		print 'USAGE: check-log.py '+type_test+' <simul-dir>'
		sys.exit(1)
	# dir where the simulation was executed
	simul_path = sys.argv[2]
	if (not os.path.isdir(simul_path)):
		print 'ERROR: 2nd parameter must be a dir'
		print 'USAGE: check-log.py '+type_test+' <simul-dir>'
		sys.exit(1)
	# report file
	report_file = open(os.path.join(simul_path,'results.txt'), 'w')
	print results(simul_path,report_file)


if (type_test == 'itc13'): 
	if (len(sys.argv) != 3):
		print 'USAGE: check-log.py '+type_test+' <mode>'
		print 'Valid modes are: results, task-time, packet-time, seek-cnt, faults, regression'
		sys.exit(1)
	mode = sys.argv[2]

	# change here to include/exclude apps
	app_list = ['disturbing','synthetic', 'mpeg', 'dtw']
	
	for app in app_list:
		# generate scenarios for disturbing
		#print app
		if (os.path.isdir(app)):
			os.chdir(app)
			for nfault in ['1-fault','2-faults']:
				#print nfault
				if (os.path.isdir(os.path.join(nfault))):
					os.chdir(nfault)
					command = 'python '+hemps_path+'/scripts/fault/check-log.py mega . ' + mode
					print '#########################################'
					print 'Executing ', app, nfault
					print '#########################################'
					print command
					os.system(command)
					os.chdir('..')
			os.chdir('..')




if (type_test == 'mega'): 
	if (len(sys.argv) != 4):
		print 'USAGE: check-log.py '+type_test+' <simul-dir> <mode>'
		print 'Valid modes are: results, task-time, packet-time, seek-cnt, faults, regression'
		sys.exit(1)
	simul_path = sys.argv[2]
	if (not os.path.isdir(simul_path)):
		print 'ERROR: 2nd parameter must be a dir'
		print 'USAGE: check-log.py '+type_test+' <simul-dir> <mode>'
		sys.exit(1)
	mode = sys.argv[3]
	if (mode == 'results'):
		results_file = open(simul_path+'/report.txt','w')
		result_list = []
	# list files that start with scenario* only
	files = [f for f in glob.glob(simul_path+'/scenario*') if os.path.isfile(f)]
	for f in files:
		print 'Executing scenario ', f
		if (not os.path.isdir(os.path.join(simul_path,f+'-log'))):
			print 'scenario', f, 'not found!'
			continue
		if (mode == 'results'):
			sim_time = results(os.path.join(simul_path,f+'-log'),results_file)
			if (sim_time != -1):
				# save only the scenario number and its sim time
				scenario_id = f.split("-")[1]
				print sim_time
				result_list.append((int(scenario_id),sim_time))
		else:
			command = 'python '+hemps_path+'/scripts/fault/check-log.py ' + mode + ' ' + os.path.join(simul_path,f+'-log')
			os.system(command)
	if (mode == 'results'):
		# generate charts
		#plt.title("Application Execution Time")
		plt.xlabel("Scenario id")
		plt.ylabel("Application Execution Time (ms)")
		# sort results by scenario id
		result_list.sort(key=lambda i: i[0])
		result_list2 = []
		print result_list

		for i in range(len(result_list)):
			# idx must start in 0, not in 1
			# plot time in ms, so we have to divide bt 1 million
			result_list2.append((i,float(float(result_list[i][1])/100000.0)))
		print result_list2

		#print the points to a file
		f_aet = open('aet_time','w+')
		for i in result_list2:
			print>>f_aet, i[1]

		# split tuple 
		x,y = [[z[i] for z in result_list2] for i in (0,1)]
		# i am assuming that the scenario-0 is always faulty-free
		plt.scatter(x,y,s=1,marker='o', c='k')
		#plt.bar(x,y,width=0.05,color='k')
		# caculate the average sim time
		line = [stats.mean([tup[1] for tup in result_list2])]*len(result_list2)
		#plt.plot(line)
		#plt.legend(loc='upper right',label='avg')
		#plt.show()
		#set limits from 0 to number of scenario
		plt.xlim([0,len(files)])
		plt.savefig('app-time.png', format='png')   
		#calculate % of scenarios affected by faults
		#print x
		#print y
		faulty_free_time = y[0]
		faulty_free = 0
		faulty = 0
		for i in y:
			if (i == faulty_free_time):
				faulty_free = faulty_free + 1
		print 'total number of scenarios: %d' % (len(files))
		print 'number of finished apps: %d' % (len(y))
		print 'number of scenarios not affected by faults: %d' % (faulty_free)
		print 'scenarios not affected by faults(faulty-free/total, %):', '%0.2f' % (float(faulty_free*100.0)/float(len(files)))
		print 'scenarios not affected by faults(faulty-free/passed, %):', '%0.2f' % (float(faulty_free*100.0)/float(len(y)))
		print 'faulty free simulation time:', str(faulty_free_time)
		print 'max simulation time:', str(max(y))
		# print 'item with max sim time:', max((x,y),key=itemgetter(1))[0]
		print 'max simulation time overhead(%):', '%0.2f' % ((float(max(y)*100.0)/float(faulty_free_time))-100.0)

		results_file.write('total number of scenarios: %d \n' % (len(files)))
		results_file.write('number of finished apps: %d \n' % (len(y)))
		results_file.write('number of scenarios not affected by faults: %d \n' % (faulty_free))
		results_file.write('scenarios not affected by faults(faulty-free/total, %): '+'%0.2f \n' % (float(faulty_free*100.0)/float(len(files))))
		results_file.write('scenarios not affected by faults(faulty-free/passed, %): '+'%0.2f \n' % (float(faulty_free*100.0)/float(len(y))))
		results_file.write('faulty free simulation time: '+'%0.4f \n' % (faulty_free_time))
		results_file.write('max simulation time: ' + str(max(y)) + ' \n')
		results_file.write('max simulation time overhead(%): '+'%0.2f \n' % ((float(max(y)*100.0)/float(faulty_free_time))-100.0))

		results_file.close()
		


if (type_test == 'faults'): # tested
	# dir where the simulation was executed
	if (len(sys.argv) != 3):
		print 'USAGE: check-log.py '+type_test+' <simul-dir>'
		sys.exit(1)
	simul_path = sys.argv[2]
	if (not os.path.isdir(simul_path)):
		print 'ERROR: 2nd parameter must be a dir'
		print 'USAGE: check-log.py '+type_test+' <simul-dir>'
		sys.exit(1)

	# get the list of scenarios
	scenario_file = open(os.path.join(simul_path,'signals'), 'r')
	for line in scenario_file:
		# the signal name is always the 1st word of the line
		sig_name = line.split(' ')[0]
		# extract the router id and the port id from the signal name
		router_id = int(sig_name.split('(')[1].split(')')[0])
		port_id = int(sig_name.split('(')[2].split(')')[0])
		if port_id == 0:
			port = 'E0'
		elif port_id == 1:
			port = 'E1'
		elif port_id == 2:
			port = 'W0'
		elif port_id == 3:
			port = 'W1'
		elif port_id == 4:
			port = 'N0'
		elif port_id == 5:
			port = 'N1'
		elif port_id == 6:
			port = 'S0'
		elif port_id == 7:
			port = 'S1'
		else:
			print "ERROR: invalid port", port_id
			sys.exit(1)
		print 'fault at router ' + str(router_id) + ' port ' + port

if (type_test == 'seek-cnt'): # tested
	if (len(sys.argv) != 3):
		print 'USAGE: check-log.py '+type_test+' <simul-dir>'
		sys.exit(1)
	# dir where the simulation was executed
	simul_path = sys.argv[2]
	if (not os.path.isdir(simul_path)):
		print 'ERROR: 2nd parameter must be a dir'
		print 'USAGE: check-log.py '+type_test+' <simul-dir>'
		sys.exit(1)

	# counts how many seeks affected the simulation
	seeks = os.popen('grep \"SEEK to\" '+simul_path+'/log/* | wc -l').read()
	faults = os.popen('wc -l '+simul_path+'/signals').read()
	faults = faults.split(' ')[0]
	print 'Number of faults:', faults
	print 'Number of seeks:', seeks

if (type_test == 'task-time'): # tested
	### The log file is expected to have msgs like this. at the beginning it has task_allocated msg, the task id, and the event time. 
	### At the end of the file it will have task_terminated, task_id, and event time. Recall that a PE might have multiple tasks.
	### Also, recall that faults might cause a task not to finish
	### This check only shows tasks that succesfully finished
	# TASK_ALLOCATED	0	10212
	# ....
	#TASK_TERMINATED	0	42430
	if (len(sys.argv) != 3):
		print 'USAGE: check-log.py '+type_test+' <simul-dir>'
		sys.exit(1)
	# dir where the simulation was executed
	simul_path = sys.argv[2]
	if (not os.path.isdir(simul_path)):
		print 'ERROR: 2nd parameter must be a dir'
		print 'USAGE: check-log.py '+type_test+' <simul-dir>'
		sys.exit(1)
	
	task_alloc_list = os.popen('grep TASK_ALLOCATION '+simul_path+'/log/*').read().splitlines()
	task_term_list = os.popen('grep TASK_TERMINATED '+simul_path+'/log/*').read().splitlines()
	#print task_alloc_list
	#print task_term_list
	task_time_list = []
	for task_alloc in task_alloc_list:
		#print 'alloc', task_alloc
		task_id = task_alloc.split('\t')[1]
		task_time = task_alloc.split('\t')[2]
		# search for the corresponding task_terminated
		for task_term in task_term_list:
			#print 'term', task_term
			if (task_id == task_term.split('\t')[1]):
				print 'task:', task_id, 'exec_time:', int(task_term.split('\t')[2])-int(task_time)
				task_time_list.append((int(task_time), int(task_term.split('\t')[2])))
	#print task_time_list
	#print [tup[0] for tup in task_time_list]
	# get the minimal task allocation time
	print 'app_start_time:', min([tup[0] for tup in task_time_list])
	# get the maximal task termination time
	print 'app_end_time:', max([tup[1] for tup in task_time_list])



if (type_test == 'regression'): # tested
	### Checks whether the scenario finished or not based on the number of TASK_ALLOCATED and TASK_TERMINATED for each task. 
	### It generates ok, nok for each case
	if (len(sys.argv) != 3):
		print 'USAGE: check-log.py '+type_test+' <simul-dir>'
		sys.exit(1)
	# dir where the simulation was executed
	simul_path = sys.argv[2]
	if (not os.path.isdir(simul_path)):
		print 'ERROR: 2nd parameter must be a dir'
		print 'USAGE: check-log.py '+type_test+' <simul-dir>'
		sys.exit(1)
	
	task_alloc_list = os.popen('grep TASK_ALLOCATION '+simul_path+'/log/*').read().splitlines()
	task_term_list = os.popen('grep TASK_TERMINATED '+simul_path+'/log/*').read().splitlines()
	sucessfull_task_counter = 0
	#print task_alloc_list
	#print task_term_list
	task_time_list = []
	for task_alloc in task_alloc_list:
		#print 'alloc', task_alloc
		task_id = task_alloc.split('\t')[1]
		task_time = task_alloc.split('\t')[2]
		# search for the corresponding task_terminated
		for task_term in task_term_list:
			#print 'term', task_term
			if (task_id == task_term.split('\t')[1]):
				#print 'task:', task_id, 'exec_time:', int(task_term.split('\t')[2])-int(task_time)
				task_time_list.append((int(task_time), int(task_term.split('\t')[2])))
				sucessfull_task_counter = sucessfull_task_counter + 1
	if ((sucessfull_task_counter == len(task_alloc_list)) and (sucessfull_task_counter == len(task_term_list))):
		print simul_path + ' is ' + colored(' OK', 'green')
	else:
		print simul_path + ' is ' + colored('NOK', 'red')

if (type_test == 'packet-time'): # not tested
	##### the log delivers msg like this example shown below. the relevant information is the 'time' field (time to deliver the packet) 
	##### and the 'avg' field (current avg representing expected relivery time)
    #req-delivery:time:20746;total:26305;avg:13152;msg#:3,src:2
    #delivery-ack:time:6091;total:6091;avg:6091;msg#:3,src:2
	if (len(sys.argv) != 6):
		print 'USAGE: check-log.py '+type_test+' <simul-dir> <x> <y> <target-task-id>'
		sys.exit(1)
	# dir where the simulation was executed
	simul_path = sys.argv[2]
	if (not os.path.isdir(simul_path)):
		print 'ERROR: 2nd parameter must be a dir'
		print 'USAGE: check-log.py '+type_test+' <simul-dir> <x> <y> <target-task-id>'
		sys.exit(1)
	if (not sys.argv[3].isdigit()):
		print 'ERROR: 3rd parameter must be an interger'
		print 'USAGE: check-log.py '+type_test+' <simul-dir> <x> <y> <target-task-id>'
		sys.exit(1)
	else:
		x = int(sys.argv[3])
	if (not sys.argv[4].isdigit()):
		print 'ERROR: 4th parameter must be an integer'
		print 'USAGE: check-log.py '+type_test+' <simul-dir> <x> <y> <target-task-id>'
		sys.exit(1)
	else:
		y = int(sys.argv[4])
	if (not sys.argv[5].isdigit()):
		print 'ERROR: 5th parameter must be an integer'
		print 'USAGE: check-log.py '+type_test+' <simul-dir> <x> <y> <target-task-id>'
		sys.exit(1)
	else:
		target_task_id = int(sys.argv[5])

	msg_time_list = []
	avg_time_list = []
	list_ack_time = os.popen('grep req-delivery '+os.path.join(simul_path,'log','log_file'+str(x)+'x'+str(y)+'.txt')).read().splitlines()
	for line in list_ack_time:
		line_list = line.rstrip().split(';') # split the line in 4 parts

		aux = line_list[3].split(':')[2] # get the target task id
		print 'target:', aux
		if (int(aux) == target_task_id):
			aux = line_list[0].split(':')[2]
			print 'msg_time:', aux # get the time to deliver the packet
			msg_time_list.append(int(aux))

			aux = line_list[2].split(':')[1]
			print 'avg_time:', aux # get the current avg representing expected relivery time
			avg_time_list.append(int(aux))

	# generate charts
	plt.title("Packet Latency")
	plt.xlabel("Packet #")
	plt.ylabel("Latency (ms)")
	plt.plot(msg_time_list,label='msg-time')
	plt.plot(avg_time_list,label='avg-time')
	line = [stats.mean(msg_time_list)]*len(msg_time_list)
	plt.plot(line)
	plt.legend(loc='upper right')
	plt.show()
	plt.savefig('test.eps', format='eps')   


