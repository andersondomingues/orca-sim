#!/bin/python

import sys,os
import re                         # reg exp
from statlib import stats
import matplotlib.pyplot as plt
from termcolor import colored     # terminal color
import glob
import operator

# if you dont have any of these packages, just type
# sudo easy_install <pack-name>

if (os.environ['HEMPS_PATH'] == ''):
	print 'HEMPS_PATH  not defined'
	sys.exit(1)
hemps_path = os.environ['HEMPS_PATH']

app_list = ['disturbing','synthetic', 'mpeg']
#app_list = ['synthetic']
k_values = [12,8,4,2]


##################################################################
# extract simulation time from log files of a single scenario
##################################################################
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
		# show terminated tasks, 
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

##################################################################
# generate log file and chart per K value
##################################################################
def k_results(result_list,results_file):
	# generate charts
	#plt.title("Application Execution Time")
	plt.xlabel("Scenario id")
	plt.ylabel("Application Execution Time (ms)")
	# sort results by scenario id
	result_list.sort(key=lambda i: i[0])
	result_list2 = []
	#print result_list
	for i in range(len(result_list)):
		# idx must start in 0, not in 1
		# plot time in ms, so we have to divide bt 1 million
		result_list2.append((i,float(float(result_list[i][1])/100000.0)))
	#print result_list2
	# split tuple 
	x,y = [[z[i] for z in result_list2] for i in (0,1)]
	# i am assuming that the scenario-0 is always faulty-free
	plt.scatter(x,y,s=1,marker='o', c='k')
	#plt.bar(x,y,width=0.05,color='k')
	# caculate the average sim time
	line = [stats.mean([tup[1] for tup in result_list2])]*len(result_list2)
	plt.plot(line)
	#plt.legend(loc='upper right',label='avg')
	#plt.show()
	plt.savefig('app-time.pdf', format='pdf')   
	plt.clf()  # clear figure
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
	print 'max simulation time overhead(%):', '%0.2f' % ((float(max(y)*100.0)/float(faulty_free_time))-100.0)

	results_file.write('total number of scenarios: %d \n' % (len(files)))
	results_file.write('number of finished apps: %d \n' % (len(y)))
	results_file.write('number of scenarios not affected by faults: %d \n' % (faulty_free))
	results_file.write('scenarios not affected by faults(faulty-free/total, %): '+'%0.2f \n' % (float(faulty_free*100.0)/float(len(files))))
	results_file.write('scenarios not affected by faults(faulty-free/passed, %): '+'%0.2f \n' % (float(faulty_free*100.0)/float(len(y))))
	results_file.write('faulty free simulation time: '+'%0.4f \n' % (faulty_free_time))
	results_file.write('max simulation time: ' + str(max(y)) + ' \n')
	results_file.write('max simulation time overhead(%): '+'%0.2f \n' % ((float(max(y)*100.0)/float(faulty_free_time))-100.0))	

def app_results(result_list,results_file):
	# generate charts
	plt.xlabel("K")
	plt.ylabel("Application Execution Time (ms)")
	# sort results by scenario id
	result_list.sort(key=lambda i: i[0])
	#result_list2 = []
	#print result_list
	#for i in range(len(result_list)):
		# idx must start in 0, not in 1
		# plot time in ms, so we have to divide bt 1 million
		#result_list2.append((i,float(float(result_list[i][1])/100000.0)))
	#print result_list2
	for scenario  in result_list:
		# split tuple 
		# x is the scenario id
		# y is the list of times for diff Ks
		#print scenario
		x = scenario[0]
		# convert time to ms
		y = []
		for i in scenario[1]:
			y.append(float(float(i)/100000.0))
		# i am assuming that the scenario-0 is always faulty-free
		print 'scenario:',x,',',str(y)
		results_file.write('scenario: ' + str(x) + ', ' + str(y) + '\n')
		plt.plot(k_values,y,marker='.',label=str(x))
		faulty_free_time = y[0]
		print 'avg-time:', '%0.4f' % (stats.mean(y))
		print 'fault-free:', '%0.4f' % (faulty_free_time)
		results_file.write('avg-time: ' + '%0.4f' %(stats.mean(y)) + '\n')
		results_file.write('fault-free: ' + '%0.4f' % (faulty_free_time) + '\n')
	#plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
	#plt.legend(loc='upper left')
	#plt.show()
	plt.savefig('app-time.pdf', format='pdf')   
	plt.clf()  # clear figure



#######################################################################
# generate all scenarios with single fault for all apps
#######################################################################
for app in app_list:
	app_results_list = []
	scenario_id_list = []
	os.chdir(os.path.join(app))
	for k in k_values:
		print '#######################################################'
		print 'Application:', app, 'K:', str(k)
		print '#######################################################'
		os.chdir(os.path.join('1-fault','k-'+str(k)))
		# create the log files
		results_file = open(os.path.join('report.txt'), 'w')
		result_list = []
		simul_path = os.getcwd()
		# get scenarios available
		files = [f for f in glob.glob('scenario*') if os.path.isfile(f)]
		for f in files:
			sim_time = results(os.path.join(f+'-log'),results_file)
			if (sim_time != -1):
				# save only the scenario number and its sim time
				scenario_id = f.split("-")[1]
				scenario_id_list.append(scenario_id)
				print sim_time
				result_list.append((int(scenario_id),sim_time))
			
		if (len(result_list) == 0):
			os.chdir(os.path.join('..','..'))
			continue # none scenario finished!
		k_results(result_list,results_file)
		app_results_list.append(result_list)
		results_file.close()		
		os.chdir(os.path.join('..','..'))
	
	# generate results for each app sorted by scenario
	
	# remove duplicated entries
	scenario_id_list = list(set(scenario_id_list))
	k_result_list = []
	results_file = open(os.path.join('report.txt'), 'w')
	#redirect stdout to file
	#sys.stdout = open('report.txt', 'w')
	print '#######################################################'
	print 'Application Chart:', app
	print '#######################################################'
	for scenario in scenario_id_list:
		result_list = []
		for k in k_values:
			os.chdir(os.path.join('1-fault','k-'+str(k)))
			sim_time = results(os.path.join('scenario-'+scenario+'-log'),results_file)
			if (sim_time != -1):
				#print 'k: ' + str(k) + ' , simtime: ' + str(sim_time)
				#results_file.write('k: ' + str(k) + ' , simtime: ' + str(sim_time))
				result_list.append(sim_time)
			os.chdir(os.path.join('..','..'))
		
		#print 'scenario:', scenario, result_list
		#results_file.write('scenario: ' + scenario + ', ' + str(result_list)+'\n')
		k_result_list.append((int(scenario),result_list))
			
	print 'creating log and charts ...'
	#print k_result_list
	# example of k_result_list. scenario 39 has times 136977, 124166, 111174, 93375
	# [(39, [136977, 124166, 111174, 93375]), (27, [151741, 134185, 113281, 93373]), (17, [136985, 124171, 111190, 93381]), (47, [136985, 124171, 111190, 93381]), (37, [140352, 140379, 140379, 93381]), (29, [140271, 140325, 140325, 93375]), (0, [90615, 90611, 90611, 93373]), (5, [151741, 134185, 113281, 93373]), (13, [151741, 134212, 113281, 93373])]
	app_results(k_result_list,results_file)
	results_file.close()
	os.chdir(os.path.join('..'))
	sys.exit(1)	
