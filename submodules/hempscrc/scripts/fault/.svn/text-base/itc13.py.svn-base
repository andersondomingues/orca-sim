# executes all fault scenarios used in ITC13 paper
# example: python itc13.py

import sys
import os, errno
import random


print 'This script submit hundreds of jobs in the grid, overloading most machines and also the network.'
print 'Make sure you have tested all environment before executing this script !!!'
print 'If you need, the command \"qdel <job-id>\" can be used to kill the jobs'
raw_input("Press Enter to continue...")

if (os.environ['HEMPS_PATH'] == ''):
	print 'HEMPS_PATH  not defined'
	sys.exit(1)
hemps_path = os.environ['HEMPS_PATH']

# change here if you want to denerate new random scenarios
rnd_seed = 1234567
# change here to include/exclude apps
app_list = ['disturbing','synthetic', 'mpeg', 'dtw']


def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise

#######################################################################
# generate all scenarios with single fault for all apps
#######################################################################
for app in app_list:
	# generate scenarios for disturbing
	mkdir_p(app)
	mkdir_p(os.path.join(app,'1-fault'))
	os.chdir(os.path.join(app,'1-fault'))
	# generates scenarios
	if (app == 'disturbing'):
		os.system('python ' + hemps_path + '/scripts/fault/1-fault.py 3 3 2 1.0 \"0.4 ms\" ' + str(rnd_seed))
	elif (app == 'synthetic'):
		os.system('python ' + hemps_path + '/scripts/fault/1-fault.py 3 3 0 1.0 \"1 ms\" ' + str(rnd_seed))
	elif (app == 'mpeg'):
		os.system('python ' + hemps_path + '/scripts/fault/1-fault.py 3 3 6 1.0 \"2 ms\" ' + str(rnd_seed))
	elif (app == 'dtw'):
		os.system('python ' + hemps_path + '/scripts/fault/1-fault.py 4 4 12 1.0 \"2 ms\" ' + str(rnd_seed))
	else:
		print 'application ' + app + ' not found!'
		sys.exit(1)
	# submit the jobs
	os.system(hemps_path + '/scripts/fault/run-all ' + app)
	#print hemps_path + 'scripts/fault/run-all ' + app
	
	os.chdir(os.path.join('..','..'))


#######################################################################
# generate random scenarios with duble fault for all apps
#######################################################################
for app in app_list:
	# generate scenarios for disturbing
	mkdir_p(app)
	mkdir_p(os.path.join(app,'2-faults'))
	os.chdir(os.path.join(app,'2-faults'))
	# generates scenarios
	if (app == 'disturbing'):
		os.system('python ' + hemps_path + '/scripts/fault/2-faults.py 3 3 2 0.05 \"0.4 ms\" ' + str(rnd_seed))
	elif (app == 'synthetic'):
		os.system('python ' + hemps_path + '/scripts/fault/2-faults.py 3 3 0 0.05 \"1 ms\" ' + str(rnd_seed))
	elif (app == 'mpeg'):
		os.system('python ' + hemps_path + '/scripts/fault/2-faults.py 3 3 6 0.05 \"2 ms\" ' + str(rnd_seed))
	elif (app == 'dtw'):
		os.system('python ' + hemps_path + '/scripts/fault/2-faults.py 4 4 12 0.02 \"2 ms\" ' + str(rnd_seed))
	else:
		print 'application ' + app + ' not found!'
		sys.exit(1)
	# submit the jobs
	os.system(hemps_path + '/scripts/fault/run-all ' + app)
	#print hemps_path + 'scripts/fault/run-all ' + app
	os.chdir(os.path.join('..','..'))
