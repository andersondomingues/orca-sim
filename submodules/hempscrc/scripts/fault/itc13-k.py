# executes fault scenarios with different values of k  -  used in ITC13 paper
# example: python itc13-k.py

import sys
import os, errno
import shutil
import glob
import time

if (os.environ['HEMPS_PATH'] == ''):
	print 'HEMPS_PATH  not defined'
	sys.exit(1)
hemps_path = os.environ['HEMPS_PATH']


print 'This script submit hundreds of jobs in the grid, overloading most machines and also the network.'
print 'Make sure you have tested all environment before executing this script !!!'
print 'If you need, the command \"qdel <job-id>\" can be used to kill the jobs'
raw_input("Press Enter to continue...")

# change here to include/exclude apps
#app_list = ['disturbing','synthetic', 'mpeg']
app_list = ['synthetic', 'mpeg']
k_values = [12,8,4,2]

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
	for k in k_values:
		print '#######################################################'
		print 'Application:', app, 'K:', str(k)
		print '#######################################################'
		# submit the jobs
		os.chdir(os.path.join(app,'1-fault','k-'+str(k)))
		#os.system(hemps_path + '/scripts/fault/run-all ' + app)
		hemps_file = os.path.join(os.getcwd(),app+'.hmp')
		files = [f for f in glob.glob('scenario*') if os.path.isfile(f)]
		for f in files:
			dest_dir = os.path.join(os.getcwd(),f)
			print 'qsub '+hemps_path+'/scripts/fault/grid-k.sh '+app+' '+hemps_file+' '+dest_dir
			os.system('qsub '+hemps_path+'/scripts/fault/grid-k.sh '+app+' '+hemps_file+' '+dest_dir)
			time.sleep(1)
		
		os.chdir(os.path.join('..','..','..'))

