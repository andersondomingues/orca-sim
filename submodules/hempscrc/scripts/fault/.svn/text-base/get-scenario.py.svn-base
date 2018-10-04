import sys,os
import glob
import shutil
import errno

app_list = ['disturbing','synthetic', 'mpeg']
k_values = [12,8,4,2]

if (len(sys.argv) != 2):
	print 'ERROR: missing parameters'
	print 'USAGE: XXXX'
	sys.exit(1)

base_dir = sys.argv[1]


def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise

#######################################################################
# generate destination dir
#######################################################################
mkdir_p(os.path.join(base_dir))
for app in app_list:
	# generate scenarios for disturbing
	mkdir_p(os.path.join(base_dir,app))
	mkdir_p(os.path.join(base_dir,app,'1-fault'))
	for k in k_values:
		mkdir_p(os.path.join(base_dir,app,'1-fault','k-'+str(k)))


#######################################################################
# selects the scenarios that finished OK and were affected by the fault
#######################################################################
for app in app_list:
	# generate scenarios for disturbing
	app_path = os.path.join(app,'1-fault')
	#grep OK */1-fault/report.txt
	f = open(os.path.join(app_path+'/report.txt'),'r')
	report_file =  f.readlines()
	i = 0
	print len(report_file)
	for i in range(len(report_file)):
		# check if the line has the word ' OK'
		if (' OK' in report_file[i]):
			# get scenario number
			scenario_id = report_file[i].split('-')[1]
			# if the scenario finished, get the simulation time.
			sim_time = report_file[i+4].split(' ')[1]
			print scenario_id, sim_time
			if (not sim_time in ['90615','307666','523024']):
				# this scenario must be selected since it affects the simulation time
				# copy good scenario
				src_path = os.path.join(app_path,'scenario-'+scenario_id)
				for k in k_values:
					dest_path = os.path.join(base_dir, app_path,'k-'+str(k),'scenario-'+scenario_id)
					shutil.copyfile(src_path, dest_path)
					print src_path, dest_path
			
