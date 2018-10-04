#!/bin/bash
#$ -cwd
#$ -j y
#$ -S /bin/bash

### Job name:
##$ -N fault-sim

### this job can only be executed on Redhat or Suse. it cannot be executed on Ubuntu
##$ -q suse_excl.q
##$ -q all_half.q
#$ -q all.q


### jobs require slots with at least 5G mem available
### this parameters didnt work ... host were swapping
##$ -l mem_free=5G
##$ -l h_vmem=5G

# request exclusive access for the job
##$ -l exclusive=true

### the max execution time for jobs is 1h (syntax hr:min:sec)
#$ -l h_rt=1:00:00  

##I cant run the job in gaphl13
#$ -l h='!(GAPHL32|GAPHL13|GAPHL05|GAPHL21)'

### HOW TO RUN IT ON THE GRID: qsub grid.sh <app-name> <fault-scenario-file-with-full-path>
### OUTPUT: the output log file will be saved at home dir under the name fault-sim<$SGE_TASK_ID>.
### HOW TO RUN IT: ./compile.sh > log.txt 2>&1



#### LHECK
#### VOLTAR ISSO o TMP para o / por enquanto usando o ~/tmp
TMP_PATH=/tmp
mkdir -p ${TMP_PATH}

set -e

echo "############################################################"
echo "######################## BASIC SETUP #######################"
echo "###################### LOADING MODULES #####################"
echo "############################################################"
source /soft64/Modules/3.2.8/init/bash
# It assumes that you have the function hempslocal defined in your ~/hemps. If not, this is an example
#function hempslocal(){
#   export HEMPS_PATH=/home/aamory/repos/wachter_hemps
#   export PATH=$PATH:$HEMPS_PATH/bin
#   module load ise mips systemc/2.2.0
#   module load modelsim
#   echo $HEMPS_PATH
#}
# it set HEMPS_PATH and loads the tools
export HEMPS_PATH=~/Documents/mazembe/ft_v1/
export PATH=$PATH:$HEMPS_PATH/bin
module load ise mips systemc/2.2.0
module load modelsim
echo $HEMPS_PATH

FAULT_APP=$1

# the scenario file
FULLPATH=$2
# remove all the prefix until "/" character
FILENAME=${FULLPATH##*/}
# remove all the prefix unitl "." character
FILEEXTENSION=${FILENAME##*.}
# remove a suffix, in our case, the filename, this will return the name of the directory that contains this file
BASEDIRECTORY=$(pwd)
LOGS_DIR=${BASEDIRECTORY}/scenario-$FAULT_APP-${FILENAME}-log

echo $FULLPATH
echo $FILENAME
echo $FILEEXTENSION
echo $BASEDIRECTORY

echo Job ${FAULT_APP}-$FILENAME with ID $JOB_ID sent for execution in `/bin/hostname` at `/bin/date` >> ${BASEDIRECTORY}/sent_jobs.txt
echo "############################################################"
echo "###################### BASIC DEBUG INFO ####################"
echo "############################################################"
echo Job $FILENAME with ID $JOB_ID is running on `/bin/hostname`
echo Now it is `/bin/date`
#echo The log file will be saved at $PWD
echo SGE_STDOUT_PATH=$SGE_STDOUT_PATH
echo The user account is $USER
echo App name: ${FAULT_APP}
echo Log dir: ${LOGS_DIR}
echo Scenario file: ${FULLPATH}
cat ${FULLPATH}
echo ""

echo "############################################################"
echo "###################### CHECKING PARAMETERS #################"
echo "############################################################"
if [ "$FAULT_APP" != "" ];then
    echo "Checking fault app ${FAULT_APP} ... ok !"
else
    echo "ERROR: Fault app not defined"
    echo "USAGE: qsub run_grid_app_fault.sh <app-name> <fault-scenario-file>"
    exit 1
fi
if [ -f ${FULLPATH} ]; then
    echo "Checking fault scenario file ${FULLPATH} ... ok !"
else
    echo "ERROR: fault scenario file not found !"
    echo "USAGE: qsub run_grid_app_fault.sh <app-name> <fault-scenario-file>"
    exit 1
fi
echo ""

echo "############################################################"
echo "######################### CURRENT DIR ######################"
echo $PWD
echo $HEMPS_PATH
echo ${BASEDIRECTORY}/${FILENAME}
echo "############################################################"
echo ""

echo "############################################################"
echo "################### CHECKING DISK USAGE ####################"
echo "############################################################"
# check local disk usage
echo "show disk usage:"
echo "use(%)	 avail(GB)	 mount"
df -kh |  grep "/dev/" | awk '{ print $5 "\t " $4 "\t " $6 }'
#cd ${TMP_PATH}/
# remove the dir older than 3 hours
#echo "cleaning old scenarios ..."
#find ./wachter* -type d -mmin +180 -exec rm -rf {} \;
# check local disk usage
#echo "show disk usage:"
#echo "use(%)	 avail(GB)	 mount"
#df -kh |  grep "/dev/" | awk '{ print $5 "\t " $4 "\t " $6 }'

# download scenario
echo "############################################################"
echo "################### DOWNLOADING SCENARIO ###################"
echo "############################################################"
mkdir -p ${TMP_PATH}/wachter-$JOB_ID
cd ${TMP_PATH}/wachter-$JOB_ID
echo ${BASEDIRECTORY}
cp ${BASEDIRECTORY}/${FAULT_APP}.hmp .
echo "${TMP_PATH}/wachter-$JOB_ID"
echo ""

# compile scenario
echo "############################################################"
echo "################# GENERATING HEMPS DESIGN ##################"
echo "############################################################"
OS=$(lsb_release -si)
if [ $OS == "Ubuntu" ]; then
  echo "executing on ubuntu"
  hemps ${FAULT_APP}.hmp
else
  echo "executing on suse"
  hemps_suse ${FAULT_APP}.hmp
fi

RETVAL=$?
[ $RETVAL != 0 ] && echo "Failure. unable to run HeMPS Generator"
cd ${FAULT_APP}
cd build/
make all
cd ..
make all
cp ${BASEDIRECTORY}/${FULLPATH} signals
echo "############################################################"
echo "##################### HEMPS COMPILED #######################"
echo "############################################################"
echo ""

# run scenario
echo "############################################################"
echo "##################### SIMULATING HEMPS #####################"
echo "############################################################"
vsim -c -do sim.do > results
RETVAL=$?
[ $RETVAL != 0 ] && echo "Failure. unable to run modelsim or failure during simulation"
echo "############################################################"
echo "##################### END SIMULATION #######################"
echo "############################################################"
# when a simulation ends with  'End of simulation' in the transcript, then it is guaranteed that the app run succesfully.
# otherwise, one has to analysed the log files to check if the app finished or it crashed
RETVAL=`grep -i "End of simulation" ./transcript | wc -l`


echo Job ${FAULT_APP}-$FILENAME with ID $JOB_ID terminated execution in `/bin/hostname` at `/bin/date` >> ${BASEDIRECTORY}/terminated_jobs.txt

# echo some debuf info into the logfile
echo Job $JOB_ID executed on `/bin/hostname`

echo Now it is `/bin/date`
#echo The log file will be saved at $PWD
echo SGE_STDOUT_PATH=$SGE_STDOUT_PATH
echo The user account is $USER
echo App name: ${FAULT_APP}
echo Log dir: ${LOGS_DIR}
echo Scenario file: ${FULLPATH}
echo ""

#echo $RETVAL
if [ $RETVAL == 1 ]; then
   echo "Success. It reached end of simulation"
fi
if [ $RETVAL != 1 ]; then
   grep TASK_ log/*
   echo "Failure. Unable to reach end of simulation. Check the log files to confirm the failure!"
fi

# saving simulation results
mkdir -p $LOGS_DIR
mkdir -p ${LOGS_DIR}/log
\cp -f ${TMP_PATH}/wachter-$JOB_ID/${FAULT_APP}/signals ${TMP_PATH}/wachter-$JOB_ID/${FAULT_APP}/output_master.txt ${TMP_PATH}/wachter-$JOB_ID/${FAULT_APP}/results ${LOGS_DIR}
\cp -f ${TMP_PATH}/wachter-$JOB_ID/${FAULT_APP}/log/*.txt ${LOGS_DIR}/log
# \mv -f $SGE_STDOUT_PATH ${LOGS_DIR}

# remove project file. each one uses almost 1G of disk space !!!
#cd ${TMP_PATH}/wachter-$JOB_ID/${FAULT_APP}/build
#make clean
#cd ${TMP_PATH}/wachter-$JOB_ID/${FAULT_APP}
#make clean
rm -rf ${TMP_PATH}/wachter-$JOB_ID/

# generating exit code
if [ $RETVAL == 1 ]; then
   exit 0
fi
if [ $RETVAL != 1 ]; then
   exit 1
fi
