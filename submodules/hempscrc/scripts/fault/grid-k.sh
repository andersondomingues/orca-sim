#!/bin/bash
#$ -j y
#$ -S /bin/bash

### Job name:
#$ -N fault-sim

### this job can only be executed on Redhat or Suse. it cannot be executed on Ubuntu
#$ -q suse_excl.q

### jobs require slots with at least 5G mem available
### this parameters didnt work ... host were swapping
##$ -l mem_free=5G
##$ -l h_vmem=5G

# request exclusive access for the job
##$ -l exclusive=true

### the max execution time for jobs is 1h (syntax hr:min:sec)
#$ -l h_rt=1:00:00  

##$ -l hostname=GAPHL31,GAPHL30,GAPHL29,GAPHL28,GAPHL27,GAPHL26,GAPHL16,GAPHL10

### HOW TO RUN IT ON THE GRID: qsub grid-k.sh <app-name> <hmp-file ><fault-scenario-file-with-full-path>
### OUTPUT: the output log file will be saved at home dir under the name fault-sim<$SGE_TASK_ID>.
### HOW TO RUN IT: ./compile.sh > log.txt 2>&1

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
source ~/hemps.sh
hempslocal
echo $HEMPS_PATH

export FAULT_APP=$1

HMP_FILE=$2

# the scenario file
FULLPATH=$3
# remove all the prefix until "/" character
FILENAME=${FULLPATH##*/}
# remove all the prefix unitl "." character
FILEEXTENSION=${FILENAME##*.}
# remove a suffix, in our cas, the filename, this will return the name of the directory that contains this file
BASEDIRECTORY=${FULLPATH%$FILENAME}
LOGS_DIR=${BASEDIRECTORY}/${FILENAME}-log

echo $FULLPATH
echo $FILENAME
echo $FILEEXTENSION
echo $BASEDIRECTORY

echo "############################################################"
echo "###################### CHECKING PARAMETERS #################"
echo "############################################################"
if [ "$FAULT_APP" != "" ];then
    echo "Checking fault app ${FAULT_APP} ... ok !"
else
    echo "ERROR: Fault app not defined"
    echo "USAGE: qsub grid-k.sh <app-name>  <hmp-file <fault-scenario-file>"
    exit 1
fi
if [ -f ${FULLPATH} ]; then
    echo "Checking fault scenario file ${FULLPATH} ... ok !"
else
    echo "ERROR: fault scenario file not found !"
    echo "USAGE: qsub grid-k.sh <app-name>  <hmp-file <fault-scenario-file>"
    exit 1
fi
if [ -f ${HMP_FILE} ]; then
    echo "Checking fault scenario file ${HMP_FILE} ... ok !"
else
    echo "ERROR: hmp file not found !"
    echo "USAGE: qsub grid-k.sh <app-name>  <hmp-file <fault-scenario-file>"
    exit 1
fi
echo ""

echo "############################################################"
echo "###################### BASIC DEBUG INFO ####################"
echo "############################################################"
echo Job $JOB_ID is running on `/bin/hostname`
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
#cd /tmp/
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
mkdir -p /tmp/wachter-$JOB_ID
cd /tmp/wachter-$JOB_ID
#svn export https://corfu.pucrs.br/svn/hemps_exe/branches/wachter/testcases/${FAULT_APP}.hmp  --username gaph --password "0gaph&svn" --no-auth-cache
cp ${HMP_FILE} .
echo ""

# compile scenario
echo "############################################################"
echo "################# GENERATING HEMPS DESIGN ##################"
echo "############################################################"
hemps ${FAULT_APP}.hmp
RETVAL=$?
[ $RETVAL != 0 ] && echo "Failure. unable to run HeMPS Generator"
cd ${FAULT_APP}
cd build/
make all
cd ..
make all
cp ${FULLPATH} signals
echo "############################################################"
echo "##################### HEMPS COMPILED #######################"
echo "############################################################"
echo ""

# run scenario
echo "############################################################"
echo "##################### SIMULATING HEMPS #####################"
echo "############################################################"
vsim -c -do sim.do
RETVAL=$?
[ $RETVAL != 0 ] && echo "Failure. unable to run modelsim or failure during simulation"
echo "############################################################"
echo "##################### END SIMULATION #######################"
echo "############################################################"
# when a simulation ends with  'End of simulation' in the transcript, then it is guaranteed that the app run succesfully.
# otherwise, one has to analysed the log files to check if the app finished or it crashed
RETVAL=`grep -i "End of simulation" ./transcript | wc -l`

# echo some debuf info into the logfile
echo Job $JOB_ID is running on `/bin/hostname`
echo Now it is `/bin/date`
#echo The log file will be saved at $PWD
echo SGE_STDOUT_PATH=$SGE_STDOUT_PATH
echo The user account is $USER
echo App name: ${FAULT_APP}
echo Log dir: ${LOGS_DIR}
echo Scenario file: ${FULLPATH}
cat ${FULLPATH}
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
\cp -f /tmp/wachter-$JOB_ID/${FAULT_APP}/signals /tmp/wachter-$JOB_ID/${FAULT_APP}/transcript /tmp/wachter-$JOB_ID/${FAULT_APP}/output_master.txt ${LOGS_DIR}
\cp -f $SGE_STDOUT_PATH ${LOGS_DIR}
\cp -f /tmp/wachter-$JOB_ID/${FAULT_APP}/log/*.txt ${LOGS_DIR}/log

# remove project file. each one uses almost 1G of disk space !!!
cd /tmp/wachter-$JOB_ID/${FAULT_APP}/build
make clean
cd /tmp/wachter-$JOB_ID/${FAULT_APP}
make clean
rm -rf /tmp/wachter-$JOB_ID/repository*

# generating exit code
if [ $RETVAL == 1 ]; then
   exit 0
fi
if [ $RETVAL != 1 ]; then
   exit 1
fi
