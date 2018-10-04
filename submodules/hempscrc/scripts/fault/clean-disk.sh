#!/bin/bash
#$ -j y
#$ -S /bin/bash

### Job name:
#$ -N clean-tmp

### this job can only be executed on Redhat or Suse. it cannot be executed on Ubuntu
#$ -q suse_excl.q

# HOW tO USE IT: 
# EXAMPLE: qsub -l hostname=GAPHL26 clean-disk.sh 
# this command clean the disk of gaphl26

echo "############################################################"
echo "###################### BASIC DEBUG INFO ####################"
echo "############################################################"
echo Job $JOB_ID is running on `/bin/hostname`
echo Now it is `/bin/date`
#echo The log file will be saved at $PWD
echo SGE_STDOUT_PATH=$SGE_STDOUT_PATH
echo The user account is $USER

echo "############################################################"
echo "############### DISK SPACE BEFORE CLEANING #################"
echo "############################################################"
echo "use(%)	 avail(GB)	 mount"
df -kh |  grep "/dev/" | awk '{ print $5 "\t " $4 "\t " $6 }'

cd /tmp
rm -rf wachter*

echo "############################################################"
echo "############### DISK SPACE AFTER CLEANING ##################"
echo "############################################################"
echo "use(%)	 avail(GB)	 mount"
df -kh |  grep "/dev/" | awk '{ print $5 "\t " $4 "\t " $6 }'


echo "Cleanning Finished!!!"
