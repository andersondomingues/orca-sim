//Marcelo Mandelli
#include <stdio.h>
#include <stdlib.h>
#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cin;
#include <list>
using std::list;
#include <iterator>
using std::cerr;
using std::ios;
#include "dependence.h"
#include "task.h"
#include "application.h"
#include "pe.h"
#include <fstream>
using namespace std;
#include <sstream>
using namespace std;
#include <vector>

#define MEM_SIZE 300001

string line, projectName, procDescription, procTypes ="";
int pageSize, memorySize, plasmaNumber = 0, mbliteNumber = 0;
int x_dimension, y_dimension, x_master, y_master, x_proc, y_proc, pos;
int runtimeApps = 0, staticTasks, runtimeTasks;
string appname, path, unity;
string taskname, status, proc;
string command, file_path;
task *t;
application *a;
list<task> tasks;
list <application> applications;
list <application>::iterator ia;	
vector<task> taskIDs;
list <task>::iterator it;
list <dependence>::iterator id;
dependence *d;

int get_taskID(string name);
bool compare_stime (application first, application second);
bool compare_flits (dependence first, dependence second);
string ftoa (float val);
string removeSpaces(string stringIn);
void Repository_vhd(int argc, string projectPath);
void Repository_h(int argc, string projectPath);
void Task_Dependences(int argc, string projectPath);
void Partial_Repositories(int argc, string projectPath);
		
int main(int argc, char **argv)
{
	float stime;
	string time;
	string simtime="";
	string constant_k="12";
	string projectPath;
	string svn = ".svn";
	 	
	if(argc > 3)
	{
		fprintf(stderr, "usage: %s <hmp-file> <prj-path>\n", argv[0]);
		fprintf(stderr, "argc = %d\n",argc);
		exit(1);
    }

	ifstream infile(argv[1]);
	
	if (infile==NULL)
	{
		cerr<<"Fatal error. could not read file \""  << argv[1] << "\"\n";
		exit(1);
	}
	else
	{
		        /* Reads [project name]*/
                getline(infile,line);
                getline(infile,line);
				projectName = line;
				
				/* Reads [page size] */
				getline(infile,line);
                getline(infile,line);
				pageSize = atoi(line.c_str());
				
                /* Reads [memory size] */
                getline(infile,line);
                getline(infile,line);
                memorySize = atoi(line.c_str());

                /* Reads [processor description] */
                getline(infile,line);
                getline(infile,line);
                procDescription = line;

                /* Reads [dimensions] */
                getline(infile,line);
                getline(infile,line);
                x_dimension = atoi(line.c_str());
                getline(infile,line);
                y_dimension = atoi(line.c_str());
                               
                pe *processors[x_dimension][y_dimension];
                
                /* Reads [processors core] */
                getline(infile,line);
                             
                for (int y = 0; y < y_dimension; y++)
                    for (int x = 0; x < x_dimension; x++)
                    {
                        getline(infile,line);
                        procTypes = procTypes + "\"" + line  + "\"" + ", ";
                        if (line == "plasma")
                        {
                            processors[x][y] = new pe(0, 0);
                            plasmaNumber++;
                        }
                        else if (line == "mblite")
                        {
                            processors[x][y] = new pe(1, 0);
                            mbliteNumber++;
						}
						else
						{
							cerr<<"Fatal error. Cant read the processor " << line << endl; 
							exit(1);
						}
					}
											
				getline(infile,line);
				
				if (line == "plasma" || line == "mblite") 
				{
					cerr<<"Fatal error. Number of processors are wrong. The number of processors must be equals " << x_dimension*y_dimension << "\n";
					exit(1);
				}
								
				/* Reads [master processor] */
                getline(infile,line);
                x_master = atoi(line.c_str());
                getline(infile,line);
                y_master = atoi(line.c_str());
                
                if (x_master > x_dimension-1 || y_master > y_dimension-1) 
				{
					cerr<<"Fatal error. Processor where master is allocated doesn't exist\n";
					exit(1);
				}
                
                processors[x_master][y_master]->set_master(1);
                
                /*for (int y = 0; y < y_dimension; y++)
				{
                    for (int x = 0; x < x_dimension; x++)
                    {
                            cout<<x<<" "<<y<<" : "<<processors[x][y]->get_type()<<" "<<processors[x][y]->get_master()<<endl;
                    }
                }*/
                
                /* Reads the applications infos */
                while (true)
                { 
					/* Reads [application] */
					getline(infile,line);
				    if(line=="[end]") break;
					
					/* Reads the application name */
                    getline(infile,line);
                    appname = line; 

                    /* Reads the application start time */
					getline(infile,line);
					time = line; 
					pos = time.find(" ", 0);
					stime = atof(time.substr(0,pos).c_str());
					unity = time.substr(pos+1, time.size());
					if (stime != 0)
					{
						runtimeApps++;
						if (unity == "ns") stime = stime / 1000000;
						if (unity == "us") stime = stime / 1000;
						if (unity == "s") stime = stime * 1000;
					}	
					
                    /* Reads [application directory] */
                    getline(infile,line);
                    getline(infile,line);
                    path = line; 
                    
                    /* Reads [allocated tasks] */
                    getline(infile,line);
					tasks.clear();
                    while (true)
                    {
                        getline(infile,line);
                        if(line=="[not allocated tasks]") break;
                        taskname = line;
                        
                        getline(infile,line);
                        x_proc = atoi(line.c_str());
                        getline(infile,line);
						y_proc = atoi(line.c_str());
						
						if (x_proc > x_dimension-1 || y_proc > y_dimension-1) 
						{
							cerr<<"Fatal error. Processor where the task is allocated doesn't exist\n";
							exit(1);
						}
						
						if (x_master == x_proc && y_master == y_proc) 
						{
							cerr<<"Fatal error. Master can't be allocated\n";
							exit(1);
						}
						
                        if (x_proc != -1)
                        {
							t = new task(taskname, appname, -1, x_proc, y_proc);  /* Task staticaly allocated */
                        }
                        else
                            t = new task(taskname, appname, y_proc, x_proc, y_proc);   /* Task dynamic allocated */

                        getline(infile,line);
                        getline(infile,line);
                       
                        tasks.push_back(*t);
					}

                    /* Reads the not allocated tasks */
                    getline(infile,line);
                    while (line!="[end application]") getline(infile,line);
                    
                    /* Reads simulation time  */
                    getline(infile,line);
                    if(line=="[sim time]") {
						getline(infile,line);
						simtime = line;
					}else{
						cerr<<"Fatal error. [sim time] not found\n";
						exit(1);
					}
                    
                    /* Reads K  */
                    getline(infile,line);
                    if(line=="[K]") {
						getline(infile,line);
						constant_k = line;
					}

                    a = new application(appname, path, time, stime);
                    a->tasks=tasks;
                    applications.push_back(*a);
                }

                infile.close();
                
            if (argc == 3)
			{
					stringstream aux;
					aux << argv[2];
					aux >> projectPath;
					
					int projectPath_size = projectPath.size();
					if (projectPath[projectPath_size-1] != '/') projectPath = projectPath + "/";
					
					command = "mkdir " + projectPath;
					system(command.c_str());
					command = "mkdir " + projectPath + projectName;
					system(command.c_str());
					command = "mkdir ./" + projectPath + projectName + "/plasma_ram";
					system(command.c_str());
					command = "mkdir ./" + projectPath + projectName + "/plasma_ram/sc";
					system(command.c_str());
					command = "mkdir ./" + projectPath + projectName + "/plasma_ram/rtl";
					system(command.c_str());
					command = "mkdir ./" + projectPath + projectName + "/mblite_ram";
					system(command.c_str());
					command = "mkdir ./" + projectPath + projectName + "/mblite_ram/sc";
					system(command.c_str());
					command = "mkdir ./" + projectPath + projectName + "/mblite_ram/rtl";
					system(command.c_str());
					command = "mkdir ./" + projectPath + projectName + "/build";
					system(command.c_str());
					command = "mkdir ./" + projectPath + projectName + "/log";
					system(command.c_str());
					command = "mkdir ./" + projectPath + projectName + "/applications";
					system(command.c_str());
					
			}
			
			else
			{              	
					command = "mkdir " + projectName;
					system(command.c_str());
					command = "mkdir ./" + projectName + "/plasma_ram";
					system(command.c_str());
					command = "mkdir ./" + projectName + "/plasma_ram/sc";
					system(command.c_str());
					command = "mkdir ./" + projectName + "/plasma_ram/rtl";
					system(command.c_str());
					command = "mkdir ./" + projectName + "/mblite_ram";
					system(command.c_str());
					command = "mkdir ./" + projectName + "/mblite_ram/sc";
					system(command.c_str());
					command = "mkdir ./" + projectName + "/mblite_ram/rtl";
					system(command.c_str());
					command = "mkdir ./" + projectName + "/build";
					system(command.c_str());
					command = "mkdir ./" + projectName + "/log";
					system(command.c_str());
					command = "mkdir ./" + projectName + "/applications";
					system(command.c_str());
			}
				
			if (argc == 3)
			{	
					for(ia=applications.begin();ia!=applications.end();ia++)
					{
						command = "mkdir " + projectPath + projectName + "/applications/" + appname;
						system(command.c_str());
						command = "cp -r $HEMPS_PATH/applications/" + ia->get_name() + "/*[^\".*\"]* ./" + projectPath + projectName + "/applications/" + appname;
						system(command.c_str());
					}		
			}			
			
			else
			{					
				for(ia=applications.begin();ia!=applications.end();ia++)
                {
					command = "mkdir " + projectName + "/applications/" + appname;
					system(command.c_str());
					command = "cp -fdpR $HEMPS_PATH/applications/" + ia->get_name() + "/*[^\".*\"]* ./" + projectName + "/applications/" + appname;
					system(command.c_str());
				}
			}

				applications.sort(compare_stime);
                
                Task_Dependences(argc,projectPath);
                
                staticTasks = 0;
                runtimeTasks = 0;
                
                for(ia=applications.begin();ia!=applications.end();ia++)
                {
					for(it=ia->tasks.begin();it!=ia->tasks.end();it++)
					{
						if(ia->get_stime()==0)
						{
							it->set_insertion_time(0);
							staticTasks++;
						}
						else
						{
							it->set_insertion_time(1);
							runtimeTasks++;
						}
						taskIDs.push_back(*it);
					}
				}
				
				/*for(int i=0;i<taskIDs.size();i++)
				{
					cout<<taskIDs[i].get_name()<<endl;
				}*/
				
				/* Generates the HeMPS_PKG.vhd */
				
				if (argc == 3) file_path = "./" + projectPath + projectName + "/HeMPS_PKG.vhd";	
				else file_path = "./" + projectName + "/HeMPS_PKG.vhd";
				ofstream hempspkg(file_path.c_str());
				
				int memSize, kernelPages, page_size_h_index, maxTasksSlave, page_number_h_index, masterAddress, masterAddress_h;
				
				memSize = memorySize * 1024;
				
				/* Calculates the kernel pages */
				if ((32 % pageSize) == 0)
					kernelPages = 32 / pageSize;
				else
					kernelPages = 32 / pageSize + 1;

				/* defines the maximum tasks/slave */
				maxTasksSlave = memorySize / pageSize - kernelPages;

				/* gets the page_size_h_index */
				if (pageSize == 32)
					page_size_h_index = 14;
				else
					page_size_h_index = 13;

				/* gets the page_number_h_index */
				if (maxTasksSlave < 4)
					page_number_h_index = page_size_h_index + 2;
				else
					page_number_h_index = page_size_h_index + 3;
					
				/* gets the master adress */
				masterAddress = y_master * x_dimension + x_master;
				masterAddress_h = x_master * 10 + y_master;

                hempspkg<<"\n";
                hempspkg<<"--------------------------------------------------------------------------\n";
                hempspkg<<"-- package com tipos basicos\n";
                hempspkg<<"--------------------------------------------------------------------------\n";
                hempspkg<<"library IEEE;\n";
                hempspkg<<"use IEEE.Std_Logic_1164.all;\n";
                hempspkg<<"use IEEE.std_logic_unsigned.all;\n";
                hempspkg<<"use IEEE.std_logic_arith.all;\n";
                hempspkg<<"\n";
                hempspkg<<"package HeMPS_PKG is\n";
                hempspkg<<"\n";
                hempspkg<<"--------------------------------------------------------\n";
                hempspkg<<"-- HEMPS CONSTANTS\n";
                hempspkg<<"--------------------------------------------------------\n";
                hempspkg<<"\t-- paging definitions\n";
                hempspkg<<"\tconstant PAGE_SIZE_H_INDEX\t\t: integer := "<<page_size_h_index<<";\n";
                hempspkg<<"\tconstant PAGE_NUMBER_H_INDEX\t: integer := "<<page_number_h_index<<";\n";
                hempspkg<<"\n";
                hempspkg<<"\t-- Hemps top definitions\n";
                hempspkg<<"\tconstant NUMBER_PROCESSORS_X\t: integer := "<<x_dimension<<"; \n";
                hempspkg<<"\tconstant NUMBER_PROCESSORS_Y\t: integer := "<<y_dimension<<"; \n";
                hempspkg<<"\n";
                hempspkg<<"\tconstant MASTER_ADDRESS\t\t\t: integer := "<<masterAddress<<";\n";
                hempspkg<<"\tconstant NUMBER_PROCESSORS\t\t: integer := NUMBER_PROCESSORS_Y*NUMBER_PROCESSORS_X;\n";
                hempspkg<<"\n";
                hempspkg<<"\tconstant NUMBER_OF_APPS\t\t\t: integer := "<<runtimeApps<<";\n";
                if (runtimeApps!= 0)
                {
                    string aux = "";
                    float pastTime = 0;
                    float delay = 0;
                    for(ia=applications.begin();ia!=applications.end();ia++)
					{	
                        if (ia->get_stime() != 0)
                        {
                            delay = ia->get_stime() - pastTime;
                            aux = aux + ftoa(delay) + " ms,";
                            pastTime = ia->get_stime();
                        }
                    }
                    aux = "(" + aux + "0 ms);";
                    hempspkg<<"\ttype timearray is array(0 to NUMBER_OF_APPS) of time;\n";
                    hempspkg<<"\tconstant appstime : timearray := "<<aux<<"\n";
                }
                else
                {
                    hempspkg<<"\ttype timearray is array(0 to 1) of time;\n";
                    hempspkg<<"\tconstant appstime : timearray := (0ms, 0ms);\n";
                }
                hempspkg<<"\n";
                hempspkg<<"\tsubtype core_str is string(1 to 6);\n";
                hempspkg<<"\ttype core_type_type is array(0 to NUMBER_PROCESSORS-1) of core_str;\n"; 
                procTypes = "(" + procTypes + ");";
                pos = procTypes.find(", );",0);
                procTypes = procTypes.substr(0, pos) + ");";
                hempspkg<<"\tconstant core_type : core_type_type := "<<procTypes<<"\n";
                hempspkg<<"end HeMPS_PKG;\n";
				
				hempspkg.close();
	
				/* Genearating the HeMPS_PKG_H */
				
				if (argc == 3) file_path = "./" + projectPath + projectName + "/HeMPS_PKG.h";
				else file_path = "./" + projectName + "/HeMPS_PKG.h";
				ofstream hempspkgh(file_path.c_str());

                hempspkgh<<"#define N_PE_X "<<x_dimension<<"\n";
                hempspkgh<<"#define N_PE_Y "<<y_dimension<<"\n";
                hempspkgh<<"#define N_PE N_PE_Y*N_PE_X\n";
                hempspkgh<<"#define MASTER "<<masterAddress<<"\n";
                hempspkgh<<"\n";
				
				hempspkgh.close();
				
				
				/* Generates the makefile */
				string all;
				string taskInclude, taskName;
				all = "all: ";
				
				if (argc == 3) file_path = "./" + projectPath + projectName + "/build/makefile";
				else file_path = "./" + projectName + "/build/makefile";
				ofstream makefile(file_path.c_str());

				makefile<<"#this environment variable must point to the hemps path, where the hardware, software and tools folders are located"<<endl;
				makefile<<endl;
				makefile<<"#Definition of Plasma toolchain"<<endl;
				makefile<<"CFLAGS     = -O2 -Wall -c -s"<<endl;
				makefile<<"GCC_MIPS   = mips-elf-gcc $(CFLAGS)"<<endl;
				makefile<<"AS_MIPS    = mips-elf-as"<<endl;
				makefile<<"LD_MIPS    = mips-elf-ld"<<endl;
				makefile<<"DUMP_MIPS  = mips-elf-objdump"<<endl;
				makefile<<"COPY_MIPS = mips-elf-objcopy -I elf32-bigmips -O binary\n"<<endl;
					
				makefile<<"#Definition of MB-Lite toolchain"<<endl;
				makefile<<"MB         = mb-gcc"<<endl;
				makefile<<"AS         = mb-as"<<endl;
				makefile<<"LD         = mb-ld"<<endl;
				makefile<<"MB_OBJCOPY = mb-objcopy"<<endl;
				makefile<<"MB_OBJDUMP = mb-objdump"<<endl;

				makefile<<"XILFLAGS   =-mxl-soft-div -msoft-float -mxl-barrel-shift -mno-xl-soft-mul"<<endl;
				makefile<<"CXXFLAGS   =-g -std=c99 -pedantic -Wall -O2 "<<endl;
				makefile<<"LNKFLAGS   =-Wl,-defsym -Wl,_STACK_SIZE=0x3000 -Wl,-defsym -Wl,_HEAP_SIZE=0x0000"<<endl;
				makefile<<"LNKFLAGS2  =-Wl,-defsym -Wl,_STACK_SIZE=0x2000 -Wl,-defsym -Wl,_HEAP_SIZE=0x0000"<<endl;

				makefile<<"MB_GCC     = $(MB) $(XILFLAGS) $(CXXFLAGS) $(LNKFLAGS2) $(LIBFLAGS) $(INCFLAGS) $(CCFLAGS)"<<endl;

				makefile<<endl;

				makefile<<"#TOOLS"<<endl;
				makefile<<"BIN2MEM       = bin2mem"<<endl;
				makefile<<"RAM_GENERATOR = ram_generator"<<endl;
				makefile<<endl;

				makefile<<"INCLUDE       = $(HEMPS_PATH)/software/include"<<endl;
				makefile<<endl;

				makefile<<"#TASKS"<<endl;

				int num_tasks = 0;

				for(int i=0;i<taskIDs.size();i++)
				{

					num_tasks = i;

					/* sets the task path */
					makefile<<"TASK"<<num_tasks<<"_PATH = "<<"../applications/"<< taskIDs[i].get_application()<<"/"<<taskIDs[i].get_name()<<endl;

					/* sets the task include files */
					taskInclude = "ids_" + taskIDs[i].get_application() + ".h";
					makefile<<"TASK"<<num_tasks<<"_INCLUDE = "<<taskInclude<<endl;

					/* sets the task source name */
					taskName = taskIDs[i].get_name();
					pos = taskName.find(".",0);
					taskName = taskName.substr(0,pos);
					makefile<<"TASK"<<num_tasks<<"_NAME = "<<taskName<<endl;

					/* sets the task ID */
					makefile<<"TASK"<<num_tasks<<"_ID = "<<i<<endl;

					/* sets the task make target */
					makefile<<"TASK"<<num_tasks<<"_TARGET = $(TASK"<<num_tasks<<"_NAME)_$(TASK"<<num_tasks<<"_ID)"<<endl;

					makefile<<endl;

				}

				makefile<<"#tasks boot code for Plasma processor"<<endl;
				makefile<<"BOOT_TASK_SRC     = $(HEMPS_PATH)/software/include/bootTask.asm"<<endl;
				makefile<<"BOOT_TASK         = bootTask"<<endl;
				makefile<<"#kernel master source files"<<endl;
				makefile<<"BOOT_MASTER_SRC   = $(HEMPS_PATH)/software/kernel/master/boot.S"<<endl;
				makefile<<"BOOT_MASTER       = boot_master"<<endl;
				makefile<<"KERNEL_MASTER_SRC = $(HEMPS_PATH)/software/kernel/master/kernel.c"<<endl;
				makefile<<"KERNEL_MASTER     = kernel_master"<<endl;
				makefile<<"#kernel slave plasma source files"<<endl;
				makefile<<"BOOT_PLASMA_SRC   = $(HEMPS_PATH)/software/kernel/slave/boot.S"<<endl;
				makefile<<"BOOT_PLASMA       = boot_plasma"<<endl;
				makefile<<"KERNEL_PLASMA_SRC = $(HEMPS_PATH)/software/kernel/slave/kernel.c"<<endl;
				makefile<<"KERNEL_PLASMA     = kernel_plasma"<<endl;
				makefile<<"#kernel slave mblite source files"<<endl;
				makefile<<"KERNEL_MBLITE     = kernel_mblite"<<endl;
				makefile<<"INTERRUPT_SRC     = $(HEMPS_PATH)/software/kernel/slave/interrupt.s"<<endl;
				makefile<<"INTERRUPT         = interrupt"<<endl;
				makefile<<endl;


				/* Task boot make target */
				makefile<<"bootTask:"<<endl;
				makefile<<"\t$(AS_MIPS) -o $(BOOT_TASK).o $(BOOT_TASK_SRC)"<<endl;
				makefile<<endl;
				all = all +"bootTask ";

				/* Kernel master make target */
				makefile<<"kernel_master:"<<endl;
				makefile<<"\t$(AS_MIPS) -o $(BOOT_MASTER).o $(BOOT_MASTER_SRC)"<<endl;
				makefile<<"\t$(GCC_MIPS) -o $(KERNEL_MASTER).o $(KERNEL_MASTER_SRC) --include ids_master.h --include InitializeVectorsMaster.h"<<endl;
				makefile<<"\t$(LD_MIPS) -Ttext 0 -eentry -Map $(KERNEL_MASTER).map -s -N -o $(KERNEL_MASTER).bin  $(BOOT_MASTER).o $(KERNEL_MASTER).o"<<endl;
				makefile<<"\t$(LD_MIPS) -Ttext 0 -eentry -Map $(KERNEL_MASTER)_debug.map -o $(KERNEL_MASTER)_debug.bin  $(BOOT_MASTER).o $(KERNEL_MASTER).o"<<endl;
				makefile<<"\t$(DUMP_MIPS) -S $(KERNEL_MASTER)_debug.bin > $(KERNEL_MASTER).lst"<<endl;
				makefile<<"\t$(COPY_MIPS) $(KERNEL_MASTER).bin $(KERNEL_MASTER).dump"<<endl;
				makefile<<"\thexdump -v -e '1/1 \"%02x\" 1/1 \"%02x\" 1/1 \"%02x\" 1/1 \"%02x\" \"\\n\"' $(KERNEL_MASTER).dump > $(KERNEL_MASTER).txt"<<endl;
				makefile<<"\tsed -i -e '4s/.*/37bdFFFF/' $(KERNEL_MASTER).txt"<<endl;
				makefile<<endl;
				all = all +  "kernel_master ";


				/* Kernel slave make target - Plasma */
				makefile<<"kernel_plasma:"<<endl;
				makefile<<"\t$(AS_MIPS) -o $(BOOT_PLASMA).o $(BOOT_PLASMA_SRC)"<<endl;
				makefile<<"\t$(GCC_MIPS) -o $(KERNEL_PLASMA).o $(KERNEL_PLASMA_SRC) --include ids_slave.h --include InitializeVectorsSlave.h -D PLASMA"<<endl;
				makefile<<"\t$(LD_MIPS) -Ttext 0 -eentry -Map $(KERNEL_PLASMA).map -s -N -o $(KERNEL_PLASMA).bin  $(BOOT_PLASMA).o $(KERNEL_PLASMA).o"<<endl;
				makefile<<"\t$(LD_MIPS) -Ttext 0 -eentry -Map $(KERNEL_PLASMA)_debug.map -o $(KERNEL_PLASMA)_debug.bin  $(BOOT_PLASMA).o $(KERNEL_PLASMA).o"<<endl;
				makefile<<"\t$(DUMP_MIPS) -S $(KERNEL_PLASMA)_debug.bin > $(KERNEL_PLASMA).lst"<<endl;
				makefile<<"\t$(COPY_MIPS) $(KERNEL_PLASMA).bin $(KERNEL_PLASMA).dump"<<endl;
				makefile<<"\thexdump -v -e '1/1 \"%02x\" 1/1 \"%02x\" 1/1 \"%02x\" 1/1 \"%02x\" \"\\n\"' $(KERNEL_PLASMA).dump > $(KERNEL_PLASMA).txt"<<endl;
				makefile<<"\tsed -i -e '4s/.*/37bd8000/' $(KERNEL_PLASMA).txt"<<endl;
				makefile<<endl;
				all = all +  "kernel_plasma "; 

				
				/* Kernel slave make target - MBLite */
				makefile<<"kernel_mblite:"<<endl;
				if(mbliteNumber > 0)
				{
					makefile<<"\t$(AS) -o $(INTERRUPT).o $(INTERRUPT_SRC)"<<endl;
					makefile<<"\t$(MB_GCC) $(KERNEL_PLASMA_SRC) $(INTERRUPT).o -o $(KERNEL_MBLITE).elf --include ids_slave.h --include InitializeVectorsSlave.h -D MBLITE"<<endl;
					makefile<<"\t$(MB_OBJCOPY) -O binary $(KERNEL_MBLITE).elf $(KERNEL_MBLITE).bin"<<endl;
					makefile<<"\t$(MB_OBJDUMP) -DSCz $(KERNEL_MBLITE).elf > $(KERNEL_MBLITE).dump"<<endl;
					makefile<<"\t$(BIN2MEM) $(KERNEL_MBLITE).bin > $(KERNEL_MBLITE).txt"<<endl;
					makefile<<"\tgrep '<puta>' $(KERNEL_MBLITE).dump | sed 's/^....\\(....\\).*$$/\\1/' | xargs -I {} sed -i -e '5s/.*/b808{}/' $(KERNEL_MBLITE).txt"<<endl;	
					makefile<<endl;
				}else
				{
					makefile<<"\t@echo \"generating dummy kernel for mblite processor\""<<endl;
					makefile<<"\tcp $(KERNEL_MASTER).txt $(KERNEL_MBLITE).txt"<<endl;
					makefile<<endl;
				}
				
				all = all + "kernel_mblite ";

				/*  Generate the tasks make targets - tasks in the repositories */
				/*  Generate the tasks make targets - tasks in processors */
				for(int i=0;i<taskIDs.size();i++)
				{

						/* Retrives the parent application, which the task belongs */
						taskInclude = "ids_" + taskIDs[i].get_application() + ".h";

						taskName = taskIDs[i].get_name();
						pos = taskName.find(".",0);
						taskName = taskName.substr(0,pos) + "_" + ftoa(i);
						all = all + taskName + " ";	
						num_tasks = i;
						
						/* Task make target */
						makefile<<taskName<<":"<<endl;
						/*cout<<taskName<<endl;
						cout<<"Status "<<taskIDs[i].get_status()<<endl;
						cout<<"x "<<taskIDs[i].get_x_proc()<<" "<<"y "<<taskIDs[i].get_y_proc()<<" "<<processors[taskIDs[i].get_x_proc()][taskIDs[i].get_y_proc()]->get_type()<<endl; */

						if ((taskIDs[i].get_status()==0) || (taskIDs[i].get_status()==-1 && processors[taskIDs[i].get_x_proc()][taskIDs[i].get_y_proc()]->get_type()==0)) /* Plasma repository */
						{
							makefile<<"\t$(GCC_MIPS) $(TASK"<<num_tasks<<"_PATH) -o $(TASK"<<num_tasks<<"_TARGET).o --include $(TASK"<<num_tasks<<"_INCLUDE)"<<" -D PLASMA -I $(INCLUDE)"<<endl;
							makefile<<"\t$(LD_MIPS) -Ttext 0 -eentry -Map $(TASK"<<num_tasks<<"_TARGET).map "<<"-s -N -o $(TASK"<<num_tasks<<"_TARGET).bin "<<"$(BOOT_TASK).o $(TASK"<<num_tasks<<"_TARGET).o"<<endl;
							makefile<<"\t$(LD_MIPS) -Ttext 0 -eentry -Map $(TASK"<<num_tasks<<"_TARGET)_debug.map "<<"-o $(TASK"<<num_tasks<<"_TARGET)_debug.bin "<<"$(BOOT_TASK).o $(TASK"<<num_tasks<<"_TARGET).o"<<endl;
							makefile<<"\t$(DUMP_MIPS) -S $(TASK"<<num_tasks<<"_TARGET)_debug.bin > $(TASK"<<num_tasks<<"_TARGET).lst"<<endl;
							makefile<<"\t$(COPY_MIPS) $(TASK"<<num_tasks<<"_TARGET).bin $(TASK"<<num_tasks<<"_TARGET).dump"<<endl;
							makefile<<"\thexdump -v -e '1/1 \"%02x\" 1/1 \"%02x\" 1/1 \"%02x\" 1/1 \"%02x\" \"\\n\"' $(TASK"<<num_tasks<<"_TARGET).dump > $(TASK"<<num_tasks<<"_TARGET).txt"<<endl;
							makefile<<"\tsed -i -e '4s/.*/37bd4000/' $(TASK"<<num_tasks<<"_TARGET).txt"<<endl;
							makefile<<endl;

						}
						if ((taskIDs[i].get_status()==1) || (taskIDs[i].get_status()==-1 && processors[taskIDs[i].get_x_proc()][taskIDs[i].get_y_proc()]->get_type()==1)) /* MBLite repository */
						{

							makefile<<"\t$(MB_GCC) $(TASK"<<num_tasks<<"_PATH) -o $(TASK"<<num_tasks<<"_TARGET).elf --include $(TASK"<<num_tasks<<"_INCLUDE) -D MBLITE -I $(INCLUDE)"<<endl;
							makefile<<"\t$(MB_OBJCOPY) -O binary $(TASK"<<num_tasks<<"_TARGET).elf $(TASK"<<num_tasks<<"_TARGET).bin"<<endl;
							makefile<<"\t$(MB_OBJDUMP) -DSCz $(TASK"<<num_tasks<<"_TARGET).elf > $(TASK"<<num_tasks<<"_TARGET).lst"<<endl;
							makefile<<"\t$(BIN2MEM) $(TASK"<<num_tasks<<"_TARGET).bin > $(TASK"<<num_tasks<<"_TARGET).txt"<<endl;
							makefile<<endl;
						}
				}

				makefile<<"ram_gen: ram_master ram_plasma ram_mblite"<<endl;
				makefile<<"\t"<<endl;
				makefile<<"ram_master:"<<endl;
				makefile<<"\t$(RAM_GENERATOR) -64 -vhd kernel_master.txt > ram_master.vhd"<<endl;
				makefile<<"\tcp ram_master.vhd ../plasma_ram/rtl"<<endl;
				makefile<<"\t$(RAM_GENERATOR) -64 -h kernel_master.txt > ram_master.h"<<endl;
				makefile<<"\tcp ram_master.h ../plasma_ram/sc"<<endl;
				makefile<<endl;
				makefile<<"ram_plasma:"<<endl;
				makefile<<"\t$(RAM_GENERATOR) -64 -vhd kernel_plasma.txt > ram_plasma.vhd"<<endl;
				makefile<<"\tcp ram_plasma.vhd ../plasma_ram/rtl"<<endl;
				makefile<<"\t$(RAM_GENERATOR) -64 -h kernel_plasma.txt > ram_plasma.h"<<endl;
				makefile<<"\tcp ram_plasma.h ../plasma_ram/sc"<<endl;
				makefile<<endl;
				makefile<<"ram_mblite:"<<endl;
				makefile<<"\t$(RAM_GENERATOR) -64 -vhd kernel_mblite.txt > ram_mblite.vhd"<<endl;
				makefile<<"\tcp ram_mblite.vhd ../mblite_ram/rtl"<<endl;
				makefile<<"\t$(RAM_GENERATOR) -64 -h kernel_mblite.txt > ram_mblite.h"<<endl;
				makefile<<"\tcp ram_mblite.h ../mblite_ram/sc"<<endl;
				makefile<<endl;
				all = all + "ram_gen";
				
				makefile<<"clean:"<<endl;
				makefile<<"\trm -rf *.bin"<<endl;
				makefile<<"\trm -rf *.txt"<<endl;
				makefile<<"\trm -rf *.mem"<<endl;
				makefile<<"\trm -rf *.dump"<<endl;
				makefile<<"\trm -rf *.lst"<<endl;
				makefile<<"\trm -rf *.o"<<endl;
				makefile<<"\trm -rf *.map"<<endl;
				makefile<<"\trm -rf ram*.h"<<endl;
				makefile<<"\trm -rf *.vhd"<<endl;
				makefile<<"\trm -rf *.elf"<<endl;
				makefile<<endl;

				makefile<<all;
				makefile<<endl;

				makefile.close();
				
				string SimMakefile = "";
				
				SimMakefile = SimMakefile + "LIB=work\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "#this environment variable must point to the hemps path, where the hardware, software and tools folders are located\n";
				SimMakefile = SimMakefile + "BASE_PATH=$(HEMPS_PATH)\n";
				SimMakefile = SimMakefile + "HW_PATH=hardware\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "#VHDL files\n";
				SimMakefile = SimMakefile + "PKG_SRC=HeMPS_defaults.vhd\n";
				SimMakefile = SimMakefile + "PKG_DIR=$(HW_PATH)\n";
				SimMakefile = SimMakefile + "PKG_PATH=$(addprefix $(PKG_DIR)/,$(PKG_SRC))\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "SCENARIO_SRC=HeMPS_PKG repository dynamic_apps\n";
				SimMakefile = SimMakefile + "SCENARIO_DIR=./\n";
				SimMakefile = SimMakefile + "SCENARIO_PATH=$(addprefix $(SCENARIO_DIR)/,$(SCENARIO_SRC))\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "MPACK_SRC=mlite_pack.vhd UartFile.vhd\n";
				SimMakefile = SimMakefile + "MPACK_DIR=$(HW_PATH)/plasma/rtl\n";
				SimMakefile = SimMakefile + "MPACK_PATH=$(addprefix $(MPACK_DIR)/,$(MPACK_SRC))\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "MLITE_SRC=alu.vhd bus_mux.vhd control.vhd mem_ctrl.vhd mult.vhd pc_next.vhd pipeline.vhd reg_bank.vhd shifter.vhd mlite_cpu.vhd\n";
				SimMakefile = SimMakefile + "MLITE_DIR=$(HW_PATH)/plasma/rtl\n";
				SimMakefile = SimMakefile + "MLITE_PATH=$(addprefix $(MLITE_DIR)/,$(MLITE_SRC))\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "CRC_SRC=Crc_xor.vhd\n";
				SimMakefile = SimMakefile + "CRC_DIR=$(HW_PATH)/router/rtl\n";
				SimMakefile = SimMakefile + "CRC_PATH=$(CRC_DIR)/$(CRC_SRC)\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "#NI_SRC=network_interface.vhd\n";
				SimMakefile = SimMakefile + "NI_SRC=Network_Interface.vhd\n";
				SimMakefile = SimMakefile + "NI_DIR=$(HW_PATH)/ni/rtl\n";
				SimMakefile = SimMakefile + "NI_PATH=$(NI_DIR)/$(NI_SRC)\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "DMA_SRC=dma.vhd dma_master.vhd\n";
				SimMakefile = SimMakefile + "DMA_DIR=$(HW_PATH)/dma/rtl\n";
				SimMakefile = SimMakefile + "DMA_PATH=$(addprefix $(DMA_DIR)/,$(DMA_SRC))\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "#ROUTER_SRC=Hermes_buffer.vhd Hermes_crossbar.vhd Hermes_switchcontrol.vhd RouterCC.vhd\n";
				// SimMakefile = SimMakefile + "ROUTER_SRC=seek/stepdet.vhd seek/seek.vhd Input_Buffer.vhd CS_Buffer.vhd Crossbars.vhd Switch_Control.vhd mux_control.vhd local_monitor.vhd Crc_xor.vhd Router_seek.vhd\n";
				SimMakefile = SimMakefile + "ROUTER_SRC=seek/stepdet.vhd seek/seek.vhd Input_Buffer.vhd CS_Buffer.vhd Crossbars.vhd Switch_Control.vhd mux_control.vhd local_monitor.vhd Router_seek.vhd\n";
				SimMakefile = SimMakefile + "ROUTER_DIR=$(HW_PATH)/router/rtl\n";
				SimMakefile = SimMakefile + "ROUTER_PATH=$(addprefix $(ROUTER_DIR)/,$(ROUTER_SRC))\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "PLASMA_RAM_SRC=ram_master ram_plasma\n";
				SimMakefile = SimMakefile + "PLASMA_RAM_DIR=$(SCENARIO_DIR)/plasma_ram/rtl\n";
				SimMakefile = SimMakefile + "PLASMA_RAM_PATH=$(addprefix $(PLASMA_RAM_DIR)/,$(PLASMA_RAM_SRC))\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "MBLITE_RAM_SRC=ram_mblite\n";
				SimMakefile = SimMakefile + "MBLITE_RAM_DIR=$(SCENARIO_DIR)/mblite_ram/rtl\n";
				SimMakefile = SimMakefile + "MBLITE_RAM_PATH=$(MBLITE_RAM_DIR)/$(MBLITE_RAM_SRC)\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "MBLITE_SRC=mblite_soc.vhd\n";
				SimMakefile = SimMakefile + "MBLITE_DIR=$(HW_PATH)/mblite/rtl\n";
				SimMakefile = SimMakefile + "MBLITE_PATH=$(addprefix $(MBLITE_DIR)/,$(MBLITE_SRC))\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "MBLITE_CORE_SRC=../config_Pkg.vhd core_Pkg.vhd core.vhd fetch.vhd gprf.vhd decode.vhd execute.vhd mem.vhd\n";
				SimMakefile = SimMakefile + "MBLITE_CORE_DIR=$(MBLITE_DIR)/core\n";
				SimMakefile = SimMakefile + "MBLITE_CORE_PATH=$(addprefix $(MBLITE_CORE_DIR)/,$(MBLITE_CORE_SRC))\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "WRAPPER_CELL_SRC=wrapper_cell.vhd wrapper_cell_N.vhd\n";
				SimMakefile = SimMakefile + "WRAPPER_CELL_DIR=$(HW_PATH)\n";
				SimMakefile = SimMakefile + "WRAPPER_CELL_PATH=$(addprefix $(WRAPPER_CELL_DIR)/,$(WRAPPER_CELL_SRC))\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "MBLITE_STD_SRC=std_Pkg.vhd dsram.vhd\n"; 
				SimMakefile = SimMakefile + "MBLITE_STD_DIR=$(MBLITE_DIR)/std\n";
				SimMakefile = SimMakefile + "MBLITE_STD_PATH=$(addprefix $(MBLITE_STD_DIR)/,$(MBLITE_STD_SRC))\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "PLASMA_SRC=access_repository.vhd plasma.vhd\n";
				SimMakefile = SimMakefile + "PLASMA_DIR=$(HW_PATH)/plasma/rtl\n";
				SimMakefile = SimMakefile + "PLASMA_PATH=$(addprefix $(PLASMA_DIR)/,$(PLASMA_SRC))\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "TOP_WRAPPED_SRC=router_seek.vhd test_port.vhd test_port_receive.vhd fail_detect.vhd seek_manager.vhd processing_element.vhd processing_element_wrapped.vhd HeMPS.vhd insert_application.vhd test_bench.vhd\n";
				SimMakefile = SimMakefile + "TOP_WRAPPED_DIR=$(HW_PATH)\n";
				SimMakefile = SimMakefile + "TOP_WRAPPED_PATH=$(addprefix $(TOP_WRAPPED_DIR)/,$(TOP_WRAPPED_SRC))\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "#SystemC files\n";
				SimMakefile = SimMakefile + "SC_NOC_DIR=$(HW_PATH)/router/sc\n";
				SimMakefile = SimMakefile + "SC_MLITE_DIR=$(HW_PATH)/plasma/sc\n";
				SimMakefile = SimMakefile + "SC_RAM_DIR=$(SCENARIO_DIR)/plasma_ram/sc\n";
				SimMakefile = SimMakefile + "SC_MBLITE_RAM_DIR=$(SCENARIO_DIR)/mblite_ram/sc\n";
				SimMakefile = SimMakefile + "SC_PLASMA_DIR=$(HW_PATH)/plasma/sc\n";
				SimMakefile = SimMakefile + "SC_TESTBENCH_DIR=$(HW_PATH)/sc\n";
				SimMakefile = SimMakefile + "SC_NI_DIR=$(HW_PATH)/ni/sc\n";
				SimMakefile = SimMakefile + "SC_DMA_DIR=$(HW_PATH)/dma/sc\n";
				SimMakefile = SimMakefile + "SC_ACCESS_REPO_DIR=$(HW_PATH)/plasma/sc\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "SC_NOC_SRC=queue.cpp switchcontrol.cpp router_cc.cpp \n";
				SimMakefile = SimMakefile + "SC_MLITE_SRC=mlite_cpu.cpp\n";
				SimMakefile = SimMakefile + "SC_RAM_SRC=ram_master ram_plasma\n";
				SimMakefile = SimMakefile + "SC_MBLITE_RAM_SRC=ram_mblite\n";
				SimMakefile = SimMakefile + "SC_PLASMA_SRC=plasma_master.cpp plasma_slave.cpp\n";
				SimMakefile = SimMakefile + "SC_TESTBENCH_SRC=hemps.cpp test_bench.cpp\n";
				SimMakefile = SimMakefile + "SC_NI_SRC=Network_Interface.cpp\n";
				SimMakefile = SimMakefile + "SC_DMA_SRC=dma.cpp dma_master.cpp\n";
				SimMakefile = SimMakefile + "SC_ACCESS_REPO_SRC=access_repository.cpp\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "SC_NOC_PATH=$(addprefix $(SC_NOC_DIR)/,$(SC_NOC_SRC))\n";
				SimMakefile = SimMakefile + "SC_MLITE_PATH=$(addprefix $(SC_MLITE_DIR)/,$(SC_MLITE_SRC))\n";
				SimMakefile = SimMakefile + "SC_RAM_PATH=$(addprefix $(SC_RAM_DIR)/,$(SC_RAM_SRC))\n";
				SimMakefile = SimMakefile + "SC_MBLITE_RAM_PATH=$(SC_MBLITE_RAM_DIR)/$(SC_MBLITE_RAM_SRC)\n";
				SimMakefile = SimMakefile + "SC_PLASMA_PATH=$(addprefix $(SC_PLASMA_DIR)/,$(SC_PLASMA_SRC))\n";
				SimMakefile = SimMakefile + "SC_TESTBENCH_PATH=$(addprefix $(SC_TESTBENCH_DIR)/,$(SC_TESTBENCH_SRC))\n";
				SimMakefile = SimMakefile + "SC_NI_PATH=$(addprefix $(SC_NI_DIR)/,$(SC_NI_SRC))\n";
				SimMakefile = SimMakefile + "SC_DMA_PATH=$(addprefix $(SC_DMA_DIR)/,$(SC_DMA_SRC))\n";
				SimMakefile = SimMakefile + "SC_ACCESS_REPO_PATH=$(addprefix $(SC_ACCESS_REPO_DIR)/,$(SC_ACCESS_REPO_SRC))\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "#compilers definitions\n";
				SimMakefile = SimMakefile + "INC=-Iplasma_ram/sc/ -I./\n";
				SimMakefile = SimMakefile + "VHD_C=@vcom -2008\n";
				SimMakefile = SimMakefile + "GEN=g++ -o HeMPS *.o -L. -Linc/. -lsystemc\n";
				SimMakefile = SimMakefile + "\n";

				if (procDescription == "RTL" || procDescription == "ISS")
				{
					SimMakefile = SimMakefile + "#modelsim gcc compiler\n";
					SimMakefile = SimMakefile + "SC_C=@sccom -work $(LIB) -g\n";
					SimMakefile = SimMakefile + "#systemc g++ compiler\n";
					SimMakefile = SimMakefile + "#SC_C=g++ -c -g -Wall -O2\n";
				}
				else
				{
					if (procDescription == "sc")
					{
						SimMakefile = SimMakefile + "#modelsim gcc compiler\n";
						SimMakefile = SimMakefile + "#SC_C=@sccom -work $(LIB) -g\n";
						SimMakefile = SimMakefile + "#systemc g++ compiler\n";
						SimMakefile = SimMakefile + "SC_C=g++ -c -g -Wall -O2\n";
					}
					else
					{
						SimMakefile = SimMakefile + "#modelsim gcc compiler\n";
						SimMakefile = SimMakefile + "SC_C=@sccom -work $(LIB) -g\n";
						SimMakefile = SimMakefile + "#systemc g++ compiler\n";
						SimMakefile = SimMakefile + "#SC_C=g++ -c -g -Wall -O2\n";
					}
				}
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "default:\n";
				SimMakefile = SimMakefile + "	@echo \"Makefile for the hemps in systemc\"\n";
				SimMakefile = SimMakefile + "	@echo \"---------------------------------------\"\n";
				SimMakefile = SimMakefile + "	@echo \"Make options:\"\n";
				SimMakefile = SimMakefile + "	@echo \"   default: Echo these instructions\"\n";
				SimMakefile = SimMakefile + "	@echo \"       lib: Generate work dir and map its library\"\n";
				SimMakefile = SimMakefile + "	@echo \"       vhd: Compile vhd HeMPS description files\"\n";
				SimMakefile = SimMakefile + "	@echo \"       iss: Compile vhd and SystemC HeMPS description files\"\n";
				SimMakefile = SimMakefile + "	@echo \"       iss_wrapped: Compile vhd and SystemC HeMPS description files with wrapper\"\n";
				SimMakefile = SimMakefile + "	@echo \"       all: Compile sytemc and vhd files\"\n";
				SimMakefile = SimMakefile + "	@echo \"     clean: Remove all compiled and generated files\"\n";
				SimMakefile = SimMakefile + "	@echo\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "# mblite is not used yet. it uses NI with 2 physical links\n";
				SimMakefile = SimMakefile + "#iss: lib $(SCENARIO_PATH) $(PKG_PATH) $(MPACK_PATH) $(SC_MLITE_PATH) $(NI_PATH) $(DMA_PATH) $(ROUTER_PATH) $(SC_RAM_PATH) $(MBLITE_STD_PATH) $(MBLITE_CORE_PATH) $(SC_MBLITE_RAM_PATH) $(MBLITE_PATH) $(PLASMA_PATH) $(TOP_PATH)\n";
				SimMakefile = SimMakefile + "iss: lib $(SCENARIO_PATH) $(PKG_PATH) $(MPACK_PATH) $(SC_MLITE_PATH) $(NI_PATH) $(ROUTER_PATH) $(DMA_PATH) $(SC_RAM_PATH) $(PLASMA_PATH) $(TOP_PATH)\n";
				SimMakefile = SimMakefile + "	@sccom -link\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "iss_wrapped: lib $(SCENARIO_PATH) $(PKG_PATH) $(MPACK_PATH) $(SC_MLITE_PATH) $(CRC_PATH) $(NI_PATH) $(ROUTER_PATH) $(DMA_PATH) $(SC_RAM_PATH) $(PLASMA_PATH) $(WRAPPER_CELL_PATH) $(TOP_WRAPPED_PATH)\n";
				SimMakefile = SimMakefile + "	@sccom -link\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "scmod: lib $(SC_NOC_PATH) $(SC_MLITE_PATH) $(SC_RAM_PATH) $(SC_ACCESS_REPO_PATH) $(SC_DMA_PATH) $(SC_NI_PATH) $(SC_PLASMA_PATH) $(SC_TESTBENCH_PATH)\n";
				SimMakefile = SimMakefile + "	@sccom -link\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "sc: $(SC_NOC_PATH) $(SC_MLITE_PATH) $(SC_RAM_PATH) $(SC_ACCESS_REPO_PATH) $(SC_DMA_PATH) $(SC_NI_PATH) $(SC_PLASMA_PATH) $(SC_TESTBENCH_PATH)\n";
				SimMakefile = SimMakefile + "	$(GEN)\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "vhd: lib $(SCENARIO_PATH) $(PKG_PATH) $(MPACK_PATH) $(MLITE_PATH) $(NI_PATH) $(DMA_PATH) $(ROUTER_PATH) $(PLASMA_RAM_PATH) $(MBLITE_STD_PATH) $(MBLITE_CORE_PATH) $(MBLITE_RAM_PATH) $(MBLITE_PATH) $(PLASMA_PATH) $(TOP_PATH)\n";
				SimMakefile = SimMakefile + "\t\n";
				SimMakefile = SimMakefile + "$(SCENARIO_PATH):\n";
				SimMakefile = SimMakefile + "	$(VHD_C) -work $(LIB) $(@).vhd\n";
				SimMakefile = SimMakefile + "\t\n";
				SimMakefile = SimMakefile + "$(PLASMA_RAM_PATH):\n";
				SimMakefile = SimMakefile + "	$(VHD_C) -work $(LIB) $(@).vhd\n";
				SimMakefile = SimMakefile + "\t\n";
				SimMakefile = SimMakefile + "$(MBLITE_RAM_PATH):\n";
				SimMakefile = SimMakefile + "	$(VHD_C) -work $(LIB) $(@).vhd\n";
				SimMakefile = SimMakefile + "\t\n";
				SimMakefile = SimMakefile + "$(SC_RAM_PATH):\n";
				SimMakefile = SimMakefile + "	$(SC_C) $(@).cpp  $(INC)\n";
				SimMakefile = SimMakefile + "\t\n";
				SimMakefile = SimMakefile + "$(SC_MBLITE_RAM_PATH):\n";
				SimMakefile = SimMakefile + "\t$(SC_C) $(@).cpp  $(INC)\n";
				SimMakefile = SimMakefile + "\n";
				SimMakefile = SimMakefile + "%.vhd:\n";
				SimMakefile = SimMakefile + "	$(VHD_C) -work $(LIB) $(BASE_PATH)/$*.vhd\n";
				SimMakefile = SimMakefile + "	\n";
				SimMakefile = SimMakefile + "%.cpp:\n";
				SimMakefile = SimMakefile + "	$(SC_C) $(BASE_PATH)/$*.cpp  $(INC)\n";
				SimMakefile = SimMakefile + "	\n";
				SimMakefile = SimMakefile + "sim:\n";
				SimMakefile = SimMakefile + "	do sim.do\n";
				SimMakefile = SimMakefile + "	\n";
				SimMakefile = SimMakefile + "lib:\n";
				SimMakefile = SimMakefile + "	@vlib $(LIB)\n";
				SimMakefile = SimMakefile + "	@vmap $(LIB) $(LIB)\n";
				SimMakefile = SimMakefile + "	\n";
				SimMakefile = SimMakefile + "clean:\n";
				SimMakefile = SimMakefile + "	@rm -r -f $(LIB)\n";
				SimMakefile = SimMakefile + "	@rm -f transcript\n";
				SimMakefile = SimMakefile + "	@rm -f modelsim.ini\n";
				SimMakefile = SimMakefile + "	@rm -f vsim.wlf\n";
				SimMakefile = SimMakefile + "	@rm -f *~\n";
				SimMakefile = SimMakefile + "	@rm -f *.o\n";
				SimMakefile = SimMakefile + "	@rm -f *.exe\n";
				SimMakefile = SimMakefile + "	\n";
				
				if (procDescription == "rtl")
				{
					SimMakefile = SimMakefile + "all: vhd\n";
				}
				else if (procDescription == "sc")
				{
					SimMakefile = SimMakefile + "all: sc\n";
				}
				else if (procDescription == "iss")
				{
					SimMakefile = SimMakefile + "all: iss\n";
				}
				else if (procDescription == "iss_wrapped")
				{
					SimMakefile = SimMakefile + "all: iss_wrapped\n";
				}				
				else if (procDescription == "scmod")
				{
					SimMakefile = SimMakefile + "all: scmod	\n";
				}
				SimMakefile = SimMakefile + "	\n";

			   if (argc == 3) file_path = "./" + projectPath + projectName + "/Makefile";
			   else file_path = "./" + projectName + "/Makefile";
			   ofstream smakefile(file_path.c_str());
			   smakefile<<SimMakefile;
			   smakefile.close();
		   
				string sim_do = "";
		
				sim_do = sim_do + "vsim -novopt -t ps +notimingchecks work.test_bench\n"; 
				sim_do = sim_do + "\n";
				sim_do = sim_do + "do wave.do\n";
				sim_do = sim_do + "onerror {resume}\n";
				sim_do = sim_do + "radix hex\n";
				sim_do = sim_do + "set NumericStdNoWarnings 1\n";
				sim_do = sim_do + "set StdArithNoWarnings 1\n";
				sim_do = sim_do + "\n";
				sim_do = sim_do + "when -label end_of_simulation { HeMPS/proc("+ftoa(y_master*x_dimension+x_master)+")/mas/master/PE/PE_PLASMA/plasma/end_sim_reg == x\"00000000\" } {echo \"End of simulation\" ; echo \"simulation ended at time: ${now}\"; quit ;}\n";
				sim_do = sim_do + "do fault-inject.do\n"; // run fault injection if the signals file exists
				sim_do = sim_do + "run "+ simtime + "\n"; //set time!!!!!!!!!!!!!!
				sim_do = sim_do + "echo \"simulation ended at time: ${now}\"\n"; // print the current simulation time
				sim_do = sim_do + "quit\n";

				if (argc == 3) file_path = "./" + projectPath + projectName + "/sim.do";
				else file_path = "./" + projectName + "/sim.do";
				ofstream sim_do_file(file_path.c_str());
				sim_do_file<<sim_do;
				sim_do_file.close();

				// cp fault injection script to the project path
				if (argc == 3) file_path = "./" + projectPath + projectName;
				else file_path = "./" + projectName;
				system(string("cp -r $HEMPS_PATH/scripts/fault/fault-inject.do " + file_path).c_str());

				string wave_do = "";
				
				int x_slave = 0; 
				int y_slave = 0;
				
				wave_do = wave_do + "onerror {resume}\n";
				wave_do = wave_do + "onerror {resume}\n";
				wave_do = wave_do + "quietly WaveActivateNextPane {} 0\n";
				wave_do = wave_do + "add wave -noupdate -divider hemps\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/credit_in(8)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/tx(8)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/data_out(8)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/eop_out(8)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/credit_out(8)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/rx(8)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/data_in(8)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/eop_in(8)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/credit_in(9)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/tx(9)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/data_out(9)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/eop_out(9)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/credit_out(9)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/rx(9)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/data_in(9)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/eop_in(9)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/in_seek(4)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/in_source(4)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/PE/Router/in_target(4)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/faulty_port(0)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/faulty_port(1)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/faulty_port(2)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/faulty_port(3)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/faulty_port(4)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/faulty_port(5)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/faulty_port(6)\n";
				wave_do = wave_do + "add wave -noupdate -group {Master "+ ftoa(x_master) +""+ ftoa(y_master) +" - " + ftoa(masterAddress) + "} -radix hexadecimal /test_bench/HeMPS/proc("+ ftoa(masterAddress) +")/mas/master/faulty_port(7)\n";
				wave_do = wave_do + "add wave -noupdate -group repository -radix hexadecimal /test_bench/HeMPS/read_req\n";
				wave_do = wave_do + "add wave -noupdate -group repository -radix hexadecimal /test_bench/HeMPS/mem_addr\n";
				wave_do = wave_do + "add wave -noupdate -group repository -radix hexadecimal /test_bench/HeMPS/data_read\n";
				wave_do = wave_do + "add wave -noupdate -group repository -radix hexadecimal /test_bench/HeMPS/data_valid\n";
				wave_do = wave_do + "add wave -noupdate -group {debug messages} -radix hexadecimal /test_bench/HeMPS/write_enable_debug\n";
				wave_do = wave_do + "add wave -noupdate -group {debug messages} -radix hexadecimal /test_bench/HeMPS/data_out_debug\n";
				wave_do = wave_do + "add wave -noupdate -group {debug messages} -radix ascii /test_bench/HeMPS/data_out_debug\n";

				for (int i = 0; i < (x_dimension*y_dimension); i++)
				{
					if (x_slave > x_dimension-1) 
					{
						x_slave = 0;
						y_slave++;
					}
					
					if (i != masterAddress)
					{						
										
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/address\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/credit_in(8)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/tx(8)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/data_out(8)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/eop_out(8)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/credit_out(8)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/rx(8)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/data_in(8)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/eop_in(8)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/credit_in(9)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/tx(9)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/data_out(9)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/eop_out(9)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/credit_out(9)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/rx(9)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/data_in(9)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/eop_in(9)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/in_seek\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/out_ack_seek\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/in_clear\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/out_ack_clear\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/in_source\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/in_target\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/in_hop_counter\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/out_seek\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/in_ack_seek\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/out_clear\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/in_ack_clear\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/out_source\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/out_target\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/out_hop_counter\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/faulty_port(0)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/faulty_port(1)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/faulty_port(2)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/faulty_port(3)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/faulty_port(4)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/faulty_port(5)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/faulty_port(6)\n";
									wave_do = wave_do + "add wave -noupdate -group {slave "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} -radix hexadecimal /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/faulty_port(7)\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/DEBUG\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/EA_query\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/EA_search\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/PE_query\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/PE_search\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/ack_backtrack\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/addr0\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/addr1\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/addr_read\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/addr_write\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/broadcast_clear\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/clear_pulse\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/clock\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/counter\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/credit_in\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/data_out\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/eop_out\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/flit_num\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/flit_pos\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/free_port\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/free_slot\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/in_ack_clear\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/in_ack_seek\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/in_clear\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/in_hop_counter\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/in_query_table\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/in_search_hop_count\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/in_search_port\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/in_search_source\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/in_search_target\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/in_seek\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/in_source\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/in_table\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/in_target\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/out_ack_clear\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/out_ack_seek\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/out_clear\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/out_hop_counter\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/out_query_table\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/out_seek\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/out_source\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/out_table0\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/out_table1\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/out_target\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/packet_size\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/port_ctrl\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/port_dir\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/read_hop_count\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/read_port_dir\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/read_source\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/read_target\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/read_used0\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/read_used1\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/req_backtrack\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/reset\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/reset_clear\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/reset_seek\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/rx_out\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/search_hop_count\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/search_port\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/search_source\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/search_target\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/seek_address\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/seek_pulse\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/source\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/source_reg\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/treating_clear\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/we\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/we_query_table\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/write_hop_count\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/write_port_dir\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/write_source\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/write_target\n";
									wave_do = wave_do + "add wave -noupdate -group {seek "+ ftoa(x_slave) +""+ ftoa(y_slave) +" - " + ftoa(i) + "} /test_bench/HeMPS/proc(" + ftoa(i) + ")/slav/slave/PE/Router/seek/write_used\n";
									                                                                                                                                      
									x_slave++;                                                                                                                            
					}                                                                                                                                                     
				                                                                                                                                                          
				    else x_slave++;                                                                                                                                       
				}                                                                                                                                                         

				wave_do = wave_do + "TreeUpdate [SetDefaultTree]\n";
				wave_do = wave_do + "WaveRestoreCursors {{Cursor 1} {1627970253 ps} 0}\n";
				wave_do = wave_do + "configure wave -namecolwidth 190\n";
				wave_do = wave_do + "configure wave -valuecolwidth 100\n";
				wave_do = wave_do + "configure wave -justifyvalue left\n";
				wave_do = wave_do + "configure wave -signalnamewidth 1\n";
				wave_do = wave_do + "configure wave -snapdistance 10\n";
				wave_do = wave_do + "configure wave -datasetprefix 0\n";
				wave_do = wave_do + "configure wave -rowmargin 4\n";
				wave_do = wave_do + "configure wave -childrowmargin 2\n";
				wave_do = wave_do + "configure wave -gridoffset 0\n";
				wave_do = wave_do + "configure wave -gridperiod 1\n";
				wave_do = wave_do + "configure wave -griddelta 40\n";
				wave_do = wave_do + "configure wave -timeline 0\n";
				wave_do = wave_do + "configure wave -timelineunits ps\n";
				wave_do = wave_do + "update\n";
				wave_do = wave_do + "WaveRestoreZoom {0 ps} {3198211064 ps}\n";

				if (argc == 3) file_path = "./" + projectPath + projectName + "/wave.do";
				else file_path = "./" + projectName + "/wave.do";
				ofstream wave_do_file(file_path.c_str());
				wave_do_file<<wave_do;
				wave_do_file.close();
				
				string Compile = ""; 
				Compile = Compile + "#!/bin/bash\n";
				Compile = Compile + "set -e\n";
				Compile = Compile + "\n";
				Compile = Compile + "make clean\n";
				Compile = Compile + "cd build/\n";
				Compile = Compile + "make clean\n";
				Compile = Compile + "make all\n";
				Compile = Compile + "cd ..\n";
				Compile = Compile + "make all\n";
				Compile = Compile + "vsim -do sim.do\n";
				
				if (argc == 3) file_path = "./" + projectPath + projectName + "/compile.sh";
				else file_path = "./" + projectName + "/compile.sh";
				ofstream Compile_file(file_path.c_str());
				Compile_file<<Compile;
				Compile_file.close();
				
				string path_compile = "/bin/chmod +x " + file_path;
				system(path_compile.c_str());
				
				string ram_plasma_cpp = "", ram_mblite_cpp = "", ram_master_cpp = "";
				
				ram_plasma_cpp = ram_plasma_cpp + "#include \"ram_plasma.h\"\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "#ifdef MTI_SYSTEMC\n";
				ram_plasma_cpp = ram_plasma_cpp + "SC_MODULE_EXPORT(ram_plasma);\n";
				ram_plasma_cpp = ram_plasma_cpp + "#endif\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "/*** Memory read port A ***/\n";
				ram_plasma_cpp = ram_plasma_cpp + "void ram_plasma::read_a() {\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "	unsigned int address;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "	address = (unsigned int)address_a.read();\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "	if ( address < RAM_SIZE )\n";
				ram_plasma_cpp = ram_plasma_cpp + "		data_read_a.write(ram[address]);\n";
				ram_plasma_cpp = ram_plasma_cpp + "}\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "/*** Memory write port A ***/\n";
				ram_plasma_cpp = ram_plasma_cpp + "void ram_plasma::write_a() {\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "	unsigned int data, address;\n";
				ram_plasma_cpp = ram_plasma_cpp + "	unsigned char wbe;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "	wbe = (unsigned char)wbe_a.read();\n";
				ram_plasma_cpp = ram_plasma_cpp + "	address = (unsigned int)address_a.read();\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "	if ( wbe != 0 && address < RAM_SIZE) {\n";
				ram_plasma_cpp = ram_plasma_cpp + "		data = ram[address];\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "		switch(wbe) {\n";
				ram_plasma_cpp = ram_plasma_cpp + "			case 0xF:	// Write word\n";
				ram_plasma_cpp = ram_plasma_cpp + "				ram[address] = data_write_a.read();\n";
				ram_plasma_cpp = ram_plasma_cpp + "			break;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "			case 0xC:	// Write MSW\n";
				ram_plasma_cpp = ram_plasma_cpp + "				ram[address] = (data & ~half_word[1]) | (data_write_a.read() & half_word[1]);\n";
				ram_plasma_cpp = ram_plasma_cpp + "			break;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "			case 3:		// Write LSW\n";
				ram_plasma_cpp = ram_plasma_cpp + "				ram[address] = (data & ~half_word[0]) | (data_write_a.read() & half_word[0]);\n";
				ram_plasma_cpp = ram_plasma_cpp + "			break;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "			case 8:		// Write byte 3\n";
				ram_plasma_cpp = ram_plasma_cpp + "				ram[address] = (data & ~byte[3]) | (data_write_a.read() & byte[3]);\n";
				ram_plasma_cpp = ram_plasma_cpp + "			break;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "			case 4:		// Write byte 2\n";
				ram_plasma_cpp = ram_plasma_cpp + "				ram[address] = (data & ~byte[2]) | (data_write_a.read() & byte[2]);\n";
				ram_plasma_cpp = ram_plasma_cpp + "			break;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "			case 2:		// Write byte 1\n";
				ram_plasma_cpp = ram_plasma_cpp + "				ram[address] = (data & ~byte[1]) | (data_write_a.read() & byte[1]);\n";
				ram_plasma_cpp = ram_plasma_cpp + "			break;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "			case 1:		// Write byte 0\n";
				ram_plasma_cpp = ram_plasma_cpp + "				ram[address] = (data & ~byte[0]) | (data_write_a.read() & byte[0]);\n";
				ram_plasma_cpp = ram_plasma_cpp + "			break;\n";
				ram_plasma_cpp = ram_plasma_cpp + "		}\n";
				ram_plasma_cpp = ram_plasma_cpp + "	}\n";
				ram_plasma_cpp = ram_plasma_cpp + "}\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "/*** Memory read port B ***/\n";
				ram_plasma_cpp = ram_plasma_cpp + "void ram_plasma::read_b() {\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "	unsigned int address;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "	address = (unsigned int)address_b.read();\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "	if ( address < RAM_SIZE )\n";
				ram_plasma_cpp = ram_plasma_cpp + "		data_read_b.write(ram[address]);\n";
				ram_plasma_cpp = ram_plasma_cpp + "}\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "/*** Memory write port B ***/\n";
				ram_plasma_cpp = ram_plasma_cpp + "void ram_plasma::write_b() {\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "	unsigned int data, address;\n";
				ram_plasma_cpp = ram_plasma_cpp + "	unsigned char wbe;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "	wbe = (unsigned char)wbe_b.read();\n";
				ram_plasma_cpp = ram_plasma_cpp + "	address = (unsigned int)address_b.read();\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "	if ( wbe != 0 && address < RAM_SIZE) {\n";
				ram_plasma_cpp = ram_plasma_cpp + "		data = ram[address];\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "		switch(wbe) {\n";
				ram_plasma_cpp = ram_plasma_cpp + "			case 0xF:	// Write word\n";
				ram_plasma_cpp = ram_plasma_cpp + "				ram[address] = data_write_b.read();\n";
				ram_plasma_cpp = ram_plasma_cpp + "			break;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "			case 0xC:	// Write MSW\n";
				ram_plasma_cpp = ram_plasma_cpp + "				ram[address] = (data & ~half_word[1]) | (data_write_b.read() & half_word[1]);\n";
				ram_plasma_cpp = ram_plasma_cpp + "			break;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "			case 3:		// Write LSW\n";
				ram_plasma_cpp = ram_plasma_cpp + "				ram[address] = (data & ~half_word[0]) | (data_write_b.read() & half_word[0]);\n";
				ram_plasma_cpp = ram_plasma_cpp + "			break;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "			case 8:		// Write byte 3\n";
				ram_plasma_cpp = ram_plasma_cpp + "				ram[address] = (data & ~byte[3]) | (data_write_b.read() & byte[3]);\n";
				ram_plasma_cpp = ram_plasma_cpp + "			break;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "			case 4:		// Write byte 2\n";
				ram_plasma_cpp = ram_plasma_cpp + "				ram[address] = (data & ~byte[2]) | (data_write_b.read() & byte[2]);\n";
				ram_plasma_cpp = ram_plasma_cpp + "			break;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "			case 2:		// Write byte 1\n";
				ram_plasma_cpp = ram_plasma_cpp + "				ram[address] = (data & ~byte[1]) | (data_write_b.read() & byte[1]);\n";
				ram_plasma_cpp = ram_plasma_cpp + "			break;\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";
				ram_plasma_cpp = ram_plasma_cpp + "			case 1:		// Write byte 0\n";
				ram_plasma_cpp = ram_plasma_cpp + "				ram[address] = (data & ~byte[0]) | (data_write_b.read() & byte[0]);\n";
				ram_plasma_cpp = ram_plasma_cpp + "			break;\n";
				ram_plasma_cpp = ram_plasma_cpp + "		}\n";
				ram_plasma_cpp = ram_plasma_cpp + "	}\n";
				ram_plasma_cpp = ram_plasma_cpp + "}\n";
				ram_plasma_cpp = ram_plasma_cpp + "\n";

				if (argc == 3) file_path = "./" + projectPath + projectName + "/plasma_ram/sc/ram_plasma.cpp";
				else file_path = "./" + projectName + "/plasma_ram/sc/ram_plasma.cpp";
				ofstream ram_plasma_cpp_file(file_path.c_str());
				ram_plasma_cpp_file<<ram_plasma_cpp;
				ram_plasma_cpp_file.close();
		  
				ram_mblite_cpp = ram_mblite_cpp + "#include \"ram_mblite.h\"\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "#ifdef MTI_SYSTEMC\n";
				ram_mblite_cpp = ram_mblite_cpp + "SC_MODULE_EXPORT(ram_mblite);\n";
				ram_mblite_cpp = ram_mblite_cpp + "#endif\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "/*** Memory read port A ***/\n";
				ram_mblite_cpp = ram_mblite_cpp + "void ram_mblite::read_a() {\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "	unsigned int address;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "	address = (unsigned int)address_a.read();\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "	if ( address < RAM_SIZE )\n";
				ram_mblite_cpp = ram_mblite_cpp + "		data_read_a.write(ram[address]);\n";
				ram_mblite_cpp = ram_mblite_cpp + "}\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "/*** Memory write port A ***/\n";
				ram_mblite_cpp = ram_mblite_cpp + "void ram_mblite::write_a() {\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "	unsigned int data, address;\n";
				ram_mblite_cpp = ram_mblite_cpp + "	unsigned char wbe;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "	wbe = (unsigned char)wbe_a.read();\n";
				ram_mblite_cpp = ram_mblite_cpp + "	address = (unsigned int)address_a.read();\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "	if ( wbe != 0 && address < RAM_SIZE) {\n";
				ram_mblite_cpp = ram_mblite_cpp + "		data = ram[address];\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "		switch(wbe) {\n";
				ram_mblite_cpp = ram_mblite_cpp + "			case 0xF:	// Write word\n";
				ram_mblite_cpp = ram_mblite_cpp + "				ram[address] = data_write_a.read();\n";
				ram_mblite_cpp = ram_mblite_cpp + "			break;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "			case 0xC:	// Write MSW\n";
				ram_mblite_cpp = ram_mblite_cpp + "				ram[address] = (data & ~half_word[1]) | (data_write_a.read() & half_word[1]);\n";
				ram_mblite_cpp = ram_mblite_cpp + "			break;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "			case 3:		// Write LSW\n";
				ram_mblite_cpp = ram_mblite_cpp + "				ram[address] = (data & ~half_word[0]) | (data_write_a.read() & half_word[0]);\n";
				ram_mblite_cpp = ram_mblite_cpp + "			break;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "			case 8:		// Write byte 3\n";
				ram_mblite_cpp = ram_mblite_cpp + "				ram[address] = (data & ~byte[3]) | (data_write_a.read() & byte[3]);\n";
				ram_mblite_cpp = ram_mblite_cpp + "			break;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "			case 4:		// Write byte 2\n";
				ram_mblite_cpp = ram_mblite_cpp + "				ram[address] = (data & ~byte[2]) | (data_write_a.read() & byte[2]);\n";
				ram_mblite_cpp = ram_mblite_cpp + "			break;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "			case 2:		// Write byte 1\n";
				ram_mblite_cpp = ram_mblite_cpp + "				ram[address] = (data & ~byte[1]) | (data_write_a.read() & byte[1]);\n";
				ram_mblite_cpp = ram_mblite_cpp + "			break;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "			case 1:		// Write byte 0\n";
				ram_mblite_cpp = ram_mblite_cpp + "				ram[address] = (data & ~byte[0]) | (data_write_a.read() & byte[0]);\n";
				ram_mblite_cpp = ram_mblite_cpp + "			break;\n";
				ram_mblite_cpp = ram_mblite_cpp + "		}\n";
				ram_mblite_cpp = ram_mblite_cpp + "	}\n";
				ram_mblite_cpp = ram_mblite_cpp + "}\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "/*** Memory read port B ***/\n";
				ram_mblite_cpp = ram_mblite_cpp + "void ram_mblite::read_b() {\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "	unsigned int address;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "	address = (unsigned int)address_b.read();\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "	if ( address < RAM_SIZE )\n";
				ram_mblite_cpp = ram_mblite_cpp + "		data_read_b.write(ram[address]);\n";
				ram_mblite_cpp = ram_mblite_cpp + "}\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "/*** Memory write port B ***/\n";
				ram_mblite_cpp = ram_mblite_cpp + "void ram_mblite::write_b() {\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "	unsigned int data, address;\n";
				ram_mblite_cpp = ram_mblite_cpp + "	unsigned char wbe;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "	wbe = (unsigned char)wbe_b.read();\n";
				ram_mblite_cpp = ram_mblite_cpp + "	address = (unsigned int)address_b.read();\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "	if ( wbe != 0 && address < RAM_SIZE) {\n";
				ram_mblite_cpp = ram_mblite_cpp + "		data = ram[address];\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "		switch(wbe) {\n";
				ram_mblite_cpp = ram_mblite_cpp + "			case 0xF:	// Write word\n";
				ram_mblite_cpp = ram_mblite_cpp + "				ram[address] = data_write_b.read();\n";
				ram_mblite_cpp = ram_mblite_cpp + "			break;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "			case 0xC:	// Write MSW\n";
				ram_mblite_cpp = ram_mblite_cpp + "				ram[address] = (data & ~half_word[1]) | (data_write_b.read() & half_word[1]);\n";
				ram_mblite_cpp = ram_mblite_cpp + "			break;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "			case 3:		// Write LSW\n";
				ram_mblite_cpp = ram_mblite_cpp + "				ram[address] = (data & ~half_word[0]) | (data_write_b.read() & half_word[0]);\n";
				ram_mblite_cpp = ram_mblite_cpp + "			break;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "			case 8:		// Write byte 3\n";
				ram_mblite_cpp = ram_mblite_cpp + "				ram[address] = (data & ~byte[3]) | (data_write_b.read() & byte[3]);\n";
				ram_mblite_cpp = ram_mblite_cpp + "			break;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "			case 4:		// Write byte 2\n";
				ram_mblite_cpp = ram_mblite_cpp + "				ram[address] = (data & ~byte[2]) | (data_write_b.read() & byte[2]);\n";
				ram_mblite_cpp = ram_mblite_cpp + "			break;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "			case 2:		// Write byte 1\n";
				ram_mblite_cpp = ram_mblite_cpp + "				ram[address] = (data & ~byte[1]) | (data_write_b.read() & byte[1]);\n";
				ram_mblite_cpp = ram_mblite_cpp + "			break;\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";
				ram_mblite_cpp = ram_mblite_cpp + "			case 1:		// Write byte 0\n";
				ram_mblite_cpp = ram_mblite_cpp + "				ram[address] = (data & ~byte[0]) | (data_write_b.read() & byte[0]);\n";
				ram_mblite_cpp = ram_mblite_cpp + "			break;\n";
				ram_mblite_cpp = ram_mblite_cpp + "		}\n";
				ram_mblite_cpp = ram_mblite_cpp + "	}\n";
				ram_mblite_cpp = ram_mblite_cpp + "}\n";
				ram_mblite_cpp = ram_mblite_cpp + "\n";

				if (argc == 3) file_path = "./" + projectPath + projectName + "/mblite_ram/sc/ram_mblite.cpp";
				else file_path = "./" + projectName + "/mblite_ram/sc/ram_mblite.cpp";
				ofstream ram_mblite_cpp_file(file_path.c_str());
				ram_mblite_cpp_file<<ram_mblite_cpp;
				ram_mblite_cpp_file.close();
		   
				ram_master_cpp = ram_master_cpp + "#include \"ram_master.h\"\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "#ifdef MTI_SYSTEMC\n";
				ram_master_cpp = ram_master_cpp + "SC_MODULE_EXPORT(ram_master);\n";
				ram_master_cpp = ram_master_cpp + "#endif\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "/*** Memory read port A ***/\n";
				ram_master_cpp = ram_master_cpp + "void ram_master::read_a() {\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "	unsigned int address;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "	address = (unsigned int)address_a.read();\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "	if ( address < RAM_SIZE )\n";
				ram_master_cpp = ram_master_cpp + "		data_read_a.write(ram[address]);\n";
				ram_master_cpp = ram_master_cpp + "}\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "/*** Memory write port A ***/\n";
				ram_master_cpp = ram_master_cpp + "void ram_master::write_a() {\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "	unsigned int data, address;\n";
				ram_master_cpp = ram_master_cpp + "	unsigned char wbe;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "	wbe = (unsigned char)wbe_a.read();\n";
				ram_master_cpp = ram_master_cpp + "	address = (unsigned int)address_a.read();\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "	if ( wbe != 0 && address < RAM_SIZE) {\n";
				ram_master_cpp = ram_master_cpp + "		data = ram[address];\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "		switch(wbe) {\n";
				ram_master_cpp = ram_master_cpp + "			case 0xF:	// Write word\n";
				ram_master_cpp = ram_master_cpp + "				ram[address] = data_write_a.read();\n";
				ram_master_cpp = ram_master_cpp + "			break;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "			case 0xC:	// Write MSW\n";
				ram_master_cpp = ram_master_cpp + "				ram[address] = (data & ~half_word[1]) | (data_write_a.read() & half_word[1]);\n";
				ram_master_cpp = ram_master_cpp + "			break;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "			case 3:		// Write LSW\n";
				ram_master_cpp = ram_master_cpp + "				ram[address] = (data & ~half_word[0]) | (data_write_a.read() & half_word[0]);\n";
				ram_master_cpp = ram_master_cpp + "			break;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "			case 8:		// Write byte 3\n";
				ram_master_cpp = ram_master_cpp + "				ram[address] = (data & ~byte[3]) | (data_write_a.read() & byte[3]);\n";
				ram_master_cpp = ram_master_cpp + "			break;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "			case 4:		// Write byte 2\n";
				ram_master_cpp = ram_master_cpp + "				ram[address] = (data & ~byte[2]) | (data_write_a.read() & byte[2]);\n";
				ram_master_cpp = ram_master_cpp + "			break;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "			case 2:		// Write byte 1\n";
				ram_master_cpp = ram_master_cpp + "				ram[address] = (data & ~byte[1]) | (data_write_a.read() & byte[1]);\n";
				ram_master_cpp = ram_master_cpp + "			break;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "			case 1:		// Write byte 0\n";
				ram_master_cpp = ram_master_cpp + "				ram[address] = (data & ~byte[0]) | (data_write_a.read() & byte[0]);\n";
				ram_master_cpp = ram_master_cpp + "			break;\n";
				ram_master_cpp = ram_master_cpp + "		}\n";
				ram_master_cpp = ram_master_cpp + "	}\n";
				ram_master_cpp = ram_master_cpp + "}\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "/*** Memory read port B ***/\n";
				ram_master_cpp = ram_master_cpp + "void ram_master::read_b() {\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "	unsigned int address;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "	address = (unsigned int)address_b.read();\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "	if ( address < RAM_SIZE )\n";
				ram_master_cpp = ram_master_cpp + "		data_read_b.write(ram[address]);\n";
				ram_master_cpp = ram_master_cpp + "}\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "/*** Memory write port B ***/\n";
				ram_master_cpp = ram_master_cpp + "void ram_master::write_b() {\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "	unsigned int data, address;\n";
				ram_master_cpp = ram_master_cpp + "	unsigned char wbe;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "	wbe = (unsigned char)wbe_b.read();\n";
				ram_master_cpp = ram_master_cpp + "	address = (unsigned int)address_b.read();\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "	if ( wbe != 0 && address < RAM_SIZE) {\n";
				ram_master_cpp = ram_master_cpp + "		data = ram[address];\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "		switch(wbe) {\n";
				ram_master_cpp = ram_master_cpp + "			case 0xF:	// Write word\n";
				ram_master_cpp = ram_master_cpp + "				ram[address] = data_write_b.read();\n";
				ram_master_cpp = ram_master_cpp + "			break;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "			case 0xC:	// Write MSW\n";
				ram_master_cpp = ram_master_cpp + "				ram[address] = (data & ~half_word[1]) | (data_write_b.read() & half_word[1]);\n";
				ram_master_cpp = ram_master_cpp + "			break;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "			case 3:		// Write LSW\n";
				ram_master_cpp = ram_master_cpp + "				ram[address] = (data & ~half_word[0]) | (data_write_b.read() & half_word[0]);\n";
				ram_master_cpp = ram_master_cpp + "			break;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "			case 8:		// Write byte 3\n";
				ram_master_cpp = ram_master_cpp + "				ram[address] = (data & ~byte[3]) | (data_write_b.read() & byte[3]);\n";
				ram_master_cpp = ram_master_cpp + "			break;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "			case 4:		// Write byte 2\n";
				ram_master_cpp = ram_master_cpp + "				ram[address] = (data & ~byte[2]) | (data_write_b.read() & byte[2]);\n";
				ram_master_cpp = ram_master_cpp + "			break;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "			case 2:		// Write byte 1\n";
				ram_master_cpp = ram_master_cpp + "				ram[address] = (data & ~byte[1]) | (data_write_b.read() & byte[1]);\n";
				ram_master_cpp = ram_master_cpp + "			break;\n";
				ram_master_cpp = ram_master_cpp + "\n";
				ram_master_cpp = ram_master_cpp + "			case 1:		// Write byte 0\n";
				ram_master_cpp = ram_master_cpp + "				ram[address] = (data & ~byte[0]) | (data_write_b.read() & byte[0]);\n";
				ram_master_cpp = ram_master_cpp + "			break;\n";
				ram_master_cpp = ram_master_cpp + "		}\n";
				ram_master_cpp = ram_master_cpp + "	}\n";
				ram_master_cpp = ram_master_cpp + "}\n";
				ram_master_cpp = ram_master_cpp + "\n";

				if (argc == 3) file_path = "./" + projectPath + projectName + "/plasma_ram/sc/ram_master.cpp";
				else file_path = "./" + projectName + "/plasma_ram/sc/ram_master.cpp";
				ofstream ram_master_cpp_file(file_path.c_str());
				ram_master_cpp_file<<ram_master_cpp;
				ram_master_cpp_file.close();
            
            
				 /* Generates the application IDs files */
				 for(ia=applications.begin();ia!=applications.end();ia++)
				 {
					string ids_h = "";
					
					for(int i=0;i<taskIDs.size();i++)
					{
						string tname = taskIDs[i].get_name();
						pos = tname.find(".",0);
						tname = tname.substr(0,pos);
						
						if (taskIDs[i].get_application() == ia->get_name()) ids_h = ids_h + "#define " + tname + "\t" + ftoa(i) + "\n";

					}
					string ids_h_name = "ids_" + ia->get_name() + ".h";
					
					if (argc == 3) file_path = "./" + projectPath + projectName + "/build/" + ids_h_name;
					else file_path = "./" + projectName + "/build/" + ids_h_name;
					ofstream ids_h_file(file_path.c_str());
					ids_h_file<<ids_h;
					ids_h_file.close();
					
				}
				
				 /* Generates the ids_master.h file */
				string defines;
				int slaveProcessors = (x_dimension * y_dimension) - 1;

				defines = defines + "#define MAX_PROCESSORS\t\t" + ftoa(slaveProcessors) + "\t/* Number of slave processors available in the platform */\n";
				defines = defines + "#define MAX_LOCAL_TASKS\t\t" + ftoa(maxTasksSlave) + "\t/* Number of task which can be allocated simultaneously in one processor */\n";
				defines = defines + "#define MAX_GLOBAL_TASKS\t" + "MAX_LOCAL_TASKS * MAX_PROCESSORS\t" + "/* Number of task which can be allocated simultaneously in the platform */\n";
				defines = defines + "#define PLASMA\t0\n";
				defines = defines + "#define MBLITE\t1\n";
				defines = defines + "#define PLASMA_PROCESSORS\t\t" + ftoa(plasmaNumber-1) + "\n";
				defines = defines + "#define MBLITE_PROCESSORS\t\t" + ftoa(mbliteNumber) + "\n";
				defines = defines + "#define XDIMENSION\t\t" + ftoa(x_dimension) + "\n";
				defines = defines + "#define YDIMENSION\t\t" + ftoa(y_dimension) + "\n";
				defines = defines + "#define MASTERADDRESS\t\t" + "0x" + ftoa(masterAddress_h) + "\n";
				defines = defines + "#define TASK_NUMBER\t\t" + ftoa(taskIDs.size()) + "\n";
				
				defines = defines + "\n";
				
				defines = defines + "char pe_free_pages[XDIMENSION][YDIMENSION] = {";
				for(int xd=1; xd<x_dimension; xd++) 
				{
					defines = defines + "{";
					for(int yd=1; yd<y_dimension; yd++) 
					{	
						defines = defines + "MAX_LOCAL_TASKS, ";
					}
					defines = defines + "MAX_LOCAL_TASKS}, ";
				}
				defines = defines + "{";
				for(int yd=1; yd<y_dimension; yd++) 
				{	
					defines = defines + "MAX_LOCAL_TASKS, ";
				}
				defines = defines + "MAX_LOCAL_TASKS}};\n";
			
				defines = defines + "\n";
			
				defines = defines + "char pe_type[XDIMENSION][YDIMENSION] = {";
				for(int xd=0; xd<x_dimension; xd++) 
				{
					defines = defines + "{";
					for(int yd=0; yd<y_dimension; yd++) 
					{	
						if (processors[xd][yd]->get_type() == 0)
						{
								defines = defines + "PLASMA";
						}
						else
						{
								defines = defines + "MBLITE";	
						}		
						if(yd==(y_dimension-1)) defines = defines + "}";
						else defines = defines + ", ";
					}
					if(xd==(x_dimension-1)) defines = defines + "};";
					else defines = defines + ", ";
				}
				defines = defines + "\n";
				defines = defines + "\n";
				defines = defines + "int total_free_pes = MAX_GLOBAL_TASKS;\n";
				defines = defines + "int free_plasmas = PLASMA_PROCESSORS * MAX_LOCAL_TASKS;\n";
				defines = defines + "int free_mblites = MBLITE_PROCESSORS * MAX_LOCAL_TASKS;\n";
				defines = defines + "\n";
				
				if (argc == 3) file_path = "./" + projectPath + projectName + "/build/ids_master.h";
				else file_path = "./" + projectName + "/build/ids_master.h";
				ofstream ids_master_file(file_path.c_str());
				
				ids_master_file<<defines;
				

				ids_master_file<<"void InsertTaskLoc(int id, int processor);\n";
				ids_master_file<<"void PageUsed(int proc);\n\n";

				ids_master_file.close();
		
		
				/* Generates the ids_slave.h file */
				if (argc == 3) file_path = "./" + projectPath + projectName + "/build/ids_slave.h";
				else file_path = "./" + projectName + "/build/ids_slave.h";
				ofstream ids_slave_file(file_path.c_str());

                ids_slave_file<<"#define MAX_PROCESSORS\t\t"<<slaveProcessors<<"\t/* Number of slave processors available in the platform */"<<endl;
                ids_slave_file<<"#define MAXLOCALTASKS\t\t"<<maxTasksSlave<<"\t/* Number of task which can be allocated simultaneously in one processor */"<<endl;
                ids_slave_file<<"#define MAX_GLOBAL_TASKS\tMAXLOCALTASKS * MAX_PROCESSORS\t"<<"/* Number of task which can be allocated simultaneously in the platform */"<<endl;
                ids_slave_file<<"#define KERNELPAGECOUNT\t"<<kernelPages<<endl;
                ids_slave_file<<"#define PAGESIZE\t\t\t"<<pageSize * 1024<<endl;
                ids_slave_file<<"#define MASTERADDRESS\t\t0x"<<masterAddress_h<<endl;
                ids_slave_file<<"#define XDIMENSION\t\t"<<x_dimension<<endl;
                ids_slave_file<<"#define YDIMENSION\t\t"<<y_dimension<<endl;
                ids_slave_file<<"#define K\t\t"<<constant_k<<endl;

                ids_slave_file.close();
                
                /* Generates the InitializeVectorsSlave.h file */
                if (argc == 3) file_path = "./" + projectPath + projectName + "/build/InitializeVectorsSlave.h";
                else file_path = "./" + projectName + "/build/InitializeVectorsSlave.h";
				ofstream InitializeVectorsSlave_file(file_path.c_str());
            
				InitializeVectorsSlave_file<<"/*--------------------------------------------------------------------\n";
				InitializeVectorsSlave_file<<" * struct TaskLocation\n";
				InitializeVectorsSlave_file<<" *\n";
				InitializeVectorsSlave_file<<" * DESCRIPTION:\n";
				InitializeVectorsSlave_file<<" *    Makes the task versus processor association\n";
				InitializeVectorsSlave_file<<" *\n";
				InitializeVectorsSlave_file<<" *--------------------------------------------------------------------*/\n";
				InitializeVectorsSlave_file<<"typedef struct {\n";
				InitializeVectorsSlave_file<<"\tint task;\n";
				InitializeVectorsSlave_file<<"\tint processor;\n";
				InitializeVectorsSlave_file<<"} TaskLocation;\n";
				InitializeVectorsSlave_file<<"\n";
				InitializeVectorsSlave_file<<"TaskLocation task_location[MAX_GLOBAL_TASKS] = {";
				for(int v=1; v<(slaveProcessors*maxTasksSlave); v++) InitializeVectorsSlave_file<<"{-1, -1}, ";
				InitializeVectorsSlave_file<<"{-1, -1}};\n";
				InitializeVectorsSlave_file<<"\n";
				InitializeVectorsSlave_file<<"int request_task[MAX_GLOBAL_TASKS] = {";
				for(int v=1; v<(slaveProcessors*maxTasksSlave); v++) InitializeVectorsSlave_file<<"-1, ";
				InitializeVectorsSlave_file<<"-1 };\n";
				InitializeVectorsSlave_file<<"\n";
				InitializeVectorsSlave_file<<"int location_request_task[MAX_GLOBAL_TASKS] = {";
				for(int v=1; v<(slaveProcessors*maxTasksSlave); v++) InitializeVectorsSlave_file<<"-1, ";
				InitializeVectorsSlave_file<<"-1 };\n";
				
				InitializeVectorsSlave_file.close();
			
			
				/* Generates the InitializeVectorsMaster.h file */
				if (argc == 3) file_path = "./" + projectPath + projectName + "/build/InitializeVectorsMaster.h";
				else file_path = "./" + projectName + "/build/InitializeVectorsMaster.h";
				ofstream InitializeVectorsMaster_file(file_path.c_str());
				
				InitializeVectorsMaster_file<<"/*--------------------------------------------------------------------\n";
				InitializeVectorsMaster_file<<" * struct TaskLocation\n";
				InitializeVectorsMaster_file<<" *\n";
				InitializeVectorsMaster_file<<" * DESCRIPTION:\n";
				InitializeVectorsMaster_file<<" *    Makes the task versus processor association\n";
				InitializeVectorsMaster_file<<" *\n";
				InitializeVectorsMaster_file<<" *--------------------------------------------------------------------*/\n";
				InitializeVectorsMaster_file<<"typedef struct {\n";
				InitializeVectorsMaster_file<<"\tint task;\n";
				InitializeVectorsMaster_file<<"\tint processor;\n";
				InitializeVectorsMaster_file<<"} TaskLocation;\n";
				InitializeVectorsMaster_file<<"\n";
				InitializeVectorsMaster_file<<"/*--------------------------------------------------------------------\n";
				InitializeVectorsMaster_file<<" * struct TaskRequest\n";
				InitializeVectorsMaster_file<<" *\n";
				InitializeVectorsMaster_file<<" * DESCRIPTION:\n";
				InitializeVectorsMaster_file<<" *    Store the task requests\n";
				InitializeVectorsMaster_file<<" *\n";
				InitializeVectorsMaster_file<<" *--------------------------------------------------------------------*/\n";
				InitializeVectorsMaster_file<<"typedef struct {\n";
				InitializeVectorsMaster_file<<"\tint requested_task;\n";
				InitializeVectorsMaster_file<<"\tint requesting_task;\n";
				InitializeVectorsMaster_file<<"\tint source_processor;\n";
				InitializeVectorsMaster_file<<"} TaskRequest;\n";
				InitializeVectorsMaster_file<<"\n";		
				InitializeVectorsMaster_file<<"/*--------------------------------------------------------------------\n";
				InitializeVectorsMaster_file<<" * struct LocationRequest\n";
				InitializeVectorsMaster_file<<" *\n";
				InitializeVectorsMaster_file<<" * DESCRIPTION:\n";
				InitializeVectorsMaster_file<<" *    Store the task location requests\n";
				InitializeVectorsMaster_file<<" *\n";
				InitializeVectorsMaster_file<<" *--------------------------------------------------------------------*/\n";
				InitializeVectorsMaster_file<<"typedef struct {\n";
				InitializeVectorsMaster_file<<"\tint requested_task;\n";
				InitializeVectorsMaster_file<<"\tint requesting_task;\n";
				InitializeVectorsMaster_file<<"\tint source_processor;\n";
				InitializeVectorsMaster_file<<"} LocationRequest;\n";
				InitializeVectorsMaster_file<<"\n";	
				InitializeVectorsMaster_file<<"TaskLocation task_location[MAX_GLOBAL_TASKS] = {";
				for(int v=1; v<(slaveProcessors*maxTasksSlave); v++) InitializeVectorsMaster_file<<"{-1, -1}, ";
				InitializeVectorsMaster_file<<"{-1, -1}};\n";
				InitializeVectorsMaster_file<<"\n";
				InitializeVectorsMaster_file<<"int task_pes[MAX_GLOBAL_TASKS*2] = {";
				for(int v=1; v<(slaveProcessors*maxTasksSlave*2); v++) InitializeVectorsMaster_file<<"-1, ";
				InitializeVectorsMaster_file<<"-1};\n";
				InitializeVectorsMaster_file<<"\n";
				InitializeVectorsMaster_file<<"TaskRequest task_request[MAX_GLOBAL_TASKS*2] = {";
				for(int v=1; v<(slaveProcessors*maxTasksSlave*2); v++) InitializeVectorsMaster_file<<"{-1, -1, -1}, ";
				InitializeVectorsMaster_file<<"{-1, -1, -1}};\n";
				InitializeVectorsMaster_file<<"\n";
				InitializeVectorsMaster_file<<"LocationRequest location_request[MAX_GLOBAL_TASKS*2] = {";
				for(int v=1; v<(slaveProcessors*maxTasksSlave*2); v++) InitializeVectorsMaster_file<<"{-1, -1, -1}, ";
				InitializeVectorsMaster_file<<"{-1, -1, -1}};\n";
				InitializeVectorsMaster_file<<"\n";	
				InitializeVectorsMaster_file<<"char task_terminated[MAX_GLOBAL_TASKS] = {";
				for(int v=1; v<(slaveProcessors*maxTasksSlave); v++) InitializeVectorsMaster_file<<"-1, ";
				InitializeVectorsMaster_file<<"-1 };\n";
				
				InitializeVectorsMaster_file.close();
			
				/* Fires the tasks compilation */
				if (argc == 3)
				{
					command = "cd ./" + projectPath + projectName + "/build; make clean; make all > generation.log 2> error_warning.log";
					system(command.c_str());
					command = "cd ..";
					system(command.c_str());
					command = "cd ..";
					system(command.c_str());
					command = "cd ..";
					system(command.c_str());	
				}
                
                else
                {
					command = "cd ./" + projectName + "/build; make clean; make all > generation.log 2> error_warning.log";
					system(command.c_str());
					command = "cd ..";
					system(command.c_str());
					command = "cd ..";
					system(command.c_str());
				}
                
                Repository_vhd(argc,projectPath);
				Repository_h(argc,projectPath);
				
				Partial_Repositories(argc,projectPath);
				
	}
	
	
	
	return 0;
}

void Task_Dependences(int argc, string projectPath)
{
	list <task>::iterator itask;
	string depname, flits;
	int p1, p2, receiveok, sendok, dep_ok;
	int old_flits;	
		
	for(ia=applications.begin();ia!=applications.end();ia++)
	{
	   list<task> InitTasks;
	   for(it=ia->tasks.begin();it!=ia->tasks.end();it++)
	   {
				if (argc == 3) file_path = "./" + projectPath + projectName + "/applications/"+ ia->get_name() + "/" + it->get_name();
				else file_path = "./" + projectName + "/applications/"+ ia->get_name() + "/" + it->get_name();
				ifstream taskfile(file_path.c_str());
		
				receiveok = 0;
				sendok = 0;
				dep_ok = 0;
				
				while(!taskfile.eof())
				{
					getline(taskfile,line);
					
					if(line.find("/*Comm",0)!=string::npos)
					{
						p1 = line.find("/*Comm",0);
						p2 = line.find(" ", p1)+1;
						line.erase(0, p2);
						p2 = line.find(" ", p1);
						depname = line.substr(0, p2);
						line.erase(0, p2+1);
						depname = depname + ".c";
						p1 = line.find("*/",0);
						flits = line.substr(0, p1);
						//cout<<"depname "<<depname<<endl;
						//cout<<"flits "<<flits<<endl;
						int has_task = 0;
						for(id=it->dependences.begin();id!=it->dependences.end();id++) 
						{
							if(id->get_name()==depname)
							{
								old_flits = id->get_flits();
								id->set_flits(old_flits + atoi(flits.c_str()));
								has_task = 1;
							}
						}
						if(!has_task)
						{
							d = new dependence(depname, atoi(flits.c_str()));
							it->dependences.push_back(*d);
						}
						dep_ok++;
					}
					
					if(line.find("Receive",0)!=string::npos)
					{
						receiveok = 1;
					}
					if((line.find("Send",0)!=string::npos) && receiveok == 0 && sendok == 0)
					{
						int has_task = 0;
						for(itask=InitTasks.begin();itask!=InitTasks.end();itask++) 
						{
							if(itask->get_name()==it->get_name()) has_task = 1;
						}
						if(!has_task) InitTasks.push_back(*it);
						sendok = 1;
					}
				}
				taskfile.close();
				
				if (dep_ok == 0)
				{
					ifstream taskfile(file_path.c_str());
					while(!taskfile.eof())
					{
						getline(taskfile,line);
						if (line.find("Receive",0)!=string::npos || line.find("Send",0)!=string::npos)
						{
							p1 = line.find(",", 0);
							depname = line.substr(p1 + 1);
							p1 = depname.find(")",0);
							depname = depname.substr(0, p1) + ".c";
							depname = removeSpaces(depname);
							
							int has_task = 0;
							for(id=it->dependences.begin();id!=it->dependences.end();id++) 
							{
								if(id->get_name()==depname) has_task = 1;
							}
							if(!has_task)
							{
								d = new dependence(depname,100);
								it->dependences.push_back(*d);
							}
						}
					}
					taskfile.close();
				}
				it->dependences.sort(compare_flits);
				cout<<it->get_name()<<endl;
				for(id=it->dependences.begin();id!=it->dependences.end();id++) 
				{
					cout<<"\t"<<id->get_name()<<" "<<id->get_flits()<<endl;
				}
		}
		/*cout<<ia->get_name()<<endl;
		for(itask=InitTasks.begin();itask!=InitTasks.end();itask++) 
		{
			cout<<"\t"<<itask->get_name()<<endl;
		}*/
		ia->initTasks = InitTasks;
	}
	
}

int get_taskID(string name)
{
	for(int i=0;i<taskIDs.size();i++)
	{	
		if(taskIDs[i].get_name() == name) return i;
	}
}
bool compare_stime (application first, application second)
{
  if(first.get_stime() < second.get_stime()) return true;
  else return false;
}
bool compare_flits (dependence first, dependence second)
{
  if(first.get_flits() > second.get_flits()) return true;
  else return false;
}
string ftoa (float val) 
{
  stringstream ss (stringstream::in | stringstream::out);
  ss << val;
  string valstr = ss.str();
  return valstr;
}

string removeSpaces(string stringIn)
{
	string::size_type pos = 0;
	bool spacesLeft = true;

	while( spacesLeft )
	{
		pos = stringIn.find(" ");
		if( pos != string::npos ) stringIn.erase( pos, 1 );
		else spacesLeft = false;
	}
	return stringIn;
} 

void Repository_vhd(int argc, string projectPath)
{
	int memsize = MEM_SIZE;
	FILE *in,  *vhdl;
	string taskfile, tname;
	unsigned char * buffer;
	int size, initial_address_code = MEM_SIZE-1; 
	int proc_address;
	
	for(int i=0;i<taskIDs.size();i++)
	{
		if(taskIDs[i].get_insertion_time()==0)
		{
			tname = taskIDs[i].get_name().substr(0,taskIDs[i].get_name().find(".",0));
			if (argc == 3) taskfile = "./" + projectPath + projectName + "/build/" + tname + "_" + ftoa(i) + ".txt";
			else taskfile = "./" + projectName + "/build/" + tname + "_" + ftoa(i) + ".txt";
		
			if ((in = fopen(taskfile.c_str(), "r")) != NULL)
			{
				//cout<<taskfile<<endl;
				
				buffer = (unsigned char *)malloc(300001);
				size = (int)fread(buffer, 1, 30001, in);
				size = size/9; 
				//cout<<"code size: "<<size<<endl;
				taskIDs[i].set_code_size(size);
				initial_address_code = initial_address_code - size;
				taskIDs[i].set_initial_address(initial_address_code);   //atribui endereço inicial */
				//cout<<"initial_address: "<<initial_address_code<<endl;
			}
			
			fclose(in);
		}
	}
	if (argc == 3) file_path = "./" + projectPath + projectName + "/repository.vhd";
	else file_path = "./" + projectName + "/repository.vhd";
	vhdl = fopen(file_path.c_str(), "w");
	
	fprintf(vhdl, "-------------------------------------------------------------------------------------\n");
	fprintf(vhdl, "--	Tasks Repository                                                                \n");
	fprintf(vhdl, "--		Contains the object codes of all tasks                                      \n");
	fprintf(vhdl, "-------------------------------------------------------------------------------------\n");
	fprintf(vhdl, "--repository structure:                                                              \n");
	fprintf(vhdl, "--[/this structure is replicaded according the number of tasks]                      \n");
	fprintf(vhdl, "--number of tasks                                                                    \n");
	fprintf(vhdl, "--task id                                                                            \n");
	fprintf(vhdl, "--task code size                                                                     \n");
	fprintf(vhdl, "--processor (ffffffff means dynamic allocation)                                      \n");
	fprintf(vhdl, "--task code start address                                                            \n");
	fprintf(vhdl, "--[/this structure is replicaded according the number of tasks]                      \n");
	fprintf(vhdl, "--tasks codes                                                                        \n");
	fprintf(vhdl, "-------------------------------------------------------------------------------------\n");

	fprintf(vhdl, "%s\n", "library IEEE;");
	fprintf(vhdl, "%s\n\n", "use IEEE.Std_Logic_1164.all;");
	fprintf(vhdl, "%s\n", "package memory_pack is");
	fprintf(vhdl, "\t%s%d%s\n", "constant REPO_SIZE : integer := ",memsize-1,";");
	fprintf(vhdl, "\t%s%d%s\n\n", "type ram is array (0 to ",memsize-1,") of std_logic_vector(31 downto 0);");
	fprintf(vhdl, "\t%s\n", "signal memory : ram := (");
	fprintf(vhdl, "\t\tx\"%08x\",\n", staticTasks);

	for(int i = 0; i <taskIDs.size(); i++)
	{
		if(taskIDs[i].get_insertion_time()==0)
		{
			fprintf(vhdl, "\t\tx\"%08x\",--id %s\n", i, taskIDs[i].get_name().c_str());
			fprintf(vhdl, "\t\tx\"%08x\",--code size\n", taskIDs[i].get_code_size());
			if(taskIDs[i].get_x_proc() == -1)
				fprintf(vhdl, "\t\tx\"ffffffff\",--processor\n");
			else
			{
				proc_address = taskIDs[i].get_x_proc()<<4 ^ taskIDs[i].get_y_proc();
				fprintf(vhdl, "\t\tx\"%08x\",--processor\n",proc_address);
			}
			fprintf(vhdl, "\t\tx\"%08x\",--object code start address\n", taskIDs[i].get_initial_address()*4);
			
			int cont = 0;
			int nothing = -1;
			id=taskIDs[i].dependences.begin();
			while(id!=taskIDs[i].dependences.end() && cont<10) 
			{
				//cout<<"\t"<<id->get_name()<<endl;
				fprintf(vhdl, "\t\tx\"%08x\",--%s\n", get_taskID(id->get_name()),id->get_name().c_str());
				fprintf(vhdl, "\t\tx\"%08x\",\n", id->get_flits());
				cont++;
				id++;
			}
			for(int k=cont; k<10; k++)
			{
					fprintf(vhdl, "\t\tx\"%08x\",\n", nothing);
					fprintf(vhdl, "\t\tx\"%08x\",\n", nothing);
			}
		}
	}
		
	for(int i=24*staticTasks+1; i < initial_address_code; i++){
		
		fprintf(vhdl, "\t\tx\"00000000\",\n");
	}	
		
	for(int i = taskIDs.size()-1; i >= 0; i--)
	{
		if(taskIDs[i].get_insertion_time()==0)
		{
			tname = taskIDs[i].get_name().substr(0,taskIDs[i].get_name().find(".",0));
			if (argc == 3) taskfile = "./" + projectPath + projectName + "/build/" + tname + "_" + ftoa(i) + ".txt";
			else taskfile = "./" + projectName + "/build/" + tname + "_" + ftoa(i) + ".txt";
			
			ifstream taskcode(taskfile.c_str());
			string code_line;
			
			while(!taskcode.eof())
			{
				getline(taskcode, code_line);
				if(code_line!="") fprintf(vhdl, "\t\tx\"%s\",\n", code_line.c_str());
			}
		}
	}

	fprintf(vhdl, "\t\t%s\n\n", "others => x\"00000000\");");
	fprintf(vhdl, "%s\n", "end memory_pack;");

	fclose(vhdl);
	
}

void Repository_h(int argc, string projectPath)
{
	int memsize = MEM_SIZE;
	FILE *in,  *h;
	string taskfile, tname;
	unsigned char * buffer;
	int size, initial_address_code = MEM_SIZE-1; 
	int proc_address;
	
	for(int i=0;i<taskIDs.size();i++)
	{
		
		tname = taskIDs[i].get_name().substr(0,taskIDs[i].get_name().find(".",0));
		if (argc == 3) taskfile = "./" + projectPath + projectName + "/build/" + tname + "_" + ftoa(i) + ".txt";
		else taskfile = "./" + projectName + "/build/" + tname + "_" + ftoa(i) + ".txt";
	
		if ((in = fopen(taskfile.c_str(), "r")) != NULL)
		{
			//cout<<taskfile<<endl;
			
			buffer = (unsigned char *)malloc(300001);
			size = (int)fread(buffer, 1, 300001, in);
			size = size/9; 
   			//cout<<"code size: "<<size<<endl;
   			taskIDs[i].set_code_size(size);
   			initial_address_code = initial_address_code - size;
			taskIDs[i].set_initial_address(initial_address_code);   //atribui endereço inicial */
			//cout<<"initial_address: "<<initial_address_code<<endl;
		}
		
   		fclose(in);
	}
	
	if (argc == 3) file_path = "./" + projectPath + projectName + "/repository.h";
	else file_path = "./" + projectName + "/repository.h";
	h = fopen(file_path.c_str(), "w");
	
	
	fprintf(h, "#ifndef _taskRepository						\n");
	fprintf(h, "#define _taskRepository						\n");
	fprintf(h, "											\n");
	fprintf(h, "#define REPO_SIZE 300000// 300000 Positions	\n");
	fprintf(h, "unsigned int repository[REPO_SIZE] = {		\n");
	fprintf(h, "											\n");
	
	fprintf(h, "\t0x%08x,\n", staticTasks);
	for(int i = 0; i <taskIDs.size(); i++)
	{
		fprintf(h, "\t0x%08x,//id %s\n", i, taskIDs[i].get_name().c_str());
		fprintf(h, "\t0x%08x,//code size\n", taskIDs[i].get_code_size());
		if(taskIDs[i].get_x_proc() == -1)
			fprintf(h, "\t0xffffffff,//processor\n");
		else
		{
			proc_address = taskIDs[i].get_x_proc()<<4 ^ taskIDs[i].get_y_proc();
			fprintf(h, "\t0x%08x,//processor\n",proc_address);
		}
		fprintf(h, "\t0x%08x,//object code start address\n", taskIDs[i].get_initial_address()*4);
		
		int cont = 0;
		int nothing = -1;
		id=taskIDs[i].dependences.begin();
		while(id!=taskIDs[i].dependences.end() && cont<10) 
		{
			fprintf(h, "\t0x%08x,\n", get_taskID(id->get_name()));
			fprintf(h, "\t0x%08x,\n", id->get_flits());
			cont++;
			id++;
		}
		for(int k=cont; k<10; k++)
		{
				fprintf(h, "\t0x%08x,\n", nothing);
				fprintf(h, "\t0x%08x,\n", nothing);
		}
	}
		
	for(int i=24*taskIDs.size()+1; i < initial_address_code; i++){
		
		fprintf(h, "\t0x00000000,\n");
	}	
		
	for(int i = taskIDs.size()-1; i >= 0; i--)
	{
		tname = taskIDs[i].get_name().substr(0,taskIDs[i].get_name().find(".",0));
		if (argc == 3) taskfile = "./" + projectPath + projectName + "/build/" + tname + "_" + ftoa(i) + ".txt";
		else taskfile = "./" + projectName + "/build/" + tname + "_" + ftoa(i) + ".txt";
		
		ifstream taskcode(taskfile.c_str());
		string code_line;
		
		while(!taskcode.eof())
		{
			getline(taskcode, code_line);
			if(code_line!="") fprintf(h, "\t0x%s,\n", code_line.c_str());
		}
	}

	fprintf(h, "};		\n");
	fprintf(h, "		\n");
	fprintf(h, "#endif	\n");

	fclose(h);
	
}

void Partial_Repositories(int argc, string projectPath)
{
	int memsize = MEM_SIZE;
	FILE *in,  *vhdl, *h;
	string taskfile, tname;
	unsigned char * buffer;
	int size, initial_address_code; 
	int proc_address;
	int appindex = 0;
	
	if (argc == 3) file_path = "./" + projectPath + projectName + "/dynamic_apps.vhd";
	else file_path = "./" + projectName + "/dynamic_apps.vhd";
	vhdl = fopen(file_path.c_str(), "w");
	
	if (argc == 3) file_path = "./" + projectPath + projectName + "/dynamic_apps.h";
	else file_path = "./" + projectName + "/dynamic_apps.h";
	h = fopen(file_path.c_str(), "w");
	
	fprintf(vhdl, "-------------------------------------------------------------------------------------\n");
	fprintf(vhdl, "--\tPartial Repository																\n");
	fprintf(vhdl, "--\t\tContains the object codes of the tasks inserted on runtime						\n");
	fprintf(vhdl, "-------------------------------------------------------------------------------------\n");
	fprintf(vhdl, "--repository structure:																\n");
	fprintf(vhdl, "--[/this structure is replicaded according the number of tasks]						\n");
	fprintf(vhdl, "--number of tasks																	\n");
	fprintf(vhdl, "--task id																			\n");
	fprintf(vhdl, "--task code size																		\n");
	fprintf(vhdl, "--processor (ffffffff means dynamic allocation)										\n");
	fprintf(vhdl, "--task code start address															\n");
	fprintf(vhdl, "--[/this structure is replicaded according the number of tasks]						\n");
	fprintf(vhdl, "--tasks codes																		\n");
	fprintf(vhdl, "-------------------------------------------------------------------------------------\n");
	fprintf(vhdl, "library IEEE;																		\n");
	fprintf(vhdl, "use IEEE.Std_Logic_1164.all;														  \n\n");
	fprintf(vhdl, "use work.memory_pack.all;														  \n\n");
	fprintf(vhdl, "package dynamic_apps_pack is															\n");
	if (runtimeTasks != 0)
		fprintf(vhdl, "\ttype repository_array is array (0 to %d ) of ram;\n\n", runtimeApps);
	else fprintf(vhdl, "\ttype repository_array is array (0 to 1) of ram;");
	
	fprintf(h, "#ifndef _dynAppsRepository\n");
	fprintf(h, "#define _dynAppsRepository\n");
	fprintf(h, "#define NUMBER_OF_APPS %d \n", runtimeApps);
	if (runtimeTasks != 0)
	{
		string aux = "";
		float pastTime = 0;
		float delay = 0;
		string delay_str;
		for(ia=applications.begin();ia!=applications.end();ia++)
		{
        	if(ia->get_stime()!=0)
			{
				delay = ia->get_stime() - pastTime;
				delay_str = ftoa(delay);
				aux = aux + delay_str + ",";
				pastTime = ia->get_stime();
			}
		}
		aux = "{" + aux + "};";
		fprintf(h, "float appstime[%d] = %s\n", runtimeApps, aux.c_str());
	}
	else
	{
		fprintf(h,"float appstime[1] = { 0 };\n");
	}
	fprintf(h,"\n");
	
	for(ia=applications.begin();ia!=applications.end();ia++)
	{
		if(ia->get_stime()!=0)
		{
			fprintf(vhdl, "\n\t signal dynamic_app_%d : ram := (\n", appindex);
			fprintf(h, "unsigned int dynamic_app_%d[11] = {\n", appindex);

			/* Sets the first task object code start in the repositoty */
			initial_address_code = (23 * ia->tasks.size()) + 11;
			
			/* Creates the repository file */

			/* Writes the number of tasks in the repository */
			fprintf(vhdl, "\t\tx\"%08x\",\n", (unsigned int)ia->tasks.size());
			fprintf(h, "\t\t0x%08x,\n", (unsigned int)ia->tasks.size());
			
			/* Writes the application initial tasks */
			int cont = 0;
			int nothing = -1;
			it=ia->initTasks.begin();
			while(it!=ia->initTasks.end() && cont<10) 
			{
				fprintf(vhdl, "\t\tx\"%08x\",--initial task id %s\n", get_taskID(it->get_name()), it->get_name().c_str());
				fprintf(h, "\t\t0x%08x,//initial task id %s\n", get_taskID(it->get_name()), it->get_name().c_str());
				cont++;
				it++;
			}
			for(int k=cont; k<10; k++)
			{
					fprintf(vhdl, "\t\tx\"%08x\",\n", nothing);
					fprintf(h, "\t\t0x%08x,\n", nothing);
			}
			
			for(it=ia->tasks.begin(); it!=ia->tasks.end(); it++)
			{
				tname = it->get_name().substr(0,it->get_name().find(".",0));
				if (argc == 3) taskfile = "./" + projectPath + projectName + "/build/" + tname + "_" + ftoa(get_taskID(it->get_name())) + ".txt";
				else taskfile = "./" + projectName + "/build/" + tname + "_" + ftoa(get_taskID(it->get_name())) + ".txt";
			
				if ((in = fopen(taskfile.c_str(), "r")) != NULL)
				{
					buffer = (unsigned char *)malloc(30000);
					size = (int)fread(buffer, 1, 30000, in);
					size = size/9; 
					it->set_code_size(size);
					it->set_initial_address(initial_address_code);   
				}
				initial_address_code = initial_address_code + size;
				fclose(in);
			}
			
			for(it=ia->tasks.begin(); it!=ia->tasks.end(); it++)
			{
				fprintf(vhdl, "\t\tx\"%08x\",--%s\n", get_taskID(it->get_name()), it->get_name().c_str());
				fprintf(vhdl, "\t\tx\"%08x\",--code size\n", it->get_code_size());
				fprintf(vhdl, "\t\tx\"%08x\",--initial_address\n", it->get_initial_address()*4);
				
				int cont = 0;
				int nothing = -1;
				id=it->dependences.begin();
				while(id!=it->dependences.end() && cont<10) 
				{
					fprintf(vhdl, "\t\tx\"%08x\",\n", get_taskID(id->get_name()));
					fprintf(vhdl, "\t\tx\"%08x\",\n", id->get_flits());
					cont++;
					id++;
				}
				for(int k=cont; k<10; k++)
				{
						fprintf(vhdl, "\t\tx\"%08x\",\n", nothing);
						fprintf(vhdl, "\t\tx\"%08x\",\n", nothing);
				}
			}
					
			for(it=ia->tasks.begin(); it!=ia->tasks.end(); it++)
			{
				tname = it->get_name().substr(0,it->get_name().find(".",0));
				tname = tname + "_" + ftoa(get_taskID(it->get_name())) + ".txt";
				taskfile = "./" + projectName + "/build/" + tname;
				
				ifstream taskcode(taskfile.c_str());
				string code_line;
				
				while(!taskcode.eof())
				{
					getline(taskcode, code_line);
					if(code_line!="") fprintf(vhdl, "\t\tx\"%s\",\n", code_line.c_str());
				}
			}
			
			fprintf(h, "};\n\n");
			fprintf(vhdl, "\t\t%s\n\n", "others => x\"00000000\");");
			appindex++;
		}
	}
	if (runtimeApps !=0)
	{
		fprintf(vhdl, "\tsignal dynamic_apps : repository_array := (\n");
		fprintf(h, "unsigned int dynamic_apps[%d][11];\n\n", runtimeApps);
		fprintf(h, "void InitializeAppsInfo()\n");
		fprintf(h, "{\n");
		for (int i = 0; i < runtimeApps; i++)
		{
			 for (int j = 0; j < 11; j++)
			 {
				fprintf(h, "\tdynamic_apps[%d][%d] = dynamic_app_%d[%d];\n", i, j, i, j);	
			 }
			fprintf(vhdl, "\t\tdynamic_app_%d,\n", i);
		}
		fprintf(h, "}\n");
		fprintf(vhdl, "\t\t(others =>(others=>'0'))\n");
		fprintf(vhdl, "\t);\n");
	}
	else
	{
		fprintf(h, "unsigned int dynamic_apps[1][1] = { { 0 }, };\n");
		fprintf(vhdl, "\tsignal dynamic_apps : repository_array := ( (others =>(others=>'0')), (others =>(others=>'0')));\n");
		fprintf(h, "void InitializeAppsInfo() {}\n");
	}
	fprintf(vhdl, "end dynamic_apps_pack;\n");
	
	fprintf(h, "\n");
	fprintf(h, "#endif\n");
	
	fclose(vhdl);
	fclose(h);
	
}
