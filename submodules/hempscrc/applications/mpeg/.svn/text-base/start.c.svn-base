/*---------------------------------------------------------------------
TITLE: Program Scheduler
AUTHOR: Nicolas Saint-jean
EMAIL : saintjea@lirmm.fr
DATE CREATED: 04/04/06
FILENAME: task2.c
PROJECT: Network Process Unit
COPYRIGHT: Software placed into the public domain by the author.
           Software 'as is' without warranty.  Author liable for nothing.
DESCRIPTION: This file contains the task2
---------------------------------------------------------------------*/

#include <task.h>
#include <stdlib.h>

unsigned int vlc_array[128] = { // array containing the compressed data stream
                                 // It must be specified as an input
                                 0xfa,0xb8,0x20,0x05,0x20,0x20,0x02,0x38,
                                 0x20,0x7e,0x7f,0xf0,0x10,0x3f,0x54,0x8a,
                                 0x08,0x1f,0xa8,0x00,0x42,0x00,0xd2,0x80,
                                 0x3e,0xf6,0xa0,0x0e,0x3e,0x45,0x80,0x3e,
                                 0xc0,0x07,0xbc,0x79,0x00,0x3f,0xc2,0x28,
                                 0xb2,0x3f,0x0e,0x78,0xbe,0x88,0x9c,0x82,
                                 0x17,0xfc,0x11,0xbc,0x85,0x74,0x27,0xa7,
                                 0xf2,0x24,0x02,0xce,0x5f,0xc7,0xce,0x4e,
                                 0xa7,0x3c,0x73,0xb6,0x31,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
                                 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

Message msg1;

int main(){

   unsigned int time_a, time_b;
   int i;

   Echo("MPEG Task A start: Send block ");
   Echo(itoa(GetTick()));


   for(i=0; i<128; i++)
        msg1.msg[i] = vlc_array[i];


    msg1.length = 128;

    for(i=0;i<8;i++)                          // send 8 times the array to task 2
	   {
		   time_a = GetTick();
		   Send(&msg1,ivlc);
		   time_b = GetTick();

		   Echo("T1");
           Echo(itoa(time_a));
		   Echo("T2");
           Echo(itoa(time_b));

	   }

    Echo(itoa(GetTick()));
    Echo("End Task A - MPEG");

    exit();
}
