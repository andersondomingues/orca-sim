#include <task.h>
#include <stdlib.h>
#include "parameters.h"

Message msg;

int main()
{
	
	int i, j,t;

	Echo("synthetic0 task C started.");
	Echo(itoa(GetTick()));

for(i=0;i<NUMBER_OF_ITERATIONS;i++){
	
	msg.length = 30;
	for(j=0;j<30;j++) msg.msg[j]=i;

	Receive(&msg,taskA_0);
	
	for(t=0;t<1000;t++){
	}
	
	Send(&msg,taskD_0);
	
	Receive(&msg,taskB_0);
	
	for(t=0;t<1000;t++){
	}
	
	Send(&msg,taskE_0);

}
    Echo(itoa(GetTick()));
    Echo("synthetic0 task C finished.");

	exit();
}
