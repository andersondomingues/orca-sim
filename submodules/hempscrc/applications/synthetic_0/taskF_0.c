#include <task.h>
#include <stdlib.h>
#include "parameters.h"

Message msg;

int main()
{

	int i,t;

    Echo("synthetic0 task F started.");
	Echo(itoa(GetTick()));

for(i=0;i<NUMBER_OF_ITERATIONS;i++){
	
		Receive(&msg,taskD_0);
			for(t=0;t<1000;t++){
		}
		Receive(&msg,taskE_0);

	}

	Echo(itoa(GetTick()));
    Echo("synthetic0 task F finished.");

	exit();

}
