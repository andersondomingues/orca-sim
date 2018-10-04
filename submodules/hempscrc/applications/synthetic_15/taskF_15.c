#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{

	int i,t;

    Echo("synthetic15 task F started.");
	Echo(itoa(GetTick()));

for(i=0;i<100;i++){
	
		Receive(&msg,taskE_15);
			for(t=0;t<1000;t++){
		}
		Receive(&msg,taskD_15);

	}

	Echo(itoa(GetTick()));
    Echo("synthetic15 task F finished.");

	exit();

}
