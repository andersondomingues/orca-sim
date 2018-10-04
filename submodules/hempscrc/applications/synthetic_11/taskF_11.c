#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{

	int i,t;

    Echo("synthetic11 task F started.");
	Echo(itoa(GetTick()));

for(i=0;i<100;i++){
	
		Receive(&msg,taskE_11);
			for(t=0;t<1000;t++){
		}
		Receive(&msg,taskD_11);

	}

	Echo(itoa(GetTick()));
    Echo("synthetic11 task F finished.");

	exit();

}
