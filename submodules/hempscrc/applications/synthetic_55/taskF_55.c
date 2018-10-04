#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{

	int i,t;

    Echo("synthetic55 task F started.");
	Echo(itoa(GetTick()));

for(i=0;i<100;i++){
	
		Receive(&msg,taskE_55);
			for(t=0;t<1000;t++){
		}
		Receive(&msg,taskD_55);

	}

	Echo(itoa(GetTick()));
    Echo("synthetic55 task F finished.");

	exit();

}
