#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{

	int i,t;

    Echo("synthetic17 task F started.");
	Echo(itoa(GetTick()));

for(i=0;i<100;i++){
	
		Receive(&msg,taskE_17);
			for(t=0;t<1000;t++){
		}
		Receive(&msg,taskD_17);

	}

	Echo(itoa(GetTick()));
    Echo("synthetic17 task F finished.");

	exit();

}
