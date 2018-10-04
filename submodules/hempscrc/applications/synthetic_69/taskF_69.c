#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{

	int i,t;

    Echo("synthetic69 task F started.");
	Echo(itoa(GetTick()));

for(i=0;i<100;i++){
	
		Receive(&msg,taskE_69);
			for(t=0;t<1000;t++){
		}
		Receive(&msg,taskD_69);

	}

	Echo(itoa(GetTick()));
    Echo("synthetic69 task F finished.");

	exit();

}
