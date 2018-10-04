#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{

	int i,t;

    Echo("synthetic0 task F started.");
	Echo(itoa(GetTick()));

for(i=0;i<100;i++){
	
		Receive(&msg,taskE_0);
			for(t=0;t<1000;t++){
		}
		Receive(&msg,taskD_0);

	}

	Echo(itoa(GetTick()));
    Echo("synthetic0 task F finished.");

	exit();

}
