#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{

	int i;

    Echo("communication72 task D started.");
	Echo(itoa(GetTick()));

for(i=0;i<50;i++){
		Receive(&msg,taskC_72);
	}

	Echo(itoa(GetTick()));
    Echo("communication72 task D finished.");

	exit();

}
