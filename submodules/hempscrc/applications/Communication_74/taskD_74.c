#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{

	int i;

    Echo("communication74 task D started.");
	Echo(itoa(GetTick()));

for(i=0;i<50;i++){
		Receive(&msg,taskC_74);
	}

	Echo(itoa(GetTick()));
    Echo("communication74 task D finished.");

	exit();

}
