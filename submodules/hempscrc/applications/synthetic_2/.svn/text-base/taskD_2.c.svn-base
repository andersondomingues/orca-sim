#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{

	int i, j,t;

    Echo("synthetic2 task D started.");
	Echo(itoa(GetTick()));

for(i=0;i<100;i++){
	msg.length = 30;
	for(j=0;j<30;j++) msg.msg[j]=i;
	
		Receive(&msg,taskC_2);
			for(t=0;t<1000;t++){
		}
		Send(&msg,taskF_2);

	}

	Echo(itoa(GetTick()));
    Echo("synthetic2 task D finished.");

	exit();

}
