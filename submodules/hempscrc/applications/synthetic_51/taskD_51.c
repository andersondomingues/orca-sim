#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{

	int i, j,t;

    Echo("synthetic51 task D started.");
	Echo(itoa(GetTick()));

for(i=0;i<100;i++){
	msg.length = 30;
	for(j=0;j<30;j++) msg.msg[j]=i;
	
		Receive(&msg,taskC_51);
			for(t=0;t<1000;t++){
		}
		Send(&msg,taskF_51);

	}

	Echo(itoa(GetTick()));
    Echo("synthetic51 task D finished.");

	exit();

}