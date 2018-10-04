#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{
	
	int i, j;

	Echo("communication74 task C started.");
	Echo(itoa(GetTick()));

for(i=0;i<50;i++){
	
	msg.length = 30;
	for(j=0;j<30;j++) msg.msg[j]=i;

	Receive(&msg,taskA_74);
	Send(&msg,taskD_74);
	Receive(&msg,taskB_74);
}
    Echo(itoa(GetTick()));
    Echo("communication74 task C finished.");

	exit();
}
