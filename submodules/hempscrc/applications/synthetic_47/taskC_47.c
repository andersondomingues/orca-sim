#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{
	
	int i, j,t;

	Echo("synthetic47 task C started.");
	Echo(itoa(GetTick()));

for(i=0;i<100;i++){
	
	msg.length = 30;
	for(j=0;j<30;j++) msg.msg[j]=i;

	Receive(&msg,taskA_47);
	
	for(t=0;t<1000;t++){
	}
	
	Send(&msg,taskD_47);
	
	Receive(&msg,taskB_47);
	
	for(t=0;t<1000;t++){
	}
	
	Send(&msg,taskE_47);

}
    Echo(itoa(GetTick()));
    Echo("synthetic47 task C finished.");

	exit();
}
