#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{
	
	int i, j,t;

	Echo("synthetic33 task C started.");
	Echo(itoa(GetTick()));

for(i=0;i<100;i++){
	
	msg.length = 30;
	for(j=0;j<30;j++) msg.msg[j]=i;

	Receive(&msg,taskA_33);
	
	for(t=0;t<1000;t++){
	}
	
	Send(&msg,taskD_33);
	
	Receive(&msg,taskB_33);
	
	for(t=0;t<1000;t++){
	}
	
	Send(&msg,taskE_33);

}
    Echo(itoa(GetTick()));
    Echo("synthetic33 task C finished.");

	exit();
}
