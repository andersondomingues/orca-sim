#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{
	
	int i, j,t;

	Echo("synthetic81 task C started.");
	Echo(itoa(GetTick()));

for(i=0;i<100;i++){
	
	msg.length = 30;
	for(j=0;j<30;j++) msg.msg[j]=i;

	Receive(&msg,taskA_81);
	
	for(t=0;t<1000;t++){
	}
	
	Send(&msg,taskD_81);
	
	Receive(&msg,taskB_81);
	
	for(t=0;t<1000;t++){
	}
	
	Send(&msg,taskE_81);

}
    Echo(itoa(GetTick()));
    Echo("synthetic81 task C finished.");

	exit();
}
