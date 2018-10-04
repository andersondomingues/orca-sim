#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{
	
	int i, j,t;

	Echo("synthetic56 task C started.");
	Echo(itoa(GetTick()));

for(i=0;i<100;i++){
	
	msg.length = 30;
	for(j=0;j<30;j++) msg.msg[j]=i;

	Receive(&msg,taskA_56);
	
	for(t=0;t<1000;t++){
	}
	
	Send(&msg,taskD_56);
	
	Receive(&msg,taskB_56);
	
	for(t=0;t<1000;t++){
	}
	
	Send(&msg,taskE_56);

}
    Echo(itoa(GetTick()));
    Echo("synthetic56 task C finished.");

	exit();
}
