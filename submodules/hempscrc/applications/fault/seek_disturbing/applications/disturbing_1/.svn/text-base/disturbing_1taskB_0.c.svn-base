#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{
	
	int i, j,t;

	Echo("disturbing1 task B started.");
	Echo(itoa(GetTick()));

for(i=0;i<100;i++){
	
	msg.length = 30;
	for(j=0;j<30;j++) msg.msg[j]=i;

	Receive(&msg,disturbing_1taskA_0);
	
	for(t=0;t<10000;t++){
	}

}
    Echo(itoa(GetTick()));
    Echo("disturbing1 task B finished.");

	exit();
}
