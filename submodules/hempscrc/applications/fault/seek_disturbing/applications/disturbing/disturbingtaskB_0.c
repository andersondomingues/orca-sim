#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{
	
	int i, j,t;

	Echo("disturbing0 task B started.");
	Echo(itoa(GetTick()));

for(i=0;i<10;i++){
	
	msg.length = 30;
	for(j=0;j<30;j++) msg.msg[j]=i;

	Receive(&msg,disturbingtaskA_0);
	
	for(t=0;t<10000;t++){
	}

}
    Echo(itoa(GetTick()));
    Echo("disturbing0 task B finished.");

	exit();
}
