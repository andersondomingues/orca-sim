#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{
	
	int i, j,t;

    Echo("disturbing0 task A started.");
	Echo(itoa(GetTick()));

for(i=0;i<10;i++){
	for(t=0;t<1000;t++){
	}
	msg.length = 30;
	for(j=0;j<30;j++) msg.msg[j]=i;
	
	Send(&msg,disturbingtaskB_0);
    Echo("iteration:");
    Echo(itoa(i));
}

    Echo(itoa(GetTick()));
    Echo("disturbing0 task A finished.");
	exit();
}