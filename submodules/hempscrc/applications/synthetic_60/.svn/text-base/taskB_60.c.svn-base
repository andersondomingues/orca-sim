#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{
	
	int i, j,t;

    Echo("synthetic60 task B started.");
	Echo(itoa(GetTick()));
	
for(i=0;i<100;i++){
	for(t=0;t<1000;t++){
	}	
	msg.length = 30;
	for(j=0;j<30;j++) msg.msg[j]=i;

	Send(&msg,taskC_60);
}

    Echo(itoa(GetTick()));
    Echo("synthetic60 task B finished.");
	exit();
}
