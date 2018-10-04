#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{
	
	int i, j;

    Echo("communication74 task A started.");
	Echo(itoa(GetTick()));

for(i=0;i<50;i++){
	
	msg.length = 30;
	for(j=0;j<30;j++) msg.msg[j]=i;
	
	Send(&msg,taskC_74);
}

    Echo(itoa(GetTick()));
    Echo("communication74 task A finished.");
	exit();
}
