#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{

int i, j;

Echo(strcat("b,TC,",itoa(GetTick())));

for(i=0;i<10;i++)
{
	for(j=0;j<128;j++) msg.msg[j]=i;

	/*Comm B 100*/
	msg.length=100;
	Send(&msg,B);
	Echo(strcat("s,Circuito_m2(100),",itoa(GetTick())));
	Echo(strcat(strcat(strcat("i,",itoa(i)),","),itoa(GetTick())));
	
	for(j=0;j<10000;j++);
}

Echo(strcat("e,TC,",itoa(GetTick())));

exit();

}
