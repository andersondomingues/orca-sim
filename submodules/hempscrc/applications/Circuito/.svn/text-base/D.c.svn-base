#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{

int i, j;

Echo(strcat("b,D,",itoa(GetTick())));

for(i=0;i<10;i++)
{
	for(j=0;j<128;j++) msg.msg[j]=i;

	/*Comm A 100*/
	msg.length=100;
	Send(&msg,A);
	Echo(strcat("s,Circuito_m(100),",itoa(GetTick())));
	/*Comm B 100*/
	msg.length=100;
	Receive(&msg,B);
	Echo(strcat("r,Circuito_m3(100),",itoa(GetTick())));
	/*Comm B 100*/
	msg.length=100;
	Send(&msg,B);
	Echo(strcat("s,Circuito_m5(100),",itoa(GetTick())));
	/*Comm A 100*/
	msg.length=100;
	Receive(&msg,A);
	Echo(strcat("r,Circuito_m6(100),",itoa(GetTick())));
	Echo(strcat(strcat(strcat("i,",itoa(i)),","),itoa(GetTick())));
	
	for(j=0;j<10000;j++);
}

Echo(strcat("e,D,",itoa(GetTick())));

exit();

}
