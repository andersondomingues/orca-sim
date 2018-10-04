#include <task.h>
#include <stdlib.h> 

Message msg;

int main()
{

int i, j;

Echo(strcat("b,A,",itoa(GetTick())));

for(i=0;i<10;i++)
{
	for(j=0;j<128;j++) msg.msg[j]=i;

	/*Comm D 100*/
	msg.length=100;
	Receive(&msg,D);
	Echo(strcat("r,Circuito_m(100),",itoa(GetTick())));
	/*Comm B 100*/
	msg.length=100;
	Receive(&msg,B);
	Echo(strcat("r,Circuito_m4(100),",itoa(GetTick())));
	/*Comm D 100*/
	msg.length=100;
	Send(&msg,D);
	Echo(strcat("s,Circuito_m6(100),",itoa(GetTick())));
	Echo(strcat(strcat(strcat("i,",itoa(i)),","),itoa(GetTick())));
}

Echo(strcat("e,A,",itoa(GetTick())));

exit();

}
