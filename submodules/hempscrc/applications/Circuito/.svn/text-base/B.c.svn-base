#include <task.h>
#include <stdlib.h>

Message msg;

int main()
{

int i, j;

Echo(strcat("b,B,",itoa(GetTick())));

for(i=0;i<10;i++)
{
	for(j=0;j<128;j++) msg.msg[j]=i;

	/*Comm TC 100*/
	msg.length=100;
	Receive(&msg,TC);
	Echo(strcat("r,Circuito_m2(100),",itoa(GetTick())));
	/*Comm D 100*/
	msg.length=100;
	Send(&msg,D);
	Echo(strcat("s,Circuito_m3(100),",itoa(GetTick())));
	/*Comm A 100*/
	msg.length=100;
	Send(&msg,A);
	Echo(strcat("s,Circuito_m4(100),",itoa(GetTick())));
	/*Comm D 100*/
	msg.length=100;
	Receive(&msg,D);
	Echo(strcat("r,Circuito_m5(100),",itoa(GetTick())));
	Echo(strcat(strcat(strcat("i,",itoa(i)),","),itoa(GetTick())));
}

Echo(strcat("e,B,",itoa(GetTick())));

exit();

}
