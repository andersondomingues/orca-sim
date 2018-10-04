#include <task.h>
#include <stdlib.h>
#include "dtw.h"

Message msg;

/*int test[SIZE][SIZE] = {
		{7200, 4600},
		{1900, 5800}
};*/

int test[SIZE][SIZE] = {
	{0,1,2,3,4,5,6,7,8,9,0},
	{0,1,2,3,4,5,6,7,8,9,0},
	{0,1,2,3,4,5,6,7,8,9,0},
	{0,1,2,3,4,5,6,7,8,9,0},
	{0,1,2,3,4,5,6,7,8,9,0},
	{0,1,2,3,4,5,6,7,8,9,0},
	{0,1,2,3,4,5,6,7,8,9,0},
	{0,1,2,3,4,5,6,7,8,9,0},
	{0,1,2,3,4,5,6,7,8,9,0},
	{0,1,2,3,4,5,6,7,8,9,0},
	{0,1,2,3,4,5,6,7,8,9,0}
};

int main(){

	int i, j, d_count = 0;
	int distances[NUM_PATTERNS];
	char d[50];


	msg.length = SIZE*SIZE; //SIZE*SIZE nao pode ser maior que 128, senao usar o 

	//memcopy(msg.msg, test, sizeof(test));

	//for(i=0; i<TOTAL_TASKS; i++){
		Send(&msg,p1);
		Send(&msg,p2);
		Send(&msg,p3);
		Send(&msg,p4);
		Send(&msg,p5);
		Send(&msg,p6);
		Send(&msg,p7);
		Send(&msg,p8);
	//}

	Echo("Test edd to all tasks\n");

	for(j=0; j<PATTERN_PER_TASK; j++){
        
        Receive(&msg, p1);
        Receive(&msg, p2);
        Receive(&msg, p3);
        Receive(&msg, p4);
        Receive(&msg, p5);
        Receive(&msg, p6);
        Receive(&msg, p7);
        Receive(&msg, p8);
		for(i=0; i<TOTAL_TASKS; i++){
			distances[d_count] = msg.msg[0];
			//sprintf(d, "DTW entre amostra de teste e o padrão %d = %d  TICK = %d", d_count, distances[d_count], GetTick());
			//Echo(d);
			d_count++;
		}
	}

	j = distances[0];

	for(i=1; i<TOTAL_TASKS; i++){
		if(j<distances[i])
			j = distances[i];
	}

	Echo(d);

	return 0;
}
