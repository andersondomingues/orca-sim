#include <task.h>
#include <stdlib.h>
#include "dtw.h"


/*int pattern[SIZE][SIZE] = {
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
};*/

void randPattern(int in[SIZE][SIZE]){
	int i,j;

	for(i=0; i<SIZE; i++){
		for(j=0; j<SIZE; j++){
			in[i][j] = abs(5);
		}
	}
}


Message msg;

int main(){

	int i, j;
	int pattern[SIZE][SIZE];
	char a[50];


	msg.length = SIZE * SIZE; //SIZE*SIZE nao pode ser maior que 128, senao usar o Data

	for(j=0; j<PATTERN_PER_TASK; j++){
			randPattern(pattern); //gera uma matriz de valores aleatorios, poderiam ser coeficientes MFCC
			Send(&msg, p1);
			randPattern(pattern); //gera uma matriz de valores aleatorios, poderiam ser coeficientes MFCC
			Send(&msg, p2);
			randPattern(pattern); //gera uma matriz de valores aleatorios, poderiam ser coeficientes MFCC
			Send(&msg, p3);
			randPattern(pattern); //gera uma matriz de valores aleatorios, poderiam ser coeficientes MFCC
			Send(&msg, p4);
			randPattern(pattern); //gera uma matriz de valores aleatorios, poderiam ser coeficientes MFCC
			Send(&msg, p5);
			randPattern(pattern); //gera uma matriz de valores aleatorios, poderiam ser coeficientes MFCC
			Send(&msg, p6);
			randPattern(pattern); //gera uma matriz de valores aleatorios, poderiam ser coeficientes MFCC
			Send(&msg, p7);
			randPattern(pattern); //gera uma matriz de valores aleatorios, poderiam ser coeficientes MFCC
			Send(&msg, p8);
	}

	Echo("Bank edd all patterns\n");

	return 0;
}
