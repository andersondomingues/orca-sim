#include <task.h>
#include <stdlib.h>

#define SIZE	11	//tamanho da matriz
#define TOTAL_TASKS	8	//deve ser PAR para dividir igualmente o numero de padroes por task
#define NUM_PATTERNS	40	//numero de padroes deve ser multiplo de TOTAL_TASKS
#define PATTERN_PER_TASK	NUM_PATTERNS/TOTAL_TASKS

int p[TOTAL_TASKS] = {p1,p2,p3,p4,p5,p6,p7,p8};

int euclideanDistance(int *x, int *y) {
	int ed = 0.0f;
	int aux = 0.0f;
	int i;
	for (i = 0; i < SIZE; i++) {
		aux = x[i] - y[i];
		ed += aux * aux;
	}
	return ed;
}

int min(int x, int y) {
	if (x > y)
		return y;
	return x;
}

int dynamicTimeWarping(int x[SIZE][SIZE], int y[SIZE][SIZE]) {
	int lastCol[SIZE];
	int currCol[SIZE];
	int temp[SIZE];
	int maxI = SIZE - 1;
	int maxJ = SIZE - 1;
	int minGlobalCost;
	int i, j;

	currCol[0] = euclideanDistance(x[0], y[0]);
	for (j = 1; j <= maxJ; j++) {
		currCol[j] = currCol[j - 1] + euclideanDistance(x[0], y[j]);
	}

	for (i = 1; i <= maxI; i++) {

		//memcpy(temp, lastCol, sizeof(lastCol));
		//memcpy(lastCol,currCol, sizeof(lastCol));
		//memcpy(currCol,currCol, sizeof(lastCol));

		currCol[0] = lastCol[0] + euclideanDistance(x[i], y[0]);

		for (j = 1; j <= maxJ; j++) {
			minGlobalCost = min(lastCol[j], min(lastCol[j - 1], currCol[j - 1]));
			currCol[j] = minGlobalCost + euclideanDistance(x[i], y[j]);
		}
	}

	return currCol[maxJ];
}

