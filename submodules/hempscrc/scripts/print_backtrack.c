#include <stdio.h>
#include <stdlib.h>
#include <string.h>

char backtrack[97];
long back_number;
int direction=-1, last_direction=-1; 
int x,y=0, last_x=0, last_y=0, pos_x=0, pos_y=0;
short hemps[12][12], length = 4;;

void print_hemps();
void print_directions();

int main(int argc, char *argv[]){

	if(argc == 2){
		strcpy(backtrack, argv[1]);
		if(strcmp(backtrack, "--help") == 0 || strcmp(backtrack, "--HELP") == 0){
			printf("\n  print_backtrack <hemps side length> <backtrack word> <initial X> <initial Y>\n");
			printf("  eg: print_backtrack 4 075A 3 0\n");			
			printf("      print_backtrack 5 2FC2 4 1\n");	
		}
		else
			printf("type: print_backtrack --help\n");	
		exit(0);
	}
	if(argc != 5){
		printf("type: print_backtrack --help\n");	
		exit(0);
	}
	length = (int)strtol(argv[1], NULL, 10);  		// copy hemps length (is square: length x length)
	// initialize matrix hemps
	for(x=0;x<length;x++)
		for(y=0;y<length;y++){
		hemps[x][y] = -1;
		
	}	
	strcpy(backtrack, argv[2]);				  		// copy backtrack word 

	back_number = (long)strtol(backtrack, NULL, 16); // convert backtrack word do hexa
	pos_x = (int)strtol(argv[3], NULL, 10);			// copy X initial position
	pos_y = (int)strtol(argv[4], NULL, 10);			// copy Y inicital position

//	printf ("\n%d:%d --> %x\n", pos_x, pos_y, back_number);
		
	for(x = 0; x < (strlen(backtrack))*2; x++){ // each hexa digit have 2 backtrack steps 
		last_direction = direction;
		direction = back_number & 3;				// mask backtrack to use last 2 bits
		if(last_direction == -1)				// if first router add 10 to direction (permit print a "S" (source) on router)
			hemps[pos_x][pos_y] = direction+10;
		else
			hemps[pos_x][pos_y] = direction;	
		
		if(direction == 0){							// if direction = 0 then print "E" (EAST)
			printf("E ");						// and new X position is X+1
			last_x = pos_x; last_y = pos_y;
			pos_x = pos_x+1;
		}
		if(direction == 1){							// if direction = 0 then print "W" (WEST)
			printf("W ");						// and new X position is X-1
			last_x = pos_x; last_y = pos_y;
			pos_x = pos_x-1;			
		}
		if(direction == 2){							// if direction = 0 then print "N" (NORTH)
			printf("N ");						// and new Y position is Y+1
			last_x = pos_x; last_y = pos_y;
			pos_y = pos_y+1;			
		}
		if(direction == 3){							// if direction = 0 then print "S" (SOUTH)
			printf("S ");						// and new Y position is Y-1
			last_x = pos_x; last_y = pos_y;
			pos_y = pos_y-1;			
		}

		// if find inconsistent direction then finish
		if((((last_direction == 0) & (direction == 1)) || ((last_direction == 1) && (direction == 0))) || (((last_direction == 2) & (direction == 3)) || ((last_direction == 3) && (direction == 2))))
			break;
		back_number = back_number >> 2;			// shift backtrack direction to process next step
	}
	hemps[last_x][last_y] = 100+hemps[last_x][last_y];  // // when finish add 100 to direction (permit print a "T" (target) on router)

	printf ("\n");
	print_directions();
    print_hemps();
    printf ("\n");
	return 0;
}

void print_directions(){						// print directions of matrix hemps
    for(y=length-1;y>=0;y--){
	    for(x=0;x<length;x++){
			printf("%3d ", hemps[x][y]);	
	    }
    printf("\n");
	}
}

void print_hemps(){							// prints routers, backtrack path, init router and end router
	int x, y;
	
	for(y=length-1;y>=0;y--){
	  printf("\n");
	  for(x = 0;x<length; x++){
		  if(hemps[x][y] == 2 || hemps[x][y] == 12 || hemps[x][y] == 102)
			printf("    |    ");		  
		  else
			printf("         ");
	  }
	  printf("\n");
	  for(x = 0;x<length; x++)
			printf("   ---   ");
	  printf("\n");
	  for(x = 0;x<length; x++){
		  if(hemps[x][y] == 1)		  
			printf("--|   |  ");
		  else if(hemps[x][y] == 11)
			printf("--| S |  ");
		  else if(hemps[x][y] == 101)
			printf("--| T |  ");			
		  else if(hemps[x][y] == 0)
			printf("  |   |--");	
		  else if(hemps[x][y] == 10)
			printf("  | S |--");
		  else if(hemps[x][y] == 100)
			printf("  | T |--");
		  else if(hemps[x][y] == 12 || hemps[x][y] == 13)
			printf("  | S |  ");	
		  else if(hemps[x][y] == 102 || hemps[x][y] == 103)
			printf("  | T |  ");			
		  else				
			printf("  |   |  ");			
	  }
	  printf("\n");
	  for(x = 0;x<length; x++)	
			printf("   ---   ");
	  printf("\n");			
	  for(x = 0;x<length; x++){
		  if(hemps[x][y] == 3 || hemps[x][y] == 13 || hemps[x][y] == 103)
			printf("    |    ");		  
		  else
			printf("         ");
	  }	  
	}		
}
