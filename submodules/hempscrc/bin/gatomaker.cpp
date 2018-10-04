#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>


using namespace std;


int main(int argc, char *argv[]){
	int found = 0;
	int i =0;
	int location;
	string address;
	int address_i;
	char str[8];
	
	FILE *mem;//input file descriptor
	ifstream dump;//input file descriptor
	
	string str_in;//string to be read
	
	dump.open(argv[1], ios::out);
	mem=fopen(argv[2], "r");

	if (argc != 3){
		printf("syntax: gatomaker *.dump *.mem > *.mem\n");
		return 0;
	}
			
	if(!dump){
        printf("Can't open file %s!\n",argv[1]);
        return 0;
    }
	if(!mem){
		printf("Can't open file %s!\n", argv[2]);
		return 0;
	}
	
	/* Search for the value of <puta> int the first argument (kernel.dump)*/
	while(!found){
		getline(dump,str_in);
		//cout << i << ":" << str_in << endl;
		location=str_in.find(" <puta>",0);
		if(location!=string::npos){
			str_in.find(" <puta>",0);
			address=str_in.substr(0,location);
			address=address.substr(4,location);
			//cout << "found!" << endl;
			address_i = atoi(address.c_str());
			//cout << address << endl;
			//cout << address_i << endl;
			found = 1;
			
		}
		i++;
	}
		
	i=0;
	while(fscanf(mem,"%s",str)!=EOF){
		//fscanf(mem,"%s",str);
		if(i==4){
			//cout << "aqui!";
			printf("b808%s\n",address.c_str());
			//break;
			//return 1;
		}
		else{
			cout<<str<<endl;
		}
		i++;
	}
	
	return 1;
}
	
	
