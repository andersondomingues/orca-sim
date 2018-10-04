/****************************************************************************************** 
*   syntax: bin2mem <filename1.bin  > filename2.mem
*   author: Rene van Leuken
*   modified: Tamar Kranenburg
*   modified: Eduardo Wachter
*   modified: Carlo Lucas
*   February, 2008: header string provided, so ModelSim can recognize the file's format
*                   (= Veriloh hex) when 'Importing' into memory ... (Huib)
*   September, 2008: prevent reversing byte order
*   June, 2010: generate data only
*   September, 2011: Windows don't like this program :(
*
*******************************************************************************************/

#include <stdio.h>

main(int argc, char* argv[])
{
    unsigned char c1, c2, c3, c4;
    
	FILE *fp;
	fp = fopen(argv[1],"rb");
	if(fp==NULL){
		printf("Cannot open %s file",argv[1]);
	}
	
    if (argc < 2){
        printf("// memory data file (do not edit the following line - required for mem load use)\n");
        printf("// format=hex addressradix=h dataradix=h version=1.0 wordsperline=1\n");
        printf("@00000000\n");
    }
    while (!feof(fp)) {
		
		//fread(&c1,1,2,fp);
		//fread(&c2,1,2,fp);
		//fread(&c3,1,2,fp);
		//fread(&c4,1,2,fp);
        c1 = fgetc(fp) & 0x0ff;
        c2 = fgetc(fp) & 0x0ff;
        c3 = fgetc(fp) & 0x0ff;
        c4 = fgetc(fp) & 0x0ff;
        printf ("%.2x%.2x%.2x%.2x\n", c1, c2, c3, c4);
    }
    return 0;
}

