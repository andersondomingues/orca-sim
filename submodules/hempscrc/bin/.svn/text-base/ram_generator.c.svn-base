#include <stdio.h>
#include <stdlib.h>
#define BUF_SIZE (1024*1024)

void vhd_ram64(unsigned int* code, int count, char *type)
{
    unsigned char init[128][32];
    unsigned char mask;
    int j, i, k, line, index;
		
	printf("library ieee;\n");
	printf("use ieee.std_logic_1164.all;\n");
	printf("use ieee.std_logic_misc.all;\n");
	printf("use ieee.std_logic_arith.all;\n");
	printf("use ieee.std_logic_unsigned.all;\n");
	//printf("use work.mlite_pack.all;\n\n");

	printf("library UNISIM;\n");
	printf("use UNISIM.vcomponents.all;\n\n");

	printf("entity ram_%s is\n",type);
	printf("	port(clk             : in  std_logic;\n");        
	printf("        address_a       : in  std_logic_vector(31 downto 2);\n");
	printf("        enable_a        : in  std_logic;\n");
	printf("        wbe_a           : in  std_logic_vector(3 downto 0);\n");
	printf("        data_write_a    : in  std_logic_vector(31 downto 0);\n");
	printf("        data_read_a     : out std_logic_vector(31 downto 0);\n\n");

	printf("        address_b       : in  std_logic_vector(31 downto 2);\n");
	printf("        enable_b        : in  std_logic;\n");
	printf("        wbe_b           : in  std_logic_vector(3 downto 0);\n");
	printf("        data_write_b    : in  std_logic_vector(31 downto 0);\n");
	printf("        data_read_b     : out std_logic_vector(31 downto 0));\n");
	printf("end; --entity ram     \n\n");
	printf("architecture ram_%s of ram_%s is\n",type,type);
	printf("begin\n");


    for (j=0; j<32; j++)
	    {
	        printf("ram_bit_%d : RAMB16_S1_S1\n",j);
	        printf("generic map (\n");

	        for (k=0; k<64; k++)         // initializes vectors with zeros
	            for (i = 0; i < 32; i++)
	                init[k][i] = 0;

	        for (i=0; i<count; i++)
	        {
	            mask = 1 << (i % 8);          // masks the init bit that should be evaluated
	            line = i / 256;               // init line number being generated
	            index = 31 - (i % 256) / 8;   // init index being generated

	            if (code[i] & (1 << j))       // sets an init bit
	                init[line][index] = init[line][index] | mask;
	        }

	        for (k = 0; k < 64; k++)
	        {
	            printf("INIT_%2.2X => X\"",k);
	            for (i = 0; i < 32; i++)
	                printf("%2.2X",init[k][i]);

	            printf("\"");
	            if (k != 63)
	                printf(",\n");
	        }

	        printf(")\n");
	        printf("port map (\n");

	        printf("CLKA     => clk, 					-- Port A clock input\n");
	        printf("ADDRA    => address_a(15 downto 2), -- Port A address input\n");
	        printf("ENA      => enable_a,               -- Port A enable input\n");
	        printf("WEA      => wbe_a(%d),               -- Port A write enable input\n",j/8);
	        printf("DIA(0)   => data_write_a(%d),        -- Port A data input\n",j);
	        printf("DOA(0)   => data_read_a(%d),         -- Port A data output\n",j);
	        printf("SSRA     => '0',                    -- Port A reset input\n");

	        printf("CLKB     => clk,                    -- Port B clock input\n");
	        printf("ADDRB    => address_b(15 downto 2), -- Port B address input\n");
	        printf("ENB      => enable_b,               -- Port B enable input\n");
	        printf("WEB      => wbe_b(%d),               -- Port B write enable input\n",j/8);
	        printf("DIB(0)   => data_write_b(%d),        -- Port B data input\n",j);
	        printf("DOB(0)   => data_read_b(%d),         -- Port B data output\n",j);
	        printf("SSRB     => '0'                     -- Port B reset input\n");
	        printf(");\n\n\n");
    }
	printf("end;");
}

void vhd_ram128(unsigned int* code, int count, char *type)
{
    unsigned char init[128][32];
    unsigned char mask;
    int j, i, k, line, index;
	
	printf("library ieee;\n");
	printf("use ieee.std_logic_1164.all;\n");
	printf("use ieee.std_logic_misc.all;\n");
	printf("use ieee.std_logic_arith.all;\n");
	printf("use ieee.std_logic_unsigned.all;\n");
	//printf("use work.mlite_pack.all;\n\n");

	printf("library UNISIM;\n");
	printf("use UNISIM.vcomponents.all;\n\n");

	printf("entity ram_%s is\n",type);
	printf("	port(clk             : in  std_logic;\n");        
	printf("        address_a       : in  std_logic_vector(31 downto 2);\n");
	printf("        enable_a        : in  std_logic;\n");
	printf("        wbe_a           : in  std_logic_vector(3 downto 0);\n");
	printf("        data_write_a    : in  std_logic_vector(31 downto 0);\n");
	printf("        data_read_a     : out std_logic_vector(31 downto 0);\n\n");

	printf("        address_b       : in  std_logic_vector(31 downto 2);\n");
	printf("        enable_b        : in  std_logic;\n");
	printf("        wbe_b           : in  std_logic_vector(3 downto 0);\n");
	printf("        data_write_b    : in  std_logic_vector(31 downto 0);\n");
	printf("        data_read_b     : out std_logic_vector(31 downto 0));\n");
	printf("end; --entity ram     \n\n");
	printf("architecture ram_%s of ram_%s is\n",type,type);
	printf("begin\n");
	printf("signal enable_a_lo       : std_logic;\n");
    printf("signal wbe_a_lo          : std_logic_vector(3 downto 0);\n");
    printf("signal data_write_a_lo   : std_logic_vector(31 downto 0);\n");
    printf("signal data_read_a_lo    : std_logic_vector(31 downto 0);\n");
    
    printf("signal enable_b_lo       : std_logic;\n");
    printf("signal wbe_b_lo          : std_logic_vector(3 downto 0);\n");
    printf("signal data_read_b_lo    : std_logic_vector(31 downto 0);\n");
    
    printf("signal enable_a_hi       : std_logic;\n");
    printf("signal wbe_a_hi          : std_logic_vector(3 downto 0);\n");
    printf("signal data_read_a_hi   : std_logic_vector(31 downto 0);\n");
        
    printf("signal enable_b_hi       : std_logic;\n");
    printf("signal wbe_b_hi          : std_logic_vector(3 downto 0);\n");
    printf("signal data_read_b_hi    : std_logic_vector(31 downto 0);\n");
    
    printf("signal address_a_reg     : std_logic_vector(31 downto 2);\n");
    printf("signal address_b_reg     : std_logic_vector(31 downto 2);\n");
    
	printf("begin\n");
	printf("process(clk)\n");
    printf("begin\n");
    printf("if rising_edge(clk) then\n");
    printf("	address_a_reg <= address_a;\n");
    printf("	address_b_reg <= address_b;\n");
    printf("	end if;\n");
    printf("end process;\n");
    
    printf("data_read_a <= data_read_a_lo when (address_a_reg < x\"0001000\"&\"00\") else\n");
    printf("        data_read_a_hi when ((address_a_reg >= x\"0001000\"&\"00\") and (address_a_reg < x\"0002000\"&\"00\"));\n");
    printf("data_read_b <= data_read_b_lo when (address_b_reg < x\"0001000\"&\"00\") else\n");
	printf("            data_read_b_hi when ((address_b_reg >= x\"0001000\"&\"00\") and (address_b_reg < x\"0002000\"&\"00\"));\n");
   
    printf("enable_a_lo <= enable_a when (address_a < x\"0001000\"&\"00\") else '0';\n");
    printf("enable_b_lo <= enable_b when (address_b < x\"0001000\"&\"00\") else '0';\n");
    
    printf("enable_a_hi <= enable_a when ((address_a >= x\"0001000\"&\"00\") and (address_a < x\"0002000\"&\"00\")) else '0';\n");
    printf("enable_b_hi <= enable_b when ((address_b >= x\"0001000\"&\"00\") and (address_b < x\"0002000\"&\"00\")) else '0';\n");
               
    printf("wbe_a_lo <= wbe_a when  enable_a_lo='1' else x\"0\";\n");
    printf("wbe_a_hi <= wbe_a when  enable_a_hi='1' else x\"0\";\n");
    printf("wbe_b_lo <= wbe_b when  enable_b_lo='1' else x\"0\";\n");
	printf("wbe_b_hi <= wbe_b when  enable_b_hi='1' else x\"0\";\n");
    for (j=0; j<32; j++)
    {
        printf("ram_bit_lo_%d : RAMB16_S1_S1\n",j);
        printf("generic map (\n");

        for (k=0; k<64; k++)         // initializes vectors with zeros
            for (i = 0; i < 32; i++)
                init[k][i] = 0;

        for (i=0; i<count; i++)
        {
            mask = 1 << (i % 8);          // masks the init bit that should be evaluated
            line = i / 256;               // init line number being generated
            index = 31 - (i % 256) / 8;   // init index being generated

            if (code[i] & (1 << j))       // sets an init bit
                init[line][index] = init[line][index] | mask;
        }

        for (k = 0; k < 64; k++)
        {
            printf("INIT_%2.2X => X\"",k);
            for (i = 0; i < 32; i++)
                printf("%2.2X",init[k][i]);

            printf("\"");
            if (k != 63)
                printf(",\n");
        }

        printf(")\n");
        printf("port map (\n");

        printf("CLKA     => clk, 					-- Port A clock input\n");
        printf("ADDRA    => address_a(15 downto 2), -- Port A address input\n");
        printf("ENA      => enable_a_lo,               -- Port A enable input\n");
        printf("WEA      => wbe_a_lo(%d),               -- Port A write enable input\n",j/8);
        printf("DIA(0)   => data_write_a(%d),        -- Port A data input\n",j);
        printf("DOA(0)   => data_read_a_lo(%d),         -- Port A data output\n",j);
        printf("SSRA     => '0',                    -- Port A reset input\n");

        printf("CLKB     => clk,                    -- Port B clock input\n");
        printf("ADDRB    => address_b(15 downto 2), -- Port B address input\n");
        printf("ENB      => enable_b_lo,               -- Port B enable input\n");
        printf("WEB      => wbe_b_lo(%d),               -- Port B write enable input\n",j/8);
        printf("DIB(0)   => data_write_b(%d),        -- Port B data input\n",j);
        printf("DOB(0)   => data_read_b_lo(%d),         -- Port B data output\n",j);
        printf("SSRB     => '0'                     -- Port B reset input\n");
        printf(");\n\n\n");
    }




	    for (j=0; j<32; j++)
	    {
	        printf("ram_bit_hi_%d : RAMB16_S1_S1\n",j);
	        printf("generic map (\n");

	        for (k=0; k<64; k++)         // initializes vectors with zeros
	            for (i = 0; i < 32; i++)
	                init[k][i] = 0;

	        for (i=0; i<count; i++)
	        {
	            mask = 1 << (i % 8);          // masks the init bit that should be evaluated
	            line = i / 256;               // init line number being generated
	            index = 31 - (i % 256) / 8;   // init index being generated

	            //if (code[i] & (1 << j))       // sets an init bit
	              //  init[line][index] = init[line][index] | mask;
	        }

	        for (k = 0; k < 64; k++)
	        {
	            printf("INIT_%2.2X => X\"",k);
	            for (i = 0; i < 32; i++)
	                printf("%2.2X",init[k][i]);

	            printf("\"");
	            if (k != 63)
	                printf(",\n");
	        }

	        printf(")\n");
	        printf("port map (\n");

	        printf("CLKA     => clk, 					-- Port A clock input\n");
	        printf("ADDRA    => address_a(15 downto 2), -- Port A address input\n");
	        printf("ENA      => enable_a_hi,               -- Port A enable input\n");
	        printf("WEA      => wbe_a_hi(%d),               -- Port A write enable input\n",j/8);
	        printf("DIA(0)   => data_write_a(%d),        -- Port A data input\n",j);
	        printf("DOA(0)   => data_read_a_hi(%d),         -- Port A data output\n",j);
	        printf("SSRA     => '0',                    -- Port A reset input\n");

	        printf("CLKB     => clk,                    -- Port B clock input\n");
	        printf("ADDRB    => address_b(15 downto 2), -- Port B address input\n");
	        printf("ENB      => enable_b_hi,               -- Port B enable input\n");
	        printf("WEB      => wbe_b_hi(%d),               -- Port B write enable input\n",j/8);
	        printf("DIB(0)   => data_write_b(%d),        -- Port B data input\n",j);
	        printf("DOB(0)   => data_read_b_hi(%d),         -- Port B data output\n",j);
	        printf("SSRB     => '0'                     -- Port B reset input\n");
	        printf(");\n\n\n");
    }
}

void fill_c_model(unsigned int *code, unsigned int code_size, char ram_size, char *type) {

	unsigned int i;

	printf("/*** RAM memory initialized with kernel object code ***/\n\n");

	printf("#include <systemc.h>\n\n");

	printf("#define RAM_SIZE\t%d*1024\n\n",ram_size);

	printf("SC_MODULE(ram_%s) {\n\n",type);

	printf("\tsc_in< bool >\t\t\tclk;\n");

	printf("\tsc_in< sc_uint<30> >\taddress_a;\n");
	printf("\tsc_in< bool >\t\t\tenable_a;\n");
	printf("\tsc_in < sc_uint<4> >\twbe_a;\n");
	printf("\tsc_in < sc_uint<32> >\tdata_write_a;\n");
	printf("\tsc_out < sc_uint<32> >\tdata_read_a;\n\n");

	printf("\tsc_in< sc_uint<30> >\taddress_b;\n");
	printf("\tsc_in< bool >\t\t\tenable_b;\n");
	printf("\tsc_in < sc_uint<4> >\twbe_b;\n");
	printf("\tsc_in < sc_uint<32> >\tdata_write_b;\n");
	printf("\tsc_out < sc_uint<32> >\tdata_read_b;\n\n");

	printf("\tunsigned long ram[RAM_SIZE];\n");
	printf("\tunsigned long byte[4];\n");
	printf("\tunsigned long half_word[2];\n\n");

	printf("\tSC_CTOR(ram_%s) {\n\n",type);

	printf("\t\tSC_METHOD(read_a);\n");
	printf("\t\tsensitive << clk.pos();\n\n");

	printf("\t\tSC_METHOD(write_a);\n");
	printf("\t\tsensitive << clk.pos();\n\n");

	printf("\t\tSC_METHOD(read_b);\n");
	printf("\t\tsensitive << clk.pos();\n\n");

	printf("\t\tSC_METHOD(write_b);\n");
	printf("\t\tsensitive << clk.pos();\n\n");

	printf("\t\t// Byte masks.\n");

	printf("\t\tbyte[0] = 0x000000FF;\n");
	printf("\t\tbyte[1] = 0x0000FF00;\n");
	printf("\t\tbyte[2] = 0x00FF0000;\n");
	printf("\t\tbyte[3] = 0xFF000000;\n\n");

	printf("\t\t// Half word masks.\n");
	printf("\t\thalf_word[0] = 0x0000FFFF;\n");
	printf("\t\thalf_word[1] = 0xFFFF0000;\n\n");

	printf("\t\t// Initializes RAM memory with kernel object code.\n");
	for(i=0; i<code_size; i++)
		printf("\t\tram[0x%X] = 0x%08X;\n",i,code[i]);

	printf("\t}\n\n");

	printf("\t/*** Process functions ***/\n");
	printf("\tvoid read_a();\n");
	printf("\tvoid write_a();\n\n");
	printf("\tvoid read_b();\n");
	printf("\tvoid write_b();\n};\n");

}

void usage()
{
    printf("Usage: ram_generator <-64|-128> <-vhd|-ucf> <model.txt>  <kernel_master.txt> <kernel_slave.txt> <repository.txt>\n\n");
    printf("  model.txt must have the tags $ram_master$ and $ram_slave$ when used\n ");
    printf("  with -vhd option and also $repository$ when used with -ucf \n");
    printf("  option.\n\n");
}

int main(int argc, char* argv[]){

    FILE *file;
    char *buf, *ram;
    unsigned int *code;
    int size;
	int i;

    buf = (char*)malloc(BUF_SIZE);
    code = (unsigned int*) malloc(BUF_SIZE);

    if(argc < 2)
        usage();

	//for(i=2;i<argc;i++){
		file = fopen(argv[3], "r");
		if(file == NULL){
			printf("Can't open file %s!\n",argv[3]);
			exit(-1);
		}

		for(size = 0; size < 16*1024; ++size){
			if(feof(file))
				break;
			fscanf(file, "%x", &code[size]);
		}
		fclose(file);
	
		if (strcmp(argv[1],"-64") == 0){
			if (strcmp(argv[2],"-vhd") == 0){
				if (strcmp(argv[3],"kernel_master.txt") == 0){
					vhd_ram64(code, size, "master");
				}
				else if(strcmp(argv[3],"kernel_mblite.txt") == 0){
					vhd_ram64(code, size, "mblite");
				}
				else if(strcmp(argv[3],"kernel_plasma.txt") == 0){
					vhd_ram64(code, size, "plasma");
				}
			}
			else if(strcmp(argv[2],"-h") == 0){
				if (strcmp(argv[3],"kernel_master.txt") == 0){
					fill_c_model(code, size, 16, "master");
				}
				else if(strcmp(argv[3],"kernel_mblite.txt") == 0){
					fill_c_model(code, size, 16, "mblite");
				}
				else if(strcmp(argv[3],"kernel_plasma.txt") == 0){
					fill_c_model(code, size, 16, "plasma");
				}
			}
		}
		else if (strcmp(argv[1],"-128") == 0){
			if (strcmp(argv[2],"-vhd") == 0){
				if (strcmp(argv[2],"kernel_master.txt") == 0){
					vhd_ram128(code, size, "master");
				}
				else if(strcmp(argv[2],"kernel_mblite.txt") == 0){
					vhd_ram128(code, size, "mblite");
				}
				else if(strcmp(argv[2],"kernel_plasma.txt") == 0){
					vhd_ram128(code, size, "plasma");
				}
			}
			else if(strcmp(argv[2],"-h") == 0){
				if (strcmp(argv[3],"kernel_master.txt") == 0){
					fill_c_model(code, size, 32, "master");
				}
				else if(strcmp(argv[3],"kernel_mblite.txt") == 0){
					fill_c_model(code, size, 32, "mblite");
				}
				else if(strcmp(argv[3],"kernel_plasma.txt") == 0){
					fill_c_model(code, size, 32, "plasma");
				}
			}
		}
		else
			usage();
	//}
    return 0;
}
