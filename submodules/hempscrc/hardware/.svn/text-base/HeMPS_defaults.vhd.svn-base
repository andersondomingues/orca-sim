------------------------------------------------------------------
---
--- HEMPS 4.0  - October 25 2011
---
--- File function: defines the Hermes constants and auxiliary functions 
---
--- Responsibles: Eduardo Wachter, Marcelo Mandelli, Carlo Lucas, Fernando Moraes
--- 
--- Contact: fernando.moraes@pucrs.br
---
------------------------------------------------------------------
library IEEE;
use IEEE.Std_Logic_1164.all;
use IEEE.std_logic_unsigned.all;
use IEEE.std_logic_arith.all;
use work.HeMPS_PKG.all;

package HeMPS_defaults is

--------------------------------------------------------
-- HERMES CONSTANTS
--------------------------------------------------------

--------------------------------------------------------------------------------
-- Router position constants - FIXED - it is not a function of the NoC size
--------------------------------------------------------------------------------
        constant BL: integer := 0;
        constant BC: integer := 1;
        constant BR: integer := 2;
        constant CL: integer := 3;
        constant CC: integer := 4;
        constant CRX: integer := 5; 
        constant TL: integer := 6;
        constant TC: integer := 7;
        constant TR: integer := 8;
        
-------------------------------------------------------------------------
-- 	Header flit types
-----------------------------------------------------------------------
	constant PACKET_SWITCHING			: std_logic_vector(3 downto 0) := x"0";
    --header type for seek module
	constant BACKTRACK_PATH     		: std_logic_vector(3 downto 0) := x"6";
	constant SOURCE_ROUTING     		: std_logic_vector(3 downto 0) := x"7";
    
---------------------------------------------------------
-- CONSTANTS INDEPENDENTES
---------------------------------------------------------
        --constant NPORT: integer := 5;

        --constant EAST: integer := 0;
        --constant WEST: integer := 1;
        --constant NORTH : integer := 2;
        --constant SOUTH : integer := 3;
        --constant LOCAL : integer := 4;
        
        constant NPORT  : integer := 10; 

        constant EAST0  : integer := 0;
        constant EAST1  : integer := 1;
        constant WEST0  : integer := 2;
        constant WEST1  : integer := 3;
        constant NORTH0 : integer := 4;
        constant NORTH1 : integer := 5;
        constant SOUTH0 : integer := 6;
        constant SOUTH1 : integer := 7;
        constant LOCAL0 : integer := 8;
        constant LOCAL1 : integer := 9;
        
        constant SEEK_PORT : integer := LOCAL1;
        
---------------------------------------------------------
-- CONSTANT DEPENDENTE DA LARGURA DE BANDA DA REDE - FIXED FOR HEMPS
---------------------------------------------------------
        constant TAM_FLIT : integer range 1 to 64 := 20; -- fochi
        constant TAM_FLITwoCRC : integer range 1 to 64 := 16; -- AUGUSTO
        constant METADEFLIT : integer range 1 to 32 := (TAM_FLIT/2)-2;-- fochi
        constant QUARTOFLIT : integer range 1 to 16 := (TAM_FLIT/4)-1;-- fochi
	
        constant FLIT_WIDTH : integer range 1 to 32 := TAM_FLIT;
        constant FLIT_WIDTHwoCRC : integer range 1 to 32 := TAM_FLITwoCRC;
        constant HALF_FLIT : integer range 1 to 16 := (FLIT_WIDTH/2)-2;-- fochi
        constant QUARTER_FLIT : integer range 1 to 16 := (FLIT_WIDTH/4)-1;-- fochi
---------------------------------------------------------
-- CONSTANTS DEPENDENTES DA PROFUNDIDADE DA FILA
---------------------------------------------------------
        constant TAM_BUFFER: integer := FLIT_WIDTH; --fochi
        constant TAM_POINTER : integer range 1 to 32 := 4;
        constant BUFFER_DEEP: integer := 16;
        constant POINTER_SIZE : integer range 1 to 32 := TAM_POINTER;

---------------------------------------------------------
-- CONSTANTS DEPENDENTES DO NUMERO DE ROTEADORES
---------------------------------------------------------
        constant NROT: integer := NUMBER_PROCESSORS;

        constant MIN_X : integer := 0;
        constant MIN_Y : integer := 0;
        constant MAX_X : integer := NUMBER_PROCESSORS_X-1;
        constant MAX_Y : integer := NUMBER_PROCESSORS_Y-1;

---------------------------------------------------------
-- CONSTANT TB
---------------------------------------------------------
        constant TAM_LINHA : integer := 2; --4;

---------------------------------------------------------
-- SUBTIPOS, TIPOS E FUNCOES
---------------------------------------------------------

        subtype reg3 is std_logic_vector(2 downto 0);
        subtype reg8 is std_logic_vector(7 downto 0);
        subtype reg32 is std_logic_vector(31 downto 0); 
        subtype regNrot is std_logic_vector((NROT-1) downto 0);
        subtype regNport is std_logic_vector((NPORT-1) downto 0); 
        subtype regflit is std_logic_vector((TAM_FLIT-1) downto 0); 
        subtype regflitWoCRC is std_logic_vector((TAM_FLITwoCRC-1) downto 0);  --AUGUSTO
--        subtype regmetadeflit is std_logic_vector(((TAM_FLIT/2)-1) downto 0);METADEFLIT
		subtype regmetadeflit is std_logic_vector((METADEFLIT-1) downto 0);
        --subtype regquartoflit is std_logic_vector((QUARTOFLIT-1) downto 0);
        subtype regquartoflit is std_logic_vector((QUARTOFLIT-2) downto 0);
        subtype pointer is std_logic_vector((TAM_POINTER-1) downto 0);

        type buff is array(0 to TAM_BUFFER-1) of regflit;

        type arrayNport_reg3 is array((NPORT-1) downto 0) of reg3;
        type arrayNport_reg8 is array((NPORT-1) downto 0) of reg8;
        type arrayNport_regflit is array((NPORT-1) downto 0) of regflit; 
        type arrayNrot_reg3 is array((NROT-1) downto 0) of reg3;
        type arrayNrot_regflit is array((NROT-1) downto 0) of regflit;
        type arrayNrot_regmetadeflit is array((NROT-1) downto 0) of regmetadeflit; 

        -- number of ports of the processing_element - 4 - north/south/west/east
        type arrayNPORT_1_regflit is array(3 downto 0) of regflit;
        
        
        type matrixNportNport_std_logic is array((NPORT-1) downto 0) of std_logic_vector((NPORT-1) downto 0);  
        subtype regQuarterFlit is std_logic_vector(((QUARTER_FLIT)-1) downto 0);--fochi
        subtype regHalfFlit is std_logic_vector(((HALF_FLIT)-1) downto 0); --fochi
        
        subtype regNport_neighbor is std_logic_vector((NPORT-3) downto 0);
                
        type arrayNport_regflit_neighbor is array(NPORT-3 downto 0) of regflit;
        
        

        
---------------------------------------------------------
-- seek constants
---------------------------------------------------------

    constant NPORT_SEEK                 : integer := 5;
    constant SEEK_BIT_SIZE              : integer := 2;
    
    constant SEEK                       : std_logic_vector := "01";
    constant TABLE_SEEK_LENGHT  			: integer := 8;
    
    -- constant SEEK_UNREACHABLE

    constant SOURCE_TARGET_SIZE         : integer := HALF_FLIT;
    constant HOP_SIZE                   : integer := 6;
    constant TABLE_HEIGHT               : integer := 8;
    constant SEEK_HIGH                  : integer := 2;
    constant SEEK_CODE			        : std_logic_vector(1 downto 0) := "01";
    constant SEEK_UNREACHABLE_CODE	    : std_logic_vector(1 downto 0) := "10";
    constant SEEK_RESEND_CODE		    : std_logic_vector(1 downto 0) := "11";
    constant NOT_SEEK			        : std_logic_vector(1 downto 0) := "00";
    
    constant EAST                       : integer := 0;
    constant WEST                       : integer := 1;
    constant NORTH                      : integer := 2;
    constant SOUTH                      : integer := 3;
    constant LOCAL                      : integer := 4;
    
     constant ROW0  : integer := 0;
     constant ROW1  : integer := 1;
     constant ROW2  : integer := 2;
     constant ROW3  : integer := 3;
     constant ROW4  : integer := 4;
     constant ROW5  : integer := 5;
     constant ROW6  : integer := 6;
     constant ROW7  : integer := 7;
    
    
    constant LUTRAM_WIDTH               : integer := 9;
    
---------------------------------------------------------
-- router_seek fochi / caimi
---------------------------------------------------------   
--SEEK_PATH          -- serviço que busca um novo caminho
--TARGET_UNREACHABLE -- falha no enlace avisa a origem que é necessário iniciar a busca por um novo caminho
--PACKET_RESEND      -- solicita o reenvio do pacote
--CLEAR              -- limpa as tabelas daquele par origem e destino
--BACKTRACK          -- caminho de volta do novo caminho encontrado 
    
    constant TAM_SERVICE_SEEK					: integer := 3; 
    constant PACKET_RESEND_SERVICE			    : std_logic_vector(TAM_SERVICE_SEEK-1 downto 0) := "001";
    constant TARGET_UNREACHABLE_SERVICE			: std_logic_vector(TAM_SERVICE_SEEK-1 downto 0) := "010";
    constant CLEAR_SERVICE			        	: std_logic_vector(TAM_SERVICE_SEEK-1 downto 0) := "011";
    constant BACKTRACK_SERVICE			        : std_logic_vector(TAM_SERVICE_SEEK-1 downto 0) := "100";
    constant SEARCHPATH_SERVICE			        : std_logic_vector(TAM_SERVICE_SEEK-1 downto 0) := "101";
---------------------------------------------------------
-- fault-tolerant constants
---------------------------------------------------------    
    
    constant MAX_NI_THRESHOLD           : std_logic_vector := x"00002000";
    
---------------------------------------------------------
-- seek types 
---------------------------------------------------------    

--subtype	 new_seek_service     					 is std_logic_vector(NPORT_SEEK-1 downto 0);


--subtype     regNport_seek_neighbor              is std_logic_vector((NPORT_SEEK-2) downto 0);
--fochi

		

-----------------

    type T_search_choice        is (EAST_DIR,WEST_DIR,NORTH_DIR,SOUTH_DIR);
    
    type slot_table is record
        used        : std_logic;
        source      : std_logic_vector(SOURCE_TARGET_SIZE-1 downto 0);
        target      : std_logic_vector(SOURCE_TARGET_SIZE-1 downto 0);
        port_dir    : T_search_choice;
        hop_count   : std_logic_vector(HOP_SIZE-1 downto 0);
    end record;
    
    --<used/unused><S><T><port><hop_counter>
    --< 1 bit     ><8><8>< 2  ><    6      >
    type        table                   is array (0 to TABLE_HEIGHT-1) of  slot_table;
    
    --types for each router
    subtype     regNport_seek                       is std_logic_vector((NPORT_SEEK-1) downto 0);
    subtype	 regNBit_seek			     is std_logic_vector(SEEK_BIT_SIZE-1 downto 0);
    
    type	 regNport_seek_NBit      	     is array (NPORT_SEEK-1 downto 0) of regNBit_seek;
    type	 regNrot_N_Seek_Bit 		     is array (NROT-1 downto 0) of regNBit_seek;
    subtype     regNsource_target                   is std_logic_vector((SOURCE_TARGET_SIZE-1) downto 0);
    subtype     regNhop                             is std_logic_vector((HOP_SIZE-1) downto 0);
    type        regNportNhop                        is array (0 to NPORT_SEEK-1) of regNhop;
    type        regNportNsource_target              is array (0 to NPORT_SEEK-1) of regNsource_target;
    
    --types for each noc seek
    type        regNrotNhop                         is array (0 to NROT-1) of regNhop;
    type        regNrotNsource_target               is array (0 to NROT-1) of regNsource_target;
    
    subtype     regNport_seek_neighbor              is std_logic_vector((NPORT_SEEK-2) downto 0);
    type	 regNport_seek_neighbor_NBit	     is array ((NPORT_SEEK-2) downto 0) of regNBit_seek;
    type        regNportNsource_target_neighbor     is array (0 to NPORT_SEEK-2) of regNsource_target;
    type        regNportNhop_neighbor               is array (0 to NPORT_SEEK-2) of regNhop;
    
    
    -- new seek
    
    type service_req_seek is array (NUMBER_PROCESSORS-1 downto 0) of std_logic_vector(4 downto 0);
    type service_ack_seek is array (NUMBER_PROCESSORS-1 downto 0) of std_logic_vector(4 downto 0);
	type service_target_source is array (NUMBER_PROCESSORS-1 downto 0) of regNportNsource_target ;
	    
    type        regNportNsource_target_neighbor_new     is array (0 to NPORT_SEEK-1) of regNsource_target;
    type        regNportNhop_neighbor_new               is array (0 to NPORT_SEEK-1) of regNhop;
    subtype	 seek_bitN_service    					 	is std_logic_vector(TAM_SERVICE_SEEK-1 downto 0);
    type	 regNport_seek_bitN_service      	    	 is array (NPORT_SEEK-1 downto 0) of seek_bitN_service;
    
    type regNportNsource_target_neighbor_Nrot_new is array(NUMBER_PROCESSORS-1 downto 0) of regNportNsource_target_neighbor_new;
    type regNportNhop_neighbor_Nrot_new is array(NUMBER_PROCESSORS-1 downto 0) of regNportNhop_neighbor_new;
    type regNportNservice_Nrot_new   is array(NUMBER_PROCESSORS-1 downto 0) of regNport_seek_bitN_service;
    
    
	
    
    --type for out of mux from buffer to crossbar
    type arrayNhalf_port_regflit is array(((NPORT-1)/2) downto 0) of regflit;
    
    --types used for seek manager
    type regNport_neighborNsource_target_neighbor is array (0 to NPORT-3) of regNsource_target;
    
    type regNport_regNBit_seek is array (0 to NPORT-3) of regNBit_seek;    
    type regNport_neighborNsource_target_neighbor_regNBit_seek is array (0 to NPORT-3) of regNport_regNBit_seek;
    
---------------------------------------------------------
-- HERMES FUCTIONS 
---------------------------------------------------------
        Function CONV_VECTOR( int: integer ) return std_logic_vector;
        function CONV_VECTOR( letra : string(1 to TAM_LINHA);  pos: integer ) return std_logic_vector;
        function CONV_HEX( int : integer ) return string;
        function CONV_STRING_4BITS( dado : std_logic_vector(3 downto 0)) return string; 
        function CONV_STRING_8BITS( dado : std_logic_vector(7 downto 0)) return string; 
        function CONV_STRING_16BITS( dado : std_logic_vector(15 downto 0)) return string;
        function CONV_STRING_32BITS( dado : std_logic_vector(31 downto 0)) return string;

---------------------------------------------------------
-- HEMPS FUCTIONS
---------------------------------------------------------
        function RouterPosition(router: integer) return integer;
        function RouterAddress(router: integer) return std_logic_vector; 
        function log_filename(i: integer) return string;

end HeMPS_defaults;

package body HeMPS_defaults is 
        --
        -- converte um inteiro em um std_logic_vector(2 downto 0) 
        --
        function CONV_VECTOR( int: integer ) return std_logic_vector is
                variable bin: reg3;
        begin 
                case(int) is
                        when 0 => bin := "000";
                        when 1 => bin := "001";
                        when 2 => bin := "010";
                        when 3 => bin := "011";
                        when 4 => bin := "100";
                        when 5 => bin := "101";
                        when 6 => bin := "110";
                        when 7 => bin := "111";
                        when others => bin := "000";
                end case;
                return bin; 
        end CONV_VECTOR;
        --------------------------------------------------------- 
        -- FUNCOES TB
        --------------------------------------------------------- 
        --
        -- converte um caracter de uma dada linha em um std_logic_vector 
        --
        function CONV_VECTOR( letra:string(1 to TAM_LINHA);pos: integer ) return std_logic_vector is
                variable bin: std_logic_vector(3 downto 0); 
        begin 
                case (letra(pos)) is
                        when '0' => bin := "0000";
                        when '1' => bin := "0001";
                        when '2' => bin := "0010";
                        when '3' => bin := "0011";
                        when '4' => bin := "0100";
                        when '5' => bin := "0101";
                        when '6' => bin := "0110";
                        when '7' => bin := "0111";
                        when '8' => bin := "1000";
                        when '9' => bin := "1001";
                        when 'A' => bin := "1010";
                        when 'B' => bin := "1011";
                        when 'C' => bin := "1100";
                        when 'D' => bin := "1101";
                        when 'E' => bin := "1110";
                        when 'F' => bin := "1111";
                        when others =>bin := "0000";
                end case;
                return bin; 
        end CONV_VECTOR;

-- converte um inteiro em um string 
        function CONV_HEX( int: integer ) return string is
                variable str: string(1 to 1);
        begin 
                case(int) is
                        when 0 => str := "0";
                        when 1 => str := "1";
                        when 2 => str := "2";
                        when 3 => str := "3";
                        when 4 => str := "4";
                        when 5 => str := "5";
                        when 6 => str := "6";
                        when 7 => str := "7";
                        when 8 => str := "8";
                        when 9 => str := "9";
                        when 10 => str := "A";
                        when 11 => str := "B";
                        when 12 => str := "C";
                        when 13 => str := "D";
                        when 14 => str := "E";
                        when 15 => str := "F";
                        when others =>str := "U";
                end case;
                return str; 
        end CONV_HEX;

        function CONV_STRING_4BITS(dado : std_logic_vector(3 downto 0)) return string is
                variable str: string(1 to 1);
        begin 
                str := CONV_HEX(CONV_INTEGER(dado));
                return str; 
        end CONV_STRING_4BITS; 

        function CONV_STRING_8BITS(dado : std_logic_vector(7 downto 0)) return string is
                variable str1,str2: string(1 to 1);
                variable str: string(1 to 2);
        begin 
                str1 := CONV_STRING_4BITS(dado(7 downto 4));
                str2 := CONV_STRING_4BITS(dado(3 downto 0));
                str := str1 & str2;
                return str; 
        end CONV_STRING_8BITS; 

        function CONV_STRING_16BITS(dado : std_logic_vector(15 downto 0)) return string is
                variable str1,str2: string(1 to 2);
                variable str: string(1 to 4);
        begin 
                str1 := CONV_STRING_8BITS(dado(15 downto 8));
                str2 := CONV_STRING_8BITS(dado(7 downto 0));
                str := str1 & str2;
                return str; 
        end CONV_STRING_16BITS;

        function CONV_STRING_32BITS(dado : std_logic_vector(31 downto 0)) return string is
                variable str1,str2: string(1 to 4);
                variable str: string(1 to 8);
        begin 
                str1 := CONV_STRING_16BITS(dado(31 downto 16));
                str2 := CONV_STRING_16BITS(dado(15 downto 0));
                str := str1 & str2;
                return str; 
        end CONV_STRING_32BITS;

                -- Returns the router position in the mesh
        -- BR: Botton Right
        -- BL: Botton Left
        -- TR: Top Right
        -- TL: Top Left 
        -- CRX: Center Right 
        -- CL: Center Left
        -- CC: Center
        -- 4x4 positions exemple
        --              TL TC TC TR
        --              CL CC CC CRX 
        --              CL CC CC CRX 
        --              BL BC BC BR
        function RouterPosition(router: integer) return integer is
                variable pos: integer range 0 to TR;
                variable line, column: integer;
                begin
                        
                        --line := router/NUMBER_PROCESSORS_Y;
                        column := router mod NUMBER_PROCESSORS_X;
                        
                        if router >= NUMBER_PROCESSORS-NUMBER_PROCESSORS_X then --TOP 
                                if column = NUMBER_PROCESSORS_X-1 then --RIGHT
                                        pos := TR;
                                elsif column = 0 then--LEFT
                                        pos := TL;
                                else--CENTER_X
                                        pos := TC;
                                end if;
                        elsif router < NUMBER_PROCESSORS_X then --BOTTOM
                                if column = NUMBER_PROCESSORS_X-1 then --RIGHT
                                        pos := BR;
                                elsif column = 0 then--LEFT
                                        pos := BL;
                                else--CENTER_X
                                        pos := BC;
                                end if;
                        else --CENTER_Y
                                if column = NUMBER_PROCESSORS_X-1 then --RIGHT
                                        pos := CRX; 
                                elsif column = 0 then--LEFT
                                        pos := CL;
                                else--CENTER_X
                                        pos := CC;
                                end if;
                        end if; 
                        
                        return pos;
                        
        end RouterPosition;

        function RouterAddress(router: integer) return std_logic_vector is
                variable pos_x, pos_y   : std_logic_vector(3 downto 0); 
                variable addr                   : std_logic_vector(7 downto 0); 
                variable aux                    : integer;
        begin 
                aux := (router/NUMBER_PROCESSORS_X);
                pos_x := conv_std_logic_vector((router mod NUMBER_PROCESSORS_X),4);
                pos_y := conv_std_logic_vector(aux,4); 
                
                addr := pos_x & pos_y;
                return addr;
        end RouterAddress;
        
        function log_filename(i: integer) return string is
                variable filename               : string(1 to 19);
                variable aux_x                  : integer;
                variable aux_y                  : integer;
        begin
                aux_x := (i mod NUMBER_PROCESSORS_X);
                aux_y := (i/NUMBER_PROCESSORS_X);
                filename := "log/log_file" & CONV_HEX(aux_x) & "x" & CONV_HEX(aux_y)  & ".txt";
                return filename;
        end log_filename;
end HeMPS_defaults;
