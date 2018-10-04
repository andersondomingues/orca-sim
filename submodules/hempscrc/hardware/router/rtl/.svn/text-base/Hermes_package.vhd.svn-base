---------------------------------------------------------
-- Basic types package
---------------------------------------------------------
library IEEE;
use IEEE.Std_Logic_1164.all;
use IEEE.std_logic_unsigned.all;
use IEEE.std_logic_arith.all;
use work.cfg_pkg.all;

package HermesPackage is

---------------------------------------------------------
-- NoC dimensions (minimum size supported: 2x2)
---------------------------------------------------------
	constant NROT: integer := NROT_X * NROT_Y;
	constant NUMBER_PROCESSORS: integer := NROT;
	constant NUMBER_PROCESSORS_X: integer := NROT_X;
	constant NUMBER_PROCESSORS_Y: integer := NROT_Y;
    
--------------------------------------------------------- 
-- Routers IDs
--------------------------------------------------------- 	      
	--constant R0: integer := 0;
	--constant R1: integer := 1;
	--constant R2: integer := 2;
	--constant R3: integer := 3;
	--constant R4: integer := 4;
	--constant R5: integer := 5;
	--constant R6: integer := 6;
	--constant R7: integer := 7;
	--constant R8: integer := 8;
	--constant R9: integer := 9;
	--constant RA: integer := 10;
	--constant RB: integer := 11;
	--constant RC: integer := 12;
	--constant RD: integer := 13;
	--constant RE: integer := 14;
	--constant RF: integer := 15;

---------------------------------------------------------
-- Flit width constants
---------------------------------------------------------
	constant FLIT_WIDTH : integer range 1 to 32 := 16;
	constant HALF_FLIT : integer range 1 to 16 := (FLIT_WIDTH/2);
	constant QUARTER_FLIT : integer range 1 to 16 := (FLIT_WIDTH/4);

-----------------------------------------------------------------------
-- 	Header structure
-----------------------------------------------------------------------
--
--	|  4 bits   |   4 bits    |         8 bits          |
--	+-----------+-----+---+---+-------------------------+
--	| Flit Type |UN   | R | P |         Target          |
--	|   (15-12) | USED|(9)|(8)|          (7-0)          |
--	+-----------+-----+---+---+-------------------------+
--
--	R: Routing type (ADAPT: Adaptive/DETER: Deterministic)
-- 	P: Priority (HIGH/LOW)

-------------------------------------------------------------------------
-- 	Header flit types
-----------------------------------------------------------------------
	constant PACKET_SWITCHING			: std_logic_vector(3 downto 0) := x"0";
	--constant CONNECTION_ESTABLISHMENT 	: std_logic_vector(3 downto 0) := x"1";	
	--constant CIRCUIT_SWITCHING			: std_logic_vector(3 downto 0) := x"2";
	--constant CONNECTION_RELEASE			: std_logic_vector(3 downto 0) := x"3";
	--constant MULTICAST_DEST				: std_logic_vector(3 downto 0) := x"4";	
	--constant LAST_MULTICAST_DEST		: std_logic_vector(3 downto 0) := x"5";
    --header type for seek module
	constant BACKTRACK_PATH     		: std_logic_vector(3 downto 0) := x"6";
	constant SOURCE_ROUTING     		: std_logic_vector(3 downto 0) := x"7";
	
-----------------------------------------------------------------------
-- 	Channels priorities (header flit bit P)
-----------------------------------------------------------------------
	--constant PRIORITY_BIT	: integer := HALF_FLIT;
	constant HIGH			: std_logic := '0';	
	constant LOW			: std_logic := '1';
	
-----------------------------------------------------------------------
-- 	Routing type (header flit bit R)
-----------------------------------------------------------------------	
	--constant ROUTING_BIT	: integer := HALF_FLIT+1;
	--constant ADAPT			: std_logic := '0';
	--constant DETER			: std_logic := '1';
	
---------------------------------------------------------
-- Router ports constants
---------------------------------------------------------
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
-- Input buffer constants
---------------------------------------------------------
	constant BUFFER_DEEP: integer := 16; 
    constant POINTER_SIZE : integer range 1 to 32 := 4;	
    
--------------------------------------------------------
-- Router position constants
--------------------------------------------------------
	constant BL: integer := 0;
	constant BC: integer := 1;
	constant BR: integer := 2;
	constant CL: integer := 3;
	constant CC: integer := 4;
	constant CRX: integer := 5;
	constant TL: integer := 6;
	constant TC: integer := 7;
	constant TR: integer := 8;
	
--------------------------------------------------------
-- Types, subtypes and funtions
---------------------------------------------------------     
    subtype regNrot is std_logic_vector((NROT-1) downto 0);
    subtype regNport is std_logic_vector((NPORT-1) downto 0);
    subtype regflit is std_logic_vector((FLIT_WIDTH-1) downto 0);
    subtype regHalfFlit is std_logic_vector(((FLIT_WIDTH/2)-1) downto 0);  
    subtype regQuarterFlit is std_logic_vector(((FLIT_WIDTH/4)-1) downto 0);  
    subtype pointer is std_logic_vector((POINTER_SIZE-1) downto 0);

    type buff is array(0 to BUFFER_DEEP-1) of regflit;	

    type arrayNport_regflit is array((NPORT-1) downto 0) of regflit;
    type arrayNrot_regflit is array((NROT-1) downto 0) of regflit;	  
    type matrixNportNport_std_logic is array((NPORT-1) downto 0) of std_logic_vector((NPORT-1) downto 0);  
	
	type arrayNport_data_bus is array (0 to NROT-1) of  arrayNport_regflit;
	type arrayNport_bit is array (0 to NROT-1) of regNport;
    
---------------------------------------------------------
-- seek constants
---------------------------------------------------------
    constant NPORT_SEEK                 : integer := 5;
    constant SOURCE_TARGET_SIZE         : integer := HALF_FLIT;
    constant HOP_SIZE                   : integer := 6;
    constant TABLE_HEIGHT               : integer := 4;
    constant SEEK_HIGH                  : integer := 2;
    
    constant EAST                       : integer := 0;
    constant WEST                       : integer := 1;
    constant NORTH                      : integer := 2;
    constant SOUTH                      : integer := 3;
    constant LOCAL                      : integer := 4;
    
    constant LUTRAM_WIDTH               : integer := 9;
    
---------------------------------------------------------
-- seek types
---------------------------------------------------------    
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
    subtype     regNport_seek           is std_logic_vector((NPORT_SEEK-1) downto 0);
    subtype     regNsource_target       is std_logic_vector((SOURCE_TARGET_SIZE-1) downto 0);
    subtype     regNhop                 is std_logic_vector((HOP_SIZE-1) downto 0);
    type        regNportNhop            is array (0 to NPORT_SEEK-1) of std_logic_vector((HOP_SIZE-1) downto 0);
    type        regNportNsource_target  is array (0 to NPORT_SEEK-1) of std_logic_vector((SOURCE_TARGET_SIZE-1) downto 0);
    
    --types for each noc seek
    type        regNrotNhop             is array (0 to NROT-1) of std_logic_vector((HOP_SIZE-1) downto 0);
    type        regNrotNsource_target   is array (0 to NROT-1) of std_logic_vector((SOURCE_TARGET_SIZE-1) downto 0);
    
    --type for out of mux from buffer to crossbar
    type arrayNhalf_port_regflit is array(((NPORT-1)/2) downto 0) of regflit;
    
	
	--function Neighbour(router: integer; direction: integer) return integer;
	function RouterPosition(router: integer) return integer;
    function RouterAddress(router: integer) return std_logic_vector; 
    function to_string(sv: Std_Logic_Vector) return string;
    function CONV_HEX( int: integer ) return string;
    function CONV_STRING_4BITS(dado : std_logic_vector(3 downto 0)) return string;
    function CONV_STRING_8BITS(dado : std_logic_vector(7 downto 0)) return string;
    
end HermesPackage; 

package body HermesPackage is

	
	-- Returns the neighbour router ID in 'direction'
	--function Neighbour(router: integer; direction: integer) return integer is
	--variable r: integer;
	--begin
		--case direction is
			--when NORTH0 =>
				--r := router + NUMBER_PROCESSORS_X;
			--when SOUTH0 =>
				--r := router - NUMBER_PROCESSORS_X;
			--when EAST0 =>
                --r := router + 1;
			--when WEST0 =>
                --r := router - 1;
			--when others=>
		--end case;
		
		--return r;
	--end Neighbour;

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
	
	-- Returns the router position in the mesh
	-- BR: Botton Right
	-- BL: Botton Left
	-- TR: Top Right
	-- TL: Top Left
	-- CRX: Center Right
	-- CL: Center Left
	-- CC: Center
	-- 4x4 positions exemple
	-- 		TL TC TC TR
	-- 		CL CC CC CRX
	-- 		CL CC CC CRX
	-- 		BL BC BC BR
        function RouterPosition(router: integer) return integer is
                variable pos: integer range 0 to TR;
                variable column: integer;
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
                        
                        report "position: " & CONV_HEX(pos) severity note;
                        return pos;
                        
        end RouterPosition;

    function to_string(sv: Std_Logic_Vector) return string is
        use Std.TextIO.all;
        variable bv: bit_vector(sv'range) := to_bitvector(sv);
        variable lp: line;
    begin
        write(lp, bv);
        return lp.all;
    end;
    
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
    
    
end	HermesPackage;
