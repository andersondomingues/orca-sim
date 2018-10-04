---------------------------------------------------------------------------------------------------
--
-- Title       : Input Buffer
-- Design      : QoS
-- Company     : GAPH
--
---------------------------------------------------------------------------------------------------
--
-- File        : Input_Buffer.vhd
-- Generated   : Thu Mar  6 17:08:38 2008
-- From        : interface description file
-- By          : Itf2Vhdl ver. 1.20
--
---------------------------------------------------------------------------------------------------
--							   
-- Description : Temporary flit storage.
--
---------------------------------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use IEEE.std_logic_arith.all;
use work.HeMPS_defaults.all;
--
library UNISIM;
use UNISIM.vcomponents.all;	 
							   

entity Input_Buffer is
generic	(buffer_type : string := "SIM");
port(
    clock		: in  std_logic;
    reset		: in  std_logic;	  
    rx			: in  std_logic;
    eop_in		: in  std_logic;
    data_in		: in  regflit;
    credit_out	: out std_logic;
    error	    : in  std_logic; -- modificado fochi
    req_routing	: out std_logic;
    ack_routing	: in  std_logic; 
	tx			: out std_logic;
	eop_out		: out std_logic;
    data_out	: out regflit;
    credit_in	: in  std_logic;
    sender		: out std_logic;
	next_flit	: in  std_logic;
	fail_detect	: out std_logic);
end Input_Buffer;

architecture Input_Buffer of Input_Buffer is

type state is (S0, S1, S2, S3);
signal EA : state;

signal eop: std_logic_vector(BUFFER_DEEP-1 downto 0);
signal first,last: pointer;
signal available_slot, one_flit_in_buffer, tx_sig: std_logic;
signal data_read: regflit;
signal	contador :std_logic_vector(7 downto 0);
signal reseta_buffer: std_logic;

-- Flit Buffer implemented using array.
signal flit_buff: buff;

	component RAM16X1D is			
		generic (
			INIT : bit_vector(15 downto 0) := X"0000"
		);			
		port (
			DPO   : out std_ulogic;        
			SPO   : out std_ulogic;			        
			A0    : in std_ulogic;
			A1    : in std_ulogic;
			A2    : in std_ulogic;
			A3    : in std_ulogic;
			D     : in std_ulogic;        
			DPRA0 : in std_ulogic;
			DPRA1 : in std_ulogic;
			DPRA2 : in std_ulogic;
			DPRA3 : in std_ulogic;        
			WCLK  : in std_ulogic;
			WE    : in std_ulogic
		);
	end component;
		 					   
begin 
	
	-- Flit Buffer implemented using LUT RAM.	
	buffer_RTL: if buffer_type = "RTL" generate	
		flit_buff: for i in 0 to FLIT_WIDTH-1 generate
			RAM16X1D_inst : RAM16X1D
			port map (
				DPO 	=> data_read(i), 		
				SPO 	=> open, 		
				A0 		=> last(0), 		
				A1 		=> last(1), 		
				A2 		=> last(2), 		
				A3 		=> last(3), 		
				D 		=> data_in(i), 			
				DPRA0 	=> first(0), 		
				DPRA1 	=> first(1), 		
				DPRA2 	=> first(2), 		
				DPRA3 	=> first(3), 		
				WCLK 	=> clock, 		
				WE 		=> '1' 			
			);
		end generate flit_buff;
	end generate;
	
--	buffer_SIM: if buffer_type = "SIM" generate
		data_read <= flit_buff(CONV_INTEGER(first));
--	end generate;
  

	-- Controls flit storing and avaliable buffer slots. 
	process(reset, clock)
	variable last_var,first_var: pointer;	-- Used only to update 'available_slot'.
	begin
		if reset='1' then			 	
			last <= (others=>'0'); 
			eop <= (others=>'0');
			available_slot <= '1';
			first_var := (others=>'0');
			last_var := (others=>'0');			
		elsif rising_edge(clock) then
			-- Stores a flit.
			if rx = '1' and available_slot = '1' then 
			--if rx = '1' and available_slot = '1' and error='0' then --modificado fochi
				if buffer_type = "SIM" then
					flit_buff(CONV_INTEGER(last)) <= data_in;
				end if;
				eop(CONV_INTEGER(last)) <= eop_in;
				last <= last + 1;
				last_var := last + 1;										
			end if;
						
			-- Verifies if a flit is transmited.
			if tx_sig = '1' and credit_in = '1'  then
				first_var := first + 1;
			else
				first_var := first; 
			end if;
			
			-- Update the 'available_slot'.
			--if (first_var = x"0" and last_var = BUFFER_DEEP-1) or (first_var = last_var+1) then
				--available_slot <= '0';
			--else
				--available_slot <= '1';
			--end if;
            if ((first_var = x"0" and last_var = BUFFER_DEEP-1) or (first_var = last_var+1) ) then
				available_slot <= '0';
			elsif (((last_var - first_var) = 2) and ((first_var - last_var) = BUFFER_DEEP-2))   then
				available_slot <= '1';
			end if;
			
			if reseta_buffer = '1' then
				last <= last -1;
				last_var := last - 1;	
				
			end if;									
		end if;
	end process;
	
   
	-- Data ready to be transmited.
	--data_out <= data_read when error='0' else (others=>'0');
	data_out <= data_read;
	
	-- Indicates if the current flit is the last packet flit ('1').
	--eop_out <= eop(CONV_INTEGER(first))when error='0' else '1' ;
	eop_out <= eop(CONV_INTEGER(first));
	
	--Indicates if exist at least one free slot in buffer ('1').
	credit_out <= available_slot;	
	
	-- Indicates one flit stored in the buffer.
	one_flit_in_buffer <= '1' when first+1 = last else '0';
		
	tx <= tx_sig;
	-- Routing request and packet flits transmition.
	process(reset, clock)
	begin
		if reset='1' then
			sender <='0';
			req_routing <= '0';
			tx_sig <= '0';
			first <= (others=>'0');
			EA <= S0;
			contador <= (others=>'0');
			fail_detect <= '0';
			reseta_buffer <= '0';
			
		elsif rising_edge(clock) then
			case EA is
				-- Request routing.
				when S0 =>		 					
					-- Waits for a stored flit.
					contador <= (others=>'0');
					reseta_buffer <='0';
					
					if first /= last and reseta_buffer ='0' then
						req_routing <= '1';
						EA <= S1;  
					else
						EA<= S0;
					end if;
					fail_detect <= '0';
				
				-- Waits routing request acknolegment from Switch Contol.	
				when S1 => 	
					contador <= contador + 1;
					if ack_routing = '1' then	
						req_routing <= '0';	 
						tx_sig <= '1';
						sender <= '1';
						EA <= S2 ;     
					
					-- Removes the first flit (MULTICAST_DEST) and returns to S0 to request routing for the another multicast destination.
					elsif next_flit = '1' then
						first <= first + 1;
						sender <= '1';
						req_routing <= '0';
						tx_sig <= '0';
						EA <= S0;
					--elsif contador = 254 then
					elsif error = '1' then
						
						req_routing <= '0';	 
						tx_sig <= '1';
						sender <= '1';
						EA <= S2 ;
					--	EA <= S0;
--						reseta_buffer <= '1';
						--req_routing <= '0';											
					else
						EA <= S1;
					end if;
				 
				-- Send packet flits.
				when S2 =>
                    -- if free_sender = '1' or (available_slot = '1' and rx = '0') then--added in case of fault in the middle of transmition of a packet)
     --                if free_sender = '1' then--added in case of fault in the middle of transmition of a packet)
					-- 	tx_sig <= '0';
     --                    sender <= '0';
     --                    EA <= S0;
     --                    report "free_senderS2" severity note;
                    
					-- elsif credit_in = '1' then
					if credit_in = '1' then
						first <= first + 1;
						
						-- Verifies if the current flit is the last packet flit.
						if eop(CONV_INTEGER(first)) = '1' then
							tx_sig <= '0';
							sender <= '0';
							EA <= S0;
						
						-- Verifies if the current flit is the only stored flit.
						elsif one_flit_in_buffer = '1' then
							tx_sig <= '0';
							EA <= S3;
						else
							EA <= S2;
						end if;					
					
					else
						EA <= S2;
					end if;
					 
				-- Waits for a stored flit.
				when S3 =>
                    --if available_slot = '1' and rx = '0'  and error = '0' then
                    if available_slot = '1' and rx = '0'  then
                    -- if free_sender = '1' then
						tx_sig <= '0';
                        sender <= '0';
                        EA <= S0;
                        fail_detect <= '1';
                        report "fail_detectS3" severity note;
					--elsif first /= last and error='0' then
					elsif first /= last  then
						tx_sig <= '1';
						EA <= S2;	
					else
						EA <= S3;
					end if;
						
					
				when others =>
					EA <= S0;
			
			end case;
		end if;
	end process;              
end Input_Buffer;
