---------------------------------------------------------------------------------------------------
--
-- Title       : CS Buffer
-- Design      : QoS
-- Company     : GAPH
--
---------------------------------------------------------------------------------------------------
--
-- File        : CS_Buffer.vhd
-- Generated   : Thu Mar  6 17:08:38 2008
-- From        : interface description file
-- By          : Itf2Vhdl ver. 1.20
--
---------------------------------------------------------------------------------------------------
--							   
-- Description : Input buffer for circuit switching with only one position and flow control.
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
							   

entity CS_Buffer is
generic	(buffer_type : string := "SIM");
port(
    clock		: in  std_logic;
    reset		: in  std_logic;	  
    rx			: in  std_logic;
    eop_in		: in  std_logic;
    data_in		: in  regflit;
    credit_out	: out std_logic;
    req_routing	: out std_logic;
    ack_routing	: in  std_logic; 
	tx			: out std_logic;
	eop_out		: out std_logic;
    data_out	: out regflit;
    credit_in	: in  std_logic;
    sender		: out std_logic;
	free_sender	: in  std_logic;
	next_flit	: in  std_logic);
end CS_Buffer;

architecture CS_Buffer of CS_Buffer is

type state is (S0, S1, S2, S3);
signal EA : state;				  													  	

begin 			
	
	
	credit_out <= credit_in when EA /= S1 else '0';
	
	-- Routing request and packet flits transmition.
	process(reset, clock)
	begin
		if reset='1' then
			sender <='0';
			req_routing <= '0';
			tx <= '0';									
			EA <= S0;		  
			
		elsif rising_edge(clock) then
			case EA is
				-- Request routing.
				when S0 =>
					tx <= '0';
					sender <= '0';
					eop_out <= '0';
					
					-- Waits for a stored flit.
					if rx = '1' then
						data_out <= data_in;
						eop_out <= eop_in;
						req_routing <= '1';	   		  
						EA <= S1;					  
					else
						EA <= S0;
					end if;
				
				-- Waits routing request acknolegment from Switch Contol.	
				when S1 => 	
					if ack_routing = '1' then	
						req_routing <= '0';	 
						tx <= '1';
						sender <= '1';		   							   						
						EA <= S3;											
					else
						EA <= S1;
					end if;
				 
				-- Send packet flits.
				when S2 =>
                    
                    if free_sender = '1' then
						tx <= '0';
                        sender <= '0';
                        EA <= S0;
					elsif credit_in = '1' then
						if rx = '1' then
							data_out <= data_in;
							eop_out <= eop_in;
						end if;				
						
						if eop_in = '1' then							
							EA <= S0;
						
						elsif rx = '0' then
							tx <= '0';							
							EA <= S3;
						
						else						
							EA <= S2;
						end if;
					else
						EA <= S2;
					end if;
					
				when S3 =>
					tx <= '0';
					
                    if free_sender = '1' then
						tx <= '0';
                        sender <= '0';
                        EA <= S0;
					elsif rx = '1' and credit_in = '1' then
						data_out <= data_in;
						eop_out <= eop_in;
						tx <= '1';
						EA <= S2;
					end if;
					
				when others =>
					EA <= S0;
			
			end case;
		end if;
	end process;              
end CS_Buffer;
