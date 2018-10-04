---------------------------------------------------------------------------------------------------
--
-- Title       : Test_port
-- Design      : QoS
-- Company     : GAPH
--
---------------------------------------------------------------------------------------------------
--
-- File        : test_port_receive.vhd
-- Generated   : Tue Out  28 11:42:38 2014
-- From        : Test port description file
-- By          : Fochi 
--
---------------------------------------------------------------------------------------------------
--							   
--
---------------------------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_unsigned.all;
use ieee.numeric_std.all;
use work.HeMPS_defaults.all;


entity test_port_receive is
    port(
        clock                   : in  std_logic;
        reset                   : in  std_logic;
        data_in					: in  regflit;
        in_failed_port_test		: in  std_logic;
		eop_in_test_recv    	: in  std_logic;
		in_failed_port          : in  std_logic;
		out_reset_stepdet		: out std_logic;
        out_test_port_recv		: out std_logic
    );
end test_port_receive; 

architecture test_port_receive of test_port_receive is

type state_test_port_receive is (IDLE, WAIT_EOP, WAIT_HEADER, RELEASE_FAULTY_PORT, FAULT_STATE);
signal test_port_SM   	: state_test_port_receive;

begin
   
    process(clock,reset)
    begin 
        if (reset='1') then
            test_port_SM					<= IDLE;
			out_test_port_recv				<= '0';
                                
        elsif (clock'event and clock='1') then
            case test_port_SM is  
              
                when IDLE =>					
										
					if  in_failed_port = '1' then 
							out_test_port_recv <= '1' ;
							test_port_SM			<= WAIT_HEADER;	 
					end if;		

					when WAIT_HEADER =>
						if data_in(19 downto 0) = x"3FEFE" then
							test_port_SM <= WAIT_EOP;
						end if;

                when WAIT_EOP =>
					 if in_failed_port_test	= '1' then 
						test_port_SM			<= FAULT_STATE;
						report "TEST PORT FAILED " severity note;
					 elsif	eop_in_test_recv = '1' then
					 	test_port_SM			<= RELEASE_FAULTY_PORT;	 
					 	out_reset_stepdet		<= '0';
					 end if;
					 
                when RELEASE_FAULTY_PORT =>
                 	if  in_failed_port = '0' then 
							test_port_SM			<= IDLE;	 
					end if;	
				when FAULT_STATE =>
					
					
			end case;		
        end if;
    end process;
end test_port_receive;
