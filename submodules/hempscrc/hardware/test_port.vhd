---------------------------------------------------------------------------------------------------
--
-- Title       : Test_port
-- Design      : QoS
-- Company     : GAPH
--
---------------------------------------------------------------------------------------------------
--
-- File        : test_port.vhd
-- Generated   : Tue Out  28 11:42:38 2014
-- From        : Test port description file
-- By          : Fochi 
--
---------------------------------------------------------------------------------------------------
--							   
--
---------------------------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use IEEE.STD_LOGIC_unsigned.all;
use work.HeMPS_defaults.all;

entity test_port is
    port(
        clock                   : in  std_logic;
        reset                   : in  std_logic;
		data_out_test           : out regflit;  
		eop_out_test            : out std_logic;
		in_failed_port          : in  regNport_neighbor;
        out_test_port			: out regNport_neighbor;
        rx_out					: out std_logic
    );
end test_port; 

architecture test_port of test_port is

type state_test_port is (IDLE, WAIT_TIMER, ROUND_ROBIN, SEND_TEST_PORT, SEND_TEST_HEADER, SEND_TEST_PAYLOAD, SEND_TEST_PAYLOAD2);
signal test_port_SM   	: state_test_port;
signal counter			: std_logic_vector(9 downto 0);
signal packet_size      : std_logic_vector(4 downto 0);
signal ask		 		: std_logic;
signal tested			: regNport_neighbor;
signal try_again		: boolean;
signal sel,prox			: integer range 0 to (NPORT-1);

begin
   
   ask <=	in_failed_port(EAST0) or in_failed_port(WEST0) or in_failed_port(NORTH0) or in_failed_port(SOUTH0) or
			in_failed_port(EAST1) or in_failed_port(WEST1) or in_failed_port(NORTH1) or in_failed_port(SOUTH1);
   
    -- Round Robin: seleciona uma das portas de entrada para o roteamento (prox).	
	process(sel,in_failed_port)
    begin
        case sel is
                           
            when EAST0 => 
				if in_failed_port(EAST1)='1' then prox<=EAST1;
				elsif in_failed_port(WEST0)='1' then  prox<=WEST0;
				elsif in_failed_port(WEST1)='1' then  prox<=WEST1;
                elsif in_failed_port(NORTH0)='1' then prox<=NORTH0; 
				elsif in_failed_port(NORTH1)='1' then prox<=NORTH1;
                elsif in_failed_port(SOUTH0)='1' then prox<=SOUTH0;
				elsif in_failed_port(SOUTH1)='1' then prox<=SOUTH1;
				else prox<=EAST0; end if;
					
			when EAST1 => 
				if in_failed_port(WEST0)='1' then  prox<=WEST0;
				elsif in_failed_port(WEST1)='1' then  prox<=WEST1;
                elsif in_failed_port(NORTH0)='1' then prox<=NORTH0; 
				elsif in_failed_port(NORTH1)='1' then prox<=NORTH1;
                elsif in_failed_port(SOUTH0)='1' then prox<=SOUTH0;
				elsif in_failed_port(SOUTH1)='1' then prox<=SOUTH1;
				elsif in_failed_port(EAST0)='1' then prox<=EAST0;				
				else prox<=EAST1; end if;
							
            when WEST0 => 
				if in_failed_port(WEST1)='1' then  prox<=WEST1;
				elsif in_failed_port(NORTH0)='1' then prox<=NORTH0; 
				elsif in_failed_port(NORTH1)='1' then prox<=NORTH1;
                elsif in_failed_port(SOUTH0)='1' then prox<=SOUTH0;
				elsif in_failed_port(SOUTH1)='1' then prox<=SOUTH1;
				elsif in_failed_port(EAST0)='1' then prox<=EAST0;
				elsif in_failed_port(EAST1)='1' then prox<=EAST1;
				else prox<=WEST0; end if;
					
			when WEST1 => 
				if in_failed_port(NORTH0)='1' then prox<=NORTH0; 
				elsif in_failed_port(NORTH1)='1' then prox<=NORTH1;
                elsif in_failed_port(SOUTH0)='1' then prox<=SOUTH0;
				elsif in_failed_port(SOUTH1)='1' then prox<=SOUTH1;
				elsif in_failed_port(EAST0)='1' then prox<=EAST0;
				elsif in_failed_port(EAST1)='1' then prox<=EAST1;
				elsif in_failed_port(WEST0)='1' then  prox<=WEST0;				
				else prox<=WEST1; end if;
							
            when NORTH0 =>
				if in_failed_port(NORTH1)='1' then prox<=NORTH1;
                elsif in_failed_port(SOUTH0)='1' then prox<=SOUTH0;
				elsif in_failed_port(SOUTH1)='1' then prox<=SOUTH1;
				elsif in_failed_port(EAST0)='1' then prox<=EAST0;
				elsif in_failed_port(EAST1)='1' then prox<=EAST1;
				elsif in_failed_port(WEST0)='1' then  prox<=WEST0;
				elsif in_failed_port(WEST1)='1' then  prox<=WEST1;
				else prox<=NORTH0; end if;
					
			when NORTH1 =>
				if in_failed_port(SOUTH0)='1' then prox<=SOUTH0;
				elsif in_failed_port(SOUTH1)='1' then prox<=SOUTH1;
				elsif in_failed_port(EAST0)='1' then prox<=EAST0;
				elsif in_failed_port(EAST1)='1' then prox<=EAST1;
				elsif in_failed_port(WEST0)='1' then  prox<=WEST0;
				elsif in_failed_port(WEST1)='1' then  prox<=WEST1;
				elsif in_failed_port(NORTH0)='1' then prox<=NORTH0;                
				else prox<=NORTH1; end if;
						   
             when SOUTH0 => 
				if in_failed_port(SOUTH1)='1' then prox<=SOUTH1;
				elsif in_failed_port(EAST0)='1' then prox<=EAST0;
				elsif in_failed_port(EAST1)='1' then prox<=EAST1;
				elsif in_failed_port(WEST0)='1' then  prox<=WEST0;
				elsif in_failed_port(WEST1)='1' then  prox<=WEST1;
				elsif in_failed_port(NORTH0)='1' then prox<=NORTH0; 
				elsif in_failed_port(NORTH1)='1' then prox<=NORTH1;
				else prox<=SOUTH0; end if;
					
			when SOUTH1 => 
				if in_failed_port(EAST0)='1' then prox<=EAST0;
				elsif in_failed_port(EAST1)='1' then prox<=EAST1;
				elsif in_failed_port(WEST0)='1' then  prox<=WEST0;
				elsif in_failed_port(WEST1)='1' then  prox<=WEST1;
				elsif in_failed_port(NORTH0)='1' then prox<=NORTH0; 
				elsif in_failed_port(NORTH1)='1' then prox<=NORTH1;
				elsif in_failed_port(SOUTH0)='1' then prox<=SOUTH0;				
				else prox<=SOUTH1; end if;
					
			when others =>
        end case;
    end process; 
    
    process(clock,reset)
    begin 
        if (reset='1') then
            test_port_SM					<= IDLE;
            counter 						<= (others=>'0');
			out_test_port					<= (others=>'0');
			try_again       				<= false;
			tested							<= (others=>'0');
                                
        elsif (clock'event and clock='1') then
            case test_port_SM is  
              
                when IDLE =>					
					if (in_failed_port /= "00000000") then
						counter  				<= (others=>'0');
						test_port_SM			<= WAIT_TIMER;
					end if;  
					
					if sel = EAST0	then
							out_test_port(EAST0) <= '0' ;
					end if;					
					if sel = EAST1	then
							out_test_port(EAST1) <= '0' ;
					end if;					
					if sel = WEST0 	then
							out_test_port(WEST0) <= '0' ;
					end if;					
					if sel = WEST1	then
							out_test_port(WEST1) <= '0' ;
					end if;
					if sel = NORTH1	then
							out_test_port(NORTH1) <= '0' ;
					end if;
					if sel = NORTH0	then
							out_test_port(NORTH0) <= '0' ;
					end if;
					if sel = SOUTH0	then
							out_test_port(SOUTH0) <= '0' ;
					end if;					
					if sel = SOUTH1	then
							out_test_port(SOUTH1)  <= '0' ;
					end if;

                when WAIT_TIMER =>
						counter <= counter +1;
						if counter = "1111111111" then
							test_port_SM <= ROUND_ROBIN;
						end if;
              
                when ROUND_ROBIN =>
					if ask = '1' then						
						if try_again and sel /= prox and in_failed_port(sel) = '1' then
							try_again <= false;
						else
							sel <= prox;
							try_again <= true;
							test_port_SM			<= SEND_TEST_PORT;	 
						end if;						
						
					end if;
                
                 when SEND_TEST_PORT=>
					if  sel = EAST0 and tested(EAST0) = '0' then
							out_test_port(EAST0) <= '1' ;
							test_port_SM			<= SEND_TEST_HEADER;
					elsif sel = EAST1 and tested(EAST1) = '0' 	then
							out_test_port(EAST1) <= '1' ;
							test_port_SM			<= SEND_TEST_HEADER;
					elsif sel = WEST0 and tested(WEST0) = '0' 	then
							out_test_port(WEST0) <= '1' ;
							test_port_SM			<= SEND_TEST_HEADER;
					elsif sel = WEST1 and tested(WEST1) = '0' 	then
							out_test_port(WEST1) <= '1' ;
							test_port_SM			<= SEND_TEST_HEADER;
					elsif sel = NORTH1 and tested(NORTH1) = '0' 	then
							out_test_port(NORTH1) <= '1' ;
							test_port_SM			<= SEND_TEST_HEADER;
					elsif sel = NORTH0 and tested(NORTH0) = '0' 	then
							out_test_port(NORTH0) <= '1' ;
							test_port_SM			<= SEND_TEST_HEADER;
					elsif sel = SOUTH0 and tested(SOUTH0) = '0' 	then
							out_test_port(SOUTH0) <= '1' ;
							test_port_SM			<= SEND_TEST_HEADER;
					elsif sel = SOUTH1 and tested(SOUTH1) = '0' 	then
							out_test_port(SOUTH1)  <= '1' ;
							test_port_SM			<= SEND_TEST_HEADER;
					else
							test_port_SM		<= IDLE;	
					end if;
				
				when SEND_TEST_HEADER =>		
					data_out_test	<= x"3FEFE";
				    eop_out_test            <= '0';
				    rx_out					<= '1';
                    packet_size				<= "01010";
                    test_port_SM			<= SEND_TEST_PAYLOAD; 
                        
                when SEND_TEST_PAYLOAD =>
                    packet_size         <= packet_size - 1;
                    if packet_size = 0 then
                        eop_out_test        <= '0';
   						test_port_SM		<= IDLE;	
   						tested(sel)			<= '1';
   						rx_out				<= '0';
                    elsif packet_size = 1 then
                        eop_out_test                <= '1';
                        data_out_test	<= (others => '0');                          
                    else 
						data_out_test	<= x"CA5A5";  
						test_port_SM		<=  SEND_TEST_PAYLOAD2;               
                    end if;
                    
				when SEND_TEST_PAYLOAD2 =>
					packet_size         <= packet_size - 1;
                    if packet_size = 0 then
                        eop_out_test        <= '0';
   						test_port_SM		<= IDLE;
   						tested(sel)			<= '1';	
   						rx_out				<= '0';   						
                    elsif packet_size = 1 then
                        eop_out_test                <= '1';
                        data_out_test	<= (others => '0');                          
                    else 
						data_out_test <= x"55A5A";                   
						test_port_SM		<=  SEND_TEST_PAYLOAD;               
                    end if;
			end case;		
        end if;
    end process;
end test_port;
