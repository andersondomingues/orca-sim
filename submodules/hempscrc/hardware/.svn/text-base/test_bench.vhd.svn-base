------------------------------------------------------------------
---
--- HEMPS 4.0  - October 25 2011
---
--- File function: instantiates the HeMPS and the module responsible to dynamically insert new applications,
--                  and generates the clock/reset (the access to the repository is faster than the MPSoC, to simulte real memories)
---
--- Responsibles: Eduardo Wachter, Marcelo Mandelli, Carlo Lucas, Fernando Moraes
--- 
--- Contact: fernando.moraes@pucrs.br
---
------------------------------------------------------------------
library IEEE;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use std.textio.all;

use work.memory_pack.all;
use work.dynamic_apps_pack.all;
use work.HeMPS_PKG.all;
use work.HeMPS_defaults.all;

entity test_bench is
        generic(
                  log_file            : string := "output_master.txt";
                  MAX_LINE_SIZE       : integer := 231;
                  mlite_description   : string := "RTL";
                  ram_description     : string := "RTL";
                  router_description  : string := "RTL"
        );
end;

--entity test_bench_seek is
        --generic(
			----router_address        : regmetadeflit := "00000001"	
			----router_address        : regmetadeflit
        --);
--end;

architecture test_bench of test_bench is
        signal clock            : std_logic := '0';
        signal clock_200        : std_logic := '1';
        signal reset            : std_logic;

        signal control_write_enable_debug      : std_logic;
        signal control_data_out_debug          : std_logic_vector(31 downto 0);
        signal control_busy_debug              : std_logic;

        signal control_hemps_data_avail        : std_logic;
        signal control_hemps_addr              : std_logic_vector(29 downto 0);
        signal control_hemps_read_req          : std_logic;
        signal control_hemps_read_req_ant      : std_logic;
        signal control_hemps_data_valid        : std_logic;
        signal control_hemps_data_valid_ant    : std_logic;
        signal control_hemps_data              : std_logic_vector(31 downto 0);

        type state is (LER, WAIT_DDR, WR_HEMPS, START);
        signal EA                                              : state;
        
        type state2 is (S0, S1);
        signal CS: state2;
        
        signal counter                                                  : integer :=0;
        
        signal ack_task                 : std_logic;
        signal req_task                 : std_logic_vector(31 downto 0);
        
        signal app_allocated    : std_logic;
        signal new_app                  : std_logic;
        
        signal app                              : ram;
        
        
		signal tb_in_source_router_seek         :	regNsource_target;
		signal tb_in_target_router_seek         :	regNsource_target;
		signal tb_in_hop_router_seek			:	regNhop;
		signal tb_in_service_router_seek		:	seek_bitN_service;
		signal tb_in_req_router_seek			:	std_logic;
		signal tb_in_ack_router_seek			:	std_logic;
		
		signal tb_in_fail_router_seek			:	std_logic_vector(NPORT_SEEK-1 downto 0); 

		signal tb_out_req_router_seek			: std_logic;
		signal tb_out_ack_router_seek			:std_logic;
		signal tb_out_service_router_seek		: regNport_seek_bitN_service;
		signal tb_out_source_router_seek        : regNportNsource_target_neighbor_new;
		signal tb_out_target_router_seek        : regNportNsource_target_neighbor_new;       
		signal tb_out_hop_router_seek			: regNportNhop_neighbor_new;
		signal tb_router_address        		: regmetadeflit; 		
		signal counter_seek						: std_logic_vector(63 downto 0);
        
        
		signal tb_in_source_router_seek2    :	regNsource_target;
		signal tb_in_target_router_seek2   :	regNsource_target;
		signal tb_in_hop_router_seek2		:	regNhop;
		signal tb_in_service_router_seek2	:	seek_bitN_service;
		signal tb_in_req_router_seek2		:	std_logic;
		
		signal tb_in_source_router_seek3    :	regNsource_target;
		signal tb_in_target_router_seek3   :	regNsource_target;
		signal tb_in_hop_router_seek3		:	regNhop;
		signal tb_in_service_router_seek3	:	seek_bitN_service;
		signal tb_in_req_router_seek3		:	std_logic;
		
		signal tb_in_source_router_seek4   	:	regNsource_target;
		signal tb_in_target_router_seek4  	:	regNsource_target;
		signal tb_in_hop_router_seek4		:	regNhop;
		signal tb_in_service_router_seek4	:	seek_bitN_service;
		signal tb_in_req_router_seek4		:	std_logic;
		
        
        
begin

		--tb_router_address <= "00010000";

       reset <= '1', '0' after 100 ns;

       -- 100 MHz
       clock <= not clock after 5 ns;
       
       -- 200 MHz
       clock_200 <= not clock_200 after 1.25 ns;

       --
       --  access the repository considering that the HeMPS and the external memory are running at different frequencies
       --
       process(clock_200, reset)
       begin
               if reset = '1' then
                       control_hemps_data_valid <= '0';
                       EA <= START;
               elsif rising_edge(clock_200) then
                       case EA is
                               when START  =>   if control_hemps_read_req_ant = '1' then
                                                       EA <= LER;
                                               else
                                                       EA <= START;
                                               end if;
                                               
                               when LER     =>  control_hemps_data_valid <= '0';
                                                EA <= WAIT_DDR;
                                                
                               when WAIT_DDR => EA <= WR_HEMPS;
                               
                               when WR_HEMPS =>  control_hemps_data_valid <= '1';
                                                 if control_hemps_read_req_ant = '0' then
                                                         EA <= START;
                                                 else
                                                         EA <= WR_HEMPS;
                                                 end if;
                       end case;
                       control_hemps_read_req_ant <= control_hemps_read_req;
               end if;
       end process;
       
       control_hemps_data <= memory(CONV_INTEGER(control_hemps_addr(25 downto 0)));
       
       control_busy_debug <= '0';

       --
       --  HeMPS instantiation 
       --
       HeMPS: entity work.HeMPS
        generic map(
                mlite_description               => mlite_description,
                ram_description                 => ram_description,
                router_description              => router_description
             --   router_address        			=> tb_router_address
        )
        port map(
                clock                   => clock,
                reset                   => reset,
                --repository
                mem_addr                => control_hemps_addr,
                read_req                                => control_hemps_read_req,
                data_read               => control_hemps_data,
                data_valid              => control_hemps_data_valid,
                --debug
                write_enable_debug              => control_write_enable_debug,
                data_out_debug					 => control_data_out_debug,
                busy_debug    					=> control_busy_debug,
                ack_task      					=> ack_task,
                req_task      					=> req_task,
              -- ---------router seek_pulse
               in_source_router_seek_local   	  		=>	tb_in_source_router_seek   ,
               in_target_router_seek_local   	  		=>	tb_in_target_router_seek   ,
               in_hop_router_seek_local					=>	tb_in_hop_router_seek		,	
               in_service_router_seek_local				=>	tb_in_service_router_seek	,
               in_req_router_seek_local					=>	tb_in_req_router_seek		,	
               
               
              
               in_source_router_seek_local2   	  		=>	tb_in_source_router_seek2   ,
               in_target_router_seek_local2   	  		=>	tb_in_target_router_seek2   ,
               in_hop_router_seek_local2				=>	tb_in_hop_router_seek2		,	
               in_service_router_seek_local2			=>	tb_in_service_router_seek2	,
               in_req_router_seek_local2				=>	tb_in_req_router_seek2	,
               
               in_source_router_seek_local3   	  		=>	tb_in_source_router_seek3   ,
               in_target_router_seek_local3   	  		=>	tb_in_target_router_seek3   ,
               in_hop_router_seek_local3				=>	tb_in_hop_router_seek3		,	
               in_service_router_seek_local3			=>	tb_in_service_router_seek3	,
               in_req_router_seek_local3				=>	tb_in_req_router_seek3		,
               
               in_source_router_seek_local4   	  		=>	tb_in_source_router_seek4   ,
               in_target_router_seek_local4   	  		=>	tb_in_target_router_seek4   ,
               in_hop_router_seek_local4				=>	tb_in_hop_router_seek4		,	
               in_service_router_seek_local4			=>	tb_in_service_router_seek4	,
               in_req_router_seek_local4				=>	tb_in_req_router_seek4							
               
               
              
              
              
              
               
        );
        
        process(clock, reset)
			begin
			
			if reset = '1' then
					tb_in_source_router_seek        <= (others => '0');
					tb_in_target_router_seek        <= (others => '0');
					tb_in_hop_router_seek			<= (others => '0');
					tb_in_service_router_seek		<= (others => '0');
					tb_in_req_router_seek 			<= '0';
					tb_in_ack_router_seek			<= '0';
					counter_seek 					<= (others => '0');
			elsif clock'event and clock='1' then   
				counter_seek <= counter_seek +1;
				
				if  tb_in_ack_router_seek = '0' and unsigned(counter_seek) = 4 then
					
					-------------------------------------------
					--------------SERVIÇOS---------------------
					--- PACKET_RESEND_SERVICE			
					--- TARGET_UNREACHABLE_SERVICE		
					--- CLEAR_SERVICE			      
					--- BACKTRACK_SERVICE			  
					--- SEARCHPATH_SERVICE			  
					-------------------------------------------
					-------------------------------------------					
					--0 to 49
					--21 to 58
					--71 to 48
					--90 to 59
					--tb_in_source_router_seek  <= "--X--Y--";					
					--	tb_in_source_router_seek    	<= "00000000"; -- PE 00 LOCAL: router_address = source
					--	tb_in_target_router_seek    	<= "10010100"; -- PE 49
					--	tb_in_hop_router_seek			<= "000000";
					--	tb_in_service_router_seek		<= SEARCHPATH_SERVICE;
					--	tb_in_req_router_seek 			<= '1';
					--	
					--	tb_in_source_router_seek2  	<= "00010010"; -- PE 21 LOCAL: router_address = source
					--	tb_in_target_router_seek2   <= "10000101"; -- PE 58
					--	tb_in_hop_router_seek2		<= "000000";
					--	tb_in_service_router_seek2	<= SEARCHPATH_SERVICE;
					--	tb_in_req_router_seek2		<= '1';
					--				
					--	tb_in_source_router_seek3  	<= "00010111"; -- PE 71 LOCAL: router_address = source
					--	tb_in_target_router_seek3   <= "10000100"; -- pe 48
					--	tb_in_hop_router_seek3		<= "000000";
					--	tb_in_service_router_seek3	<= SEARCHPATH_SERVICE;
					--	tb_in_req_router_seek3		<= '1';
					--	
					--	tb_in_source_router_seek4  	<= "00001001"; -- PE 90 LOCAL: router_address = source
					--	tb_in_target_router_seek4   <= "10010101"; -- PE 59
					--	tb_in_hop_router_seek4		<= "000000";
					--	tb_in_service_router_seek4	<= SEARCHPATH_SERVICE;
					--	tb_in_req_router_seek4		<= '1';
					--
					--elsif unsigned(counter_seek) = 10	then 
					--	tb_in_req_router_seek 		<= '0';					
					--	tb_in_req_router_seek2 		<= '0';	
					--	tb_in_req_router_seek3 		<= '0';	
					--	tb_in_req_router_seek4 		<= '0';	
					--	
					--end if;	
					tb_in_source_router_seek    	<= "00000000"; -- PE 00 LOCAL: router_address = source
					tb_in_target_router_seek    	<= "00110011"; -- PE 15
					tb_in_hop_router_seek			<= "000000";
					tb_in_service_router_seek		<= SEARCHPATH_SERVICE;
					tb_in_req_router_seek 			<= '1';
					
					tb_in_source_router_seek2  	<= "00000011"; -- PE 12 LOCAL: router_address = source
					tb_in_target_router_seek2   <= "00110000"; --PE 03
					tb_in_hop_router_seek2		<= "000000";
					tb_in_service_router_seek2	<= SEARCHPATH_SERVICE;
					tb_in_req_router_seek2		<= '1';
								
					tb_in_source_router_seek3  	<= "00110011"; --PE 15 LOCAL: router_address = source
					tb_in_target_router_seek3   <= "00000000";  -- pe 00
					tb_in_hop_router_seek3		<= "000000";
					tb_in_service_router_seek3	<= SEARCHPATH_SERVICE;
					tb_in_req_router_seek3		<= '1';
					
					tb_in_source_router_seek4  	<= "00110000"; -- PE 03 LOCAL: router_address = source
					tb_in_target_router_seek4   <= "00000011"; --PE 12
					tb_in_hop_router_seek4		<= "000000";
					tb_in_service_router_seek4	<= SEARCHPATH_SERVICE;
					tb_in_req_router_seek4		<= '1';
            
				elsif unsigned(counter_seek) = 10	then 
					  tb_in_req_router_seek 		<= '0';					
					  tb_in_req_router_seek2 		<= '0';	
					  tb_in_req_router_seek3 		<= '0';	
					  tb_in_req_router_seek4 		<= '0';	
					  
				end if;				
				
				
				
				
				
			end if;
			
		end process;
        
        
        process
              variable k : integer := 0;
              variable j : integer := 0;
        begin
                if reset = '1' then
                        req_task <= (others=>'0');
                        new_app <= '0';
                        k:=0;
                        j:=0;
                        app <=((others=> (others=>'0')));
                else
                        loop1: while j<NUMBER_OF_APPS loop
                                wait for appstime(j);
                                app <= dynamic_apps(j);
                                new_app <= '1';
                                wait until app_allocated'event and app_allocated= '1';
                                new_app <= '0';
                                k := 1;
                                loop2: while app(k)/=x"ffffffff" and k<12 loop
                                        req_task <= app(k) or x"80000000";
                                        wait until ack_task'event and ack_task= '1';
                                        req_task <= (others=>'0');
                                        k:= k + 1;
                                        wait for 10 ns;
                                end loop loop2;
                                j:= j + 1;
                        end loop loop1;
                        wait;
                end if;
        end process;
        
       --
       --  instantiates the module responsible to insert new applications into the repository 
       --
       Insert_Application: entity work.insert_application
       port map(
               clock                   => clock,
               reset                   => reset,
               new_app                 => new_app,
               app                             => app,
               app_allocated   => app_allocated
       );
      
      
     --
     -- creates the output file 
     --
     process(control_write_enable_debug,reset)
       file store_file : text open write_mode is log_file;
       variable file_line : line;
       variable line_type: character;
       variable line_length : natural := 0;
       variable str: string (1 to 4);
       variable str_end: boolean;
     begin
        if reset = '1' then
                str_end := false;
                CS <= S0;      
        elsif rising_edge(control_write_enable_debug) then
                case CS is
                  when S0 =>
                          -- Reads the incoming string
                          line_type := character'val(conv_integer(control_data_out_debug(7 downto 0)));
                          
                          -- Verifies if the string is from Echo()
                          if line_type = '$' then 
                                  write(file_line, line_type);
                                  line_length := line_length + 1;
                                  CS <= S1;
                          
                          -- Writes the string to the file
                          else                                                                    
                                  str(4) := character'val(conv_integer(control_data_out_debug(7 downto 0)));
                                  str(3) := character'val(conv_integer(control_data_out_debug(15 downto 8)));
                                  str(2) := character'val(conv_integer(control_data_out_debug(23 downto 16)));
                                  str(1) := character'val(conv_integer(control_data_out_debug(31 downto 24)));
                                  
                                  str_end := false;
                                  
                                  for i in 1 to 4 loop                                                            
                                          -- Writes a string in the line
                                          if str(i) /= lf and str(i) /= nul and not str_end then
                                                  write(file_line, str(i));
                                                  line_length := line_length + 1;
                                  
                                          -- Detects the string end
                                          elsif str(i) = nul then
                                                  str_end := true;
                                          
                                          -- Line feed detected. Writes the line in the file
                                          elsif str(i) = lf then                                                              
                                                  writeline(store_file, file_line);
                                                  line_length := 0;
                                          end if;
                                  end loop;
                          end if;
                                                                  
                  -- Receives from plasma the source processor, source task and writes them to the file
                  when S1 =>
                          write(file_line, ',');
                          write(file_line, conv_integer(control_data_out_debug(7 downto 0)));                                                             
                          line_length := line_length + 1;
                          
                          if line_length = 3 then 
                                  write(file_line, ',');
                                  CS <= S0;
                          else
                                  CS <= S1;
                          end if;
               end case;
        end if;
      end process;

end test_bench;
