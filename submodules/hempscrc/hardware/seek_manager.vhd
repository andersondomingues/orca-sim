library ieee;
use ieee.std_logic_1164.all;
use IEEE.STD_LOGIC_unsigned.all;
use work.HeMPS_defaults.all;

entity seek_manager is
   port(
		clock       			 : in  std_logic;
		reset					 : in  std_logic;
	
		--seek pe ports
		in_seek_pe		   		 : in  regNBit_seek;
		out_ack_seek_pe			 : out std_logic;
		in_clear_pe		   		 : in  std_logic;
		out_ack_clear_pe		 : out std_logic;
		in_source_pe   			 : in  regNsource_target;
   		in_target_pe   			 : in  regNsource_target;
		 
		-- seek manager ports 
		in_seek_fail_detect		 : in  regNport_regNBit_seek; 
		in_source_fail_detect    : in  regNport_neighborNsource_target_neighbor;
		in_target_fail_detect    : in  regNport_neighborNsource_target_neighbor;
		out_ack_seek_fail_detect : out regNport_neighbor;
		 
		-- interface seek manager - router seeksim:/test_bench/HeMPS/proc(11)/slav/slave/PE/generation_of_fail_detect(5)/HeMPS_fail_detect/out_seek_fail_detect

		out_seek_manager	     : out regNBit_seek;
		in_ack_seek_manager		 : in  std_logic;
		out_clear_manager	     : out std_logic;
		in_ack_clear_manager	 : in  std_logic;
		out_source_seek_manager  : out regNsource_target;
		out_target_seek_manager	 : out regNsource_target
        );
end seek_manager; 

architecture seek_manager of seek_manager is

type state_seek_manager is (waitSeek, waitAck, waitSeekAckPE, waitClearAckPE, waitClearOrSeekDown);

signal seek_manager_SM    		: state_seek_manager;
--signal ack_seek_fail_detect     : regNsource_target;
signal free_port: regNport;	 
signal port_ack : integer; 

begin
	process(clock,reset)
   begin
  
        -- state machine of seek manager
        if (reset='1') then
			out_source_seek_manager 				<= (others => '0');
   	    	out_target_seek_manager 				<= (others => '0');
   	    	out_ack_seek_fail_detect				<= (others => '0');
			out_seek_manager 						<= (others => '0');
			out_ack_clear_pe 						<= '0';
			out_ack_seek_pe 						<= '0';
			out_clear_manager 						<= '0';
            seek_manager_SM  						<= waitSeek;
        elsif (clock'event and clock='1') then
            case seek_manager_SM is 
                when waitSeek =>
					out_clear_manager			<= '0';
					out_seek_manager			<= (others => '0');
         			out_ack_seek_fail_detect    <= (others => '0');
         			out_ack_clear_pe			<= '0';
         			out_ack_seek_pe				<= '0';
         			if in_seek_pe = SEEK_CODE then
        				out_seek_manager   		<= in_seek_pe; -- if not seek unreacleble, the seek manager just propagates the seek of PE
					  	out_source_seek_manager	<= in_source_pe;
						out_target_seek_manager	<= in_target_pe;
						seek_manager_SM         <= waitSeekAckPE;
					elsif in_clear_pe = '1' then
						out_clear_manager		<= in_clear_pe;
					  	out_source_seek_manager	<= in_source_pe;
						out_target_seek_manager	<= in_target_pe;
						seek_manager_SM         <= waitClearAckPE;
         			elsif (free_port(EAST0)='1' and free_port(EAST1)='1' and (in_seek_fail_detect(0) = "01" or in_seek_fail_detect(1) = "01") )    then
						if (in_seek_fail_detect(0) = "01") then
							out_source_seek_manager <= in_source_fail_detect(0);
							out_target_seek_manager <= in_target_fail_detect(0);
							seek_manager_SM         <= waitAck;   
							port_ack 				<= 0;
							out_seek_manager 		<= SEEK_UNREACHABLE_CODE;
						elsif ( in_seek_fail_detect(1) = "01") then 
							out_source_seek_manager <= in_source_fail_detect(1);
							out_target_seek_manager <= in_target_fail_detect(1);
							seek_manager_SM         <= waitAck;   
							port_ack 				<= 1;
							out_seek_manager 		<= SEEK_UNREACHABLE_CODE;
						end if;
						report "Seek Manager 0 e 1" severity note;
					elsif (free_port(WEST0)='1' and free_port(WEST1)='1' and (in_seek_fail_detect(2) = "01" or in_seek_fail_detect(3) = "01") )   then
						if (in_seek_fail_detect(2) = "01") then
							out_source_seek_manager <= in_source_fail_detect(2);
							out_target_seek_manager <= in_target_fail_detect(2);
							seek_manager_SM         <= waitAck;   
							port_ack 				<= 2;
							out_seek_manager 		<= SEEK_UNREACHABLE_CODE;
						elsif ( in_seek_fail_detect(3) = "01") then 
							out_source_seek_manager <= in_source_fail_detect(3);
							out_target_seek_manager <= in_target_fail_detect(3);
							seek_manager_SM         <= waitAck;   
							port_ack 				<= 3;
							out_seek_manager 		<= SEEK_UNREACHABLE_CODE;
						end if;
					  report "Seek Manager 2 e 3" severity note;
					elsif (free_port(NORTH0)='1' and free_port(NORTH1)='1' and (in_seek_fail_detect(4) = "01" or in_seek_fail_detect(5) = "01") )   then
						if (in_seek_fail_detect(4) = "01") then
							out_source_seek_manager <= in_source_fail_detect(4);
							out_target_seek_manager <= in_target_fail_detect(4);
							seek_manager_SM         <= waitAck;   
							port_ack 				<= 4;
							out_seek_manager 		<= SEEK_UNREACHABLE_CODE;
						elsif ( in_seek_fail_detect(5) = "01") then 
							out_source_seek_manager <= in_source_fail_detect(5);
							out_target_seek_manager <= in_target_fail_detect(5);
							seek_manager_SM         <= waitAck;   
							port_ack 				<= 5;
							out_seek_manager 		<= SEEK_UNREACHABLE_CODE;
						end if;
					    report "Seek Manager 4 e 5" severity note;
					--elsif (free_port(SOUTH0)='1' and free_port(SOUTH1)='1' and (in_seek_fail_detect(6) = "01" or in_seek_fail_detect(7) = "01") )  then
					elsif (free_port(SOUTH0)='1' and free_port(SOUTH1)='1' and (in_seek_fail_detect(6) = "01" or in_seek_fail_detect(7) = "01") )  then
						if (in_seek_fail_detect(6) = "01") then
							out_source_seek_manager <= in_source_fail_detect(6);
							out_target_seek_manager <= in_target_fail_detect(6);
							seek_manager_SM         <= waitAck;   
							port_ack 				<= 6;
							out_seek_manager 		<= SEEK_UNREACHABLE_CODE;
							--report "Seek Manager 6 e 7 --- caiu no 6" severity note;
						elsif ( in_seek_fail_detect(7) = "01") then 
							out_source_seek_manager <= in_source_fail_detect(7);
							out_target_seek_manager <= in_target_fail_detect(7);
							seek_manager_SM         <= waitAck;   
							port_ack 				<= 7;
							out_seek_manager 		<= SEEK_UNREACHABLE_CODE;
							--report "Seek Manager 6 e 7 --- caiu no 7" severity note;
						end if;
					    --report "Seek Manager 6 e 7" severity note;
					elsif (in_seek_fail_detect(0) = "01") then
						out_source_seek_manager <= in_source_fail_detect(0);
					    out_target_seek_manager  <= in_target_fail_detect(0);
					    seek_manager_SM          <= waitAck;   
					    port_ack 				<= 0;
					    out_seek_manager 		<= SEEK_RESEND_CODE; 					 
					    --report "Seek Manager 0 " severity note;
					elsif (in_seek_fail_detect(1) = "01") then
						out_source_seek_manager <= in_source_fail_detect(1);
					    out_target_seek_manager  <= in_target_fail_detect(1);
					    seek_manager_SM          <= waitAck;   
					    port_ack 				<= 1;
					    out_seek_manager 		<= SEEK_RESEND_CODE;   
					    --report "Seek Manager 1" severity note;
					elsif (in_seek_fail_detect(2) = "01") then
						out_source_seek_manager <= in_source_fail_detect(2);
					    out_target_seek_manager  <= in_target_fail_detect(2);
					    seek_manager_SM          <= waitAck;   
					    port_ack 				<= 2;
					    out_seek_manager 		<= SEEK_RESEND_CODE;
					    --report "Seek Manager 2" severity note;
					elsif (in_seek_fail_detect(3) = "01") then
						out_source_seek_manager <= in_source_fail_detect(3);
					    out_target_seek_manager  <= in_target_fail_detect(3);
					    seek_manager_SM          <= waitAck;   
					    port_ack 				<= 3;
					    out_seek_manager 		<= SEEK_RESEND_CODE;
					    --report "Seek Manager 3" severity note;
					elsif (in_seek_fail_detect(4) = "01") then
						out_source_seek_manager <= in_source_fail_detect(4);
					    out_target_seek_manager  <= in_target_fail_detect(4);
					    seek_manager_SM          <= waitAck;   
					    port_ack 				<= 4;
					    out_seek_manager 		<= SEEK_RESEND_CODE;
					    report "Seek Manager 4" severity note;
					elsif (in_seek_fail_detect(5) = "01") then
						out_source_seek_manager <= in_source_fail_detect(5);
					    out_target_seek_manager  <= in_target_fail_detect(5);
					    seek_manager_SM          <= waitAck;   
					    port_ack 				<= 5;
						out_seek_manager 		<= SEEK_RESEND_CODE;
					    --out_seek_manager 		<= SEEK_UNREACHABLE_CODE;
					    report "Seek Manager 5" severity note;
					elsif (in_seek_fail_detect(6) = "01") then
						out_source_seek_manager <= in_source_fail_detect(6);
					    out_target_seek_manager  <= in_target_fail_detect(6);
					    seek_manager_SM          <= waitAck;   
					    port_ack 				<= 6;
					    out_seek_manager 		<= SEEK_RESEND_CODE;
					    --report "Seek Manager 6" severity note;
					elsif (in_seek_fail_detect(7) = "01") then
						out_source_seek_manager <= in_source_fail_detect(7);
					    out_target_seek_manager  <= in_target_fail_detect(7);
					    seek_manager_SM          <= waitAck;   
					    port_ack 				<= 7;
					    out_seek_manager 		<= SEEK_RESEND_CODE;
					    -- out_seek_manager 		<= SEEK_UNREACHABLE_CODE;
					    --report "Seek Manager 7" severity note;					   
					end if;

                when waitAck =>
							--out_ack_seek_fail_detect(port_ack) <= in_ack_seek_manager;
							if (in_ack_seek_manager = '1') then
								out_ack_seek_fail_detect(port_ack)  <= in_ack_seek_manager;
						  		seek_manager_SM      	  			<= waitClearOrSeekDown;
                    end if;
                    
               when waitSeekAckPE => 
               		if (in_ack_seek_manager = '1') then
               			out_ack_seek_pe			<= in_ack_seek_manager;
						out_clear_manager		<= '0';
               			seek_manager_SM			<= waitClearOrSeekDown;
               		end if;

               	--added by wachter	
               	when waitClearAckPE => 
               		if (in_ack_clear_manager = '1') then
               			out_ack_clear_pe    <= in_ack_clear_manager;
						out_seek_manager	<= (others => '0');
               			seek_manager_SM		<= waitClearOrSeekDown;
               		end if;
               	when waitClearOrSeekDown =>
               		seek_manager_SM		<= waitSeek;
            end case;
        end if;
    end process;
   
  free_port(EAST0)  <= '1' when in_seek_fail_detect(0) = "01";
  free_port(EAST1)  <= '1' when in_seek_fail_detect(1) = "01";
  free_port(WEST0)  <= '1' when in_seek_fail_detect(2) = "01";
  free_port(WEST1)  <= '1' when in_seek_fail_detect(3) = "01";
  free_port(NORTH0)  <= '1' when in_seek_fail_detect(4) = "01";
  free_port(NORTH1)  <= '1' when in_seek_fail_detect(5) = "01";
  free_port(SOUTH0)  <= '1' when in_seek_fail_detect(6) = "01";
  free_port(SOUTH1)  <= '1' when in_seek_fail_detect(7) = "01";    
  
		--if (in_seek_fail_detect(0) = "01") then
			--free_port(EAST0)  <= '1';
		--end if;
		--if (in_seek_fail_detect(1) = "01") then
			--free_port(EAST1)  <= '1';
		--end if;
		--if (in_seek_fail_detect(2) = "01") then
			--free_port(WEST0)  <= '1';
		--end if;
		--if (in_seek_fail_detect(3) = "01") then
			--free_port(WEST1)  <= '1';
		--end if;
		--if (in_seek_fail_detect(4) = "01") then
			--free_port(NORTH0)  <= '1';
		--end if;
		--if (in_seek_fail_detect(5) = "01") then
			--free_port(NORTH1)  <= '1';
		--end if;			
		--if (in_seek_fail_detect(6) = "01") then
			--free_port(SOUTH0)  <= '1';
		--end if;
		--if (in_seek_fail_detect(7) = "01") then
			--free_port(SOUTH1)  <= '1';
		--end if;
		
   --end process; 
   
   
          
end seek_manager;


