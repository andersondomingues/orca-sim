------------------------------------------------------------------------
-- test: /test_bench/HeMPS/proc(7)/slav/slave/faulty_port(7) 1 0.4 ms --
------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use IEEE.STD_LOGIC_unsigned.all;
use work.HeMPS_defaults.all;

entity fail_detect is
    port(
        clock                   : in  std_logic;
        reset                   : in  std_logic;

        -- fail detect ports
        in_failed_port          : in  std_logic;
        tx                      : in  std_logic;
        in_eop_in               : in  std_logic;
        in_data_in              : in  regflit;

        --seek manager ports
        out_source	            : out regNsource_target;  
        out_target              : out regNsource_target;   
        out_seek_fail_detect	: out regNBit_seek;
        in_ack_seek_fail_detect : in  std_logic
    );
end fail_detect; 

architecture fail_detect of fail_detect is

type state_fail_detect is (WaitPacket, ReadSourceXY, ReadSourceSR, WaitFault, FaultDetect, WaitEOP);
signal fail_detect_SM   : state_fail_detect;

signal count_flit       : std_logic_vector (3 downto 0);
signal count_seven      : std_logic_vector (3 downto 0);
signal eop_in_stepdet   : std_logic;
signal reset_eop        : std_logic;

begin
   
    stepdet: entity work.stepdet
        port map(
            ck          => clock,
            rst         => reset_eop,
            in_sig      => in_eop_in,
            out_sig     => eop_in_stepdet
        );

   --state machine of fail detect module
    process(clock,reset)
    begin 


        if (reset='1') then
            count_flit       				<= (others => '0');
            count_seven      				<= (others => '0');
            out_source 		 				<= (others => '0');
            out_target 		 				<= (others => '0');
            out_seek_fail_detect            <= NOT_SEEK;
            fail_detect_SM  				<= WaitPacket;
            reset_eop                       <= '1';
        
        elsif (clock'event and clock='1') then
            case fail_detect_SM is 
              
                when WaitPacket =>
                    count_flit              <= (others => '0');
                    count_seven				<= (others => '0');
                    reset_eop               <= '0';
                    if tx = '1' then--read target
                        count_flit          <= count_flit + 1;
                        out_target          <= in_data_in(7 downto 0);
                        if(in_data_in(15 downto 12) = "0111") then
							fail_detect_SM      <= ReadSourceSR;
							count_seven         <= count_seven + 1;
						else
							fail_detect_SM 		<= ReadSourceXY;
						end if;
						
                    end if;

                when ReadSourceXY =>
                    count_flit              <= count_flit + 1;
                    if count_flit = x"5" then
                        out_source          <= in_data_in(7 downto 0);
                        fail_detect_SM      <= WaitFault;
                    end if;
             
				when ReadSourceSR =>
				
				   	if(in_data_in(15 downto 12) = "0111") then
						count_seven         <= count_seven + 1;
					else
						count_flit          <= count_flit + 1;
					end if;
					if count_flit = x"5" then
                        out_source          <= in_data_in(7 downto 0);
                        out_target          <= in_data_in(7 downto 0);
                        fail_detect_SM      <= WaitFault;
                    end if;
					
                  
                    
                when WaitFault =>
                    if (eop_in_stepdet = '1') then-- end of packet, read another packet
                        fail_detect_SM          <= WaitPacket;
                        reset_eop               <= '1';
                    elsif in_failed_port = '1' then
                        fail_detect_SM          <= FaultDetect;
                        out_seek_fail_detect    <= SEEK_CODE;
                        report "fail detected!" severity note;
                    end if;

                when FaultDetect =>
                    if in_ack_seek_fail_detect = '1' then
                        out_seek_fail_detect    <= (others => '0');
                        fail_detect_SM          <= WaitEOP;
                        out_seek_fail_detect    <= NOT_SEEK;
                    end if;

                when WaitEOP =>
                    if eop_in_stepdet = '1' then
                        reset_eop               <= '1';
                        fail_detect_SM          <= WaitPacket;
                    end if;

            end case;
        end if;
    end process;

end fail_detect;


