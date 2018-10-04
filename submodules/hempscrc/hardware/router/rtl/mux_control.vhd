---------------------------------------------------------------------------------------------------
--
-- Title       : Router
-- Design      : QoS
-- Company     : GAPH
--
---------------------------------------------------------------------------------------------------
--
-- File        : Router.vhd
-- Generated   : Thu Mar  6 17:08:38 2008
-- From        : interface description file
-- By          : Itf2Vhdl ver. 1.20
--
---------------------------------------------------------------------------------------------------
--							   
-- Description : Implements a NoC router based on its mesh position. 
-- 	Connects the router components: Input Buffers, Crossbar and Switch Control.
--
---------------------------------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use work.HeMPS_defaults.all;

entity mux_control is
    port(
        clock		        : in  std_logic;
        reset		        : in  std_logic;
        in_buffer           : in  arrayNport_regflit;
        out_buffer          : out arrayNport_regflit;
        mux_buffer_ctrl     : out regNport;
        tx_buffer           : in  regNport;
        credit_buffer       : in  regNport;
        ack_backtrack       : in  std_logic;
        seek_resend_sr		: in  std_logic;--fochi
        flit_num            : in  std_logic_vector(2 downto 0);
        enable_mux          : in  regNport;
        enable_shift        : in  regNport;
        mask                : in  regflit
    );
end mux_control;

architecture mux_control of mux_control is

--signals of mux from buffer to crossbar
type array_counter is array(NPORT-1 downto 0) of std_logic_vector(2 downto 0);
signal counter              : array_counter;
signal shift_mux_buffer     : arrayNport_regflit;
signal in_mux_buffer        : arrayNport_regflit;


begin

    mux_gen_source_routing:for i in 0 to NPORT-1 generate
        
        --mux to insert the port directly on the out flit from buffer to crossbar
        MUX_BACKTRACK_PATH:if i = EAST1 or i = WEST1 or i = NORTH1 or i = SOUTH1 generate
            process(clock,reset,seek_resend_sr)
            begin
                if reset = '1' then
                    counter(i) <= (others => '0');
                elsif clock = '1' and clock'event then
                    if tx_buffer(i) = '1' and credit_buffer(i) = '1' then
                        counter(i) <= counter(i) - 1;
                    elsif ack_backtrack = '1' then
                        counter(i) <= flit_num;
                    elsif enable_mux(i) = '0' then
                        counter(i) <= (others => '0');
                    end if;
                end if;
                
               
                
                
                
                
            end process;
            in_mux_buffer(i)    <=  mask or in_buffer(i);
            shift_mux_buffer(i) <= "0000" & in_buffer(i)((FLIT_WIDTH-5) downto 12) & in_buffer(i)(7 downto 0) & "1111";
            
            out_buffer(i)   <=  in_mux_buffer(i) when counter(i) = "000" and enable_mux(i) = '1' else
                                    shift_mux_buffer(i) when counter(i) = "000" and enable_shift(i) = '1' else
                                    in_buffer(i);
            mux_buffer_ctrl(i)  <=  '1' when counter(i) = "000" else
                                    '0';
            
            --process(clock,reset,seek_resend_sr)
            --begin
					--if (seek_resend_sr	= '1') then
						--in_mux_buffer(i)    <= mask or in_buffer(i);--ok
						--shift_mux_buffer(i) <= "0000" & in_buffer(i)((FLIT_WIDTH-5) downto 12) & in_buffer(i)(7 downto 0) & "1111";--ok
					  
						--if(enable_mux(i) = '1') then
									--out_buffer(i)   <=  in_mux_buffer(i);
							--elsif(enable_shift(i) = '1') then
									--out_buffer(i)   <=  shift_mux_buffer(i);
							--else
									--out_buffer(i)   <= in_buffer(i);
						--end if;  
						   	
					--elsif(seek_resend_sr	= '0') then
			
						--in_mux_buffer(i)    <=  mask or in_buffer(i);
						--shift_mux_buffer(i) <= "0000" & in_buffer(i)((FLIT_WIDTH-5) downto 12) & in_buffer(i)(7 downto 0) & "1111";
			
						--if(counter(i) = "000" and enable_mux(i) = '1') then
									--out_buffer(i)   <=  in_mux_buffer(i);
							--elsif(counter(i) = "000" and enable_shift(i) = '1') then
									--out_buffer(i)   <=  shift_mux_buffer(i);
							--else
									--out_buffer(i)   <= in_buffer(i);
						--end if;   
			
						--if(counter(i) = "000" ) then
							--mux_buffer_ctrl(i)  <=  '1';
							--else
							--mux_buffer_ctrl(i)  <=  '0';
		                --end if;
					--end if;
	      --end process;                        
                                    
                                    
                                    
        end generate;
        
        --mux to reset used paths in source routing
        MUX_SOURCE_ROUTING:if i = EAST0 or i = WEST0 or i = NORTH0 or i = SOUTH0 or i = LOCAL0 or i = LOCAL1  generate
        
            in_mux_buffer(i)    <= mask or in_buffer(i);
            shift_mux_buffer(i) <= "0000" & in_buffer(i)((FLIT_WIDTH-5) downto 12) & in_buffer(i)(7 downto 0) & "1111";
            
            out_buffer(i)   <=  in_mux_buffer(i) when enable_mux(i) = '1' else
                                    shift_mux_buffer(i) when enable_shift(i) = '1' else
                                    in_buffer(i);
        end generate;
    end generate;
     
    
end mux_control;
