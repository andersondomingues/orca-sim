------------------------------------------------------------------
---
--- seek router module 01/03/2012
---
--- File function: searches the network for a possible path between a pair of source - target
---
--- Responsibles: Eduardo Wachter
--- 
--- Contact: eduardo.wachter@acad.pucrs.br
---
---1 fio:  seek
---1 fio:  clear
---16 fios:  para origem/destino
---6 bits: hop counter
------------------------------------------------------------------
library ieee;
use work.HeMPS_defaults.all;
use ieee.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
--use IEEE.std_logic_arith.all;
use ieee.numeric_std.all;
library unisim;
use UNISIM.vcomponents.all;


entity seek is
        generic(
            seek_address                : regHalfFlit   := (others => '0');
            DEBUG                       : boolean       := true
        );  
        port(   
            clock                       : in  std_logic;
            reset                       : in  std_logic;
            in_seek                     : in  regNport_seek_NBit;
            out_ack_seek                : out regNport_seek;
            in_clear                    : in  regNport_seek;
            out_ack_clear               : out regNport_seek;
            in_source                   : in  regNportNsource_target;
            in_target                   : in  regNportNsource_target;
            in_hop_counter              : in  regNportNhop;
                
            out_seek                    : out regNport_seek_NBit;
            in_ack_seek                 : in  regNport_seek;
            out_clear                   : out regNport_seek;
            in_ack_clear                : in  regNport_seek;
            out_source                  : out regNportNsource_target;
            out_target                  : out regNportNsource_target;
            out_hop_counter             : out regNportNhop;
                
            req_backtrack               : in  std_logic;
            ack_backtrack               : out std_logic;
            source                      : in  regNsource_target;
            flit_num                    : out std_logic_vector(2 downto 0);
            flit_pos                    : out std_logic_vector(2 downto 0);
            port_dir                    : out std_logic_vector(1 downto 0);

            seek_unreachable_interrupt  : out std_logic;
            seek_unreachable_target     : out regNsource_target;
			seek_resend_interrupt  		: out std_logic;
            
            data_out                    : out regflit;  
            rx_out                      : out std_logic;
            eop_out                     : out std_logic;
            credit_in                   : in  std_logic;
            port_ctrl                   : out std_logic;
            free_port                   : in  std_logic
        );
end entity;

architecture seek of seek is
    
    type T_ea_search            is (S_INIT, S_READ_AVAIL, S_SEARCH_0, S_SEARCH_1, S_SEARCH_2, S_SEARCH_3, S_FILL_TABLE_0, S_FILL_TABLE_1, S_FILL_TABLE_2, S_ABORT, S_AVAIL_SPACE, S_FOUND, S_WAIT_FREE_L0, S_SEND_BACKTRACK_HEADER, S_SEND_BACKTRACK_PAYLOAD, S_BACKTRACK_REQUEST, S_SEARCH_BACKTRACK, S_PROPAGATE, S_PROPAGATE_CLEAR, S_SET_UNUSED_0, S_SET_UNUSED_1, S_SET_UNUSED_2, S_WAIT_BACKTRACK_REQUEST_DOWN, S_WAIT_ACK_SEEK, S_WAIT_ACK_CLEAR);
    signal EA_search,PE_search  : T_ea_search;
    
    type T_ea_query            is (INIT, S_CLEAR_PULSE, S_SEEK_PULSE);
    signal EA_query,PE_query  : T_ea_query;

    signal seek_pulse           : regNport_seek_NBit;
    signal reset_seek           : regNport_seek;
       
    signal clear_pulse          : regNport_seek;
    signal reset_clear          : regNport_seek;
    signal treating_clear       : std_logic;
    signal seek_unreachable	 	: std_logic;
    signal seek_resend			: std_logic;
    
    --signal i                    : integer range 0 to NPORT/2-1;
    
    signal counter              : std_logic_vector(2 downto 0);
    
    signal free_slot            : integer range -1 to TABLE_HEIGHT;
    
    --LUTRAM table signals
    signal out_table0           : std_logic_vector(LUTRAM_WIDTH-1 downto 0);
    signal out_table1           : std_logic_vector(LUTRAM_WIDTH-1 downto 0);
    signal addr0                : std_logic_vector(3 downto 0);
    signal in_table             : std_logic_vector(LUTRAM_WIDTH-1 downto 0);
    signal addr1                : std_logic_vector(3 downto 0);
    signal we                   : std_logic;
    
    --aliases to access the LUTRAM
    alias read_used0:std_logic                                          is out_table0(SOURCE_TARGET_SIZE);
    alias read_used1:std_logic                                          is out_table1(SOURCE_TARGET_SIZE);
    alias read_target:std_logic_vector(SOURCE_TARGET_SIZE-1 downto 0)   is out_table0(SOURCE_TARGET_SIZE-1 downto 0);
    alias read_source:std_logic_vector(SOURCE_TARGET_SIZE-1 downto 0)   is out_table1(SOURCE_TARGET_SIZE-1 downto 0);
    alias read_port_dir:std_logic_vector(1 downto 0)                    is out_table0(HOP_SIZE+1 downto HOP_SIZE);
    alias read_hop_count:std_logic_vector(HOP_SIZE-1 downto 0)          is out_table0(HOP_SIZE-1 downto 0);
    
    alias write_used                                                    is in_table(SOURCE_TARGET_SIZE);
    alias write_unreachable                                             is in_table(SOURCE_TARGET_SIZE);
    alias write_target:std_logic_vector(SOURCE_TARGET_SIZE-1 downto 0)  is in_table(SOURCE_TARGET_SIZE-1 downto 0);
    alias write_source:std_logic_vector(SOURCE_TARGET_SIZE-1 downto 0)  is in_table(SOURCE_TARGET_SIZE-1 downto 0);
    alias write_port_dir:std_logic_vector(1 downto 0)                   is in_table(HOP_SIZE+1 downto HOP_SIZE);
    alias write_hop_count:std_logic_vector(HOP_SIZE-1 downto 0)         is in_table(HOP_SIZE-1 downto 0);
    
    --signal for BACKTRACK
    signal packet_size          	: std_logic_vector(2 downto 0);
    signal source_reg           	: regNsource_target;
    signal broadcast_clear      	: std_logic;
   	
    
    --DEBUG signals
    signal DEBUG_table          : table;
    
    --signals for query seek/clear table
    signal out_query_table      : std_logic_vector(26 downto 0);
    signal addr_read            : std_logic_vector(3 downto 0);
    signal in_query_table       : std_logic_vector(26 downto 0);
    signal addr_write           : std_logic_vector(3 downto 0);
    signal we_query_table       : std_logic;
    
    alias search_target         :regNsource_target is out_query_table(HOP_SIZE+SOURCE_TARGET_SIZE+SOURCE_TARGET_SIZE-1 downto HOP_SIZE+SOURCE_TARGET_SIZE);
    alias search_source         :regNsource_target is out_query_table(HOP_SIZE+SOURCE_TARGET_SIZE-1 downto HOP_SIZE);
    alias search_hop_count      :regNhop is out_query_table(HOP_SIZE-1 downto 0);
    alias search_port           :std_logic_vector(2 downto 0) is out_query_table(HOP_SIZE+SOURCE_TARGET_SIZE+SOURCE_TARGET_SIZE+2 downto HOP_SIZE+SOURCE_TARGET_SIZE+SOURCE_TARGET_SIZE);
    
    alias in_search_target         :regNsource_target is in_query_table(HOP_SIZE+SOURCE_TARGET_SIZE+SOURCE_TARGET_SIZE-1 downto HOP_SIZE+SOURCE_TARGET_SIZE);
    alias in_search_source         :regNsource_target is in_query_table(HOP_SIZE+SOURCE_TARGET_SIZE-1 downto HOP_SIZE);
    alias in_search_hop_count      :regNhop is in_query_table(HOP_SIZE-1 downto 0);
    alias in_search_port           :std_logic_vector(2 downto 0) is in_query_table(HOP_SIZE+SOURCE_TARGET_SIZE+SOURCE_TARGET_SIZE+2 downto HOP_SIZE+SOURCE_TARGET_SIZE+SOURCE_TARGET_SIZE);
begin

    gen_stepdet: for i in 0 to NPORT_SEEK-1 generate
        stepdet_seek0: entity work.stepdet
        port map(
            ck          => clock,
            rst         => reset_seek(i),
            in_sig      => in_seek(i)(0),
            out_sig     => seek_pulse(i)(0)
        );
        stepdet_seek1: entity work.stepdet
        port map(
            ck          => clock,
            rst         => reset_seek(i),
            in_sig      => in_seek(i)(1),
            out_sig     => seek_pulse(i)(1)
        );
        
        stepdet_clear: entity work.stepdet
        port map(
            ck          => clock,
            rst         => reset_clear(i),
            in_sig      => in_clear(i),
            out_sig     => clear_pulse(i)
        );
    end generate;
    
    out_ack_seek <= (seek_pulse(LOCAL)(0) or seek_pulse(LOCAL)(1)) & (seek_pulse(SOUTH)(0) or seek_pulse(SOUTH)(1))
		 & (seek_pulse(NORTH)(0) or seek_pulse(NORTH)(1)) & (seek_pulse(WEST)(0) or seek_pulse(WEST)(1)) & (seek_pulse(EAST)(0) or seek_pulse(EAST)(1));
    out_ack_clear <= clear_pulse;

    gen_table: for i in 0 to LUTRAM_WIDTH-1 generate
        u_RAM16X1D_1 : RAM16X1D_1
        generic map(
            INIT => X"0000"
        )
        port map(
            DPO     => out_table1(i),   -- Read port 1-bit output
            SPO     => out_table0(i),   -- Read/Write port 1-bit ouput
            A0      => addr0(0),        -- Read/Write port 1-bit address input
            A1      => addr0(1),        -- Read/Write port 1-bit address input
            A2      => addr0(2),        -- Read/Write port 1-bit address input
            A3      => addr0(3),        -- Read/Write port 1-bit address input
            D       => in_table(i),     -- RAM data input
            DPRA0   => addr1(0),        -- Read port 1-bit address input
            DPRA1   => addr1(1),        -- Read port 1-bit address input
            DPRA2   => addr1(2),        -- Read port 1-bit address input
            DPRA3   => addr1(3),        -- Read port 1-bit address input
            WCLK    => clock,           -- Write clock input
            WE      => we               -- RAM data input
        );
    end generate;
                
           
    	
    process(reset, clock)
    begin
        if reset = '1' then
            EA_search                   <= S_INIT;
                
            counter                     <= (others => '0');
                
            out_seek                    <= (others => NOT_SEEK);
            out_clear                   <= (others => '0');
            out_source                  <= (others => (others => '0'));
            out_target                  <= (others => (others => '0'));
            out_hop_counter             <= (others => (others => '0'));
                
            --reset_clear                 <= (others => '1');
            free_slot                   <= -1;
                
            addr0                       <= (others => '0');
            addr1                       <= (others => '0');
            in_table                    <= (others => '0');
            we                          <= '0';
                
            data_out(15 downto 0)	    <= (others => '0'); 
            
            rx_out                      <= '0';
            eop_out                     <= '0';
            port_ctrl                   <= '0';
                
            ack_backtrack               <= '0';
            flit_num                    <= (others => '0');
            flit_pos                    <= (others => '0');
            port_dir                    <= (others => '0');
            addr_read                   <= (others => '1');

            seek_unreachable_interrupt  <= '0';
            seek_unreachable_target     <= (others => '0');
			seek_resend_interrupt  		<= '0';
            
        elsif clock'event and clock='1' then
            case EA_search is
                when S_INIT =>
                    --i                         <= 0;
                    --reset_clear               <= (others => '0');
                    counter                     <= (others => '0');
                    out_seek                    <= (others => NOT_SEEK);
                    out_clear                   <= (others => '0');
                    free_slot                   <= -1;
                    port_ctrl                   <= '0';
                    ack_backtrack               <= '0';
                    treating_clear              <= '0';
                    broadcast_clear             <= '0';
                    seek_unreachable            <= '0';
                    seek_unreachable_interrupt  <= '0';
					seek_resend					<= '0';
                    seek_resend_interrupt  		<= '0';
					
                    seek_unreachable_target     <= (others => '0');
                    
                    if addr_write /= addr_read and req_backtrack = '0' then
                        addr_read <= addr_read + 1;
                    end if;
                
                when S_READ_AVAIL => --verifica o tipo de serviço do seek foi solicitado.
                    
                    if out_query_table(26 downto 25) = "00" then
                        treating_clear          <= '1';
                    elsif out_query_table(26 downto 25) = SEEK_UNREACHABLE_CODE then
                        seek_unreachable        <= '1';
                    elsif out_query_table(26 downto 25) = SEEK_RESEND_CODE then
						seek_resend        		<= '1';
                    end if;
                    addr0                   <= x"0";
                    addr1                   <= x"1";

        		-- when S_SEEK_UNREACHABLE =>

        		--    	if search_source = seek_address then -- if the seek unreachable source address is my address
          --               seek_unreachable_interrupt  <= '1';
          --               seek_unreachable_target     <= search_target;

        		-- 	else-- propagate the seek unreachable, except for the incomming port
    		    --         for j in 0 to (NPORT/2)-1 loop 
    		    --             out_source(j)        <= search_source;
    		    --             out_target(j)        <= search_target;
    		    --         end loop;

        		-- 	    if search_port /= "000" then
    		    --             out_seek(EAST)		<= SEEK_UNREACHABLE_CODE;
    		    --         else
    		    --             out_seek(EAST)		<=  NOT_SEEK;
    		    --         end if;
    		            
    		    --         if search_port /= "001" then
    		    --             out_seek(WEST)		<= SEEK_UNREACHABLE_CODE;
    		    --         else
    		    --             out_seek(WEST)		<= NOT_SEEK;
    		    --         end if;
    		                
    		    --         if search_port /= "010" then
    		    --             out_seek(NORTH)		<= SEEK_UNREACHABLE_CODE;
    		    --         else
    		    --             out_seek(NORTH)		<= NOT_SEEK;
    		    --         end if;
    		            
    		    --         if search_port /= "011" then
    		    --             out_seek(SOUTH)		<= SEEK_UNREACHABLE_CODE;
    		    --         else
    		    --             out_seek(SOUTH)		<= NOT_SEEK;
          --               end if;
        		-- 	end if;
                    
                    
                when S_SEARCH_0 => --searchs the position 0 of the table
                
                    if read_used0 = '1' and read_source = search_source then
                        if treating_clear = '1' then
                            we              <= '1';
                            write_used      <= '0';
                            for j in 0 to (NPORT/2)-1 loop 
                                out_source(j)        <= read_source;
                            end loop;
                        end if;
                    else
                            addr0           <= x"4";
                            addr1           <= x"5";
                    end if;
                    if read_used0 = '0' then
                        free_slot <= 0;
                    end if;
                    
                when S_SEARCH_1 => --searchs the table
                                    
                    if read_used0 = '1' and read_source = search_source then
                        if treating_clear = '1' then
                            we              <= '1';
                            write_used      <= '0';
                    for j in 0 to (NPORT/2)-1 loop 
                        out_source(j)        <= read_source;
                    end loop;
                        end if;
                    else
                            addr0           <= x"8";
                            addr1           <= x"9";
                    end if;
                    if read_used0 = '0' then
                        free_slot <= 1;
                    end if;
                    
                when S_SEARCH_2 => --searchs the table
                    if read_used0 = '1' and read_source = search_source then
                        if treating_clear = '1' then
                            we              <= '1';
                            write_used      <= '0';
                    for j in 0 to (NPORT/2)-1 loop 
                        out_source(j)        <= read_source;
                    end loop;
                    end if;
                    else
                            addr0           <= x"C";
                            addr1           <= x"D";
                    end if;
                    if read_used0 = '0' then
                        free_slot <= 2;
                    end if;
                    
                when S_SEARCH_3 => --searchs the table
                    if read_used0 = '1' and read_source = search_source then
                        if treating_clear = '1' then
                            we              <= '1';
                            write_used      <= '0';
							for j in 0 to (NPORT/2)-1 loop 
								out_source(j)        <= read_source;
							end loop;
                        end if;
					end if;
                    if read_used0 = '0' then
                        free_slot <= 3;
                    end if;
                    
                when S_AVAIL_SPACE => null;
                                       
                    
                when S_FILL_TABLE_0 =>--this state writes the used and the target
                    case free_slot is
                        when 0 =>
                            addr0 <= x"0";
                        when 1 =>
                            addr0 <= x"4";
                        when 2 =>
                            addr0 <= x"8";
                        when others =>
                            addr0 <= x"C";
                    end case;
                    
                    we              <= '1';
                    write_used      <= '1';
                    write_target    <= search_target;
                    
                    if DEBUG then
                        DEBUG_table(free_slot).used         <= '1';
                        DEBUG_table(free_slot).source       <= search_source;
                        DEBUG_table(free_slot).target       <= search_target;
                        DEBUG_table(free_slot).hop_count    <= search_hop_count;
                        case search_port is
                            when "000"   =>  DEBUG_table(free_slot).port_dir <= EAST_DIR;
                            when "001"   =>  DEBUG_table(free_slot).port_dir <= WEST_DIR;
                            when "010"   =>  DEBUG_table(free_slot).port_dir <= NORTH_DIR;
                            when others =>  DEBUG_table(free_slot).port_dir <= SOUTH_DIR;
                        end case;
                    end if;
                
                when S_FILL_TABLE_1 =>--this state writes the source
                    addr0               <= addr0 + 1;
                    we                  <= '1';
					--write_resend? <= seek_resend ??
                    write_unreachable   <= seek_unreachable;
                    write_source        <= search_source;
                                    
                when S_FILL_TABLE_2 =>--this state writes the port and the hop_count
                    addr0           <= addr0 + 1;
                    we              <= '1';
                    write_used      <= '1';
                    write_port_dir  <= search_port(1 downto 0);
                    write_hop_count <= search_hop_count;
                
                when S_PROPAGATE =>
                    for j in 0 to (NPORT/2)-1 loop 
                        out_source(j)        <= search_source;
                        out_target(j)        <= search_target;
                        out_hop_counter(j)   <= search_hop_count+1;
                    end loop;
                    
                    if search_port /= "000" then
                        if seek_unreachable = '1' then --propagate the seek unreachable, except for the incomming port
                            out_seek(EAST)          <= SEEK_UNREACHABLE_CODE;
                        elsif seek_resend = '1' then
							out_seek(EAST)          <= SEEK_RESEND_CODE;
                        else
                            out_seek(EAST)          <= SEEK_CODE;
                        end if;
                    else
                        out_seek(EAST)          <= NOT_SEEK;
                    end if;
                    
                    if search_port /= "001" then
                        if seek_unreachable = '1' then --propagate the seek unreachable, except for the incomming port
                            out_seek(WEST)          <= SEEK_UNREACHABLE_CODE;
						elsif seek_resend = '1' then
							out_seek(WEST)          <= SEEK_RESEND_CODE;
                        else
                            out_seek(WEST)          <= SEEK_CODE;
                        end if;
                    else
                        out_seek(WEST)          <= NOT_SEEK;
                    end if;
                        
                    if search_port /= "010" then
                        if seek_unreachable = '1' then --propagate the seek unreachable, except for the incomming port
                            out_seek(NORTH)          <= SEEK_UNREACHABLE_CODE;
                        elsif seek_resend = '1' then
							out_seek(NORTH)          <= SEEK_RESEND_CODE;
                        else
                            out_seek(NORTH)          <= SEEK_CODE;
                        end if;
                    else
                        out_seek(NORTH)         <= NOT_SEEK;
                    end if;
                    
                    if search_port /= "011" then
                        if seek_unreachable = '1' then --propagate the seek unreachable, except for the incomming port
                            out_seek(SOUTH)          <= SEEK_UNREACHABLE_CODE;
                        elsif seek_resend = '1' then
							out_seek(SOUTH)          <= SEEK_RESEND_CODE;
                        else
                            out_seek(SOUTH)          <= SEEK_CODE;
                        end if;
                    else
                        out_seek(SOUTH)         <= NOT_SEEK;
                    end if;
                    
                    we                      <= '0';
                    
                when S_WAIT_ACK_SEEK =>
                    
                    --out_seek            <= (others => '0');
                    
                when S_WAIT_ACK_CLEAR =>
                    
                    --out_seek            <= (others => '0');
                
                when S_FOUND =>				---			QUANDO REQUISICAO CHEGA NA ORIGEM
                    we                      <= '0';
                    if seek_unreachable = '1' then
                        seek_unreachable_interrupt  <= '1';
                        seek_unreachable_target     <= search_target;
                        report "UNREACHABLE FOUND A PATH: from " & CONV_STRING_8BITS(search_source) & " to " & CONV_STRING_8BITS(search_target) severity note;
                    else
                        report "SEEK FOUND A PATH: from " & CONV_STRING_8BITS(search_source) & " to " & CONV_STRING_8BITS(search_target) severity note;
                    end if;
				  
 					if seek_resend = '1' then
                        seek_resend_interrupt  <= '1';
                        seek_unreachable_target     <= search_target;
                        report "RESEND: from " & CONV_STRING_8BITS(search_source) & " to " & CONV_STRING_8BITS(search_target) severity note;
                    end if;

                
                when S_WAIT_FREE_L0 =>
                    packet_size <= search_hop_count(HOP_SIZE-1 downto 3) + 1;
                    if free_port = '1' then
                        port_ctrl           <= '1';
                    else
                        port_ctrl           <= '0';
                    end if;
        	
                when S_SEND_BACKTRACK_HEADER =>
                    data_out(15 downto 0)				<= BACKTRACK_PATH & "0001" & search_source;-- fochi
                    rx_out                  <= '1';
                    eop_out                 <= '0';
                    port_ctrl               <= '1';
                    port_dir                <= write_port_dir;
                
                when S_SEND_BACKTRACK_PAYLOAD =>
                    if credit_in = '1' then
                        packet_size         <= packet_size - 1;
                    end if;
                    
                    if packet_size = 0 then
                        if credit_in = '1' then --last packet
                            rx_out              <= '0';
                            eop_out             <= '0';
                        end if;
                    elsif packet_size = 1 then
                        rx_out                  <= '1';
                        eop_out                 <= '1';
                        case search_hop_count(2 downto 0) is
                            when "000" =>
								data_out(15 downto 0)	<= write_port_dir & "00000000000000";                            
                            when "001" =>							
								data_out(15 downto 0)	<= "00" & write_port_dir & "000000000000";                                                            
                            when "010" =>								
								data_out(15 downto 0)	<= "0000" & write_port_dir & "0000000000";                            
                            when "011" =>								
								data_out(15 downto 0)	<= "000000" & write_port_dir & "00000000";                            
                            when "100" =>								
								data_out(15 downto 0)   <= "00000000" & write_port_dir & "000000";                            
                            when "101" =>								
								data_out(15 downto 0)   <= "0000000000" & write_port_dir & "0000";                            
                            when "110" =>								
                                data_out(15 downto 0)  <=  "000000000000" & write_port_dir & "00";                           
                            when others =>								
								data_out(15 downto 0)	<= "00000000000000" & write_port_dir;

                            
                        end case;
                       
                    else
                        rx_out                  <= '1';
                        data_out(15 downto 0)   <= (others => '0'); -- fochi
                    end if;
                
                when S_BACKTRACK_REQUEST =>
                    addr1                       <= x"1";--source is addressed by addr1
                    addr0                       <= x"2";--port_dir and hop_count are addressed by addr0
                    source_reg                  <= source;
                    
                when S_SEARCH_BACKTRACK =>
                    if source_reg = read_source and read_used0 = '1' then
                        ack_backtrack           <= '1';
                        flit_num                <= read_hop_count(HOP_SIZE-1 downto 3)+1;
                        flit_pos                <= read_hop_count(2 downto 0);
                        port_dir                <= read_port_dir;
                    else
                        addr0                   <= addr0 + 4;
                        addr1                   <= addr1 + 4;
                    end if;
                    
                when S_WAIT_BACKTRACK_REQUEST_DOWN =>
                    -- if source_reg = seek_address and req_backtrack = '0' then
                    --     treating_clear  <= '1';
                    --     we              <= '1';
                    --     write_used      <= '0';
                    --     addr0           <= addr0 - 2;
                    --     addr1           <= addr0 - 2;
                    --     for j in 0 to (NPORT/2)-1 loop 
                    --         out_source(j)        <= read_source;
                    --     end loop;
                    -- end if;
                    -- broadcast_clear     <= '1';
                    
                when S_SET_UNUSED_0 =>
                    ack_backtrack   <= '0';
                    addr0           <= addr0 + 1;
                    we              <= '1';
                    write_used      <= '0';
                    
                    if DEBUG then
                        DEBUG_table(to_integer(unsigned(addr0))/4).used         <= '0';
                        DEBUG_table(to_integer(unsigned(addr0))/4).source       <= (others => '0');
                        DEBUG_table(to_integer(unsigned(addr0))/4).target       <= (others => '0');
                        DEBUG_table(to_integer(unsigned(addr0))/4).hop_count    <= (others => '0');
                        --case addr0 is
                            --when x"0" =>
                                --if(DEBUG_table(1).used = '0' and DEBUG_table(2).used = '0' and DEBUG_table(3).used = '0') then
                                    --report "all slots of table from source " & CONV_STRING_8BITS(seek_address) & " are cleared!" severity note;
                                --end if;
                            --when x"4" =>
                                --if(DEBUG_table(0).used = '0' and DEBUG_table(2).used = '0' and DEBUG_table(3).used = '0') then
                                    --report "all slots of table from source " & CONV_STRING_8BITS(seek_address) & " are cleared!" severity note;
                                --end if;
                            --when x"8" =>
                                --if(DEBUG_table(0).used = '0' and DEBUG_table(1).used = '0' and DEBUG_table(3).used = '0') then
                                    --report "all slots of table from source " & CONV_STRING_8BITS(seek_address) & " are cleared!" severity note;
                                --end if;
                            --when x"C" =>
                                --if(DEBUG_table(0).used = '0' and DEBUG_table(1).used = '0' and DEBUG_table(2).used = '0') then
                                    --report "all slots of table from source " & CONV_STRING_8BITS(seek_address) & " are cleared!" severity note;
                                --end if;
                            --when others =>
                        --end case;
                    end if;
                
                when S_SET_UNUSED_1 =>
                    addr0           <= addr0 + 1;
                    we              <= '1';
                    write_used      <= '0';
                
                when S_SET_UNUSED_2 =>
                    we              <= '0';
                    
                when S_PROPAGATE_CLEAR =>
                    
                    if search_port /= "000" or broadcast_clear = '1' then
                        out_clear(EAST)          <= '1';
                    else
                        out_clear(EAST)          <= '0';
                    end if;
                    
                    if search_port /= "001" or broadcast_clear = '1' then
                        out_clear(WEST)          <= '1';
                    else
                        out_clear(WEST)          <= '0';
                    end if;
                        
                    if search_port /= "010" or broadcast_clear = '1' then
                        out_clear(NORTH)         <= '1';
                    else
                        out_clear(NORTH)         <= '0';
                    end if;
                    
                    if search_port /= "011" or broadcast_clear = '1' then
                        out_clear(SOUTH)         <= '1';
                    else
                        out_clear(SOUTH)         <= '0';
                    end if;
                
                when S_ABORT =>
                    report "NO SPACE ON THE TABLE!!" severity failure;           
                
            end case;
            EA_search <= PE_search;
            
        end if;
    end process;

    process(EA_search, seek_pulse, seek_unreachable, seek_resend, clear_pulse, read_source, read_target, read_used0, read_used1, search_source, search_target, packet_size, credit_in, free_slot, req_backtrack, treating_clear, in_ack_seek, free_port)
    begin
        case EA_search is
            when S_INIT =>
                if req_backtrack = '1' then
                    PE_search <= S_BACKTRACK_REQUEST;
                elsif addr_write /= addr_read then
                    PE_search <= S_READ_AVAIL;
                else
                    PE_search <= S_INIT;
                end if;
                
            when S_READ_AVAIL =>
        		-- if out_query_table(26 downto 25) = SEEK_UNREACHABLE_CODE then
        		-- 	PE_search <= S_SEEK_UNREACHABLE;
        		-- else
                    PE_search <= S_SEARCH_0;
        		-- end if;

    	    -- when S_SEEK_UNREACHABLE =>
         --        PE_search <= S_INIT;
                    
                --busca em todas as 4 posicoes da tabela, se a origem esta armazenada
            when S_SEARCH_0 => --searchs the table position 0
                if treating_clear = '1' and read_used0 = '1' and read_source = search_source then
                    PE_search <= S_SET_UNUSED_0;
                elsif (read_used0 = '1' and (read_source /= search_source)) or read_used0 = '0' then
                    PE_search <= S_SEARCH_1;
                else
                    PE_search <= S_INIT;
                end if;
                
            when S_SEARCH_1 => --searchs the table position 1
                if treating_clear = '1' and read_used0 = '1' and read_source = search_source then
                    PE_search <= S_SET_UNUSED_0;
                elsif (read_used0 = '1' and (read_source /= search_source)) or read_used0 = '0' then
                    PE_search <= S_SEARCH_2;
                else
                    PE_search <= S_INIT;
                end if;
                
            when S_SEARCH_2 => --searchs the table position 2
                if treating_clear = '1' and read_used0 = '1' and read_source = search_source then
                    PE_search <= S_SET_UNUSED_0;
                elsif (read_used0 = '1' and (read_source /= search_source)) or read_used0 = '0' then
                    PE_search <= S_SEARCH_3;
                else
                    PE_search <= S_INIT;
                end if;
                
            when S_SEARCH_3 => --searchs the table position 3
                if treating_clear = '1' then
                    if read_used0 = '1' and read_source = search_source then
                        PE_search <= S_SET_UNUSED_0;
                    else
                        PE_search <= S_INIT;
                    end if;

                --elsif (read_used0 = '1' and ((read_source /= search_source) or seek_unreachable = '1')) or read_used0 = '0' then --in the case of seek_unreachable, I just need to save the source, there is no problem with rewriting
                elsif (read_used0 = '1' and (read_source /= search_source)) or read_used0 = '0' then
                    PE_search <= S_AVAIL_SPACE;
                else
                    PE_search <= S_INIT;
                end if;
                
            when S_AVAIL_SPACE =>
                if free_slot = -1 then
                    PE_search <= S_ABORT;
                else
                    PE_search <= S_FILL_TABLE_0;
                end if;

            when S_FILL_TABLE_0 =>
                PE_search <= S_FILL_TABLE_1;
                
            when S_FILL_TABLE_1 =>
                PE_search <= S_FILL_TABLE_2;
            
            -- 33         =  33              and                     0             1    
            -- 10         =  33                                  0             1
            --search_target = 33
            --seek_address  = 33
            
               --if search_target = seek_address and seek_unreachable = '0' then -- chegou na origem
               --     PE_search <= S_FOUND;
              --  elsif search_source = seek_address and seek_unreachable = '1' then -- chegou no destino.
               --     PE_search <= S_FOUND;
             --   else
               --     PE_search <= S_WAIT_ACK_SEEK;
              --  end if;
            --search_source = 10
            --seek_resend 
            when S_FILL_TABLE_2 =>
            
				if  search_target = seek_address and seek_unreachable = '0' and seek_resend = '0' then -- não entrar no seek alvo quando bater a 1 onda, e entrar aqui quando bater uma terceira onda.
                    PE_search <= S_FOUND;                    
                elsif search_source = seek_address and (seek_resend = '1' or seek_unreachable = '1') then ---- entrar quando a onda bater na origem.
                    PE_search <= S_FOUND;
                else
                    PE_search <= S_WAIT_ACK_SEEK;
                end if;
                
            when S_PROPAGATE =>
                PE_search <= S_INIT;
                
            when S_WAIT_ACK_SEEK =>
                case search_port is
                    when "000" =>
                        if in_ack_seek(WEST) = '0' and in_ack_seek(NORTH) = '0' and in_ack_seek(SOUTH) = '0' then
                            PE_search <= S_PROPAGATE;
                        else
                            PE_search <= S_WAIT_ACK_SEEK;
                        end if;
                    when "001" =>
                        if in_ack_seek(EAST) = '0' and in_ack_seek(NORTH) = '0' and in_ack_seek(SOUTH) = '0' then
                            PE_search <= S_PROPAGATE;
                        else
                            PE_search <= S_WAIT_ACK_SEEK;
                        end if;
                    when "010" =>
                        if in_ack_seek(EAST) = '0' and in_ack_seek(WEST) = '0' and in_ack_seek(SOUTH) = '0' then
                            PE_search <= S_PROPAGATE;
                        else
                            PE_search <= S_WAIT_ACK_SEEK;
                        end if;
                    when "011" =>
                        if in_ack_seek(EAST) = '0' and in_ack_seek(WEST) = '0' and in_ack_seek(NORTH) = '0' then
                            PE_search <= S_PROPAGATE;
                        else
                            PE_search <= S_WAIT_ACK_SEEK;
                        end if;
                    when others =>
                        if in_ack_seek(EAST) = '0' and in_ack_seek(WEST) = '0' and in_ack_seek(NORTH) = '0' and in_ack_seek(SOUTH) = '0' then
                            PE_search <= S_PROPAGATE;
                        else
                            PE_search <= S_WAIT_ACK_SEEK;
                        end if;
                end case;
                
            when S_WAIT_ACK_CLEAR =>
                case search_port is
                    when "000" =>
                        if in_ack_clear(WEST) = '0' and in_ack_clear(NORTH) = '0' and in_ack_clear(SOUTH) = '0' then
                            PE_search <= S_PROPAGATE_CLEAR;
                        else
                            PE_search <= S_WAIT_ACK_CLEAR;
                        end if;
                    when "001" =>
                        if in_ack_clear(EAST) = '0' and in_ack_clear(NORTH) = '0' and in_ack_clear(SOUTH) = '0' then
                            PE_search <= S_PROPAGATE_CLEAR;
                        else
                            PE_search <= S_WAIT_ACK_CLEAR;
                        end if;
                    when "010" =>
                        if in_ack_clear(EAST) = '0' and in_ack_clear(WEST) = '0' and in_ack_clear(SOUTH) = '0' then
                            PE_search <= S_PROPAGATE_CLEAR;
                        else
                            PE_search <= S_WAIT_ACK_CLEAR;
                        end if;
                    when "011" =>
                        if in_ack_clear(EAST) = '0' and in_ack_clear(WEST) = '0' and in_ack_clear(NORTH) = '0' then
                            PE_search <= S_PROPAGATE_CLEAR;
                        else
                            PE_search <= S_WAIT_ACK_CLEAR;
                        end if;
                    when others =>
                        if in_ack_clear(EAST) = '0' and in_ack_clear(WEST) = '0' and in_ack_clear(NORTH) = '0' and in_ack_clear(SOUTH) = '0' then
                            PE_search <= S_PROPAGATE_CLEAR;
                        else
                            PE_search <= S_WAIT_ACK_CLEAR;
                        end if;
                end case;
                
            when S_FOUND =>
                if seek_unreachable = '1' or seek_resend = '1' then
                    PE_search <= S_INIT;
                else
                    PE_search <= S_WAIT_FREE_L0;
                end if;
                
            when S_WAIT_FREE_L0 =>
                if free_port = '1' then
                    PE_search <= S_SEND_BACKTRACK_HEADER;
                else
                    PE_search <= S_WAIT_FREE_L0;
                end if;
            
            when S_SEND_BACKTRACK_HEADER =>
                PE_search <= S_SEND_BACKTRACK_PAYLOAD;
                
            when S_SEND_BACKTRACK_PAYLOAD =>
                if packet_size = 0 and credit_in = '1' then
                    PE_search <= S_INIT;
                else
                    PE_search <= S_SEND_BACKTRACK_PAYLOAD;
                end if;
                
            when S_BACKTRACK_REQUEST =>
                PE_search <= S_SEARCH_BACKTRACK;
            
            when S_SEARCH_BACKTRACK =>
                if source_reg = read_source and read_used0 = '1' then
                    PE_search <= S_WAIT_BACKTRACK_REQUEST_DOWN;
                else
                    PE_search <= S_SEARCH_BACKTRACK;
                end if;
                
            when S_WAIT_BACKTRACK_REQUEST_DOWN =>
                if req_backtrack = '0' then
                    -- if source_reg = seek_address then
                    --     --report "iniciando clear!" severity note;
                    --     PE_search <= S_SET_UNUSED_0;
                    -- else
                        PE_search <= S_INIT;
                    -- end if;
                else
                    PE_search <= S_WAIT_BACKTRACK_REQUEST_DOWN;
                end if;
                    
            when S_SET_UNUSED_0 =>
                PE_search <= S_SET_UNUSED_1;
            
            when S_SET_UNUSED_1 =>
                PE_search <= S_SET_UNUSED_2;
            
            when S_SET_UNUSED_2 =>
                PE_search <= S_WAIT_ACK_CLEAR;
            
            when S_PROPAGATE_CLEAR =>
                PE_search <= S_INIT;
                
            when S_ABORT =>
                PE_search <= S_ABORT;
                
        end case;
    end process;
    
 ------ acima ele trata  
 ------ recebe solicitações abaixo   
    
    gen_query_table: for i in 0 to 26 generate
    u_RAM16X1D_1 : RAM16X1D_1
    generic map(
        INIT => X"0000"
    )
    port map(
        DPO     => out_query_table(i),   -- Read port 1-bit output
        SPO     => open,   -- Read/Write port 1-bit ouput
        A0      => addr_write(0),        -- Read/Write port 1-bit address input
        A1      => addr_write(1),        -- Read/Write port 1-bit address input
        A2      => addr_write(2),        -- Read/Write port 1-bit address input
        A3      => addr_write(3),        -- Read/Write port 1-bit address input
        D       => in_query_table(i),     -- RAM data input
        DPRA0   => addr_read(0),        -- Read port 1-bit address input
        DPRA1   => addr_read(1),        -- Read port 1-bit address input
        DPRA2   => addr_read(2),        -- Read port 1-bit address input
        DPRA3   => addr_read(3),        -- Read port 1-bit address input
        WCLK    => clock,           -- Write clock input
        WE      => we_query_table               -- RAM data input
    );
    end generate;
    
    process(reset, clock)
    begin
        if reset = '1' then
            EA_query        <= INIT;
            we_query_table  <= '0';
            addr_write      <= (others => '1');
            reset_seek      <= (others => '1');
            reset_clear     <= (others => '1');
        elsif clock'event and clock='1' then
            EA_query <= PE_query;
            case EA_query is
                when INIT =>
                    we_query_table  <= '0';
                    reset_seek      <= (others => '0');
                    reset_clear     <= (others => '0');
                    
                when S_CLEAR_PULSE => -- limpa o seek pulse
                    
                    if clear_pulse(EAST) = '1' then
                        in_query_table <= "00" & "000" & in_target(EAST) & in_source(EAST) & in_hop_counter(EAST);
                        reset_clear(EAST) <= '1';
                    elsif clear_pulse(WEST) = '1' then
                        in_query_table <= "00" & "001" & in_target(WEST) & in_source(WEST) & in_hop_counter(WEST);
                        reset_clear(WEST) <= '1';
                    elsif clear_pulse(NORTH) = '1' then
                        in_query_table <= "00" & "010" & in_target(NORTH) & in_source(NORTH) & in_hop_counter(NORTH);
                        reset_clear(NORTH) <= '1';
                    elsif clear_pulse(SOUTH) = '1' then
                        in_query_table <= "00" & "011" & in_target(SOUTH) & in_source(SOUTH) & in_hop_counter(SOUTH);
                        reset_clear(SOUTH) <= '1';
                    else
                        in_query_table <= "00" & "100" & in_target(LOCAL) & in_source(LOCAL) & in_hop_counter(LOCAL);
                        reset_clear(LOCAL) <= '1';
                    end if;
                    
                    we_query_table  <= '1';
                    addr_write      <= addr_write + 1;
                    
                
                when S_SEEK_PULSE => -- verifica o tipo de seek_pulse.
                    
                    if seek_pulse(EAST) = SEEK_CODE then
                        in_query_table <= SEEK_CODE & "000" & in_target(EAST) & in_source(EAST) & in_hop_counter(EAST);
                        reset_seek(EAST) <= '1';
                    elsif seek_pulse(EAST) = SEEK_UNREACHABLE_CODE then
                        in_query_table <= SEEK_UNREACHABLE_CODE & "000" & in_target(EAST) & in_source(EAST) & in_hop_counter(EAST);
                        reset_seek(EAST) <= '1';
                    elsif seek_pulse(EAST) = SEEK_RESEND_CODE then
                        in_query_table <= SEEK_RESEND_CODE & "000" & in_target(EAST) & in_source(EAST) & in_hop_counter(EAST);
                        reset_seek(EAST) <= '1';
                    elsif seek_pulse(WEST) = SEEK_CODE then
                        in_query_table <= SEEK_CODE & "001" & in_target(WEST) & in_source(WEST) & in_hop_counter(WEST);
                        reset_seek(WEST) <= '1';
                    elsif seek_pulse(WEST) = SEEK_UNREACHABLE_CODE then
                        in_query_table <= SEEK_UNREACHABLE_CODE & "001" & in_target(WEST) & in_source(WEST) & in_hop_counter(WEST);
                        reset_seek(WEST) <= '1';
                    elsif seek_pulse(WEST) = SEEK_RESEND_CODE then
                        in_query_table <= SEEK_RESEND_CODE & "001" & in_target(WEST) & in_source(WEST) & in_hop_counter(WEST);
                        reset_seek(WEST) <= '1';    
                    elsif seek_pulse(NORTH) = SEEK_CODE then
                        in_query_table <= SEEK_CODE & "010" & in_target(NORTH) & in_source(NORTH) & in_hop_counter(NORTH);
                        reset_seek(NORTH) <= '1';
                    elsif seek_pulse(NORTH) = SEEK_UNREACHABLE_CODE then
                        in_query_table <= SEEK_UNREACHABLE_CODE & "010" & in_target(NORTH) & in_source(NORTH) & in_hop_counter(NORTH);
                        reset_seek(NORTH) <= '1';
                    elsif seek_pulse(NORTH) = SEEK_RESEND_CODE then
                        in_query_table <= SEEK_RESEND_CODE & "010" & in_target(NORTH) & in_source(NORTH) & in_hop_counter(NORTH);
                        reset_seek(NORTH) <= '1';    
                    elsif seek_pulse(SOUTH) = SEEK_CODE then
                        in_query_table <= SEEK_CODE & "011" & in_target(SOUTH) & in_source(SOUTH) & in_hop_counter(SOUTH);
                        reset_seek(SOUTH) <= '1';
                    elsif seek_pulse(SOUTH) = SEEK_UNREACHABLE_CODE then
                        in_query_table <= SEEK_UNREACHABLE_CODE & "011" & in_target(SOUTH) & in_source(SOUTH) & in_hop_counter(SOUTH);
                        reset_seek(SOUTH) <= '1';
                    elsif seek_pulse(SOUTH) = SEEK_RESEND_CODE then
                        in_query_table <= SEEK_RESEND_CODE & "011" & in_target(SOUTH) & in_source(SOUTH) & in_hop_counter(SOUTH);
                        reset_seek(SOUTH) <= '1';    
                    elsif seek_pulse(LOCAL) = SEEK_CODE then
                        in_query_table <= SEEK_CODE & "100" & in_target(LOCAL) & in_source(LOCAL) & in_hop_counter(LOCAL);
                        reset_seek(LOCAL) <= '1';
                    elsif seek_pulse(LOCAL) = SEEK_UNREACHABLE_CODE then
                        in_query_table <= SEEK_UNREACHABLE_CODE & "100" & in_target(LOCAL) & in_source(LOCAL) & in_hop_counter(LOCAL);
                        reset_seek(LOCAL) <= '1';
                    else
						in_query_table <= SEEK_RESEND_CODE & "100" & in_target(LOCAL) & in_source(LOCAL) & in_hop_counter(LOCAL);
                        reset_seek(LOCAL) <= '1';
                    end if;
                    
                    we_query_table  <= '1';
                    addr_write      <= addr_write + 1;
                
            end case;
        end if;
    end process;
    
    
    process(EA_query, seek_pulse, clear_pulse)
    begin
        case EA_query is
            when INIT =>
                if seek_pulse(EAST) /= "00" or seek_pulse(WEST) /= "00" or seek_pulse(NORTH) /= "00" or seek_pulse(SOUTH) /= "00" or seek_pulse(LOCAL) /= "00" then
                    PE_query <= S_SEEK_PULSE;
                elsif clear_pulse /= "00000" then
                    PE_query <= S_CLEAR_PULSE;
                else
                    PE_query <= INIT;
                end if;
                
            when S_SEEK_PULSE =>
                PE_query <= INIT;
                
            when S_CLEAR_PULSE =>
                PE_query <= INIT;
                            
        end case;
    end process;

end architecture seek;
