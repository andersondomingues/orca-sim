---------------------------------------------------------------------
-- TITLE: Network Interface Module
-- AUTHOR: Everton Alceu Carara (everton.carara@pucrs.br)
-- DATE CREATED: 15/06/2009
-- FILENAME: Network_Interface.vhd
-- PROJECT: GAPH MPSoC
-- DESCRIPTION:
--		Send/receive and pack/unpack NoC packets.
---------------------------------------------------------------------

library ieee;
use work.HeMPS_defaults.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
--use ieee.std_logic_arith.all;
use ieee.numeric_std.all;
use work.mlite_pack.all;

--Uncomment following two lines for Xilinx RAM16X1D
library unisim;              --Xilinx
use unisim.vcomponents.all;  --Xilinx

entity Network_Interface is
	generic (
		address_router: regHalfFlit:=(others=>'0');
		buffer_type         : string := "RTL"
	);	
   	port (  
		clock		        : in  std_logic;
		reset               : in  std_logic;

		-- Interface com a NoC (Local port 0)     
		tx0                 : out std_logic;
		data_out0           : out regflitWoCRC;
		eop_out0	        : out std_logic;
		credit_i0           : in  std_logic;       
		rx0                 : in  std_logic;
		data_in0            : in  regflitWoCRC;
		eop_in0		        : in  std_logic;
		credit_o0           : out std_logic;
		
		-- Interface com a NoC (Local port 1)
		tx1                 : out std_logic;
		data_out1           : out regflitWoCRC;
		eop_out1	        : out std_logic;
		credit_i1           : in  std_logic;       
		rx1                 : in  std_logic;
		data_in1            : in  regflitWoCRC;
		eop_in1		        : in  std_logic;
		credit_o1           : out std_logic;

        receive_seek        : in  std_logic;
        seek_data           : in  std_logic_vector(31 downto 0);
        
        release_ni          : in  std_logic;
        
		-- Interface Plasma  
		send_av             : out std_logic;
		read_av             : out std_logic;
		intr                : out std_logic;
		send_data           : in  std_logic;
		read_data           : in  std_logic;     
		data_write          : in  std_logic_vector(31 downto 0);
		data_read           : out std_logic_vector(31 downto 0);      
		config              : out std_logic_vector(31 downto 0));
end;


architecture Network_Interface of Network_Interface is

    type Tpath      is array (2 downto 0) of std_logic_vector(31 downto 0);
    type slot_table_source_routing is record
        used        : std_logic;
        target      : std_logic_vector(HALF_FLIT-1 downto 0);
        size        : std_logic_vector(HOP_SIZE-1 downto 0);
        path        : Tpath;
    end record;
    type Ttable_sr  is array (TABLE_HEIGHT-1 downto 0) of  slot_table_source_routing;
    
    signal table_sr : Ttable_sr;

begin

	-- Turns available the NoC address to the Plasma processor
	config <= x"000000" & address_router;

	-- Table with the source routing for targets with faulty routers in the path
	FILL_TABLE: block
        type State      is (IDLE, WAIT_SIZE, WAIT_PATH);
		signal ST       : State;
        signal size     : std_logic_vector(HOP_SIZE-1 downto 0);
        
        signal free_slot: integer range -1 to TABLE_HEIGHT;
        
        begin
            process(clock,reset)
            begin
                if reset = '1' then
                    size                <= (others => '0');
                    ST                  <= IDLE;
                    free_slot           <= -1;
                    table_sr(0).used    <= '0';
                    table_sr(1).used    <= '0';
                    table_sr(2).used    <= '0';
                    table_sr(3).used    <= '0';
                    table_sr(4).used    <= '0';
                    table_sr(5).used    <= '0';
                    table_sr(6).used    <= '0';
                    table_sr(7).used    <= '0';
                    
                    table_sr(0).size <= (others => '0');
                    table_sr(1).size <= (others => '0');
                    table_sr(2).size <= (others => '0');
                    table_sr(3).size <= (others => '0');
                    table_sr(4).size <= (others => '0');
                    table_sr(5).size <= (others => '0');
                    table_sr(6).size <= (others => '0');
                    table_sr(7).size <= (others => '0');
                    
                elsif rising_edge(clock) then
                    case ST is
                        when IDLE =>
                            if receive_seek = '1' then
                                --if target is already stored, updates its path
                                if table_sr(0).target  = seek_data(HALF_FLIT-1 downto 0) and table_sr(0).used = '1' then
                                    free_slot <= 0;
                                    ST <= WAIT_SIZE;
                                elsif table_sr(1).target  = seek_data(HALF_FLIT-1 downto 0) and table_sr(1).used = '1' then
                                    free_slot <= 1;
                                    ST <= WAIT_SIZE;
                                elsif table_sr(2).target  = seek_data(HALF_FLIT-1 downto 0) and table_sr(2).used = '1' then
                                    free_slot <= 2;
                                    ST <= WAIT_SIZE;
                                elsif table_sr(3).target  = seek_data(HALF_FLIT-1 downto 0) and table_sr(3).used = '1' then
                                    free_slot <= 3;
                                    ST <= WAIT_SIZE;
                                elsif table_sr(4).target  = seek_data(HALF_FLIT-1 downto 0) and table_sr(4).used = '1' then
                                    free_slot <= 4;
                                    ST <= WAIT_SIZE;
                                elsif table_sr(5).target  = seek_data(HALF_FLIT-1 downto 0) and table_sr(5).used = '1' then
                                    free_slot <= 5;
                                    ST <= WAIT_SIZE;
                                elsif table_sr(6).target  = seek_data(HALF_FLIT-1 downto 0) and table_sr(6).used = '1' then
                                    free_slot <= 6;
                                    ST <= WAIT_SIZE;
                                elsif table_sr(7).target  = seek_data(HALF_FLIT-1 downto 0) and table_sr(7).used = '1' then
                                    free_slot <= 7;
                                    ST <= WAIT_SIZE;
                                --search for free slots
                                elsif table_sr(0).used = '0' then
                                    table_sr(0).used    <= '1';
                                    table_sr(0).target  <= seek_data(HALF_FLIT-1 downto 0);
                                    free_slot <= 0;
                                    ST <= WAIT_SIZE;
                                elsif table_sr(1).used = '0' then
                                    table_sr(1).used    <= '1';
                                    table_sr(1).target  <= seek_data(HALF_FLIT-1 downto 0);
                                    free_slot <= 1;
                                    ST <= WAIT_SIZE;
                                elsif table_sr(2).used = '0' then
                                    table_sr(2).used    <= '1';
                                    table_sr(2).target  <= seek_data(HALF_FLIT-1 downto 0);
                                    free_slot <= 2;
                                    ST <= WAIT_SIZE;
                                elsif table_sr(3).used = '0' then
                                    table_sr(3).used    <= '1';
                                    table_sr(3).target  <= seek_data(HALF_FLIT-1 downto 0);
                                    free_slot <= 3;
                                    ST <= WAIT_SIZE;
                                elsif table_sr(4).used = '0' then
                                    table_sr(4).used    <= '1';
                                    table_sr(4).target  <= seek_data(HALF_FLIT-1 downto 0);
                                    free_slot <= 4;
                                    ST <= WAIT_SIZE;
                                elsif table_sr(5).used = '0' then
                                    table_sr(5).used    <= '1';
                                    table_sr(5).target  <= seek_data(HALF_FLIT-1 downto 0);
                                    free_slot <= 5;
                                    ST <= WAIT_SIZE;
                                elsif table_sr(6).used = '0' then
                                    table_sr(6).used    <= '1';
                                    table_sr(6).target  <= seek_data(HALF_FLIT-1 downto 0);
                                    free_slot <= 6;
                                    ST <= WAIT_SIZE;
                                elsif table_sr(7).used = '0' then
                                    table_sr(7).used    <= '1';
                                    table_sr(7).target  <= seek_data(HALF_FLIT-1 downto 0);
                                    free_slot <= 7;
                                    ST <= WAIT_SIZE;
                                else
                                	if table_sr(3).used = '1' and table_sr(2).used = '1' and table_sr(1).used = '1' and table_sr(0).used = '1' and table_sr(7).used = '1' and table_sr(6).used = '1' and table_sr(5).used = '1' and table_sr(4).used = '1' then
                        				report "There is no space in NI's table for target:" & CONV_STRING_8BITS(seek_data(HALF_FLIT-1 downto 0)) severity error;
                        			end if;
                                    free_slot <= -1;
                                    ST <= IDLE;
                                end if;
                            else
                                ST <= IDLE;
                            end if;
                            
                        when WAIT_SIZE =>
                            if receive_seek = '1' then
                                size                        <= "000000";
                                table_sr(free_slot).size    <= seek_data(HOP_SIZE-1 downto 0);
                                ST                          <= WAIT_PATH;
                            else
                                ST                          <= WAIT_SIZE;
                            end if;
                        when WAIT_PATH =>
                            if receive_seek = '1' then
                                if (to_integer(unsigned(size+1) sll 1)) >= table_sr(free_slot).size then
                                    ST                                                  <= IDLE;
                                else
                                    ST                                                  <= WAIT_PATH;
                                end if;
                                size                                                    <= size + 1;
                                table_sr(free_slot).path(to_integer(unsigned(size)))    <= seek_data;
                            else
                                ST <= WAIT_PATH;
                            end if;
                    end case;
                end if;
            end process;
    end block FILL_TABLE;

	-- Sends data to NoC
	SEND: block
		type State is (S0, S1, S2, S3, SSEND_SR_HEADER, SSEND_SR_HEADER_1);
		signal SS: State;
		
		-- Output port in use
		signal local: std_logic;
		
		-- Output port interface signals (NI->NoC)
		signal data_out: regflitWoCRC;
		signal tx: std_logic;
		signal eop_out : std_logic;
		signal credit_i: std_logic;
		
		-- Registers
		signal low_word: regflitWoCRC;				-- Stores a low word flit
		signal payload_size : regflitWoCRC;			-- Stores the payload NoC packet size
		
		-- Used to set the eop_out signal when the connection is released
		signal releasing_connection: boolean;
        
        alias address:std_logic_vector(HALF_FLIT-1 downto 0)            is data_write(HALF_FLIT-1 downto 0);
        
        signal header_sr_size                                           : std_logic_vector(HOP_SIZE-1 downto 0);
        
        signal free_slot: integer range -1 to TABLE_HEIGHT;
		begin
			
			-- Demultiplexes the output port interface (NI->NoC)
			data_out0 <= data_out;
			tx0 <= tx when local = '0' else '0';
			eop_out0 <= eop_out;
			
			data_out1 <= data_out;
			tx1 <= tx when local = '1' else '0';
			eop_out1 <= eop_out;			
									
			credit_i <= credit_i0 when local = '0' else credit_i1;
			
			-- Warning: output not registrated
			send_av <= '0' when SS = S3 or ((SS = SSEND_SR_HEADER_1 or SS = SSEND_SR_HEADER) and not(header_sr_size = table_sr(free_slot).size)) or credit_i = '0' else '1';
			
			process (clock,reset)
			begin
				if reset = '1' then 
					tx                      <= '0';
					local                   <= '0';
					eop_out                 <= '0';
					releasing_connection    <= false;
					SS                      <= S0;
                    free_slot               <= -1;
                    header_sr_size          <= (others => '0');
				elsif rising_edge(clock) then
					case SS is
					
						-- Waits the header flit
						when S0 =>
							if send_data = '1' then
								-- Selects the output port interface (NI->NoC)
								-- Packet switching use port 1
								if credit_i1 = '1' then
									local <= '1';
								else
									local <= '0';									
								end if;
								-- Sends the header flit
								
                                if (address = table_sr(0).target and table_sr(0).used = '1') then
                                    free_slot <= 0;
                                    header_sr_size <= "000001";
                                    SS <= SSEND_SR_HEADER_1;
                                    tx <= '1';
                                    data_out <= table_sr(0).path(0)(31 downto FLIT_WIDTHwoCRC);
                                elsif (address = table_sr(1).target and table_sr(1).used = '1') then
                                    free_slot <= 1;
                                    header_sr_size <= "000001";
                                    SS <= SSEND_SR_HEADER_1;
                                    tx <= '1';
                                    data_out <= table_sr(1).path(0)(31 downto FLIT_WIDTHwoCRC);
                                elsif (address = table_sr(2).target and table_sr(2).used = '1') then
                                    free_slot <= 2;
                                    header_sr_size <= "000001";
                                    SS <= SSEND_SR_HEADER_1;
                                    tx <= '1';
                                    data_out <= table_sr(2).path(0)(31 downto FLIT_WIDTHwoCRC);
                                elsif (address = table_sr(3).target and table_sr(3).used = '1') then
                                    free_slot <= 3;
                                    header_sr_size <= "000001";
                                    SS <= SSEND_SR_HEADER_1;
                                    tx <= '1';
                                    data_out <= table_sr(3).path(0)(31 downto FLIT_WIDTHwoCRC);
                                elsif (address = table_sr(4).target and table_sr(4).used = '1') then
                                    free_slot <= 4;
                                    header_sr_size <= "000001";
                                    SS <= SSEND_SR_HEADER_1;
                                    tx <= '1';
                                    data_out <= table_sr(4).path(0)(31 downto FLIT_WIDTHwoCRC);
                                elsif (address = table_sr(5).target and table_sr(5).used = '1') then
                                    free_slot <= 5;
                                    header_sr_size <= "000001";
                                    SS <= SSEND_SR_HEADER_1;
                                    tx <= '1';
                                    data_out <= table_sr(5).path(0)(31 downto FLIT_WIDTHwoCRC);
                                elsif (address = table_sr(6).target and table_sr(6).used = '1') then
                                    free_slot <= 6;
                                    header_sr_size <= "000001";
                                    SS <= SSEND_SR_HEADER_1;
                                    tx <= '1';
                                    data_out <= table_sr(6).path(0)(31 downto FLIT_WIDTHwoCRC);
                                elsif (address = table_sr(7).target and table_sr(7).used = '1') then
                                    free_slot <= 7;
                                    header_sr_size <= "000001";
                                    SS <= SSEND_SR_HEADER_1;
                                    tx <= '1';
                                    data_out <= table_sr(7).path(0)(31 downto FLIT_WIDTHwoCRC);
                                else
                                    tx <= '1';
                                    data_out <= data_write(FLIT_WIDTHwoCRC-1 downto 0);
                                    SS <= S1;
                                end if;
							else
								SS <= S0;
							end if;
							
						
						-- Waits the payload size flit
						when S1 =>
							-- Sends the multicast header flits or payload size flit
							if credit_i = '1' and send_data = '1' then
								tx <= '1';
								data_out <= data_write(FLIT_WIDTHwoCRC-1 downto 0);
                                payload_size <= data_write(FLIT_WIDTHwoCRC-1 downto 0);
                                SS <= S2;
							-- No data to send
							elsif credit_i = '1' then								
								tx <= '0';													
								SS <= S1;
							end if;
											
						-- Sends the high word flit
						when S2 =>							
							if credit_i = '1' then								
								-- Verifies if there is a word to send
								if send_data = '1' then
									data_out <= data_write(31 downto FLIT_WIDTHwoCRC);
									low_word <= data_write(FLIT_WIDTHwoCRC-1 downto 0);
									tx <= '1';									
								else
									tx <= '0';
								end if;
								-- Decrement the payload size
								if tx = '1' then
									payload_size <= payload_size - 1;
								end if;						
								-- Sets the end of packet bit
								if payload_size = 1 and local = '1' then
									--eop_out <= '1';
								end if;
								-- Decides the next state
								if payload_size = 0 then
									eop_out <= '0';
									tx <= '0';									
									SS <= S0;
								elsif send_data = '1' then
									SS <= S3;
								else
									SS <= S2;
								end if;
							-- Waits for credit
							else
								SS <= S2;
							end if;
						-- Sends the low word flit
						when S3 =>
							if credit_i = '1' then
								-- Sends the low word 
								data_out <= low_word;									
								payload_size <= payload_size - 1;
								-- Sets the end of packet bit
								if payload_size = 1 and (local = '1' or releasing_connection) then
									eop_out <= '1';
									releasing_connection <= false;
								end if;
								
								-- Decides the next state
								if payload_size = 0 then
									eop_out <= '0';
									SS <= S0;
								else
									SS <= S2;
								end if;
																				
							-- Waits for credit
							else
								SS <= S3;
							end if;
                            
                        when SSEND_SR_HEADER =>
							if credit_i = '1' then
                                if header_sr_size = table_sr(free_slot).size then
                                    if send_data = '1' then
                                        tx              <= '1';
                                        SS              <= S2;
                                        data_out        <= data_write(FLIT_WIDTHwoCRC-1 downto 0);
                                        payload_size    <= data_write(FLIT_WIDTHwoCRC-1 downto 0);
                                    else
                                        tx              <= '0';
                                        SS              <=SSEND_SR_HEADER;
                                    end if;
                                else
                                    header_sr_size  <= header_sr_size + '1';
                                    tx              <= '1';
                                    data_out        <= table_sr(free_slot).path(to_integer(unsigned(header_sr_size) srl 1))(31 downto FLIT_WIDTHwoCRC);
                                    SS              <= SSEND_SR_HEADER_1;
                                end if;
                            end if;
                            
                        when SSEND_SR_HEADER_1 =>
							if credit_i = '1' then
                                if header_sr_size = table_sr(free_slot).size then
                                    if send_data = '1' then
                                        tx              <= '1';
                                        SS              <= S2;
                                        data_out        <= data_write(FLIT_WIDTHwoCRC-1 downto 0);
                                        payload_size    <= data_write(FLIT_WIDTHwoCRC-1 downto 0);
                                    else
                                        tx              <= '0';
                                        SS              <=SSEND_SR_HEADER_1;
                                    end if;
                                else
                                    header_sr_size  <= header_sr_size + '1';
                                    tx              <= '1';
                                    --header_sr_size  <= std_logic_vector(unsigned(header_sr_size) srl 1);
                                    data_out        <= table_sr(free_slot).path(to_integer(unsigned(header_sr_size) srl 1))(FLIT_WIDTHwoCRC-1 downto 0);
                                    SS              <= SSEND_SR_HEADER;
                                end if;
                            else
                                SS <= SSEND_SR_HEADER_1;
                            end if;
                            
						when others =>
					end case;
				end if;			
			end process;		
	end block SEND;
	
	
	-- Receives data from NoC and controls the Plasma reading
	RECEIVE: block
			type State is (S0, S1, S2, S3, S4, S5);
			signal SR, SP: State;
			
			-- Input port in use
			signal local: std_logic;	
			
			-- Input port interface signals (NoC -> NI)
			signal data_in : regflitWoCRC;
			signal rx : std_logic;
			
			-- Signals to controls the receive buffer
			signal first, last : std_logic_vector(3 downto 0);
			signal buffer_write, buffer_read : std_logic_vector(32 downto 0);
			signal we : std_logic;
			signal slot_available : std_logic;
						
			-- Registers
			--signal header_stored : boolean;			-- Store a header flit
			signal header_stored : boolean;			-- Indicates a header flit stored in the case of two header flits arriving at the same clock cycle
			signal reg_header: regflitWoCRC;				-- Store a header flit
			signal read_av_reg: std_logic;			-- Output registred			
			signal payload_size: regflitWoCRC;			-- Counter
			signal payload_size_readed: boolean;	-- Indicates a payload_size stored
			
			-- Receive buffer used for simulation
			type buffer_in is array(0 to 15) of std_logic_vector(32 downto 0);
			signal Receive_buffer: buffer_in;
            
            signal NI_threshold         : std_logic_vector (32 downto 0);
			
            signal backtrack_packet: std_logic;
            
            signal credit_o0_sig : std_logic;
            signal credit_o1_sig : std_logic;
            
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
							
				-- Multiplexes the input ports (NoC -> NI)
				data_in <= data_in1 when local = '1' else data_in0;
				rx <= rx1 when local = '1' else rx0;
				credit_o0_sig <= slot_available when SR=S0 or local = '0' else '0';
				--credit_o1_sig <= slot_available when (SR=S0 or local = '1') else '0';
				credit_o1_sig <= slot_available when (SR=S0 or local = '1') and not header_stored else '0';
                
				credit_o0 <= credit_o0_sig;
				credit_o1 <= credit_o1_sig;
				
				-- Data to Plasma
				data_read <= buffer_read(31 downto 0);
				
				-- Memory to store the input flits. Bit 32 indicates last word packet (eop).
				Buffer_RTL: if buffer_type = "RTL" generate
					Receive_buffer: for i in 0 to 32 generate
						receive_buffer: RAM16X1D
							port map (
								WCLK	=> clock,
								WE		=> we,
								D		=> buffer_write(i),
								A0		=> last(0),   
								A1		=> last(1),   
								A2		=> last(2),  
								A3		=> last(3), 
								DPRA0	=> first(0),
								DPRA1	=> first(1),
								DPRA2	=> first(2),
								DPRA3	=> first(3),
								SPO		=> open,
								DPO		=> buffer_read(i)
							);     
						end generate receive_buffer;
				end generate;
				
				Buffer_SIM: if buffer_type = "SIM" generate
						buffer_read <= Receive_buffer(CONV_INTEGER(first));
				end generate;
					
								
				-- Receives data from NoC and stores in the Receive buffer
				process (clock,reset)
				begin
					if reset = '1' then 
						local               <= '0';
						last                <= (others=>'0');
						we                  <= '0';
						slot_available      <= '1';
						--header_stored       <= false;
						payload_size_readed <= false;
						SR                  <= S0;
                        backtrack_packet    <= '0';
						header_stored       <= false;
                        
                        NI_threshold        <= (others=>'0');
                        
					elsif rising_edge(clock) then
						
						if buffer_type = "SIM" then
							Receive_buffer(CONV_INTEGER(last)) <= buffer_write;
						end if;
						
						case SR is
							-- Waits the header flit
							when S0 =>
								-- Priority to receive packets from input port 0
								if rx0 = '1' and slot_available = '1' then
                                    local <= '0';
                                    buffer_write(31 downto FLIT_WIDTHwoCRC) <= data_in0;
                                    SR <= S1;
                                    backtrack_packet <= '0';
                                    
                                    -- Stores a packet switching/multicast header if there are available flits in both ports
									if rx1 = '1' and not header_stored then
										reg_header <= data_in1;
										header_stored <= true;
									end if;									
								
								-- Handles the stored header 
								elsif header_stored then
									
                                    local <= '1';
                                    buffer_write(31 downto FLIT_WIDTHwoCRC) <= reg_header;
                                    SR <= S1;
								
									header_stored <= false;
                                    
                                    
								-- Receives a packet switching/multicast packet from port 1
								elsif rx1 = '1' and slot_available = '1' then
                                    if data_in1(FLIT_WIDTHwoCRC-1 downto FLIT_WIDTHwoCRC-4) = BACKTRACK_PATH then
                                        report CONV_STRING_8BITS(address_router) & ":received BACKTRACK_PATH:" severity note;
                                        backtrack_packet <= '1';
                                        local <= '1';
										buffer_write(32) <= '0';
                                        buffer_write(31 downto FLIT_WIDTHwoCRC) <= data_in1;
                                        SR <= S1;
                                    else
                                        local <= '1';
										buffer_write(32) <= '0';
                                        buffer_write(31 downto FLIT_WIDTHwoCRC) <= data_in1;
                                        SR <= S1;
                                        backtrack_packet <= '0';
                                    end if;
								else
									SR <= S0;
								end if;
								
								if we = '1' then
									we <= '0';
									last <= last + 1;
								end if;
								
							-- Receives the low word flit
							when S1 =>
								-- Receives the flit
								if rx = '1' and slot_available = '1' then
                                    buffer_write(FLIT_WIDTHwoCRC-1 downto 0) <= data_in;
                                    we <= '1';
                                    
                                    if backtrack_packet = '1' then
                                        --BACKTRACK_PATH do not use payload_size
                                        if local = '1' and eop_in1 = '1' then
                                            buffer_write(32) <= '1';
                                            SR <= S0;
                                        else
                                            SR <= S2;
                                        end if;
                                    else
                                        -- Stores the payload size
                                        if payload_size_readed then
                                            payload_size <= payload_size - 1;
                                        else
                                            payload_size <= data_in;
                                            payload_size_readed <= true;
                                        end if;
                                        
                                        -- Sets the last packet flit
                                        if payload_size = 1 then
                                            buffer_write(32) <= '1';
                                            payload_size_readed <= false;
                                            SR <= S0;
                                        else
                                            buffer_write(32) <= '0';
                                            SR <= S2;
                                        end if;
                                    end if;
                                    
                                elsif release_ni = '1' then
                                    SR <= S0;
                                    payload_size_readed <= false;
								else
									SR <= S1;
								end if;
								
							-- Receives the high word flit
							when S2 =>																						
								-- Stores the flit in the receive buffer
								if we = '1' then
									we <= '0';
									last <= last + 1;								
								end if;
								
								-- Receives the flit
								if rx = '1' and slot_available = '1' then 
									buffer_write(31 downto FLIT_WIDTHwoCRC) <= data_in;									
									payload_size <= payload_size - 1;									
									SR <= S1;
                                    --BACKTRACK_PATH packet can have a even number of flits
                                    if local = '1' and eop_in1 = '1' and backtrack_packet = '1' then
										buffer_write(32) <= '1';
                                        buffer_write(31 downto FLIT_WIDTHwoCRC) <= data_in;	
                                        we <= '1';
										SR <= S0;
                                    end if;
								elsif release_ni = '1' then
                                    SR <= S0;
                                    payload_size_readed <= false;
                                else
									SR <= S2;
								end if;
                                
							when others=>
						end case;
                        
                        if ((SR = S1) or (SR = S2)) then
                            if credit_o1_sig = '1' and local = '1' and rx1 = '0' then
                                NI_threshold <= NI_threshold + 1;
                            elsif credit_o0_sig = '1' and local = '0' and rx0 = '0' then
                                NI_threshold <= NI_threshold + 1;
                            end if;
                        else
                            NI_threshold <= (others=>'0');
                        end if;
						
						-- Controls the available space in the receive buffer
						if we = '1' and ((first = last+2) or (first=0 and last=x"E")) and read_data = '0' then
							slot_available <= '0';						
						
						elsif read_data = '1' then
							slot_available <= '1';
							
						else
							slot_available <= slot_available;
						end if;
					end if;				
				end process;
                	
				-- Output registred
				read_av <= read_av_reg;
				
				-- Controls the Plasma reading from the receive buffer
				process (clock, reset)
				begin
					if reset = '1' then
						intr <= '0';
						first <= (others=>'0');
						read_av_reg <= '0';						
						SP <= S0;
					
					elsif rising_edge(clock) then
						case SP is
							-- Interrupts the Plasma processor
							when S0 =>
								if first /= last then
									intr <= '1';
									SP <= S1;
								else
									SP <= S0;
								end if;
								
							-- Controls the receive buffer reading
							when S1 =>
								if read_data = '1' then
									intr <= '0';
									first <= first + 1;
									
									if buffer_read(32) = '1' then
										SP <= S0;
									else
										SP <= S1;
									end if;									
								elsif release_ni = '1' then
                                    SP <= S0;
                                else
									SP <= S1;
								end if;							
								
																	
							when others =>
						end case;
						
						-- Controls the read availability to Plasma processor
						if we = '1' then
							read_av_reg <= '1';
						elsif read_data = '1' and first+1 = last then
							read_av_reg <= '0';
						else
							read_av_reg <= read_av_reg;							
						end if;					
						
					end if;
				end process;
			
	end block RECEIVE;
end;   
