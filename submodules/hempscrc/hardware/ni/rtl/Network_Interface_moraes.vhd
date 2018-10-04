---------------------------------------------------------------------
-- TITLE: Network Interface Module
-- AUTHOR: Everton Alceu Carara (everton.carara@pucrs.br)
-- DATE CREATED: 15/06/2009
-- FILENAME: Network_Interface.vhd
-- PROJECT: GAPH MPSoC
-- DESCRIPTION:
--      Send/receive and pack/unpack NoC packets.
--
--   moraes -- o ctober / 2014
--
--      Send/receive and pack/unpack NoC packets.
--
--   moraes -- october / 2014
--
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
        clock               : in  std_logic;
        reset               : in  std_logic;

        -- Interface com a NoC (Local port 0)     
        tx0                 : out std_logic;
        data_out0           : out regflitWoCRC;
        eop_out0            : out std_logic;
        credit_i0           : in  std_logic;       
        rx0                 : in  std_logic;
        data_in0            : in  regflitWoCRC;
        eop_in0             : in  std_logic;
        credit_o0           : out std_logic;
        
        -- Interface com a NoC (Local port 1)
        tx1                 : out std_logic;
        data_out1           : out regflitWoCRC;
        eop_out1            : out std_logic;
        credit_i1           : in  std_logic;       
        rx1                 : in  std_logic;
        data_in1            : in  regflitWoCRC;
        eop_in1             : in  std_logic;
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
        signal low_word: regflitWoCRC;              -- Stores a low word flit
        signal payload_size : regflitWoCRC;         -- Stores the payload NoC packet size
        
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
    
    -- RECEPCAO POR PARTE DA NOC
    --
    --  A PRIMEIRA MAQUINA RECEBE OS PACOTES DA NOC E AO ENCONTRAR UM EOP VAI PARA O ESTADO S2 E TRANCA ATÉ TODO O PACOTE SER LIDO
    --
    -- A SEGUNDA MAQUINA ENVIA PARA O PLASMA OS PACOTE, DESTRANCANDO A PRIMEIRA QUANDO TODOS OS FLITS FOREM LIDOS
    --
    --
    --  Jan/2015 Adicionado buffer RTL. Augusto Erichsen
    RECEIVE: block
            type State is (S0, S1,S2);
            signal SR, SP: State;
            
            -- Signals to controls the receive buffer
            signal rd_pointer, wr_pointer : std_logic_vector(3 downto 0);
	        signal buffer_write, buffer_read : std_logic_vector(32 downto 0);                                      -- Augusto Erichsen
            signal reset_interno : std_logic;
	        signal we : std_logic;                              -- Augusto Erichsen 
            signal slot_available, receiving_from_port1,  receiving_from_port0,  end_of_current_packet: std_logic;
                                    
            -- Receive buffer used for simulation
            type   buffer_in is array(0 to 15) of std_logic_vector(32 downto 0);
            signal Receive_buffer: buffer_in;
            
            signal elements_in_the_buffer : std_logic_vector(3 downto 0);  
            signal	contador :std_logic_vector(7 downto 0);

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
                            
                -- credit:  havendo espaço no buffer e a outra porta não está recevendo flits
                credit_o0 <= slot_available  and  (not receiving_from_port1) ;   
                credit_o1 <= slot_available  and  (not receiving_from_port0) and ( not(rx0) );    -- se os dois rx sobem juntos o credito do 1 baixa

                 --- CALCULA O NUMERO DE ELEMENTOS LIVRES NO BUFFER E INFORMA DISPONIBILIDADE
                elements_in_the_buffer <= wr_pointer-rd_pointer  when wr_pointer>rd_pointer else 16 - (rd_pointer - wr_pointer);
                slot_available <= '1' when  elements_in_the_buffer < 15 else '0';

                -- Memory to store the input flits. Bit 32 indicates last word packet (eop).
                Buffer_RTL: if buffer_type = "RTL" generate                                      -- Augusto Erichsen
                    Receive_buffer: for i in 0 to 32 generate
                        receive_buffer: RAM16X1D
                            port map (
                                WCLK    => clock,
                                WE      => we,
                                D       => buffer_write(i),
                                A0      => wr_pointer(0),   
                                A1      => wr_pointer(1),   
                                A2      => wr_pointer(2),  
                                A3      => wr_pointer(3), 
                                DPRA0   => rd_pointer(0),
                                DPRA1   => rd_pointer(1),
                                DPRA2   => rd_pointer(2),
                                DPRA3   => rd_pointer(3),
                                SPO     => open,
                                DPO     => buffer_read(i)
                            );     
                        end generate receive_buffer;
                end generate;
                
                Buffer_SIM: if buffer_type = "SIM" generate                                      -- Augusto Erichsen
                        buffer_read <= Receive_buffer(CONV_INTEGER(rd_pointer));
                end generate;

		
                
                -- Receives data from NoC and stores in the Receive buffer
                process (clock,reset,release_ni,reset_interno)
                begin
                    if reset = '1' or release_ni='1' or reset_interno='1' then    -- "release_ni" em caso de erro liberando as portas
                        wr_pointer          <= (others=>'0');                       
                        receiving_from_port0 <= '0';
                        receiving_from_port1 <= '0';            
                        SR <= S0; 
			            we <= '0';                                      -- Augusto Erichsen
                        
                        if release_ni='1'  then
                            report  ":CPU MANDOU ABORTAR:" severity note;
                        end if;
                       
                        
                    elsif rising_edge(clock) then

                        if buffer_type = "SIM" then                                      -- Augusto Erichsen
                            Receive_buffer(CONV_INTEGER(wr_pointer)) <= buffer_write;
                        end if;

                        if we = '1' then                                      -- Augusto Erichsen
                            wr_pointer <= wr_pointer +1 ;
                        end if;
            
                            case SR is

                                when S0 =>   --  ARMAZENA O FLIT MAIS SIGNIFICATIVO
                                    -- receives in port 0 - high priority
                                    if rx0 = '1' and slot_available = '1' and receiving_from_port1 ='0' then
                                        receiving_from_port0 <= '1';
                                        buffer_write(31 downto 16) <= data_in0;
                                        buffer_write(32) <= eop_in0;
                                         SR <= S1; 
                                    --  receives in port 1 - low priority
                                    elsif rx1 = '1' and slot_available = '1' and receiving_from_port0 ='0' then
                                        receiving_from_port1 <= '1';
                                        buffer_write(31 downto 16) <= data_in1; 
                                        buffer_write(32) <= eop_in1;
                                         SR <= S1; 
                                          
                                         -- primeiro flit que entra é um backtrack packet  
                                         if data_in1(FLIT_WIDTHwoCRC-1 downto FLIT_WIDTHwoCRC-4) = BACKTRACK_PATH  and receiving_from_port1 = '0' then
                                              report CONV_STRING_8BITS(address_router) & ":received BACKTRACK_PATH:" severity note;
                                          end if;
                                         
                                    end if;
                                    
                                    -- ao vir um eop vai para o estado S2 e baixa os dois sinais de credito (evita tratar novo pacote antes de terminar o atual)
                                    if eop_in0='1'  or eop_in1='1' then   
                                                SR <= S2;
                                                receiving_from_port1 <= '1';
                                                receiving_from_port0 <= '1';
                                                -- wr_pointer <= wr_pointer +1;    -- pacote de tamanho impar pode vir a ocorrer     -- Augusto Erichsen
                                                we <= '1';                                                                               -- Augusto Erichsen
                                                
                                                report  ":EOP EM PACOTE DE TAMANHO IMPAR:" severity note;
                                    else
                                        we <= '0';                           -- Augusto Erichsen
                                    end if;
                                     
                                when S1 =>  --  ARMAZENA O FLIT MENOS SIGNIFICATIVO

                                    -- receives in port 0 - high priority
                                    if rx0 = '1' and slot_available = '1' and receiving_from_port1 ='0' then
                                        buffer_write(15 downto 0) <= data_in0;
                                        buffer_write(32) <= eop_in0;
                                        -- wr_pointer  <= wr_pointer +1 ;    -- Augusto Erichsen
                                        we <= '1';                           -- Augusto Erichsen
                                        SR <= S0; 
                                    --  receives in port 1 - low priority
                                    elsif rx1 = '1' and slot_available = '1' and receiving_from_port0 ='0' then
                                        buffer_write(15 downto 0) <= data_in1; 
                                        buffer_write(32) <= eop_in1;
                                        -- wr_pointer   <= wr_pointer +1 ;    -- Augusto Erichsen
                                        we <= '1';                            -- Augusto Erichsen
                                        SR <= S0; 
                                    end if;

                                    -- ao vir um eop vai para o estado S2 e baixa os dois sinais de credito (evita tratar novo pacote antes de terminar o atual)
                                    if eop_in0='1'  or eop_in1='1' then   
                                                 SR <= S2;
                                                 receiving_from_port1 <= '1';
                                                 receiving_from_port0 <= '1'; 
                                    end if; 
                                
                                when S2 =>   -- libera o crédito e permite a recepção de um novo pacote

                                    we <= '0';                              -- Augusto Erichsen

                                       if  end_of_current_packet = '1'  then
                                                receiving_from_port1 <= '0';
                                                receiving_from_port0 <= '0';
                                                SR <= S0; 
                                       end if;

                                when others=>  null;
                        end case;

                    end if;             
                end process;
                    
    
                -- Data to Plasma
                
                read_av <= '1' when elements_in_the_buffer>0 else '0';    -- havendo algo no buffer, o processador pode ler
                
                data_read <= buffer_read(31 downto 0);
                -- buffer_read <= Receive_buffer(CONV_INTEGER(rd_pointer));

                -- Sends data from the Receive buffer to the plasma
                process (clock,reset,release_ni,reset_interno)
                begin
                    if reset = '1' or release_ni='1' or reset_interno='1'  then    -- "release_ni" em caso de erro liberando as portas
                        intr <= '0';
                        reset_interno <= '0';
                        rd_pointer <= (others=>'0');
                        SP <= S0;
                        end_of_current_packet<='0';
                      elsif rising_edge(clock) then

                            case SP is
                                
                                --when S0 =>                    
                                    --if rd_pointer /= wr_pointer then
                                        --SP <= S2;
                                        --intr <= '1';       --- avisa o Plasma que há dados para serem lidos
                                    --end if;                                                                   
                                    --end_of_current_packet<='0';                                                                         
                                 --when S2 =>  -- Controls the receive buffer reading  -- 
                                    --if read_data = '1' then
                                        --intr <= '0';
                                        --rd_pointer <= rd_pointer + 1;
                                        --if buffer_read(32) = '1' then   ---- terminou o pacote
                                            --SP <= S0;
                                            --end_of_current_packet<='1';
                                        --end if; 
                                    --end if;                                  
                                --when others => null;
                            --end case;
                                                        
                                when S0 =>                    
                                    if rd_pointer /= wr_pointer then
                                        SP <= S1;
                                       -- intr <= '1';       --- avisa o Plasma que há dados para serem lidos
                                    end if;
                                                                   
                                    end_of_current_packet<='0';
                                    contador <= (others=>'0');
                                    
                                when S1 =>   -- neste estado temos um tempo para bufferizer os flits que entram
                                    
                                     contador <= contador + 1;

                                      if   SR=S2 or elements_in_the_buffer > 6 then     -- eop ou flits no buffer    (-> ACHO QUE COM S2 FUNCIONA )  
                                           intr <= '1';             --- avisa o Plasma que há dados para serem lidos
                                           SP <= S2;
                                      elsif contador=64  and elements_in_the_buffer >= 2 then   
                                          --  passou muito tempo, chegaram alguns flits, mas parou de chegar flits
                                           report "=====> pacote incompleto - bug bug  bug  " &  integer'image(conv_integer(elements_in_the_buffer));
                                            --FAZ UM INTERNAL RESET - sobe um reset interno  
                                            reset_interno <= '1';                                                      
                                     end if;  
                                                                                                            
                                 when S2 =>  -- Controls the receive buffer reading  -- 

                                    if read_data = '1' then
                                        intr <= '0';
                                        rd_pointer <= rd_pointer + 1;
                                        if buffer_read(32) = '1' then   ---- terminou o pacote
                                            SP <= S0;
                                            end_of_current_packet<='1';
                                        end if; 
                                    end if;
                                  
                                when others => null;
                            end case;    
                        
                        end if;
                        
                        
                        
                end process;
            
    end block RECEIVE;
end;   


