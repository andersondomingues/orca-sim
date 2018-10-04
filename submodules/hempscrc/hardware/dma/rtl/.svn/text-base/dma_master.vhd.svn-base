------------------------------------------------------------------
--- HEMPS 4.0  - October 25 2011
---------------------------------------------------------------------
-- TITLE: Direct Memory Access Unit
-- INITIAL AUTHOR: Ismael Augusto Grehs - 10/11/06
-- DESCRIPTION:
--    Implements a DMA used by NI module.
--
-- OPERATIONS:
--    0 - Copy from memory
--    1 - Copy to memory
---------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;
-- use ieee.std_logic_arith.all;

entity dma_master is
port(  
        clock          : in  std_logic;
        reset          : in  std_logic;
        -- NI interface
        read_av        : in  std_logic;
        read_data      : out std_logic;
        send_av        : in  std_logic;
        send_data      : out std_logic;
        -- Configuration pins
        set_address    : in  std_logic;
        set_size       : in  std_logic;
        set_op         : in  std_logic;
        start          : in  std_logic;
        --task_allocation signals
        set_pe_target  : in  std_logic;
        set_task_id    : in  std_logic;
        -- Input/Output data ports
        data_read      : in  std_logic_vector(31 downto 0);
        data_write     : out std_logic_vector(31 downto 0);
        active           : out std_logic;
        -- Interrupt management
        intr           : out std_logic;
        intr_ack       : in  std_logic;
        -- Memory interface
        data_valid      : in  std_logic;
        mem_address     : out std_logic_vector(31 downto 0);
        mem_addr_ddr    : out std_logic_vector(31 downto 0);
        mem_ddr_read_req: out std_logic;
        mem_ddr_access  : out std_logic;
        mem_data_write  : out std_logic_vector(31 downto 0);
        mem_data_read   : in  std_logic_vector(31 downto 0);
        mem_byte_we     : out std_logic_vector(3 downto 0)
);
        
end;

architecture dma of dma_master is

type state is (SWait, SWaitMem0, SWaitMem1, SCopyFromMem, SCopyFromRam, SCopyFromRam0, SCopyToMem, SEnd, SCopyFromRamWait, SCopyFromRamWait1, STask_Allocation_Target, STask_Allocation_Size, STask_Allocation_Service, STask_Allocation_TaskId, STask_Allocation_task_size, Swait_send_header);

signal EA: state;

signal address          : std_logic_vector(31 downto 0);

signal data_reg, data_write_reg : std_logic_vector(31 downto 0);        -- Deals with the one cycle memory latency
signal size          : std_logic_vector(15 downto 0); 
signal operation     : std_logic;
signal read_data_reg    : std_logic;

        signal mem_data_read_reg        : std_logic_vector(31 downto 0);
        signal data_valid_reg           : std_logic;
        signal send_data1: std_logic;

constant TASK_ALLOCATION        : std_logic_vector(31 downto 0) := x"00000040";
signal pe_target                : std_logic_vector(31 downto 0);
signal task_id                  : std_logic_vector(31 downto 0);
signal task_size                : std_logic_vector(15 downto 0);
signal send_av_reg              : std_logic;

begin

        process (clock,reset)
        begin
                if reset = '1' then
                        mem_data_read_reg       <= (others=>'0');
                        data_valid_reg          <= '0';
                        send_av_reg             <= '0';
                elsif rising_edge(clock) then
                        mem_data_read_reg       <= mem_data_read;
                        data_valid_reg          <= data_valid;
                        send_av_reg             <= send_av;
                end if;
        end process;

        mem_address <= address - 4 when operation = '1' else address;
        mem_addr_ddr <= address ;--when operation = '0' else (others=>'0');
        --data_write <= mem_data_read when data_valid = '1' else (others=>'0');
        --data_write <= data_reg when read_data_reg = '1' else mem_data_read;


        read_data <= '1' when EA = SCopyToMem and read_av = '1' else '0';
        --send_data <= '1' when (EA = SCopyFromMem or EA = SCopyFromRam) and send_av = '1' else '0';--and done_sig = '1' else '0';

        
        process (clock,reset)
        begin   
                if reset = '1' then
                        address        <= (others => '0');
                        mem_data_write <= (others => '0');
                        size           <= (others => '0');
                        task_size      <= (others => '0');
                        task_id        <= (others => '0');
                        pe_target      <= (others => '0');
                        --done_sig      <= '0';
                        intr            <= '0';
                        data_write_reg  <= (others=> '0');
                        operation       <= '0';
                        mem_byte_we     <= "0000";
                        data_reg        <= (others => '0');
                        read_data_reg   <= '0';
                        EA              <= SWait;
                        active          <= '0';
                        
                        data_write      <= (others => '0');
                        send_data       <= '0';
                        mem_ddr_read_req<= '0';
                        mem_ddr_access  <= '0';
                        
                elsif rising_edge(clock) then
                        case EA is
                                when SWait =>
                                        if intr_ack = '1' then
                                                intr        <= '0';
                                                EA          <= SWait;
                                        elsif set_size = '1' then
                                                size        <= data_read(15 downto 0);
                                                EA          <= SWait;
                                        elsif set_op = '1' then
                                                operation   <= data_read(0);
                                                EA          <= SWait;
                                        elsif set_address = '1' then
                                                address     <= data_read;
                                                EA          <= SWait;
                                        elsif set_pe_target = '1'then--target for task_allocation
                                                pe_target   <= data_read;
                                                EA          <= SWait;
                                        elsif set_task_id = '1'then--task_id for task_allocation
                                                task_id     <= data_read;
                                                EA          <= SWait;
                                        elsif start = '1' then
                                                active                      <= '1';
                                                if operation = '0' then
                                                    if(address(30 downto 28) = "001") then --TASK_ALLOCATION
                                                        size                <= std_logic_vector((unsigned(size - x"6")) srl 1);--size <= (size -6)/2
                                                        --task_size for task_allocation
                                                        task_size           <= size;
                                                        EA                  <= STask_Allocation_Target;
                                                        mem_ddr_access      <= '1';
                                                        mem_ddr_read_req    <= '1';
                                                    else
                                                        EA                  <= SCopyFromRam;
                                                        mem_ddr_read_req    <= '0';
                                                    end if;  
                                                else
                                                    EA                      <= SCopyToMem;
                                                end if;
                                        end if;
                                -- Ensures that the first word will not be sent 2 times.
                                when SCopyFromRam0 =>
                                        if send_av = '1' then   
                                            address <= address + 4;
                                            size    <= size - 1;
                                            EA      <= SCopyFromRam;
                                        else
                                            EA      <= SCopyFromRam0;
                                        end if;
                                        
                                when SCopyFromRam =>                                    
                                        if send_av = '1' then
                                            EA          <= SCopyFromRamWait;
                                            address     <= address + 4;
                                            size        <= size - 1;
                                            send_data   <= '1';
                                            data_write  <= mem_data_read_reg;
                                        else
                                            send_data   <= '0';
                                            data_write  <= mem_data_read_reg;
                                            EA          <= SCopyFromRam;
                                        end if;

                                when SCopyFromRamWait =>
                                        if size = 0 then
                                            EA          <= Send;
                                            send_data   <= '0';
                                        else
                                            EA          <= SCopyFromRamWait1;
                                            data_write  <= mem_data_read_reg;
                                            send_data   <= '0';
                                        end if;

                                when SCopyFromRamWait1 =>
                                        EA              <= SCopyFromRam;
                                
                                when SWaitMem0 =>--espera baixar data_valid
                                        if data_valid_reg = '0' then
                                            EA  <= SWaitMem1;
                                        else
                                            EA  <= SWaitMem0;
                                        end if;

                                when SWaitMem1 =>                               
                                        if data_valid_reg = '1' then
                                            EA                  <= SCopyFromMem;
                                            mem_ddr_read_req    <= '0';
                                        else
                                            EA                  <= SWaitMem1;
                                        end if;
                                        send_data <= '0';
                        
                                when SCopyFromMem =>
                                        if send_av = '1' and data_valid_reg = '1' then
                                            address                 <= address + 4;
                                            size                    <= size - 1;
                                            if size = 0 then
                                                EA                  <= Send;
                                                mem_ddr_read_req    <= '0';
                                            else
                                                EA                  <= SWaitMem1;
                                                mem_ddr_read_req    <= '1';
                                            end if;
                                            send_data               <= '1';
                                            data_write              <= mem_data_read_reg;
                                        else
                                            --mem_ddr_read_req <= '0';
                                            send_data               <= '0';
                                            EA                      <= SCopyFromMem;
                                        end if;
                                        
                                when SCopyToMem =>
                                        if read_av = '1' then
                                            mem_data_write  <= data_read;
                                            mem_byte_we     <= "1111";
                                            address         <= address + 4;
                                            size            <= size - '1';
                                            if size = 1 then        -- The last word is sent in the Send state
                                                EA          <= Send;
                                            else
                                                EA          <= SCopyToMem;
                                            end if;
                                        else
                                            mem_byte_we <= "0000";
                                        end if;
                                        
                                when Send =>
                                        mem_byte_we         <= "0000";
                                        intr                <= '1';
                                        active              <= '0';
                                        EA                  <= SWait;
                                        send_data           <= '0';
                                        mem_ddr_access      <= '0';

                                --Sends The TASK_ALLOCATION header: pe_target, noc size, service, task id, code size
                                When STask_Allocation_Target =>
                                        send_data       <= '1';
                                        if send_av = '1' then
                                            data_write  <= pe_target;
                                            EA          <= STask_Allocation_Size;
                                        end if;

                                When STask_Allocation_Size =>
                                        send_data       <= '1';
                                        if send_av = '1' then
                                            data_write  <= x"0000" & task_size;
                                            EA          <= STask_Allocation_Service;
                                        end if;

                                When STask_Allocation_Service =>
                                        send_data       <= '1';
                                        if send_av = '1' then
                                            data_write  <= TASK_ALLOCATION;
                                            EA          <= STask_Allocation_TaskId;
                                        end if;

                                When STask_Allocation_TaskId =>
                                        send_data       <= '1';
                                        if send_av = '1' then
                                            data_write  <= task_id;
                                            EA          <= STask_Allocation_task_size;
                                        end if;

                                When STask_Allocation_task_size =>
                                        if send_av = '1' then
                                            data_write  <= x"0000" & size;
                                            size        <= size - 1;
                                            EA          <= Swait_send_header;
                                        end if;
                                When Swait_send_header =>
                                        if send_av = '1' then
                                            EA          <= SWaitMem1;
                                            send_data   <= '0';
                                        end if;

                                when others => null;
                        end case;
                end if;
        end process;
end dma;
