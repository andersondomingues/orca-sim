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
use ieee.std_logic_arith.all;

entity dma is
   port(  
      clock             : in  std_logic;
      reset             : in  std_logic;
      -- NI interface
      read_av           : in  std_logic;
      read_data         : out std_logic;
      send_av           : in  std_logic;      
      send_data         : out std_logic;
      failed_reception  : in  std_logic;
      release_ni        : out std_logic;
      
      -- Configuration pins
      set_address       : in  std_logic;
      set_size          : in  std_logic;
      set_op            : in  std_logic;
      start             : in  std_logic;
      -- Input/Output data ports
      data_read         : in  std_logic_vector(31 downto 0);
      data_write        : out std_logic_vector(31 downto 0);
      active            : out std_logic;
      -- Interrupt management
      intr              : out std_logic;
      intr_ack          : in  std_logic;      
      -- Memory interface
      mem_address       : out std_logic_vector(31 downto 0);
      mem_data_write    : out std_logic_vector(31 downto 0);
      mem_data_read     : in  std_logic_vector(31 downto 0);
      mem_byte_we       : out std_logic_vector(3 downto 0)
      );
          
end;

architecture dma of dma is

   type state is (SWait,SCopyFromMem, SCopyFromMem0, SCopyToMem, SEnd);
   signal EA: state;
   
   signal address                         : std_logic_vector(31 downto 0);
   signal data_reg                        : std_logic_vector(31 downto 0);        -- Deals with the one cycle memory latency
   signal size                            : std_logic_vector(15 downto 0); 
   signal operation                       : std_logic;
   signal read_data_reg                   : std_logic;
   signal rst_failed_reception_stepdet    : std_logic;
   signal failed_reception_stepdet        : std_logic;

begin

        mem_address <= address - 4 when operation = '1' else address;
        data_write <= data_reg when read_data_reg = '1' else mem_data_read;
   
        read_data <= '1' when EA = SCopyToMem and read_av = '1' else '0';
        send_data <= '1' when EA = SCopyFromMem and send_av = '1' else '0';
   
        stepdet: entity work.stepdet
          port map(
            ck          => clock,
            rst         => rst_failed_reception_stepdet,
            in_sig      => failed_reception,
            out_sig     => failed_reception_stepdet
          );

        process (clock,reset)
        begin   
        if reset = '1' then
                address         <= (others => '0');
                mem_data_write  <= (others => '0');
                size            <= (others => '0');
         
                intr            <= '0';
                operation       <= '0';
                mem_byte_we     <= "0000";
                data_reg        <= (others => '0');
                read_data_reg   <= '0';
                active          <= '0';
                release_ni      <= '0';
                rst_failed_reception_stepdet  <= '1';
                
                elsif rising_edge(clock) then
                case EA is
                    when SWait =>
                        if intr_ack = '1' then
                            intr <= '0';
                            EA <= SWait;
                        elsif set_size = '1' then
                            size <= data_read(15 downto 0);
                            EA <= SWait;
                        elsif set_op = '1' then
                            operation <= data_read(0);
                            EA <= SWait;
                        elsif set_address = '1' then
                            address <= data_read;
                            EA <= SWait;
                        elsif start = '1' then
                            active <= '1';
                            if operation = '0' then
                                EA <= SCopyFromMem0;                                                    
                            else
                                EA <= SCopyToMem;
                            end if;
                        end if;
                        release_ni <= '0';
                        rst_failed_reception_stepdet  <= '0';
                                
                    -- Ensures that the first word will not be sent 2 times.
                    when SCopyFromMem0 =>
                        if send_av = '1' then   
                            address <= address + 4;
                            size <= size - 1;
                            EA <= SCopyFromMem;
                        else
                            EA <= SCopyFromMem0;
                        end if;            
            
                    when SCopyFromMem =>            
                        if send_av = '1' then                   
                            read_data_reg <= '0';
                            address <= address + 4;
                            size <= size - 1;

                            if size = 0 then
                                EA <= Send;
                            else
                                EA <= SCopyFromMem;
                            end if;
                        else
                            if read_data_reg = '0' then     -- nao grava no reg pq ele deve ser lido na sequela
                                data_reg <= mem_data_read;
                            end if;
                            
                            read_data_reg <= '1';
                            EA <= SCopyFromMem;            
                        end if;
                    
                    when SCopyToMem =>
                            if read_av = '1' then
                                mem_data_write <= data_read;
                                mem_byte_we <= "1111";
                                address <= address + 4;
                                size <= size - '1';
           
                                if size = 1 then        -- The last word is sent in the Send state
                                    EA <= Send;
                                else
                                    EA <= SCopyToMem;
                                end if;
                            elsif failed_reception_stepdet = '1' then
                                EA                            <= SWait;
                                active                        <= '0';
                                release_ni                    <= '1';
                                rst_failed_reception_stepdet  <= '1';
                            else
                               mem_byte_we <= "0000";
                               --read_data <= '0';
                            end if;
                    
     
                    when SEnd =>
                        mem_byte_we     <= "0000";
                        --read_data     <= '0';
                        intr            <= '1';
                        active          <= '0';
                        EA              <= SWait;
                end case;   
        end if;
        end process;     
end dma;
