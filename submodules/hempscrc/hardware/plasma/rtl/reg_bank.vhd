---------------------------------------------------------------------
-- TITLE: Register Bank
-- AUTHOR: Steve Rhoads (rhoadss@yahoo.com)
--         Cristiane Woszezenki (cwoszezenki@inf.pucrs.br)
--         Ismael Augusto Grehs (grehs@inf.pucrs.br)
-- DATE CREATED: 2/2/01
-- FILENAME: reg_bank.vhd
-- PROJECT: Plasma CPU core
-- COPYRIGHT: Software placed into the public domain by the author.
--    Software 'as is' without warranty.  Author liable for nothing.
-- DESCRIPTION:
--    Implements a register bank with 32 registers that are 32-bits wide.
--    There are two read-ports and one write port.
--
--
--  SIMPLIFIED ONLY FOR XILINX - ISMAEL GREHS  /  FERNANDO MORAES - 22/DEZ/2006
--
--
---------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use work.mlite_pack.all;

--Uncomment following two lines for Xilinx RAM32X1D
library unisim;              --Xilinx
use unisim.vcomponents.all;  --Xilinx

entity reg_bank is
   generic(memory_type : string := "XIL");
   port(clk            : in  std_logic;
      reset_in       : in  std_logic;
      pause          : in  std_logic;
      rs_index       : in  std_logic_vector(5 downto 0);
      rt_index       : in  std_logic_vector(5 downto 0);
      rd_index       : in  std_logic_vector(5 downto 0);
      reg_source_out : out std_logic_vector(31 downto 0);
      reg_target_out : out std_logic_vector(31 downto 0);
      reg_dest_new   : in  std_logic_vector(31 downto 0); 
      page           : out std_logic_vector(31 downto 0);
      intr_enable    : out std_logic);
end; --entity reg_bank


--------------------------------------------------------------------
-- The ram_block architecture attempts to use TWO dual-port memories.
-- Different FPGAs and ASICs need different implementations.
-- Choose one of the RAM implementations below.
-- I need feedback on this section!
--------------------------------------------------------------------
architecture ram_block of reg_bank is

   signal intr_enable_reg : std_logic;
   type ram_type is array(31 downto 0) of std_logic_vector(31 downto 0);
   
   --controls access to dual-port memories
   signal addr_read1, addr_read2 : std_logic_vector(4 downto 0);
   signal addr_write             : std_logic_vector(4 downto 0);
   signal data_out1, data_out2   : std_logic_vector(31 downto 0);
   signal write_enable           : std_logic;  
   signal page_reg               : std_logic_vector(31 downto 0);

begin
  
reg_proc: process(clk, rs_index, rt_index, rd_index, reg_dest_new, 
      intr_enable_reg, data_out1, data_out2, reset_in, pause)
begin
   page <= page_reg;

   --setup for first dual-port memory
   if rs_index = "101110" then  --reg_epc CP0 14
      addr_read1 <= "00000";
   else
      addr_read1 <= rs_index(4 downto 0);
   end if;
   
   case rs_index is
   when "000000" => reg_source_out <= ZERO;
   when "101100" => reg_source_out <= ZERO(31 downto 1) & intr_enable_reg;
   
   --interrupt vector address = 0x44 (syscall)
   when "111110" => reg_source_out <= ZERO(31 downto 8) & "01000100";
                    page <= (others => '0');
                    
   --interrupt vector address = 0x3c (hardware)
   when "111111" => reg_source_out <= ZERO(31 downto 8) & "00111100";
                    page <= (others => '0');
                    
   when others   => reg_source_out <= data_out1;
   end case;

   --setup for second dual-port memory
   addr_read2 <= rt_index(4 downto 0);
   case rt_index is
   when "000000" => reg_target_out <= ZERO;
   when others   => reg_target_out <= data_out2;
   end case;

   --setup write port for both dual-port memories
   if rd_index /= "000000" and rd_index /= "101100" and rd_index /= "101010" and pause = '0' then
      write_enable <= '1';
   else
      write_enable <= '0';
   end if;
   
   if rd_index = "101110" then      --reg_epc CP0 14
      addr_write <= "00000";
   else
      addr_write <= rd_index(4 downto 0);
   end if;

   if reset_in = '1' then
      intr_enable_reg <= '0';
      page_reg <= (others=>'0');
   elsif rising_edge(clk) then
      
      --disable interrupts
      if rd_index = "101110" then   --reg_epc CP0 14
         intr_enable_reg <= '0';       
      
      --sets interrupts register
      elsif rd_index = "101100" then
         intr_enable_reg <= reg_dest_new(0);      
        
      --sets page register
      elsif rd_index = "101010" then
         page_reg <= reg_dest_new;
      end if;
      
      --resets page on interrupts    
      if rs_index = "111110" or  rs_index = "111111" then
        page_reg <= (others => '0');      
      end if;
      
   end if;

   intr_enable <= intr_enable_reg; 
   
end process;   

--------------------------------------------------------------
---- Pick only ONE of the dual-port RAM implementations below!
--------------------------------------------------------------

   -- Option #1
   -- One tri-port RAM, two read-ports, one write-port
   -- 32 registers 32-bits wide
   tri_port_mem:
   if memory_type = "TRI" generate
      ram_proc: process(clk, addr_read1, addr_read2, 
            addr_write, reg_dest_new, write_enable)
      variable tri_port_ram : ram_type;
      begin
         data_out1 <= tri_port_ram(conv_integer(addr_read1));
         data_out2 <= tri_port_ram(conv_integer(addr_read2));
         if rising_edge(clk) then
            if write_enable = '1' then
               tri_port_ram(conv_integer(addr_write)) := reg_dest_new;
            end if;
         end if;
      end process;
   end generate; --tri_port_mem


   -- Option #3
   -- RAM32X1D: 32 x 1 positive edge write, asynchronous read dual-port 
   -- distributed RAM for all Xilinx FPGAs
   xilinx_32x1d:
   if memory_type = "XIL" generate
   begin
      reg_loop: for i in 0 to 31 generate
      begin
         --Read port 1
         reg_bit1 : RAM32X1D
         port map (
            WCLK  => clk,              -- Port A write clock input
            WE    => write_enable,              -- Port A write enable input
            A0    => addr_write(0),    -- Port A address[0] input bit
            A1    => addr_write(1),    -- Port A address[1] input bit
            A2    => addr_write(2),    -- Port A address[2] input bit
            A3    => addr_write(3),    -- Port A address[3] input bit
            A4    => addr_write(4),    -- Port A address[4] input bit                        
            D     => reg_dest_new(i),  -- Port A 1-bit data input
            DPRA0 => addr_read1(0),    -- Port B address[0] input bit
            DPRA1 => addr_read1(1),    -- Port B address[1] input bit
            DPRA2 => addr_read1(2),    -- Port B address[2] input bit
            DPRA3 => addr_read1(3),    -- Port B address[3] input bit
            DPRA4 => addr_read1(4),    -- Port B address[4] input bit                        
            DPO   => data_out1(i),     -- Port B 1-bit data output
            SPO   => open              -- Port A 1-bit data output
         );

         --Read port 2
         reg_bit2 : RAM32X1D
         port map (
            WCLK  => clk,              -- Port A write clock input
            WE    => write_enable,              -- Port A write enable input
            A0    => addr_write(0),    -- Port A address[0] input bit
            A1    => addr_write(1),    -- Port A address[1] input bit
            A2    => addr_write(2),    -- Port A address[2] input bit
            A3    => addr_write(3),    -- Port A address[3] input bit
            A4    => addr_write(4),    -- Port A address[4] input bit            
            D     => reg_dest_new(i),  -- Port A 1-bit data input
            DPRA0 => addr_read2(0),    -- Port B address[0] input bit
            DPRA1 => addr_read2(1),    -- Port B address[1] input bit
            DPRA2 => addr_read2(2),    -- Port B address[2] input bit
            DPRA3 => addr_read2(3),    -- Port B address[3] input bit
            DPRA4 => addr_read2(4),    -- Port B address[4] input bit            
            DPO   => data_out2(i),     -- Port B 1-bit data output
            SPO   => open              -- Port A 1-bit data output
         );

      end generate; --reg_loop

   end generate; --xilinx_32x1d

end; --architecture ram_block
