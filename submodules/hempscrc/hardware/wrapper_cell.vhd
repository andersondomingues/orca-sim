library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity wrapper_cell is
generic(
    faulty_value: in std_logic := '0'
);
 port(
    func_in     			: in  std_logic;
    scan_in     			: in  std_logic;
    clock       			: in  std_logic;
    shift       			: in  std_logic;
    normal_mode 			: in  std_logic;
    faulty_port 			: in  std_logic;
    out_failed_test_port	: in  std_logic;--fochi
    func_out    			: out std_logic;
    scan_out    			: out std_logic
);
end wrapper_cell;

architecture wrapper_cell of wrapper_cell is
    signal m1         : std_logic;
    signal m2         : std_logic;
    signal dff        : std_logic;
begin

mux_1 : m1 <= scan_in when shift = '1' else
         m2;

mux_2 : m2 <= func_in when normal_mode = '1' else
         dff;

mux_func_out: func_out <= faulty_value when faulty_port = '1' and out_failed_test_port = '0' else 
			m2; 

D_flip_flop: process (clock)
begin
    if (clock'event and clock='1') then
        dff <= m1;
    end if;
end process;

scan_out <= dff;
   
end wrapper_cell;

