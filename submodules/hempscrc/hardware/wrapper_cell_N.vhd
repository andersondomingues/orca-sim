library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity wrapper_cell_N is
 
generic(
    N		: in integer;
    faulty_value: in std_logic := '0'
);
 
 port(
    func_in     		: in  std_logic_vector (N-1 downto 0);
    scan_in     		: in  std_logic;
    clock       		: in  std_logic;
    shift       		: in  std_logic;
    normal_mode 		: in  std_logic;
    faulty_port 		: in  std_logic;
    out_failed_test_port: in  std_logic;--fochi
    func_out    		: out std_logic_vector (N-1 downto 0);
    scan_out    		: out std_logic
);
end wrapper_cell_N;

architecture wrapper_cell_N of wrapper_cell_N is

signal scan_in_internal   : std_logic_vector (N downto 0);
signal scan_out_internal   : std_logic_vector (N downto 0);

begin

scan_in_internal(0) <= scan_in;

wrapper_cells_generetion: for i in 0 to N-1 generate
	wrapper_cells:  Entity work.wrapper_cell
		generic map(
			faulty_value => faulty_value
		)
		port map(
			func_in     			=> func_in(i),
			scan_in     			=> scan_in_internal(i),
			clock       			=> clock,
			shift       			=> shift,
			normal_mode 			=> normal_mode,
			faulty_port 			=> faulty_port,
			out_failed_test_port	=> out_failed_test_port, --fochi
			func_out    			=> func_out(i),
			scan_out    			=> scan_out_internal(i)
			);
scan_in_internal(i+1) <= scan_out_internal(i);
 
end generate;

scan_out <= scan_out_internal(N-1);

end wrapper_cell_N;

