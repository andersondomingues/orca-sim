library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use ieee.numeric_std.all;
use work.HeMPS_defaults.all;

entity crc4p is
	port(
		input:	in	std_logic_vector(15 downto 0);
		output: out std_logic_vector(3 downto 0)
	);
end crc4p;

architecture crc4p of crc4p is

	type shift_vector is array(0 to 15) of std_logic_vector(3 downto 0);
	signal shift: shift_vector;
	
begin
	
	output(3) <= input(15) xor input(14) xor input(10) xor input(7) xor input(6) xor input(4) xor input(2) xor input(1) xor input(0) when input(15 downto 0) /= "ZZZZZZZZZZZZZZZZ" else 'Z';
	output(2) <= input(13) xor input(10) xor input(9)  xor input(7) xor input(5) xor input(4) xor input(3) xor input(2) when input(15 downto 0) /= "ZZZZZZZZZZZZZZZZ" else 'Z';
	output(1) <= input(12) xor input(9)  xor input(8)  xor input(6) xor input(4) xor input(3) xor input(2) xor input(1) when input(15 downto 0) /= "ZZZZZZZZZZZZZZZZ" else 'Z';
	output(0) <= input(15) xor input(11) xor input(8)  xor input(7) xor input(5) xor input(3) xor input(2) xor input(1) xor input(0) when input(15 downto 0) /= "ZZZZZZZZZZZZZZZZ" else 'Z';


end crc4p;

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use work.HeMPS_defaults.all;


entity deccrc is
	port(
		input	: in  std_logic_vector(15 downto 0);
		crc_in	: in  std_logic_vector(3 downto 0);
		rx		: in  std_logic;
	    error	: out std_logic
	);
end deccrc;

architecture deccrc of deccrc is

	signal header : regflit;

	signal aux		: std_logic;
	signal crc_sig	: std_logic_vector(3 downto 0);

begin

	CRCU: entity work.crc4p port map(input=>input,output=>crc_sig);
	error <= '1' when crc_sig/=crc_in else '0';



end deccrc;


library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use work.HeMPS_defaults.all;


entity ec_module is
	port(
		crc_out:   out std_logic_vector(3 downto 0);
		data_inc:  in  std_logic_vector(15 downto 0);
		crc_in:    in  std_logic_vector(3 downto 0);
		data_ind:  in  std_logic_vector(15 downto 0);
		rx_in:	   in  std_logic;
		error:	   out std_logic;
		error_in:   in  std_logic;
		credit_in:  in  std_logic;
		credit_out: out std_logic
	);
end ec_module;

architecture ec_module of ec_module is

begin
	coder: entity work.crc4p
	port map(
		input => data_inc,
		output => crc_out
	);

	decoder: entity work.deccrc
	port map(
		input => data_ind,
		crc_in => crc_in,
		rx => rx_in,
		error => error
	);

	credit_out <= credit_in;-- and (not error_in);


end ec_module;
