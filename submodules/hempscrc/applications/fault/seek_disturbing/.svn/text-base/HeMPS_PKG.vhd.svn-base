
--------------------------------------------------------------------------
-- package com tipos basicos
--------------------------------------------------------------------------
library IEEE;
use IEEE.Std_Logic_1164.all;
use IEEE.std_logic_unsigned.all;
use IEEE.std_logic_arith.all;

package HeMPS_PKG is

--------------------------------------------------------
-- HEMPS CONSTANTS
--------------------------------------------------------
	-- paging definitions
	constant PAGE_SIZE_H_INDEX		: integer := 13;
	constant PAGE_NUMBER_H_INDEX	: integer := 15;

	-- Hemps top definitions
	constant NUMBER_PROCESSORS_X	: integer := 3; 
	constant NUMBER_PROCESSORS_Y	: integer := 3; 

	constant MASTER_ADDRESS			: integer := 0;
	constant NUMBER_PROCESSORS		: integer := NUMBER_PROCESSORS_Y*NUMBER_PROCESSORS_X;

	constant NUMBER_OF_APPS			: integer := 0;
	type timearray is array(0 to 1) of time;
	constant appstime : timearray := (0 ms, 0 ms);

	subtype core_str is string(1 to 6);
	type core_type_type is array(0 to NUMBER_PROCESSORS-1) of core_str;
	constant core_type : core_type_type := (					"plasma",
											"plasma",
											"plasma",
											"plasma",
											"plasma",
											"plasma",
											"plasma",
											"plasma",
											"plasma");
											
end HeMPS_PKG;
