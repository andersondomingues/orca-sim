----------------------------------------------------------------------------------------------
--
--      Input file         : dsram.vhd
--      Design name        : dsram
--      Author             : Tamar Kranenburg
--      Company            : Delft University of Technology
--                         : Faculty EEMCS, Department ME&CE
--                         : Systems and Circuits group
--
--      Description        : Dual Port Synchronous 'read after write' Ram. 1 Read Port and 1
--                           Write Port.
--
--
----------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.ALL;

--LIBRARY mblite;
USE work.std_Pkg.ALL;

ENTITY dsram IS GENERIC
(
    WIDTH : positive := 32;
    SIZE  : positive := 8
);
PORT
(
    dat_o   : OUT std_ulogic_vector(WIDTH - 1 DOWNTO 0);
    adr_i   : IN std_ulogic_vector(SIZE - 1 DOWNTO 0);
    ena_i   : IN std_ulogic;
    dat_w_i : IN std_ulogic_vector(WIDTH - 1 DOWNTO 0);
    adr_w_i : IN std_ulogic_vector(SIZE - 1 DOWNTO 0);
    wre_i   : IN std_ulogic;
    clk_i   : IN std_ulogic
);
END dsram;

ARCHITECTURE arch OF dsram IS
    TYPE ram_type IS array(2 ** SIZE - 1 DOWNTO 0) OF std_ulogic_vector(WIDTH - 1 DOWNTO 0);
    SIGNAL ram :  ram_type;
BEGIN
    PROCESS(clk_i)
    BEGIN
        IF rising_edge(clk_i) THEN
            IF ena_i = '1' THEN
                IF wre_i = '1' THEN
                    ram(my_conv_integer(adr_w_i)) <= dat_w_i;
                END IF;
                dat_o <= ram(my_conv_integer(adr_i));
            END IF;
        END IF;
    END PROCESS;
END arch;
