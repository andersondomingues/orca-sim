----------------------------------------------------------------------------------------------
--
--      Input file         : gprf.vhd
--      Design name        : gprf
--      Author             : Tamar Kranenburg
--      Company            : Delft University of Technology
--                         : Faculty EEMCS, Department ME&CE
--                         : Systems and Circuits group
--
--      Description        : The general purpose register infers memory blocks to implement
--                           the register file. All outputs are registered, possibly by using
--                           registered memory elements.
--
----------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.ALL;

--LIBRARY mblite;
USE work.config_Pkg.ALL;
USE work.core_Pkg.ALL;
USE work.std_Pkg.ALL;

ENTITY gprf IS PORT
(
    gprf_o : OUT gprf_out_type;
    gprf_i : IN gprf_in_type;
    ena_i  : IN std_ulogic;
    clk_i  : IN std_ulogic
);
END gprf;

-- This architecture is the default implementation. It
-- consists of three dual port memories. Other
-- architectures can be added while configurations can
-- control the implemented architecture.
ARCHITECTURE arch OF gprf IS
BEGIN
    a : dsram GENERIC MAP
    (
        WIDTH   => CFG_DMEM_WIDTH,
        SIZE    => CFG_GPRF_SIZE
    )
    PORT MAP
    (
        dat_o   => gprf_o.dat_a_o,
        adr_i   => gprf_i.adr_a_i,
        ena_i   => ena_i,
        dat_w_i => gprf_i.dat_w_i,
        adr_w_i => gprf_i.adr_w_i,
        wre_i   => gprf_i.wre_i,
        clk_i   => clk_i
    );

    b : dsram GENERIC MAP
    (
        WIDTH   => CFG_DMEM_WIDTH,
        SIZE    => CFG_GPRF_SIZE
    )
    PORT MAP
    (
        dat_o   => gprf_o.dat_b_o,
        adr_i   => gprf_i.adr_b_i,
        ena_i   => ena_i,
        dat_w_i => gprf_i.dat_w_i,
        adr_w_i => gprf_i.adr_w_i,
        wre_i   => gprf_i.wre_i,
        clk_i   => clk_i
    );

    d : dsram GENERIC MAP
    (
        WIDTH   => CFG_DMEM_WIDTH,
        SIZE    => CFG_GPRF_SIZE
    )
    PORT MAP
    (
        dat_o   => gprf_o.dat_d_o,
        adr_i   => gprf_i.adr_d_i,
        ena_i   => ena_i,
        dat_w_i => gprf_i.dat_w_i,
        adr_w_i => gprf_i.adr_w_i,
        wre_i   => gprf_i.wre_i,
        clk_i   => clk_i
    );
END arch;
