----------------------------------------------------------------------------------------------
--
--      Input file         : fetch.vhd
--      Design name        : fetch
--      Author             : Tamar Kranenburg
--      Company            : Delft University of Technology
--                         : Faculty EEMCS, Department ME&CE
--                         : Systems and Circuits group
--
--      Description        : Instruction Fetch Stage inserts instruction into the pipeline. It
--                           uses a single port Random Access Memory component which holds
--                           the instructions. The next instruction is computed in the decode
--                           stage.
--
----------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.ALL;

--LIBRARY mblite;
USE work.config_Pkg.ALL;
USE work.core_Pkg.ALL;
USE work.std_Pkg.ALL;

ENTITY fetch IS PORT
(
    fetch_o : OUT fetch_out_type;
    imem_o  : OUT imem_out_type;
    fetch_i : IN fetch_in_type;
    rst_i   : IN std_ulogic;
    ena_i   : IN std_ulogic;
    clk_i   : IN std_ulogic
);
END fetch;

ARCHITECTURE arch OF fetch IS
    SIGNAL r, rin   : fetch_out_type;
BEGIN

    fetch_o.program_counter <= r.program_counter;
    imem_o.adr_o <= rin.program_counter;
    imem_o.ena_o <= ena_i;

    fetch_comb: PROCESS(fetch_i, r, rst_i)
        VARIABLE v : fetch_out_type;
    BEGIN
        v := r;
        IF fetch_i.hazard = '1' THEN
            v.program_counter := r.program_counter;
        ELSIF fetch_i.branch = '1' THEN
            v.program_counter := fetch_i.branch_target;
        ELSE
            v.program_counter := increment(r.program_counter(CFG_IMEM_SIZE - 1 DOWNTO 2)) & "00";
        END IF;
        rin <= v;
    END PROCESS;

    fetch_seq: PROCESS(clk_i)
    BEGIN
        IF rising_edge(clk_i) THEN
            IF rst_i = '1' THEN
                r.program_counter <= (OTHERS => '0');
            ELSIF ena_i = '1' THEN
                r <= rin;
            END IF;
        END IF;
    END PROCESS;

END arch;
