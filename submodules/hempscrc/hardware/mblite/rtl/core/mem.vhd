----------------------------------------------------------------------------------------------
--
--      Input file         : mem.vhd
--      Design name        : mem
--      Author             : Tamar Kranenburg
--      Company            : Delft University of Technology
--                         : Faculty EEMCS, Department ME&CE
--                         : Systems and Circuits group
--
--      Description        : Memory retrieves data words from a data memory. Memory file
--                           access of byte, halfword and word sizes is supported. The sel_o
--                           signal indicates which bytes should be read or written. The
--                           responsibility for writing the right memory address is not within
--                           this integer unit but should be handled by the external memory
--                           device. This facilitates the addition of devices with different
--                           bus sizes.
--
--                           The dmem_i signals are directly connected to the decode and
--                           execute components.
--
----------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.ALL;

--LIBRARY mblite;
USE work.config_Pkg.ALL;
USE work.core_Pkg.ALL;
USE work.std_Pkg.ALL;

ENTITY mem IS PORT
(
    mem_o  : OUT mem_out_type;
    dmem_o : OUT dmem_out_type;
    mem_i  : IN mem_in_type;
    ena_i  : IN std_ulogic;
    rst_i  : IN std_ulogic;
    clk_i  : IN std_ulogic
);
END mem;

ARCHITECTURE arch OF mem IS
    SIGNAL r, rin : mem_out_type;
    SIGNAL mem_result : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
BEGIN
    -- connect pipline signals
    mem_o.ctrl_wb     <= r.ctrl_wb;
    mem_o.ctrl_mem_wb <= r.ctrl_mem_wb;
    mem_o.alu_result  <= r.alu_result;

    -- connect memory interface signals
    dmem_o.dat_o <= mem_result;
    dmem_o.sel_o <= decode_mem_store(mem_i.alu_result(1 DOWNTO 0), mem_i.ctrl_mem.transfer_size);
    dmem_o.we_o  <= mem_i.ctrl_mem.mem_write;
    dmem_o.adr_o <= mem_i.alu_result(CFG_DMEM_SIZE - 1 DOWNTO 0);
    dmem_o.ena_o <= mem_i.ctrl_mem.mem_read OR mem_i.ctrl_mem.mem_write;

    mem_comb: PROCESS(mem_i, mem_i.ctrl_wb, mem_i.ctrl_mem, r, r.ctrl_wb, r.ctrl_mem_wb)
        VARIABLE v : mem_out_type;
        VARIABLE intermediate : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
    BEGIN

        v := r;
        v.ctrl_wb := mem_i.ctrl_wb;

        IF mem_i.branch = '1' THEN
            -- set alu result for branch and load instructions
            v.alu_result := sign_extend(mem_i.program_counter, '0', 32);
        ELSE
            v.alu_result := mem_i.alu_result;
        END IF;

        -- Forward memory result
        IF CFG_MEM_FWD_WB = true AND ( r.ctrl_mem_wb.mem_read AND compare(mem_i.ctrl_wb.reg_d, r.ctrl_wb.reg_d)) = '1' THEN
            intermediate := align_mem_load(mem_i.mem_result, r.ctrl_mem_wb.transfer_size, r.alu_result(1 DOWNTO 0));
            mem_result <= align_mem_store(intermediate, mem_i.ctrl_mem.transfer_size);
        ELSE
            mem_result <= mem_i.dat_d;
        END IF;

        v.ctrl_mem_wb.mem_read := mem_i.ctrl_mem.mem_read;
        v.ctrl_mem_wb.transfer_size := mem_i.ctrl_mem.transfer_size;

        rin <= v;

    END PROCESS;

    mem_seq: PROCESS(clk_i)
        PROCEDURE proc_mem_reset IS
        BEGIN
            r.alu_result  <= (OTHERS => '0');
            r.ctrl_wb.reg_d <= (OTHERS => '0');
            r.ctrl_wb.reg_write <= '0';
            r.ctrl_mem_wb.mem_read <= '0';
            r.ctrl_mem_wb.transfer_size <= WORD;
        END PROCEDURE proc_mem_reset;
    BEGIN
        IF rising_edge(clk_i) THEN
            IF rst_i = '1' THEN
                proc_mem_reset;
            ELSIF ena_i = '1' THEN
                r <= rin;
            END IF;
        END IF;
    END PROCESS;
END arch;
