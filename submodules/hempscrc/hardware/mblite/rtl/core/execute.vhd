----------------------------------------------------------------------------------------------
--
--      Input file         : execute.vhd
--      Design name        : execute
--      Author             : Tamar Kranenburg
--      Company            : Delft University of Technology
--                         : Faculty EEMCS, Department ME&CE
--                         : Systems and Circuits group
--
--      Description        : The Execution Unit performs all arithmetic operations and makes
--                           the branch decision. Furthermore the forwarding logic is located
--                           here. Everything is computed within a single clock-cycle
--
--
----------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.ALL;

--LIBRARY mblite;
USE work.config_Pkg.ALL;
USE work.core_Pkg.ALL;
USE work.std_Pkg.ALL;

ENTITY execute IS GENERIC
(
    G_USE_HW_MUL : boolean := CFG_USE_HW_MUL;
    G_USE_BARREL : boolean := CFG_USE_BARREL
);
PORT
(
    exec_o : OUT execute_out_type;
    exec_i : IN execute_in_type;
    ena_i  : IN std_ulogic;
    rst_i  : IN std_ulogic;
    clk_i  : IN std_ulogic
);
END execute;

ARCHITECTURE arch OF execute IS

    TYPE execute_reg_type IS RECORD
        carry      : std_ulogic;
        flush_ex   : std_ulogic;
    END RECORD;

    SIGNAL r, rin : execute_out_type;
    SIGNAL reg, regin : execute_reg_type;

BEGIN

    exec_o <= r;

    execute_comb: PROCESS(exec_i,exec_i.fwd_mem,exec_i.ctrl_ex,
            exec_i.ctrl_wb,exec_i.ctrl_mem,
            exec_i.ctrl_mem.transfer_size,
            exec_i.ctrl_mem_wb,exec_i.fwd_dec,
            r,r.ctrl_mem,r.ctrl_mem.transfer_size,
            r.ctrl_wb,reg)

        VARIABLE v : execute_out_type;
        VARIABLE v_reg : execute_reg_type;

        VARIABLE alu_src_a : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        VARIABLE alu_src_b : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        VARIABLE carry : std_ulogic;

        VARIABLE result : std_ulogic_vector(CFG_DMEM_WIDTH DOWNTO 0);
        VARIABLE result_add : std_ulogic_vector(CFG_DMEM_WIDTH DOWNTO 0);
        VARIABLE zero : std_ulogic;

        VARIABLE dat_a, dat_b : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        VARIABLE sel_dat_a, sel_dat_b, sel_dat_d : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        VARIABLE mem_result : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);

    BEGIN

        v := r;

        sel_dat_a := select_register_data(exec_i.dat_a, exec_i.reg_a, exec_i.fwd_dec_result, forward_condition(exec_i.fwd_dec.reg_write, exec_i.fwd_dec.reg_d, exec_i.reg_a));
        sel_dat_b := select_register_data(exec_i.dat_b, exec_i.reg_b, exec_i.fwd_dec_result, forward_condition(exec_i.fwd_dec.reg_write, exec_i.fwd_dec.reg_d, exec_i.reg_b));
        sel_dat_d := select_register_data(exec_i.dat_d, exec_i.ctrl_wb.reg_d, exec_i.fwd_dec_result, forward_condition(exec_i.fwd_dec.reg_write, exec_i.fwd_dec.reg_d, exec_i.ctrl_wb.reg_d));

        IF reg.flush_ex = '1' THEN
            v.ctrl_mem.mem_write := '0';
            v.ctrl_mem.mem_read := '0';
            v.ctrl_wb.reg_write := '0';
            v.ctrl_wb.reg_d := (OTHERS => '0');
        ELSE
            v.ctrl_mem := exec_i.ctrl_mem;
            v.ctrl_wb := exec_i.ctrl_wb;
        END IF;

        IF exec_i.ctrl_mem_wb.mem_read = '1' THEN
            mem_result := align_mem_load(exec_i.mem_result, exec_i.ctrl_mem_wb.transfer_size, exec_i.alu_result(1 DOWNTO 0));
        ELSE
            mem_result := exec_i.alu_result;
        END IF;

        IF forward_condition(r.ctrl_wb.reg_write, r.ctrl_wb.reg_d, exec_i.reg_a) = '1' THEN
            -- Forward Execution Result to REG a
            dat_a := r.alu_result;
        ELSIF forward_condition(exec_i.fwd_mem.reg_write, exec_i.fwd_mem.reg_d, exec_i.reg_a) = '1' THEN
            -- Forward Memory Result to REG a
            dat_a := mem_result;
        ELSE
            -- DEFAULT: value of REG a
            dat_a := sel_dat_a;
        END IF;

        IF forward_condition(r.ctrl_wb.reg_write, r.ctrl_wb.reg_d, exec_i.reg_b) = '1' THEN
            -- Forward (latched) Execution Result to REG b
            dat_b := r.alu_result;
        ELSIF forward_condition(exec_i.fwd_mem.reg_write, exec_i.fwd_mem.reg_d, exec_i.reg_b) = '1' THEN
            -- Forward Memory Result to REG b
            dat_b := mem_result;
        ELSE
            -- DEFAULT: value of REG b
            dat_b := sel_dat_b;
        END IF;

        IF forward_condition(r.ctrl_wb.reg_write, r.ctrl_wb.reg_d, exec_i.ctrl_wb.reg_d) = '1' THEN
            -- Forward Execution Result to REG d
            v.dat_d := align_mem_store(r.alu_result, exec_i.ctrl_mem.transfer_size);
        ELSIF forward_condition(exec_i.fwd_mem.reg_write, exec_i.fwd_mem.reg_d, exec_i.ctrl_wb.reg_d) = '1' THEN
            -- Forward Memory Result to REG d
            v.dat_d := align_mem_store(mem_result, exec_i.ctrl_mem.transfer_size);
        ELSE
            -- DEFAULT: value of REG d
            v.dat_d := align_mem_store(sel_dat_d, exec_i.ctrl_mem.transfer_size);
        END IF;

        -- Set the first operand of the ALU
        CASE exec_i.ctrl_ex.alu_src_a IS
            WHEN ALU_SRC_PC       => alu_src_a := sign_extend(exec_i.program_counter, '0', 32);
            WHEN ALU_SRC_NOT_REGA => alu_src_a := NOT dat_a;
            WHEN ALU_SRC_ZERO     => alu_src_a := (OTHERS => '0');
            WHEN OTHERS           => alu_src_a := dat_a;
        END CASE;

        -- Set the second operand of the ALU
        CASE exec_i.ctrl_ex.alu_src_b IS
            WHEN ALU_SRC_IMM      => alu_src_b := exec_i.imm;
            WHEN ALU_SRC_NOT_IMM  => alu_src_b := NOT exec_i.imm;
            WHEN ALU_SRC_NOT_REGB => alu_src_b := NOT dat_b;
            WHEN OTHERS           => alu_src_b := dat_b;
        END CASE;

        -- Determine value of carry in
        CASE exec_i.ctrl_ex.carry IS
            WHEN CARRY_ALU   => carry := reg.carry;
            WHEN CARRY_ONE   => carry := '1';
            WHEN CARRY_ARITH => carry := alu_src_a(CFG_DMEM_WIDTH - 1);
            WHEN OTHERS      => carry := '0';
        END CASE;

        result_add := add(alu_src_a, alu_src_b, carry);

        CASE exec_i.ctrl_ex.alu_op IS
            WHEN ALU_ADD    => result := result_add;
            WHEN ALU_OR     => result := '0' & (alu_src_a OR alu_src_b);
            WHEN ALU_AND    => result := '0' & (alu_src_a AND alu_src_b);
            WHEN ALU_XOR    => result := '0' & (alu_src_a XOR alu_src_b);
            WHEN ALU_SHIFT  => result := alu_src_a(0) & carry & alu_src_a(CFG_DMEM_WIDTH - 1 DOWNTO 1);
            WHEN ALU_SEXT8  => result := '0' & sign_extend(alu_src_a(7 DOWNTO 0), alu_src_a(7), 32);
            WHEN ALU_SEXT16 => result := '0' & sign_extend(alu_src_a(15 DOWNTO 0), alu_src_a(15), 32);
            WHEN ALU_MUL =>
                IF G_USE_HW_MUL = true THEN
                    result := '0' & multiply(alu_src_a, alu_src_b);
                ELSE
                    result := (OTHERS => '0');
                END IF;
            WHEN ALU_BS =>
                IF G_USE_BARREL = true THEN
                    result := '0' & shift(alu_src_a, alu_src_b(4 DOWNTO 0), exec_i.imm(10), exec_i.imm(9));
                ELSE
                    result := (OTHERS => '0');
                END IF;
            WHEN OTHERS =>
                result := (OTHERS => '0');
                REPORT "Invalid ALU operation" SEVERITY FAILURE;
        END CASE;

        -- Set carry register
        IF exec_i.ctrl_ex.carry_keep = CARRY_KEEP THEN
            v_reg.carry := reg.carry;
        ELSE
            v_reg.carry := result(CFG_DMEM_WIDTH);
        END IF;

        zero := is_zero(dat_a);

        -- Overwrite branch condition
        IF reg.flush_ex = '1' THEN
            v.branch := '0';
        ELSE
            -- Determine branch condition
            CASE exec_i.ctrl_ex.branch_cond IS
                WHEN BNC => v.branch := '1';
                WHEN BEQ => v.branch := zero;
                WHEN BNE => v.branch := NOT zero;
                WHEN BLT => v.branch := dat_a(CFG_DMEM_WIDTH - 1);
                WHEN BLE => v.branch := dat_a(CFG_DMEM_WIDTH - 1) OR zero;
                WHEN BGT => v.branch := NOT dat_a(CFG_DMEM_WIDTH - 1);
                WHEN BGE => v.branch := NOT dat_a(CFG_DMEM_WIDTH - 1) OR zero;
                WHEN OTHERS => v.branch := '0';
            END CASE;
        END IF;

        -- Handle CMPU
        IF ( exec_i.ctrl_ex.operation AND NOT (alu_src_a(CFG_DMEM_WIDTH - 1) XOR alu_src_b(CFG_DMEM_WIDTH - 1))) = '1' THEN
            -- Set MSB
            v.alu_result(CFG_DMEM_WIDTH - 1 DOWNTO 0) := (NOT result(CFG_DMEM_WIDTH - 1)) & result(CFG_DMEM_WIDTH - 2 DOWNTO 0);
        ELSE
            -- Use ALU result
            v.alu_result := result(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        END IF;

        v.program_counter := exec_i.program_counter;

        -- Determine flush signals
        v.flush_id := v.branch;
        v_reg.flush_ex := v.branch AND NOT exec_i.ctrl_ex.delay;

        rin <= v;
        regin <= v_reg;

    END PROCESS;

    execute_seq: PROCESS(clk_i)
        PROCEDURE proc_execute_reset IS
        BEGIN
            r.alu_result             <= (OTHERS => '0');
            r.dat_d                  <= (OTHERS => '0');
            r.branch                 <= '0';
            r.program_counter        <= (OTHERS => '0');
            r.flush_id               <= '0';
            r.ctrl_mem.mem_write     <= '0';
            r.ctrl_mem.mem_read      <= '0';
            r.ctrl_mem.transfer_size <= WORD;
            r.ctrl_wb.reg_d          <= (OTHERS => '0');
            r.ctrl_wb.reg_write      <= '0';
            reg.carry                <= '0';
            reg.flush_ex             <= '0';
        END PROCEDURE proc_execute_reset;
    BEGIN
        IF rising_edge(clk_i) THEN
            IF rst_i = '1' THEN
                proc_execute_reset;
            ELSIF ena_i = '1' THEN
                r <= rin;
                reg <= regin;
            END IF;
        END IF;
    END PROCESS;
END arch;
