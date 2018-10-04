----------------------------------------------------------------------------------------------
--
--      Input file         : decode.vhd
--      Design name        : decode
--      Author             : Tamar Kranenburg
--      Company            : Delft University of Technology
--                         : Faculty EEMCS, Department ME&CE
--                         : Systems and Circuits group
--
--      Description        : This combined register file and decoder uses three Dual Port
--                           read after write Random Access Memory components. Every clock
--                           cycle three data values can be read (ra, rb and rd) and one value
--                           can be stored.
--
----------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.ALL;

--LIBRARY mblite;
USE work.config_Pkg.ALL;
USE work.core_Pkg.ALL;
USE work.std_Pkg.ALL;

ENTITY decode IS GENERIC
(
    G_INTERRUPT  : boolean := CFG_INTERRUPT;
    G_USE_HW_MUL : boolean := CFG_USE_HW_MUL;
    G_USE_BARREL : boolean := CFG_USE_BARREL;
    G_DEBUG      : boolean := CFG_DEBUG
);
PORT
(
    decode_o : OUT decode_out_type;
    gprf_o   : OUT gprf_out_type;
    decode_i : IN decode_in_type;
    ena_i    : IN std_ulogic;
    rst_i    : IN std_ulogic;
    clk_i    : IN std_ulogic
);
END decode;

ARCHITECTURE arch OF decode IS

    TYPE decode_reg_type IS RECORD
        instruction     : std_ulogic_vector(CFG_IMEM_WIDTH - 1 DOWNTO 0);
        program_counter : std_ulogic_vector(CFG_IMEM_SIZE - 1 DOWNTO 0);
        immediate       : std_ulogic_vector(15 DOWNTO 0);
        is_immediate    : std_ulogic;
        msr_interrupt_enable : std_ulogic;
        interrupt       : std_ulogic;
        delay_interrupt : std_ulogic;
    END RECORD;

    SIGNAL r, rin : decode_out_type;
    SIGNAL reg, regin : decode_reg_type;

    SIGNAL wb_dat_d : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);

BEGIN

    decode_o.imm <= r.imm;

    decode_o.ctrl_ex <= r.ctrl_ex;
    decode_o.ctrl_mem <= r.ctrl_mem;
    decode_o.ctrl_wb <= r.ctrl_wb;

    decode_o.reg_a <= r.reg_a;
    decode_o.reg_b <= r.reg_b;
    decode_o.hazard <= r.hazard;
    decode_o.program_counter <= r.program_counter;

    decode_o.fwd_dec_result <= r.fwd_dec_result;
    decode_o.fwd_dec <= r.fwd_dec;

    decode_comb: PROCESS(decode_i,decode_i.ctrl_wb,
                         decode_i.ctrl_mem_wb,
                         decode_i.ctrl_mem_wb.transfer_size,
                         r,r.ctrl_ex,r.ctrl_mem,
                         r.ctrl_mem.transfer_size,r.ctrl_wb,
                         r.fwd_dec,reg)

        VARIABLE v : decode_out_type;
        VARIABLE v_reg : decode_reg_type;
        VARIABLE opcode : std_ulogic_vector(5 DOWNTO 0);
        VARIABLE instruction : std_ulogic_vector(CFG_IMEM_WIDTH - 1 DOWNTO 0);
        VARIABLE program_counter : std_ulogic_vector(CFG_IMEM_SIZE - 1 DOWNTO 0);
        VARIABLE mem_result : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);

    BEGIN
        v := r;
        v_reg := reg;

        -- Default register values (NOP)
        v_reg.immediate := (OTHERS => '0');
        v_reg.is_immediate := '0';
        v_reg.program_counter := decode_i.program_counter;
        v_reg.instruction := decode_i.instruction;

        IF decode_i.ctrl_mem_wb.mem_read = '1' THEN
            mem_result := align_mem_load(decode_i.mem_result, decode_i.ctrl_mem_wb.transfer_size, decode_i.alu_result(1 DOWNTO 0));
        ELSE
            mem_result := decode_i.alu_result;
        END IF;

        wb_dat_d <= mem_result;

        IF G_INTERRUPT = true THEN
            v_reg.delay_interrupt := '0';
        END IF;

        IF CFG_REG_FWD_WB = true THEN
            v.fwd_dec_result    := mem_result;
            v.fwd_dec           := decode_i.ctrl_wb;
        ELSE
            v.fwd_dec_result    := (OTHERS => '0');
            v.fwd_dec.reg_d     := (OTHERS => '0');
            v.fwd_dec.reg_write := '0';
        END IF;

        IF (NOT decode_i.flush_id AND r.ctrl_mem.mem_read AND (compare(decode_i.instruction(20 DOWNTO 16), r.ctrl_wb.reg_d) OR compare(decode_i.instruction(15 DOWNTO 11), r.ctrl_wb.reg_d))) = '1' THEN
        -- A hazard occurred on register a or b

            -- set current instruction and program counter to 0
            instruction := (OTHERS => '0');
            program_counter := (OTHERS => '0');

            v.hazard := '1';

        ELSIF CFG_MEM_FWD_WB = false AND (NOT decode_i.flush_id AND r.ctrl_mem.mem_read AND compare(decode_i.instruction(25 DOWNTO 21), r.ctrl_wb.reg_d)) = '1' THEN
        -- A hazard occurred on register d

            -- set current instruction and program counter to 0
            instruction := (OTHERS => '0');
            program_counter := (OTHERS => '0');

            v.hazard := '1';

        ELSIF r.hazard = '1' THEN
        -- Recover from hazard. Insert latched instruction

            instruction := reg.instruction;
            program_counter := reg.program_counter;
            v.hazard := '0';

        ELSE

            instruction := decode_i.instruction;
            program_counter := decode_i.program_counter;
            v.hazard := '0';

        END IF;

        v.program_counter := program_counter;
        opcode := instruction(31 DOWNTO 26);
        v.ctrl_wb.reg_d := instruction(25 DOWNTO 21);
        v.reg_a := instruction(20 DOWNTO 16);
        v.reg_b := instruction(15 DOWNTO 11);

        -- SET IMM value
        IF reg.is_immediate = '1' THEN
            v.imm := reg.immediate & instruction(15 DOWNTO 0);
        ELSE
            v.imm := sign_extend(instruction(15 DOWNTO 0), instruction(15), 32);
        END IF;

        -- Register if an interrupt occurs
        IF G_INTERRUPT = true THEN
            IF v_reg.msr_interrupt_enable = '1' AND decode_i.interrupt = '1' THEN
                v_reg.interrupt := '1';
                v_reg.msr_interrupt_enable := '0';
            END IF;
        END IF;

        v.ctrl_ex.alu_op := ALU_ADD;
        v.ctrl_ex.alu_src_a := ALU_SRC_REGA;
        v.ctrl_ex.alu_src_b := ALU_SRC_REGB;
        v.ctrl_ex.operation := '0';
        v.ctrl_ex.carry := CARRY_ZERO;
        v.ctrl_ex.carry_keep := CARRY_NOT_KEEP;
        v.ctrl_ex.delay := '0';
        v.ctrl_ex.branch_cond := NOP;
        v.ctrl_mem.mem_write := '0';
        v.ctrl_mem.transfer_size := WORD;
        v.ctrl_mem.mem_read := '0';
        v.ctrl_wb.reg_write := '0';

        IF G_INTERRUPT = true AND (v_reg.interrupt = '1' AND reg.delay_interrupt = '0' AND decode_i.flush_id = '0' AND v.hazard = '0' AND r.ctrl_ex.delay = '0' AND reg.is_immediate = '0') THEN
        -- IF an interrupt occured
        --    AND the current instruction is not a branch or return instruction,
        --    AND the current instruction is not in a delay slot,
        --    AND this is instruction is not preceded by an IMM instruction, than handle the interrupt.
            v_reg.msr_interrupt_enable := '0';
            v_reg.interrupt := '0';

            v.reg_a := (OTHERS => '0');
            v.reg_b := (OTHERS => '0');

            v.imm   := X"00000010";
            v.ctrl_wb.reg_d := "01110";

            v.ctrl_ex.branch_cond := BNC;
            v.ctrl_ex.alu_src_a := ALU_SRC_ZERO;
            v.ctrl_ex.alu_src_b := ALU_SRC_IMM;
            v.ctrl_wb.reg_write := '1';

        ELSIF (decode_i.flush_id OR v.hazard) = '1' THEN
            -- clearing these registers is not necessary, but facilitates debugging.
            -- On the other hand performance improves when disabled.
            IF G_DEBUG = true THEN
                v.program_counter := (OTHERS => '0');
                v.ctrl_wb.reg_d   := (OTHERS => '0');
                v.reg_a           := (OTHERS => '0');
                v.reg_b           := (OTHERS => '0');
                v.imm             := (OTHERS => '0');
            END IF;

        ELSIF is_zero(opcode(5 DOWNTO 4)) = '1' THEN
        -- ADD, SUBTRACT OR COMPARE

            -- Alu operation
            v.ctrl_ex.alu_op := ALU_ADD;

            -- Source operand A
            IF opcode(0) = '1' THEN
                v.ctrl_ex.alu_src_a := ALU_SRC_NOT_REGA;
            ELSE
                v.ctrl_ex.alu_src_a := ALU_SRC_REGA;
            END IF;

            -- Source operand B
            IF opcode(3) = '1' THEN
                v.ctrl_ex.alu_src_b := ALU_SRC_IMM;
            ELSE
                v.ctrl_ex.alu_src_b := ALU_SRC_REGB;
            END IF;

            IF (compare(opcode, "000101") AND instruction(1)) = '1' THEN
                v.ctrl_ex.operation := '1';
            END IF;

            -- Carry
            CASE opcode(1 DOWNTO 0) IS
                WHEN "00" => v.ctrl_ex.carry := CARRY_ZERO;
                WHEN "01" => v.ctrl_ex.carry := CARRY_ONE;
                WHEN OTHERS => v.ctrl_ex.carry := CARRY_ALU;
            END CASE;

            -- Carry keep
            IF opcode(2) = '1' THEN
                v.ctrl_ex.carry_keep := CARRY_KEEP;
            ELSE
                v.ctrl_ex.carry_keep := CARRY_NOT_KEEP;
            END IF;

            -- Flag writeback if reg_d != 0
            v.ctrl_wb.reg_write := is_not_zero(v.ctrl_wb.reg_d);

        ELSIF (compare(opcode(5 DOWNTO 2), "1000") OR compare(opcode(5 DOWNTO 2), "1010")) = '1' THEN
        -- OR, AND, XOR, ANDN
        -- ORI, ANDI, XORI, ANDNI
            CASE opcode(1 DOWNTO 0) IS
                WHEN "00" => v.ctrl_ex.alu_op := ALU_OR;
                WHEN "10" => v.ctrl_ex.alu_op := ALU_XOR;
                WHEN OTHERS => v.ctrl_ex.alu_op := ALU_AND;
            END CASE;

            IF opcode(3) = '1' AND compare(opcode(1 DOWNTO 0), "11") = '1' THEN
                v.ctrl_ex.alu_src_b := ALU_SRC_NOT_IMM;
            ELSIF opcode(3) = '1' THEN
                v.ctrl_ex.alu_src_b := ALU_SRC_IMM;
            ELSIF opcode(3) = '0' AND compare(opcode(1 DOWNTO 0), "11") = '1' THEN
                v.ctrl_ex.alu_src_b := ALU_SRC_NOT_REGB;
            ELSE
                v.ctrl_ex.alu_src_b := ALU_SRC_REGB;
            END IF;

            -- Flag writeback if reg_d != 0
            v.ctrl_wb.reg_write := is_not_zero(v.ctrl_wb.reg_d);

        ELSIF compare(opcode, "101100") = '1' THEN
        -- IMM instruction
            v_reg.immediate := instruction(15 DOWNTO 0);
            v_reg.is_immediate := '1';

        ELSIF compare(opcode, "100100") = '1' THEN
        -- SHIFT, SIGN EXTEND
            IF compare(instruction(6 DOWNTO 5), "11") = '1' THEN
                IF instruction(0) = '1' THEN
                    v.ctrl_ex.alu_op:= ALU_SEXT16;
                ELSE
                    v.ctrl_ex.alu_op:= ALU_SEXT8;
                END IF;
            ELSE
                v.ctrl_ex.alu_op:= ALU_SHIFT;
                CASE instruction(6 DOWNTO 5) IS
                    WHEN "10"   => v.ctrl_ex.carry := CARRY_ZERO;
                    WHEN "01"   => v.ctrl_ex.carry := CARRY_ALU;
                    WHEN OTHERS => v.ctrl_ex.carry := CARRY_ARITH;
                END CASE;
            END IF;

            -- Flag writeback if reg_d != 0
            v.ctrl_wb.reg_write := is_not_zero(v.ctrl_wb.reg_d);

        ELSIF (compare(opcode, "100110") OR compare(opcode, "101110")) = '1' THEN
        -- BRANCH UNCONDITIONAL

            v.ctrl_ex.branch_cond := BNC;

            IF opcode(3) = '1' THEN
                v.ctrl_ex.alu_src_b := ALU_SRC_IMM;
            ELSE
                v.ctrl_ex.alu_src_b := ALU_SRC_REGB;
            END IF;

            -- WRITE THE RESULT ALSO TO REGISTER D
            IF v.reg_a(2) = '1' THEN
                -- Flag writeback if reg_d != 0
                v.ctrl_wb.reg_write := is_not_zero(v.ctrl_wb.reg_d);
            END IF;

            IF v.reg_a(3) = '1' THEN
                v.ctrl_ex.alu_src_a := ALU_SRC_ZERO;
            ELSE
                v.ctrl_ex.alu_src_a := ALU_SRC_PC;
            END IF;

            IF G_INTERRUPT = true THEN
                v_reg.delay_interrupt := '1';
            END IF;
            v.ctrl_ex.delay := v.reg_a(4);

        ELSIF (compare(opcode, "100111") OR compare(opcode, "101111")) = '1' THEN
        -- BRANCH CONDITIONAL
            v.ctrl_ex.alu_op := ALU_ADD;
            v.ctrl_ex.alu_src_a := ALU_SRC_PC;

            IF opcode(3) = '1' THEN
                v.ctrl_ex.alu_src_b := ALU_SRC_IMM;
            ELSE
                v.ctrl_ex.alu_src_b := ALU_SRC_REGB;
            END IF;

            CASE v.ctrl_wb.reg_d(2 DOWNTO 0) IS
                WHEN "000"  => v.ctrl_ex.branch_cond := BEQ;
                WHEN "001"  => v.ctrl_ex.branch_cond := BNE;
                WHEN "010"  => v.ctrl_ex.branch_cond := BLT;
                WHEN "011"  => v.ctrl_ex.branch_cond := BLE;
                WHEN "100"  => v.ctrl_ex.branch_cond := BGT;
                WHEN OTHERS => v.ctrl_ex.branch_cond := BGE;
            END CASE;

            IF G_INTERRUPT = true THEN
                v_reg.delay_interrupt := '1';
            END IF;
            v.ctrl_ex.delay := v.ctrl_wb.reg_d(4);

        ELSIF compare(opcode, "101101") = '1' THEN
        -- RETURN

            v.ctrl_ex.branch_cond := BNC;
            v.ctrl_ex.alu_src_b := ALU_SRC_IMM;
            v.ctrl_ex.delay := '1';

            IF G_INTERRUPT = true THEN
                IF v.ctrl_wb.reg_d(0) = '1' THEN
                    v_reg.msr_interrupt_enable := '1';
                END IF;
                v_reg.delay_interrupt := '1';
            END IF;

        ELSIF compare(opcode(5 DOWNTO 4), "11") = '1' THEN
            -- SW, LW
            v.ctrl_ex.alu_op := ALU_ADD;
            v.ctrl_ex.alu_src_a := ALU_SRC_REGA;

            IF opcode(3) = '1' THEN
                v.ctrl_ex.alu_src_b := ALU_SRC_IMM;
            ELSE
                v.ctrl_ex.alu_src_b := ALU_SRC_REGB;
            END IF;

            v.ctrl_ex.carry := CARRY_ZERO;

            IF opcode(2) = '1' THEN
                -- Store
                v.ctrl_mem.mem_write := '1';
                v.ctrl_mem.mem_read := '0';
                v.ctrl_wb.reg_write := '0';
            ELSE
                -- Load
                v.ctrl_mem.mem_write := '0';
                v.ctrl_mem.mem_read := '1';
                v.ctrl_wb.reg_write := is_not_zero(v.ctrl_wb.reg_d);
            END IF;

            CASE opcode(1 DOWNTO 0) IS
                WHEN "00" => v.ctrl_mem.transfer_size := BYTE;
                WHEN "01" => v.ctrl_mem.transfer_size := HALFWORD;
                WHEN OTHERS => v.ctrl_mem.transfer_size := WORD;
            END CASE;

            v.ctrl_ex.delay := '0';

        ELSIF G_USE_HW_MUL = true AND (compare(opcode, "010000") OR compare(opcode, "011000")) = '1' THEN

            v.ctrl_ex.alu_op := ALU_MUL;

            IF opcode(3) = '1' THEN
                v.ctrl_ex.alu_src_b := ALU_SRC_IMM;
            ELSE
                v.ctrl_ex.alu_src_b := ALU_SRC_REGB;
            END IF;

            v.ctrl_wb.reg_write := is_not_zero(v.ctrl_wb.reg_d);

        ELSIF G_USE_BARREL = true AND (compare(opcode, "010001") OR compare(opcode, "011001")) = '1' THEN

            v.ctrl_ex.alu_op := ALU_BS;

            IF opcode(3) = '1' THEN
                v.ctrl_ex.alu_src_b := ALU_SRC_IMM;
            ELSE
                v.ctrl_ex.alu_src_b := ALU_SRC_REGB;
            END IF;

            v.ctrl_wb.reg_write := is_not_zero(v.ctrl_wb.reg_d);

        ELSE
            -- UNKNOWN OPCODE
            NULL;
        END IF;

        rin <= v;
        regin <= v_reg;

    END PROCESS;

    decode_seq: PROCESS(clk_i)
        PROCEDURE proc_reset_decode IS
        BEGIN
            r.reg_a                     <= (OTHERS => '0');
            r.reg_b                     <= (OTHERS => '0');
            r.imm                       <= (OTHERS => '0');
            r.program_counter           <= (OTHERS => '0');
            r.hazard                    <= '0';
            r.ctrl_ex.alu_op            <= ALU_ADD;
            r.ctrl_ex.alu_src_a         <= ALU_SRC_REGA;
            r.ctrl_ex.alu_src_b         <= ALU_SRC_REGB;
            r.ctrl_ex.operation         <= '0';
            r.ctrl_ex.carry             <= CARRY_ZERO;
            r.ctrl_ex.carry_keep        <= CARRY_NOT_KEEP;
            r.ctrl_ex.delay             <= '0';
            r.ctrl_ex.branch_cond       <= NOP;
            r.ctrl_mem.mem_write        <= '0';
            r.ctrl_mem.transfer_size    <= WORD;
            r.ctrl_mem.mem_read         <= '0';
            r.ctrl_wb.reg_d             <= (OTHERS => '0');
            r.ctrl_wb.reg_write         <= '0';
            r.fwd_dec_result            <= (OTHERS => '0');
            r.fwd_dec.reg_d             <= (OTHERS => '0');
            r.fwd_dec.reg_write         <= '0';
            reg.instruction             <= (OTHERS => '0');
            reg.program_counter         <= (OTHERS => '0');
            reg.immediate               <= (OTHERS => '0');
            reg.is_immediate            <= '0';
            reg.msr_interrupt_enable    <= '1';
            reg.interrupt               <= '0';
            reg.delay_interrupt         <= '0';
        END PROCEDURE proc_reset_decode;
    BEGIN
        IF rising_edge(clk_i) THEN
            IF rst_i = '1' THEN
                proc_reset_decode;
            ELSIF ena_i = '1' THEN
                r <= rin;
                reg <= regin;
            END IF;
        END IF;
    END PROCESS;

    gprf0 : gprf PORT MAP
    (
        gprf_o         => gprf_o,
        gprf_i.adr_a_i => rin.reg_a,
        gprf_i.adr_b_i => rin.reg_b,
        gprf_i.adr_d_i => rin.ctrl_wb.reg_d,
        gprf_i.dat_w_i => wb_dat_d,
        gprf_i.adr_w_i => decode_i.ctrl_wb.reg_d,
        gprf_i.wre_i   => decode_i.ctrl_wb.reg_write,
        ena_i          => ena_i,
        clk_i          => clk_i
    );
END arch;
