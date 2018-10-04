----------------------------------------------------------------------------------------------
--
--      Input file         : core_Pkg.vhd
--      Design name        : core_Pkg
--      Author             : Tamar Kranenburg
--      Company            : Delft University of Technology
--                         : Faculty EEMCS, Department ME&CE
--                         : Systems and Circuits group
--
--      Description        : Package with components and type definitions for the interface
--                           of the components
--
--
----------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.ALL;

--LIBRARY mblite;
USE work.config_Pkg.ALL;
USE work.std_Pkg.ALL;

PACKAGE core_Pkg IS

----------------------------------------------------------------------------------------------
-- TYPES USED IN MB-LITE
----------------------------------------------------------------------------------------------

    TYPE alu_operation     IS (ALU_ADD, ALU_OR, ALU_AND, ALU_XOR, ALU_SHIFT, ALU_SEXT8, ALU_SEXT16, ALU_MUL, ALU_BS);
    TYPE src_type_a        IS (ALU_SRC_REGA, ALU_SRC_NOT_REGA, ALU_SRC_PC, ALU_SRC_ZERO);
    TYPE src_type_b        IS (ALU_SRC_REGB, ALU_SRC_NOT_REGB, ALU_SRC_IMM, ALU_SRC_NOT_IMM);
    TYPE carry_type        IS (CARRY_ZERO, CARRY_ONE, CARRY_ALU, CARRY_ARITH);
    TYPE carry_keep_type   IS (CARRY_NOT_KEEP, CARRY_KEEP);
    TYPE branch_condition  IS (NOP, BNC, BEQ, BNE, BLT, BLE, BGT, BGE);
    TYPE transfer_size     IS (WORD, HALFWORD, BYTE);

    TYPE ctrl_execution IS RECORD
        alu_op      : alu_operation;
        alu_src_a   : src_type_a;
        alu_src_b   : src_type_b;
        operation   : std_ulogic;
        carry       : carry_type;
        carry_keep  : carry_keep_type;
        branch_cond : branch_condition;
        delay       : std_ulogic;
    END RECORD;

    TYPE ctrl_memory IS RECORD
        mem_write     : std_ulogic;
        mem_read      : std_ulogic;
        transfer_size : transfer_size;
    END RECORD;

    TYPE ctrl_memory_writeback_type IS RECORD
        mem_read      : std_ulogic;
        transfer_size : transfer_size;
    END RECORD;

    TYPE forward_type IS RECORD
        reg_d     : std_ulogic_vector(CFG_GPRF_SIZE - 1 DOWNTO 0);
        reg_write : std_ulogic;
    END RECORD;

    TYPE imem_in_type IS RECORD
        dat_i : std_ulogic_vector(CFG_IMEM_WIDTH - 1 DOWNTO 0);
    END RECORD;

    TYPE imem_out_type IS RECORD
        adr_o : std_ulogic_vector(CFG_IMEM_SIZE - 1 DOWNTO 0);
        ena_o : std_ulogic;
    END RECORD;

    TYPE fetch_in_type IS RECORD
        hazard : std_ulogic;
        branch : std_ulogic;
        branch_target : std_ulogic_vector(CFG_IMEM_SIZE - 1 DOWNTO 0);
    END RECORD;

    TYPE fetch_out_type IS RECORD
        program_counter : std_ulogic_vector(CFG_IMEM_SIZE - 1 DOWNTO 0);
    END RECORD;

    TYPE gprf_out_type IS RECORD
        dat_a_o : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        dat_b_o : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        dat_d_o : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
    END RECORD;

    TYPE decode_in_type IS RECORD
        program_counter : std_ulogic_vector(CFG_IMEM_SIZE - 1 DOWNTO 0);
        instruction     : std_ulogic_vector(CFG_IMEM_WIDTH - 1 DOWNTO 0);
        ctrl_wb         : forward_type;
        ctrl_mem_wb     : ctrl_memory_writeback_type;
        mem_result      : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        alu_result      : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        interrupt       : std_ulogic;
        flush_id        : std_ulogic;
    END RECORD;

    TYPE decode_out_type IS RECORD
        reg_a           : std_ulogic_vector(CFG_GPRF_SIZE - 1 DOWNTO 0);
        reg_b           : std_ulogic_vector(CFG_GPRF_SIZE - 1 DOWNTO 0);
        imm             : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        program_counter : std_ulogic_vector(CFG_IMEM_SIZE - 1 DOWNTO 0);
        hazard          : std_ulogic;
        ctrl_ex         : ctrl_execution;
        ctrl_mem        : ctrl_memory;
        ctrl_wb         : forward_type;
        fwd_dec_result  : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        fwd_dec         : forward_type;
    END RECORD;

    TYPE gprf_in_type IS RECORD
        adr_a_i : std_ulogic_vector(CFG_GPRF_SIZE - 1 DOWNTO 0);
        adr_b_i : std_ulogic_vector(CFG_GPRF_SIZE - 1 DOWNTO 0);
        adr_d_i : std_ulogic_vector(CFG_GPRF_SIZE - 1 DOWNTO 0);
        dat_w_i : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        adr_w_i : std_ulogic_vector(CFG_GPRF_SIZE - 1 DOWNTO 0);
        wre_i   : std_ulogic;
    END RECORD;

    TYPE execute_out_type IS RECORD
        alu_result      : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        dat_d           : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        branch          : std_ulogic;
        program_counter : std_ulogic_vector(CFG_IMEM_SIZE - 1 DOWNTO 0);
        flush_id        : std_ulogic;
        ctrl_mem        : ctrl_memory;
        ctrl_wb         : forward_type;
    END RECORD;

    TYPE execute_in_type IS RECORD
        reg_a           : std_ulogic_vector(CFG_GPRF_SIZE  - 1 DOWNTO 0);
        dat_a           : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        reg_b           : std_ulogic_vector(CFG_GPRF_SIZE  - 1 DOWNTO 0);
        dat_b           : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        dat_d           : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        imm             : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        program_counter : std_ulogic_vector(CFG_IMEM_SIZE - 1 DOWNTO 0);
        fwd_dec         : forward_type;
        fwd_dec_result  : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        fwd_mem         : forward_type;
        ctrl_ex         : ctrl_execution;
        ctrl_mem        : ctrl_memory;
        ctrl_wb         : forward_type;
        ctrl_mem_wb     : ctrl_memory_writeback_type;
        mem_result      : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        alu_result      : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);

    END RECORD;

    TYPE mem_in_type IS RECORD
        dat_d           : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        alu_result      : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        mem_result      : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        program_counter : std_ulogic_vector(CFG_IMEM_SIZE - 1 DOWNTO 0);
        branch          : std_ulogic;
        ctrl_mem        : ctrl_memory;
        ctrl_wb         : forward_type;
    END RECORD;

    TYPE mem_out_type IS RECORD
        alu_result  : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        ctrl_wb     : forward_type;
        ctrl_mem_wb : ctrl_memory_writeback_type;
    END RECORD;

    TYPE dmem_in_type IS RECORD
        dat_i : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        ena_i : std_ulogic;
    END RECORD;

    TYPE dmem_out_type IS RECORD
        dat_o : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
        adr_o : std_ulogic_vector(CFG_DMEM_SIZE - 1 DOWNTO 0);
        sel_o : std_ulogic_vector(3 DOWNTO 0);
        we_o  : std_ulogic;
        ena_o : std_ulogic;
    END RECORD;

    TYPE dmem_in_array_type IS ARRAY(NATURAL RANGE <>) OF dmem_in_type;
    TYPE dmem_out_array_type IS ARRAY(NATURAL RANGE <>) OF dmem_out_type;

    -- WB-master inputs from the wb-slaves
    TYPE wb_mst_in_type IS RECORD
        clk_i : std_ulogic;                      -- master clock input
        rst_i : std_ulogic;                      -- synchronous active high reset
        dat_i : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0); -- databus input
        ack_i : std_ulogic;                      -- buscycle acknowledge input
        int_i : std_ulogic;                      -- interrupt request input
    END RECORD;

    -- WB-master outputs to the wb-slaves
    TYPE wb_mst_out_type IS RECORD
        adr_o : std_ulogic_vector(CFG_DMEM_SIZE - 1 DOWNTO 0);  -- address bits
        dat_o : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0); -- databus output
        we_o  : std_ulogic;                      -- write enable output
        stb_o : std_ulogic;                      -- strobe signals
        sel_o : std_ulogic_vector(3 DOWNTO 0);   -- select output array
        cyc_o : std_ulogic;                      -- valid BUS cycle output
    END RECORD;

    -- WB-slave inputs, from the WB-master
    TYPE wb_slv_in_type IS RECORD
        clk_i : std_ulogic;                     -- master clock input
        rst_i : std_ulogic;                     -- synchronous active high reset
        adr_i : std_ulogic_vector(CFG_DMEM_SIZE - 1 DOWNTO 0); -- address bits
        dat_i : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0); -- Databus input
        we_i  : std_ulogic;                     -- Write enable input
        stb_i : std_ulogic;                     -- strobe signals / core select signal
        sel_i : std_ulogic_vector(3 DOWNTO 0);   -- select output array
        cyc_i : std_ulogic;                     -- valid BUS cycle input
    END RECORD;

    -- WB-slave outputs to the WB-master
    TYPE wb_slv_out_type IS RECORD
        dat_o : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0); -- Databus output
        ack_o : std_ulogic;                     -- Bus cycle acknowledge output
        int_o : std_ulogic;                     -- interrupt request output
    END RECORD;

----------------------------------------------------------------------------------------------
-- COMPONENTS USED IN MB-LITE
----------------------------------------------------------------------------------------------

    COMPONENT core GENERIC
    (
        G_INTERRUPT  : boolean := CFG_INTERRUPT;
        G_USE_HW_MUL : boolean := CFG_USE_HW_MUL;
        G_USE_BARREL : boolean := CFG_USE_BARREL;
        G_DEBUG      : boolean := CFG_DEBUG
    );
    PORT
    (
        imem_o : OUT imem_out_type;
        dmem_o : OUT dmem_out_type;
        imem_i : IN imem_in_type;
        dmem_i : IN dmem_in_type;
        int_i  : IN std_ulogic;
        rst_i  : IN std_ulogic;
        clk_i  : IN std_ulogic
    );
    END COMPONENT;

    COMPONENT core_wb GENERIC
    (
        G_INTERRUPT  : boolean := CFG_INTERRUPT;
        G_USE_HW_MUL : boolean := CFG_USE_HW_MUL;
        G_USE_BARREL : boolean := CFG_USE_BARREL;
        G_DEBUG      : boolean := CFG_DEBUG
    );
    PORT
    (
        imem_o : OUT imem_out_type;
        wb_o   : OUT wb_mst_out_type;
        imem_i : IN imem_in_type;
        wb_i   : IN wb_mst_in_type
    );
    END COMPONENT;

    COMPONENT core_wb_adapter PORT
    (
        dmem_i : OUT dmem_in_type;
        wb_o   : OUT wb_mst_out_type;
        dmem_o : IN dmem_out_type;
        wb_i   : IN wb_mst_in_type
    );
    END COMPONENT;

    COMPONENT core_wb_async_adapter PORT
    (
        dmem_i : OUT dmem_in_type;
        wb_o   : OUT wb_mst_out_type;
        dmem_o : IN dmem_out_type;
        wb_i   : IN wb_mst_in_type
    );
    END COMPONENT;

    COMPONENT fetch PORT
    (
        fetch_o : OUT fetch_out_type;
        imem_o  : OUT imem_out_type;
        fetch_i : IN fetch_in_type;
        rst_i   : IN std_ulogic;
        ena_i   : IN std_ulogic;
        clk_i   : IN std_ulogic
    );
    END COMPONENT;

    COMPONENT decode GENERIC
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
    END COMPONENT;

    COMPONENT gprf PORT
    (
        gprf_o : OUT gprf_out_type;
        gprf_i : IN gprf_in_type;
        ena_i  : IN std_ulogic;
        clk_i  : IN std_ulogic
    );
    END COMPONENT;

    COMPONENT execute GENERIC
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
    END COMPONENT;

    COMPONENT mem PORT
    (
        mem_o  : OUT mem_out_type;
        dmem_o : OUT dmem_out_type;
        mem_i  : IN mem_in_type;
        ena_i  : IN std_ulogic;
        rst_i  : IN std_ulogic;
        clk_i  : IN std_ulogic
    );
    END COMPONENT;

    COMPONENT core_address_decoder GENERIC
    (
        G_NUM_SLAVES : positive := CFG_NUM_SLAVES
    );
    PORT
    (
        m_dmem_i : OUT dmem_in_type;
        s_dmem_o : OUT dmem_out_array_type;
        m_dmem_o : IN dmem_out_type;
        s_dmem_i : IN dmem_in_array_type;
        clk_i    : IN std_ulogic
    );
    END COMPONENT;
----------------------------------------------------------------------------------------------
-- FUNCTIONS USED IN MB-LITE
----------------------------------------------------------------------------------------------

    FUNCTION select_register_data(reg_dat, reg, wb_dat : std_ulogic_vector; write : std_ulogic) RETURN std_ulogic_vector;
    FUNCTION forward_condition(reg_write : std_ulogic; reg_a, reg_d : std_ulogic_vector) RETURN std_ulogic;
    FUNCTION align_mem_load(data : std_ulogic_vector; size : transfer_size; address : std_ulogic_vector) RETURN std_ulogic_vector;
    FUNCTION align_mem_store(data : std_ulogic_vector; size : transfer_size) RETURN std_ulogic_vector;
    FUNCTION decode_mem_store(address : std_ulogic_vector(1 DOWNTO 0); size : transfer_size) RETURN std_ulogic_vector;

END core_Pkg;

PACKAGE BODY core_Pkg IS

    -- This function select the register value:
    --      A) zero
    --      B) bypass value read from register file
    --      C) value from register file
    FUNCTION select_register_data(reg_dat, reg, wb_dat : std_ulogic_vector; write : std_ulogic) RETURN std_ulogic_vector IS
        VARIABLE tmp : std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
    BEGIN
        IF CFG_REG_FORCE_ZERO = true AND is_zero(reg) = '1' THEN
            tmp := (OTHERS => '0');
        ELSIF CFG_REG_FWD_WB = true AND write = '1' THEN
            tmp := wb_dat;
        ELSE
            tmp := reg_dat;
        END IF;
        RETURN tmp;
    END select_register_data;

    -- This function checks if a forwarding condition is met. The condition is met of register A and D match
    -- and the signal needs to be written back to the register file.
    FUNCTION forward_condition(reg_write : std_ulogic; reg_a, reg_d : std_ulogic_vector ) RETURN std_ulogic IS
    BEGIN
        RETURN reg_write AND compare(reg_a, reg_d);
    END forward_condition;

    -- This function aligns the memory load operation. The load byte-order is defined here.
    FUNCTION align_mem_load(data : std_ulogic_vector; size : transfer_size; address : std_ulogic_vector ) RETURN std_ulogic_vector IS
    BEGIN
        IF CFG_BYTE_ORDER = false THEN
            -- Little endian decoding
            CASE size IS
                WHEN byte => 
                    CASE address(1 DOWNTO 0) IS
                        WHEN "00"   => RETURN "000000000000000000000000" & data(CFG_DMEM_WIDTH/4 - 1 DOWNTO 0);
                        WHEN "01"   => RETURN "000000000000000000000000" & data(CFG_DMEM_WIDTH/2 - 1 DOWNTO CFG_DMEM_WIDTH/4);
                        WHEN "10"   => RETURN "000000000000000000000000" & data(3*CFG_DMEM_WIDTH/4 - 1 DOWNTO CFG_DMEM_WIDTH/2);
                        WHEN "11"   => RETURN "000000000000000000000000" & data(CFG_DMEM_WIDTH - 1 DOWNTO 3*CFG_DMEM_WIDTH/4);
                        WHEN OTHERS => RETURN "00000000000000000000000000000000";
                    END CASE;
                WHEN halfword => 
                    CASE address(1 DOWNTO 0) IS
                        WHEN "00"   => RETURN "0000000000000000" & data(CFG_DMEM_WIDTH/2 - 1 DOWNTO 0);
                        WHEN "10"   => RETURN "0000000000000000" & data(CFG_DMEM_WIDTH - 1 DOWNTO CFG_DMEM_WIDTH/2);
                        WHEN OTHERS => RETURN "00000000000000000000000000000000";
                    END CASE;
                WHEN OTHERS =>
                    RETURN data;
            END CASE;
        ELSE
            -- Big endian decoding
            CASE size IS
                WHEN byte => 
                    CASE address(1 DOWNTO 0) IS
                        WHEN "00"   => RETURN "000000000000000000000000" & data(CFG_DMEM_WIDTH - 1 DOWNTO 3*CFG_DMEM_WIDTH/4);
                        WHEN "01"   => RETURN "000000000000000000000000" & data(3*CFG_DMEM_WIDTH/4 - 1 DOWNTO CFG_DMEM_WIDTH/2);
                        WHEN "10"   => RETURN "000000000000000000000000" & data(CFG_DMEM_WIDTH/2 - 1 DOWNTO CFG_DMEM_WIDTH/4);
                        WHEN "11"   => RETURN "000000000000000000000000" & data(CFG_DMEM_WIDTH/4 - 1 DOWNTO 0);
                        WHEN OTHERS => RETURN "00000000000000000000000000000000";
                    END CASE;
                WHEN halfword => 
                    CASE address(1 DOWNTO 0) IS
                        WHEN "00"   => RETURN "0000000000000000" & data(CFG_DMEM_WIDTH - 1 DOWNTO CFG_DMEM_WIDTH/2);
                        WHEN "10"   => RETURN "0000000000000000" & data(CFG_DMEM_WIDTH/2 - 1 DOWNTO 0);
                        WHEN OTHERS => RETURN "00000000000000000000000000000000";
                    END CASE;
                WHEN OTHERS =>
                    RETURN data;
            END CASE;
        END IF;
    END align_mem_load;

    -- This function repeats the operand to all positions memory store operation.
    FUNCTION align_mem_store(data : std_ulogic_vector; size : transfer_size) RETURN std_ulogic_vector IS
    BEGIN
        CASE size IS
            WHEN byte     => RETURN data( 7 DOWNTO 0) & data( 7 DOWNTO 0) & data(7 DOWNTO 0) & data(7 DOWNTO 0);
            WHEN halfword => RETURN data(15 DOWNTO 0) & data(15 DOWNTO 0);
            WHEN OTHERS   => RETURN data;
        END CASE;
    END align_mem_store;

    -- This function selects the correct bytes for memory writes. The store byte-order (MSB / LSB) can be defined here.
    FUNCTION decode_mem_store(address : std_ulogic_vector(1 DOWNTO 0); size : transfer_size) RETURN std_ulogic_vector IS
    BEGIN
        IF CFG_BYTE_ORDER = false THEN
            -- Little endian encoding
            CASE size IS
                WHEN BYTE =>
                    CASE address IS
                        WHEN "00"   => RETURN "0001";
                        WHEN "01"   => RETURN "0010";
                        WHEN "10"   => RETURN "0100";
                        WHEN "11"   => RETURN "1000";
                        WHEN OTHERS => RETURN "0000";
                    END CASE;
                WHEN HALFWORD =>
                    CASE address IS
                        WHEN "00"   => RETURN "0011";
                        WHEN "10"   => RETURN "1100";
                        WHEN OTHERS => RETURN "0000";
                    END CASE;
                WHEN OTHERS =>
                    RETURN "1111";
            END CASE;
        ELSE
            -- Big endian encoding
            CASE size IS
                WHEN BYTE =>
                    CASE address IS
                        WHEN "00"   => RETURN "1000";
                        WHEN "01"   => RETURN "0100";
                        WHEN "10"   => RETURN "0010";
                        WHEN "11"   => RETURN "0001";
                        WHEN OTHERS => RETURN "0000";
                    END CASE;
                WHEN HALFWORD =>
                    CASE address IS
                        -- Big endian encoding
                        WHEN "10"   => RETURN "0011";
                        WHEN "00"   => RETURN "1100";
                        WHEN OTHERS => RETURN "0000";
                    END CASE;
                WHEN OTHERS =>
                    RETURN "1111";
            END CASE;
        END IF;
    END decode_mem_store;

END core_Pkg;
