---------------------------------------------------------------------
-- TITLE: Plasma Misc. Package
-- AUTHOR: Steve Rhoads (rhoadss@yahoo.com)
-- DATE CREATED: 2/15/01
-- FILENAME: mlite_pack.vhd
-- PROJECT: Plasma CPU core
-- COPYRIGHT: Software placed into the public domain by the author.
--    Software 'as is' without warranty.  Author liable for nothing.
-- DESCRIPTION:
--    Data types, constants, and add functions needed for the Plasma CPU.
---------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use work.HeMPS_PKG.all;

package mlite_pack is
   constant ZERO          : std_logic_vector(31 downto 0) :=
      "00000000000000000000000000000000";
   constant ONES          : std_logic_vector(31 downto 0) :=
      "11111111111111111111111111111111";
   --make HIGH_Z equal to ZERO if compiler complains
   constant HIGH_Z        : std_logic_vector(31 downto 0) :=
      "ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ";
  
   subtype alu_function_type is std_logic_vector(3 downto 0);
   constant ALU_NOTHING   : alu_function_type := "0000";
   constant ALU_ADD       : alu_function_type := "0001";
   constant ALU_SUBTRACT  : alu_function_type := "0010";
   constant ALU_LESS_THAN : alu_function_type := "0011";
   constant ALU_LESS_THAN_SIGNED : alu_function_type := "0100";
   constant ALU_OR        : alu_function_type := "0101";
   constant ALU_AND       : alu_function_type := "0110";
   constant ALU_XOR       : alu_function_type := "0111";
   constant ALU_NOR       : alu_function_type := "1000";

   subtype shift_function_type is std_logic_vector(1 downto 0);
   constant SHIFT_NOTHING        : shift_function_type := "00";
   constant SHIFT_LEFT_UNSIGNED  : shift_function_type := "01";
   constant SHIFT_RIGHT_SIGNED   : shift_function_type := "11";
   constant SHIFT_RIGHT_UNSIGNED : shift_function_type := "10";

   subtype mult_function_type is std_logic_vector(3 downto 0);
   constant MULT_NOTHING       : mult_function_type := "0000";
   constant MULT_READ_LO       : mult_function_type := "0001";
   constant MULT_READ_HI       : mult_function_type := "0010";
   constant MULT_WRITE_LO      : mult_function_type := "0011";
   constant MULT_WRITE_HI      : mult_function_type := "0100";
   constant MULT_MULT          : mult_function_type := "0101";
   constant MULT_SIGNED_MULT   : mult_function_type := "0110";
   constant MULT_DIVIDE        : mult_function_type := "0111";
   constant MULT_SIGNED_DIVIDE : mult_function_type := "1000";

   subtype a_source_type is std_logic_vector(1 downto 0);
   constant A_FROM_REG_SOURCE : a_source_type := "00";
   constant A_FROM_IMM10_6    : a_source_type := "01";
   constant A_FROM_PC         : a_source_type := "10";

   subtype b_source_type is std_logic_vector(1 downto 0);
   constant B_FROM_REG_TARGET : b_source_type := "00";
   constant B_FROM_IMM        : b_source_type := "01";
   constant B_FROM_SIGNED_IMM : b_source_type := "10";
   constant B_FROM_IMMX4      : b_source_type := "11";

   subtype c_source_type is std_logic_vector(2 downto 0);
   constant C_FROM_NULL       : c_source_type := "000";
   constant C_FROM_ALU        : c_source_type := "001";
   constant C_FROM_SHIFT      : c_source_type := "001"; --same as alu
   constant C_FROM_MULT       : c_source_type := "001"; --same as alu
   constant C_FROM_MEMORY     : c_source_type := "010";
   constant C_FROM_PC         : c_source_type := "011";
   constant C_FROM_PC_PLUS4   : c_source_type := "100";
   constant C_FROM_IMM_SHIFT16: c_source_type := "101";
   constant C_FROM_REG_SOURCEN: c_source_type := "110";
   constant C_FROM_PC_PAST    : c_source_type := "111"; -- fixes interrupting jumps

   subtype pc_source_type is std_logic_vector(1 downto 0);
   constant FROM_INC4       : pc_source_type := "00";
   constant FROM_OPCODE25_0 : pc_source_type := "01";
   constant FROM_BRANCH     : pc_source_type := "10";
   constant FROM_LBRANCH    : pc_source_type := "11";

   subtype branch_function_type is std_logic_vector(2 downto 0);
   constant BRANCH_LTZ : branch_function_type := "000";
   constant BRANCH_LEZ : branch_function_type := "001";
   constant BRANCH_EQ  : branch_function_type := "010";
   constant BRANCH_NE  : branch_function_type := "011";
   constant BRANCH_GEZ : branch_function_type := "100";
   constant BRANCH_GTZ : branch_function_type := "101";
   constant BRANCH_YES : branch_function_type := "110";
   constant BRANCH_NO  : branch_function_type := "111";

   -- mode(32=1,16=2,8=3), signed, write
   subtype mem_source_type is std_logic_vector(3 downto 0);
   constant MEM_FETCH   : mem_source_type := "0000";
   constant MEM_READ32  : mem_source_type := "0100";
   constant MEM_WRITE32 : mem_source_type := "0101";
   constant MEM_READ16  : mem_source_type := "1000";
   constant MEM_READ16S : mem_source_type := "1010";
   constant MEM_WRITE16 : mem_source_type := "1001";
   constant MEM_READ8   : mem_source_type := "1100";
   constant MEM_READ8S  : mem_source_type := "1110";
   constant MEM_WRITE8  : mem_source_type := "1101";

   function bv_adder(a     : in std_logic_vector;
                     b     : in std_logic_vector;
                     do_add: in std_logic) return std_logic_vector;
   function bv_negate(a : in std_logic_vector) return std_logic_vector;
   function bv_increment(a : in std_logic_vector(31 downto 2)
                         ) return std_logic_vector;
   function bv_inc(a : in std_logic_vector
                  ) return std_logic_vector;
     
   component pc_next
      port(clk         : in std_logic;
           reset_in    : in std_logic;
           pc_new      : in std_logic_vector(31 downto 2);
           take_branch : in std_logic;
           pause_in    : in std_logic;
           opcode25_0  : in std_logic_vector(25 downto 0);
           pc_source   : in pc_source_type;
           pc_future   : out std_logic_vector(31 downto 2);
           pc_current  : out std_logic_vector(31 downto 2);
         pc_past     : out std_logic_vector(31 downto 2);
           pc_plus4    : out std_logic_vector(31 downto 2));
   end component;

   component mem_ctrl
      port(clk          : in std_logic;
           reset_in     : in std_logic;
           pause_in     : in std_logic;
           nullify_op   : in std_logic;
           address_pc   : in std_logic_vector(31 downto 2);
           opcode_out   : out std_logic_vector(31 downto 0);

           address_in   : in std_logic_vector(31 downto 0);
           mem_source   : in mem_source_type;
           data_write   : in std_logic_vector(31 downto 0);
           data_read    : out std_logic_vector(31 downto 0);
           pause_out    : out std_logic;
        
           mem_address  : out std_logic_vector(31 downto 2);
           mem_data_w   : out std_logic_vector(31 downto 0);
           mem_data_r   : in std_logic_vector(31 downto 0);
           mem_byte_we  : out std_logic_vector(3 downto 0));
   end component;

   component control 
      port(opcode       : in  std_logic_vector(31 downto 0);
           intr_signal  : in  std_logic;
           rs_index     : out std_logic_vector(5 downto 0);
           rt_index     : out std_logic_vector(5 downto 0);
           rd_index     : out std_logic_vector(5 downto 0);
           imm_out      : out std_logic_vector(15 downto 0);
           alu_func     : out alu_function_type;
           shift_func   : out shift_function_type;
           mult_func    : out mult_function_type;
           branch_func  : out branch_function_type;
           a_source_out : out a_source_type;
           b_source_out : out b_source_type;
           c_source_out : out c_source_type;
           pc_source_out: out pc_source_type;
           mem_source_out:out mem_source_type);
   end component;

   component reg_bank
      generic(memory_type : string := "XIL");
      port(clk            : in  std_logic;
           reset_in       : in  std_logic;
           pause          : in  std_logic;
           rs_index       : in  std_logic_vector(5 downto 0);
           rt_index       : in  std_logic_vector(5 downto 0);
           rd_index       : in  std_logic_vector(5 downto 0);
           reg_source_out : out std_logic_vector(31 downto 0);
           reg_target_out : out std_logic_vector(31 downto 0);
           reg_dest_new   : in  std_logic_vector(31 downto 0);
         page           : out std_logic_vector(31 downto 0); -- CRIS
           intr_enable    : out std_logic);
   end component;

   component bus_mux 
      port(imm_in       : in  std_logic_vector(15 downto 0);
           reg_source   : in  std_logic_vector(31 downto 0);
           a_mux        : in  a_source_type;
           a_out        : out std_logic_vector(31 downto 0);

           reg_target   : in  std_logic_vector(31 downto 0);
           b_mux        : in  b_source_type;
           b_out        : out std_logic_vector(31 downto 0);

           c_bus        : in  std_logic_vector(31 downto 0);
           c_memory     : in  std_logic_vector(31 downto 0);
           c_pc         : in  std_logic_vector(31 downto 2);
           c_pc_past    : in  std_logic_vector(31 downto 2);
           c_pc_plus4   : in  std_logic_vector(31 downto 2);
           c_mux        : in  c_source_type;
           reg_dest_out : out std_logic_vector(31 downto 0);

           branch_func  : in  branch_function_type;
           take_branch  : out std_logic);
   end component;

   component alu
      generic(alu_type  : string := "DEF");
      port(a_in         : in  std_logic_vector(31 downto 0);
           b_in         : in  std_logic_vector(31 downto 0);
           alu_function : in  alu_function_type;
           c_alu        : out std_logic_vector(31 downto 0));
   end component;

   component shifter
      generic(shifter_type : string := "DEF" );
      port(value        : in  std_logic_vector(31 downto 0);
           shift_amount : in  std_logic_vector(4 downto 0);
           shift_func   : in  shift_function_type;
           c_shift      : out std_logic_vector(31 downto 0));
   end component;

   component mult
      generic(mult_type  : string := "DEF"); 
      port(clk       : in  std_logic;
           reset_in  : in  std_logic;
           a, b      : in  std_logic_vector(31 downto 0);
           mult_func : in  mult_function_type;
           c_mult    : out std_logic_vector(31 downto 0);
           pause_out : out std_logic); 
   end component;

   component pipeline
      port(clk            : in  std_logic;
           reset          : in  std_logic;
           a_bus          : in  std_logic_vector(31 downto 0);
           a_busD         : out std_logic_vector(31 downto 0);
           b_bus          : in  std_logic_vector(31 downto 0);
           b_busD         : out std_logic_vector(31 downto 0);
           alu_func       : in  alu_function_type;
           alu_funcD      : out alu_function_type;
           shift_func     : in  shift_function_type;
           shift_funcD    : out shift_function_type;
           mult_func      : in  mult_function_type;
           mult_funcD     : out mult_function_type;
           reg_dest       : in  std_logic_vector(31 downto 0);
           reg_destD      : out std_logic_vector(31 downto 0);
           rd_index       : in  std_logic_vector(5 downto 0);
           rd_indexD      : out std_logic_vector(5 downto 0);

           rs_index       : in  std_logic_vector(5 downto 0);
           rt_index       : in  std_logic_vector(5 downto 0);
           pc_source      : in  pc_source_type;
           mem_source     : in  mem_source_type;
           a_source       : in  a_source_type;
           b_source       : in  b_source_type;
           c_source       : in  c_source_type;
           c_bus          : in  std_logic_vector(31 downto 0);
           pause_any      : in  std_logic;
           pause_pipeline : out std_logic);
   end component;

   

end; --package mlite_pack

package body mlite_pack is

function bv_adder(a     : in std_logic_vector;
                  b     : in std_logic_vector;
                  do_add: in std_logic) return std_logic_vector is
   variable carry_in : std_logic;
   variable bb       : std_logic_vector(a'length-1 downto 0);
   variable result   : std_logic_vector(a'length downto 0);
begin
   if do_add = '1' then
      bb := b;
      carry_in := '0';
   else
      bb := not b;
      carry_in := '1';
   end if;
   for index in 0 to a'length-1 loop
      result(index) := a(index) xor bb(index) xor carry_in;
      carry_in := (carry_in and (a(index) or bb(index))) or
                  (a(index) and bb(index));
   end loop;
   result(a'length) := carry_in xnor do_add;
   return result;
end; --function


function bv_negate(a : in std_logic_vector) return std_logic_vector is
   variable carry_in : std_logic;
   variable not_a    : std_logic_vector(a'length-1 downto 0);
   variable result   : std_logic_vector(a'length-1 downto 0);
begin
   not_a := not a;
   carry_in := '1';
   for index in a'reverse_range loop
      result(index) := not_a(index) xor carry_in;
      carry_in := carry_in and not_a(index);
   end loop;
   return result;
end; --function


function bv_increment(a : in std_logic_vector(31 downto 2)
                     ) return std_logic_vector is
   variable carry_in : std_logic;
   variable result   : std_logic_vector(31 downto 2);
begin
   carry_in := '1';
   for index in 2 to 31 loop
      result(index) := a(index) xor carry_in;
      carry_in := a(index) and carry_in;
   end loop;
   return result;
end; --function


function bv_inc(a : in std_logic_vector
                ) return std_logic_vector is
   variable carry_in : std_logic;
   variable result   : std_logic_vector(a'length-1 downto 0);
begin
   carry_in := '1';
   for index in 0 to a'length-1 loop
      result(index) := a(index) xor carry_in;
      carry_in := a(index) and carry_in;
   end loop;
   return result;
end; --function

end; --package body


