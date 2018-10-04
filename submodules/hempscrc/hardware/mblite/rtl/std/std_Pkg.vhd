----------------------------------------------------------------------------------------------
--
--      Input file         : std_Pkg.vhd
--      Design name        : std_Pkg
--      Author             : Tamar Kranenburg
--      Company            : Delft University of Technology
--                         : Faculty EEMCS, Department ME&CE
--                         : Systems and Circuits group
--
--      Description        : Package with several standard components.
--
----------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

PACKAGE std_Pkg IS

----------------------------------------------------------------------------------------------
-- STANDARD COMPONENTS IN STD_PKG
----------------------------------------------------------------------------------------------

    COMPONENT sram GENERIC
    (
        WIDTH : positive;
        SIZE  : positive
    );
    PORT
    (
        dat_o : OUT std_ulogic_vector(WIDTH - 1 DOWNTO 0);
        dat_i : IN std_ulogic_vector(WIDTH - 1 DOWNTO 0);
        adr_i : IN std_ulogic_vector(SIZE - 1 DOWNTO 0);
        wre_i : IN std_ulogic;
        ena_i : IN std_ulogic;
        clk_i : IN std_ulogic
    );
    END COMPONENT;

    COMPONENT sram_4en GENERIC
    (
        WIDTH : positive;
        SIZE  : positive
    );
    PORT
    (
        dat_o : OUT std_ulogic_vector(WIDTH - 1 DOWNTO 0);
        dat_i : IN std_ulogic_vector(WIDTH - 1 DOWNTO 0);
        adr_i : IN std_ulogic_vector(SIZE - 1 DOWNTO 0);
        wre_i : IN std_ulogic_vector(3 DOWNTO 0);
        ena_i : IN std_ulogic;
        clk_i : IN std_ulogic
    );
    END COMPONENT;

    COMPONENT dsram GENERIC
    (
        WIDTH : positive;
        SIZE  : positive
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
    END COMPONENT;

----------------------------------------------------------------------------------------------
-- FUNCTIONS IN STD_PKG
----------------------------------------------------------------------------------------------

    FUNCTION v_or(d : std_ulogic_vector) RETURN std_ulogic;
    FUNCTION is_zero(d : std_ulogic_vector) RETURN std_ulogic;
    FUNCTION is_not_zero(d : std_ulogic_vector) RETURN std_ulogic;
    FUNCTION my_conv_integer(a: std_ulogic_vector) RETURN integer;
    FUNCTION notx(d : std_ulogic_vector) RETURN boolean;
    FUNCTION compare(a, b : std_ulogic_vector) RETURN std_ulogic;
    FUNCTION multiply(a, b : std_ulogic_vector) RETURN std_ulogic_vector;
    FUNCTION sign_extend(value: std_ulogic_vector; fill: std_ulogic; size: positive) RETURN std_ulogic_vector;
    FUNCTION add(a, b : std_ulogic_vector; ci: std_ulogic) RETURN std_ulogic_vector;
    FUNCTION increment(a : std_ulogic_vector) RETURN std_ulogic_vector;
    FUNCTION shift(value : std_ulogic_vector(31 DOWNTO 0); shamt: std_ulogic_vector(4 DOWNTO 0); s: std_ulogic; t: std_ulogic) RETURN std_ulogic_vector;
    FUNCTION shift_left(value : std_ulogic_vector(31 DOWNTO 0); shamt : std_ulogic_vector(4 DOWNTO 0)) RETURN std_ulogic_vector;
    FUNCTION shift_right(value : std_ulogic_vector(31 DOWNTO 0); shamt : std_ulogic_vector(4 DOWNTO 0); padding: std_ulogic) RETURN std_ulogic_vector;

END std_Pkg;

PACKAGE BODY std_Pkg IS

-- Unary OR reduction
    FUNCTION v_or(d : std_ulogic_vector) RETURN std_ulogic IS
        VARIABLE z : std_ulogic;
    BEGIN
        z := '0';
        IF notx (d) THEN
            FOR i IN d'range LOOP
                z := z OR d(i);
            END LOOP;
        END IF;
        RETURN z;
    END;

-- Check for ones in the vector
    FUNCTION is_not_zero(d : std_ulogic_vector) RETURN std_ulogic IS
        VARIABLE z : std_ulogic_vector(d'range);
    BEGIN
        z := (OTHERS => '0');
        IF notx(d) THEN

            IF d = z THEN
                RETURN '0';
            ELSE
                RETURN '1';
            END IF;

        ELSE
            RETURN '0';
        END IF;
    END;

-- Check for ones in the vector
    FUNCTION is_zero(d : std_ulogic_vector) RETURN std_ulogic IS
    BEGIN
        RETURN NOT is_not_zero(d);
    END;

    -- rewrite conv_integer to avoid modelsim warnings
    FUNCTION my_conv_integer(a : std_ulogic_vector) RETURN integer IS
        VARIABLE res : integer RANGE 0 TO 2**a'length-1;
    BEGIN
        res := 0;
        IF (notx(a)) THEN
            res := to_integer(unsigned(a));
        END IF;
        RETURN res;
    END;

    FUNCTION compare(a, b : std_ulogic_vector) RETURN std_ulogic IS
        VARIABLE z : std_ulogic;
    BEGIN

        IF notx(a & b) AND a = b THEN
            RETURN '1';
        ELSE
            RETURN '0';
        END IF;

    END;

-- Unary NOT X test
    FUNCTION notx(d : std_ulogic_vector) RETURN boolean IS
        VARIABLE res : boolean;
    BEGIN
        res := true;
-- pragma translate_off
        res := NOT is_x(d);
-- pragma translate_on
        RETURN (res);
    END;

-- -- 32 bit shifter
-- -- SYNOPSIS:
-- --    value: value to be shifted
-- --    shamt: shift amount
-- --    s 0 / 1: shift right / left
-- --    t 0 / 1: shift logical / arithmetic
-- -- PSEUDOCODE (from microblaze reference guide)
-- --     if S = 1 then
-- --          (rD) ← (rA) << (rB)[27:31]
-- --     else
-- --      if T = 1 then
-- --         if ((rB)[27:31]) ≠ 0 then
-- --              (rD)[0:(rB)[27:31]-1] ← (rA)[0]
-- --              (rD)[(rB)[27:31]:31] ← (rA) >> (rB)[27:31]
-- --         else
-- --              (rD) ← (rA)
-- --      else
-- --         (rD) ← (rA) >> (rB)[27:31]

    FUNCTION shift(value: std_ulogic_vector(31 DOWNTO 0); shamt: std_ulogic_vector(4 DOWNTO 0); s: std_ulogic; t: std_ulogic) RETURN std_ulogic_vector IS
    BEGIN
        IF s = '1' THEN
            -- left arithmetic or logical shift
            RETURN shift_left(value, shamt);
        ELSE
            IF t = '1' THEN
                -- right arithmetic shift
                RETURN shift_right(value, shamt, value(31));
            ELSE
                -- right logical shift
                RETURN shift_right(value, shamt, '0');
            END IF;
        END IF;
    END;

    FUNCTION shift_left(value: std_ulogic_vector(31 DOWNTO 0); shamt: std_ulogic_vector(4 DOWNTO 0)) RETURN std_ulogic_vector IS
        VARIABLE result: std_ulogic_vector(31 DOWNTO 0);
        VARIABLE paddings: std_ulogic_vector(15 DOWNTO 0);
    BEGIN

        paddings := (OTHERS => '0');
        result := value;
        IF (shamt(4) = '1') THEN result := result(15 DOWNTO 0) & paddings(15 DOWNTO 0); END IF;
        IF (shamt(3) = '1') THEN result := result(23 DOWNTO 0) & paddings( 7 DOWNTO 0); END IF;
        IF (shamt(2) = '1') THEN result := result(27 DOWNTO 0) & paddings( 3 DOWNTO 0); END IF;
        IF (shamt(1) = '1') THEN result := result(29 DOWNTO 0) & paddings( 1 DOWNTO 0); END IF;
        IF (shamt(0) = '1') THEN result := result(30 DOWNTO 0) & paddings( 0 );         END IF;
        RETURN result;

    END;

    FUNCTION shift_right(value: std_ulogic_vector(31 DOWNTO 0); shamt: std_ulogic_vector(4 DOWNTO 0); padding: std_ulogic) RETURN std_ulogic_vector IS
        VARIABLE result: std_ulogic_vector(31 DOWNTO 0);
        VARIABLE paddings: std_ulogic_vector(15 DOWNTO 0);
    BEGIN

        paddings := (OTHERS => padding);
        result := value;
        IF (shamt(4) = '1') THEN result := paddings(15 DOWNTO 0) & result(31 DOWNTO 16); END IF;
        IF (shamt(3) = '1') THEN result := paddings( 7 DOWNTO 0) & result(31 DOWNTO  8); END IF;
        IF (shamt(2) = '1') THEN result := paddings( 3 DOWNTO 0) & result(31 DOWNTO  4); END IF;
        IF (shamt(1) = '1') THEN result := paddings( 1 DOWNTO 0) & result(31 DOWNTO  2); END IF;
        IF (shamt(0) = '1') THEN result := paddings( 0 )         & result(31 DOWNTO  1); END IF;
        RETURN result;

    END;

    FUNCTION multiply(a, b: std_ulogic_vector) RETURN std_ulogic_vector IS
        VARIABLE x: std_ulogic_vector (a'length + b'length - 1 DOWNTO 0);
    BEGIN
        x := std_ulogic_vector(signed(a) * signed(b));
        RETURN x(31 DOWNTO 0);
    END;

    FUNCTION sign_extend(value: std_ulogic_vector; fill: std_ulogic; size: positive) RETURN std_ulogic_vector IS
        VARIABLE a: std_ulogic_vector (size - 1 DOWNTO 0);
    BEGIN
        a(size - 1 DOWNTO value'length) := (OTHERS => fill);
        a(value'length - 1 DOWNTO 0) := value;
        return a;
    END;

    FUNCTION add(a, b : std_ulogic_vector; ci: std_ulogic) RETURN std_ulogic_vector IS
        VARIABLE x : std_ulogic_vector(a'length + 1 DOWNTO 0);
    BEGIN
        x := (OTHERS => '0');
        IF notx (a & b & ci) THEN
            x := std_ulogic_vector(signed('0' & a & '1') + signed('0' & b & ci));
        END IF;
        RETURN x(a'length + 1 DOWNTO 1);
    END;

    FUNCTION increment(a : std_ulogic_vector) RETURN std_ulogic_vector IS
        VARIABLE x : std_ulogic_vector(a'length-1 DOWNTO 0);
    BEGIN
        x := (OTHERS => '0');
        IF notx (a) THEN
            x := std_ulogic_vector(signed(a) + 1);
        END IF;
        RETURN x;
    END;

END std_Pkg;