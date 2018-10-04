----------------------------------------------------------------------------------------------
--
--      Input file         : config_Pkg.vhd
--      Design name        : config_Pkg
--      Author             : Tamar Kranenburg
--      Company            : Delft University of Technology
--                         : Faculty EEMCS, Department ME&CE
--                         : Systems and Circuits group
--
--      Description        : Configuration parameters for the design
--
----------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.ALL;

PACKAGE config_Pkg IS

    ----------------------------------------------------------------------------------------------
    -- CORE PARAMETERS
    ----------------------------------------------------------------------------------------------
    -- Implement external interrupt
    CONSTANT CFG_INTERRUPT : boolean := true;      -- Disable or enable external interrupt [0,1]

     -- Implement hardware multiplier
    CONSTANT CFG_USE_HW_MUL : boolean := true;     -- Disable or enable multiplier [0,1]

    -- Implement hardware barrel shifter
    CONSTANT CFG_USE_BARREL : boolean := true;     -- Disable or enable barrel shifter [0,1]

    -- Debug mode
    CONSTANT CFG_DEBUG : boolean := true;          -- Resets some extra registers for better readability
                                                   -- and enables feedback (report) [0,1]
                                                   -- Set CFG_DEBUG to zero to obtain best performance.

    -- Memory parameters
    CONSTANT CFG_DMEM_SIZE  : positive := 32;      -- Data memory bus size in 2LOG # elements
    CONSTANT CFG_IMEM_SIZE  : positive := 32;      -- Instruction memory bus size in 2LOG # elements
    CONSTANT CFG_BYTE_ORDER : boolean := true;     -- Switch between MSB (1, default) and LSB (0) byte order policy

    -- Register parameters
    CONSTANT CFG_REG_FORCE_ZERO : boolean := true; -- Force data to zero if register address is zero [0,1]
    CONSTANT CFG_REG_FWD_WB     : boolean := true; -- Forward writeback to loosen register memory requirements [0,1]
    CONSTANT CFG_MEM_FWD_WB     : boolean := true; -- Forward memory result in stead of introducing stalls [0,1]

    ----------------------------------------------------------------------------------------------
    -- CONSTANTS (currently not configurable / not tested)
    ----------------------------------------------------------------------------------------------
    CONSTANT CFG_DMEM_WIDTH : positive := 32;   -- Data memory width in bits
    CONSTANT CFG_IMEM_WIDTH : positive := 32;   -- Instruction memory width in bits
    CONSTANT CFG_GPRF_SIZE  : positive :=  5;   -- General Purpose Register File Size in 2LOG # elements

    ----------------------------------------------------------------------------------------------
    -- BUS PARAMETERS
    ----------------------------------------------------------------------------------------------

    TYPE memory_map_type IS ARRAY(natural RANGE <>) OF std_ulogic_vector(CFG_DMEM_WIDTH - 1 DOWNTO 0);
    CONSTANT CFG_NUM_SLAVES : positive := 2;
    CONSTANT CFG_MEMORY_MAP : memory_map_type(0 TO CFG_NUM_SLAVES) := (X"00000000", X"00FFFFFF", X"FFFFFFFF");

END config_Pkg;
