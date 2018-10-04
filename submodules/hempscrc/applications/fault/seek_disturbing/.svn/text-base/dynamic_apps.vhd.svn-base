-------------------------------------------------------------------------------------
--	Tasks Repository
--		Contains the object codes of all tasks
-------------------------------------------------------------------------------------
--repository structure:
--[/this structure is replicaded according the number of tasks]
--number of tasks
--task id
--task code size
--processor (ffffffff means dynamic allocation)
--task code start address
--[/this structure is replicaded according the number of tasks]
--tasks codes
-------------------------------------------------------------------------------------
library IEEE;
use IEEE.Std_Logic_1164.all;

use work.memory_pack.all;

package dynamic_apps_pack is
	type repository_array is array (0 to 1) of ram;
	signal dynamic_apps : repository_array := ( (others =>(others=>'0')), (others =>(others=>'0')));
end dynamic_apps_pack;
