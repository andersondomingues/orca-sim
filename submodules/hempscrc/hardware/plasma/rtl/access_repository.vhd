library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

entity access_repository is
	port (
		clock			: in  std_logic;
		reset			: in  std_logic;
		--access to repository in ddr2
		read_req		: out std_logic;
		address			: out std_logic_vector(31 downto 2);
		data_valid		: in  std_logic;
		data_read		: in  std_logic_vector(31 downto 0);
		--dma acess
		mem_ddr_access		: in  std_logic;
		dma_mem_addr_ddr	: in  std_logic_vector(31 downto 0);
		dma_mem_ddr_read_req	: in  std_logic;
		--plasma interface
		cpu_mem_address		: in  std_logic_vector(31 downto 0);
		cpu_mem_address_reg	: in  std_logic_vector(31 downto 0);
		mem_hold		: out std_logic;
		data_read_reg		: out std_logic_vector(31 downto 0)
	);
end entity access_repository;

architecture access_repository of access_repository is
	type state is (wait_addr, set_req, wait_data, set_done, dma_access);
	signal ea, pe : state;
begin
	process(clock, reset)
	begin
		if reset = '1' then
			ea <= wait_addr;
			mem_hold <= '0';
			read_req <= '0';
			address <= (others => '0');
			data_read_reg <= (others => '0');
		elsif rising_edge(clock) then
			ea <= pe;
			--data_read registered to dma
			if data_valid = '1' then
				data_read_reg <= data_read;
			end if;
			
			case ea is
			when wait_addr =>
				if (cpu_mem_address(30 downto 28) = "001") then
					mem_hold <= '1';
				elsif mem_ddr_access = '1' then
					read_req <= '1';
					address <= dma_mem_addr_ddr(31 downto 2);
					mem_hold <= '0';
				end if;
			when set_req =>
				address <= cpu_mem_address_reg(31 downto 2);
				read_req <= '1';
			when wait_data =>
				address <= cpu_mem_address_reg(31 downto 2);
				if(data_valid = '1') then
					mem_hold <= '0';
					read_req <= '0';
				else
					read_req <= '1';
				end if;
			when dma_access =>
				read_req <= dma_mem_ddr_read_req;
				address <= dma_mem_addr_ddr(31 downto 2);
			when set_done =>
				read_req <= '0';
				
			end case;
		end if;
	end process;

	process(ea,cpu_mem_address,data_valid,mem_ddr_access)
	begin
		case ea is
		when wait_addr =>
			if(cpu_mem_address(30 downto 28) = "001") then
				pe <= set_req;
			elsif mem_ddr_access = '1' then
				pe <= dma_access;
			else
				pe <= wait_addr;
			end if;
		when set_req =>
			pe <= wait_data;
		when wait_data =>
			if(data_valid = '1') then
				pe <= set_done;
			else
				pe <= wait_data;
			end if;
		when dma_access =>
			if mem_ddr_access = '1' then
				pe <= dma_access;
			else
				pe <= set_done;
			end if;
		when set_done =>
			pe <= wait_addr;
		end case;
	end process;

end architecture;