----------------------------------------------------------------------------------------------
--
--      Input file         : config_Pkg.vhd
--      Design name        : config_Pkg
--      Author             : Tamar Kranenburg
--      Company            : Delft University of Technology
--                         : Faculty EEMCS, Department ME&CE
--                         : Systems and Circuits group
--
--      Description        : Instantiates instruction- and datamemories and the core
--
----------------------------------------------------------------------------------------------
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

--LIBRARY ieee;
--USE ieee.std_logic_1164.ALL;
--USE ieee.std_logic_unsigned.ALL;

--LIBRARY mblite;

--USE mblite.core_Pkg.ALL;

--PACKAGE mb_Pkg IS

    ----FUNCTION align_mem_store(data : std_ulogic_vector; size : transfer_size) RETURN std_ulogic_vector;
    ----procedure set_register(value_set: std_logic_vector; dmem_o : dmem_out_type; REG : std_ulogic_vector(31 downto 0));
    
--END mb_Pkg;

--PACKAGE BODY mb_Pkg IS
	
    ---- 
    --procedure set_register(variable value_set: out std_logic_vector; variable dmem_o : in dmem_out_type; constant REG : in std_ulogic_vector(31 downto 0)) is
	--begin
		--if dmem_o.adr_o = REG and dmem_o.we_o = '1' then
				--value_set := std_logic_vector(dmem_o.dat_o);
		--end if;
    ----BEGIN
        ----CASE size IS
            ----WHEN byte     => RETURN data( 7 DOWNTO 0) & data( 7 DOWNTO 0) & data(7 DOWNTO 0) & data(7 DOWNTO 0);
            ----WHEN halfword => RETURN data(15 DOWNTO 0) & data(15 DOWNTO 0);
            ----WHEN OTHERS   => RETURN data;
        ----END CASE;
    --end set_register;
--END mb_Pkg;


LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.ALL;

--LIBRARY mblite;
USE work.config_Pkg.ALL;
USE work.core_Pkg.ALL;
USE work.std_Pkg.ALL;

use work.HeMPS_defaults.all;

ENTITY mblite_soc IS
	GENERIC(
		address_router	: regmetadeflit:= (others=>'0');
		log_file		: string
	);
	PORT(
		sys_clk_i	: in STD_LOGIC;
		sys_rst_i	: in STD_LOGIC;
		-- NoC Interface
		clock_tx	: out std_logic;
		tx      	: out std_logic;
		data_out	: out regflit;
		credit_i	: in  std_logic;
		clock_rx	: in  std_logic;
		rx      	: in  std_logic;
		data_in 	: in  regflit;
		credit_o	: out std_logic
	);
END mblite_soc;

ARCHITECTURE arch OF mblite_soc IS

	procedure set_register(signal value_set: out std_logic_vector; signal dmem_o : in dmem_out_type; constant REG : in std_ulogic_vector(31 downto 0)) is
	begin
		if dmem_o.adr_o = REG and dmem_o.we_o = '1' then
				value_set <= std_logic_vector(dmem_o.dat_o);
		end if;
    end set_register;
    
    TYPE data_ram_type IS RECORD
		addr	: std_logic_vector(31 downto 2);
		en		: std_logic;
		we		: std_logic_vector(3 downto 0);
		dataw	: std_logic_vector(31 downto 0);
		datar	: std_ulogic_vector(31 downto 0);
	END RECORD;

    SIGNAL dmem_o : dmem_out_type;
    SIGNAL imem_o : imem_out_type;
    SIGNAL dmem_i : dmem_in_type;
    SIGNAL imem_i : imem_in_type;

    SIGNAL mem_enable : std_ulogic;
    SIGNAL sel_o : std_ulogic_vector(3 DOWNTO 0);

    CONSTANT std_out_adr : std_ulogic_vector(CFG_DMEM_SIZE - 1 DOWNTO 0) := X"FFFFFFC0";
    CONSTANT rom_size : integer := 32;
    CONSTANT ram_size : integer := 32;
	
	--added by wachter
	CONSTANT CURRENT_PAGE		: std_ulogic_vector(CFG_DMEM_SIZE - 1 DOWNTO 0) := X"20000180";
	CONSTANT NEXT_PAGE			: std_ulogic_vector(CFG_DMEM_SIZE - 1 DOWNTO 0) := X"20000190";
	SIGNAL current_page_reg		: std_logic_vector(31 downto 0);
	SIGNAL next_page_reg		: std_logic_vector(31 downto 0);
	SIGNAL imem_adr				: std_logic_vector(rom_size - 1 downto 2);
	SIGNAL dmem_adr				: std_logic_vector(ram_size - 1 downto 2);
	SIGNAL dmem_o_data			: std_ulogic_vector(CFG_DMEM_SIZE - 1 DOWNTO 0);
	SIGNAL dmem_o_adr_reg		: std_ulogic_vector(CFG_DMEM_SIZE - 1 DOWNTO 0);
	SIGNAL dmem_o_data_reg		: std_ulogic_vector(CFG_DMEM_SIZE - 1 DOWNTO 0);
	
	-- Netwok interface mapping.
	constant NI_STATUS_READ		: std_ulogic_vector(31 downto 0):=x"20000100";
	constant NI_STATUS_SEND		: std_ulogic_vector(31 downto 0):=x"20000110";
	constant NI_READ			: std_ulogic_vector(31 downto 0):=x"20000120";
	constant NI_WRITE			: std_ulogic_vector(31 downto 0):=x"20000130";
	constant NI_CONFIGURATION	: std_ulogic_vector(31 downto 0):=x"20000140";
	constant NI_ACK				: std_ulogic_vector(31 downto 0):=x"20000150";
	constant NI_NACK			: std_ulogic_vector(31 downto 0):=x"20000160";
	constant NI_END				: std_ulogic_vector(31 downto 0):=x"20000170";
	
	constant DEBUG 				: std_ulogic_vector(31 downto 0):=x"20000000";
	constant IRQ_STATUS			: std_ulogic_vector(31 downto 0):=x"20000020";
	constant TICK_COUNTER_ADDR	: std_ulogic_vector(31 downto 0):=x"20000300";
	constant COUNTER			: std_ulogic_vector(31 downto 0):=x"20000060";
	constant IRQ_MASK			: std_ulogic_vector(31 downto 0):=x"20000010";
	constant SYS_CALL			: std_ulogic_vector(31 downto 0):=x"20000070";
	
	-- DMA mapping.
	constant DMA_SIZE			: std_ulogic_vector(31 downto 0):=x"20000200";
	constant DMA_ADDR			: std_ulogic_vector(31 downto 0):=x"20000210";
	constant DMA_OP				: std_ulogic_vector(31 downto 0):=x"20000220";
	constant START_DMA			: std_ulogic_vector(31 downto 0):=x"20000230";
	constant DMA_ACK			: std_ulogic_vector(31 downto 0):=x"20000240";
	constant DMA_ACTIVE			: std_ulogic_vector(31 downto 0):=x"20000250";

	--network interface
	signal plasma_hold		: std_logic;
	signal ni_send_av		: std_logic;
	signal ni_read_av		: std_logic;
	signal ni_intr			: std_logic;
	signal ni_send_data		: std_logic;
	signal ni_read_data		: std_logic;
	signal ni_data_write	: std_logic_vector(31 downto 0);
	signal ni_data_read		: std_logic_vector(31 downto 0);
	signal ni_config		: std_logic_vector(31 downto 0);
	
	--cpu signals to ni
	signal cpu_read_data             : std_logic;
	signal cpu_send_data             : std_logic;
	signal cpu_packet_ack            : std_logic;
	signal cpu_packet_nack           : std_logic;
	signal cpu_packet_end            : std_logic;

	--subtype std_logic is resolved std_ulogic;
	
	--debug signal
	signal imem_adr_32_debug		: std_logic_vector(rom_size - 1 downto 2);
	signal dmem_reg					: dmem_out_type;
	signal page_debug				: std_logic_vector(1 downto 0);
	
	--irq signals
	signal irq_reg					: std_logic_vector(31 downto 0);
	signal irq_mask_reg				: std_logic_vector(31 downto 0);
	signal irq						: std_logic;

	--debug only signals
	--return from interruption signal
	signal rtid						: std_logic;
	signal enter_int				: std_logic;
	signal enter_exception			: std_logic;
	signal return_exception			: std_logic;
		
	--time_slice counter
	signal time_slice				: std_logic_vector(31 downto 0);
	
	--tick_counter counter 
	signal tick_counter				: std_logic_vector(31 downto 0);
	
	--sys_call register
	signal sys_call_reg				: std_logic_vector(31 downto 0);
	
	--data ram signals
	signal dram						: data_ram_type;
	
		--dma    
	signal dma_read_data             : std_logic;
	signal dma_send_data             : std_logic;
	signal dma_intr                  : std_logic;
	signal dma_mem_address           : std_logic_vector(31 downto 0);
	signal dma_mem_addr_ddr		 : std_logic_vector(31 downto 0);
	signal dma_mem_ddr_read_req	 : std_logic;
	signal mem_ddr_access		 : std_logic;
	signal dma_mem_write_byte_enable : std_logic_vector(3 downto 0);
	signal dma_mem_data_write        : std_logic_vector(31 downto 0);
	signal dma_mem_data_read         : std_logic_vector(31 downto 0);
	signal dma_data_read             : std_logic_vector(31 downto 0);
	signal dma_data_write            : std_logic_vector(31 downto 0);
	signal dma_enable_internal_ram   : std_logic;
	signal dma_active_sig  			 : std_logic;
	signal data_avail_sig  			 : std_logic; 
	signal busy_sig  			 	 : std_logic;
	signal mem_hold					 : std_logic;
	signal dma_waiting				 : std_logic;
	signal data_read_reg			 : std_logic_vector(31 downto 0);
	signal empty_out				 : std_logic;
	signal cpu_set_size              : std_logic;
	signal cpu_set_address           : std_logic;
	signal cpu_set_op                : std_logic;
	signal cpu_start                 : std_logic;
	signal cpu_ack                   : std_logic;
	signal mem_data_read             : std_logic_vector(31 downto 0);
	signal data_read                 : std_logic_vector(31 downto 0);
	
	signal data_read_b               : std_logic_vector(31 downto 0);
	signal data_read_a               : std_logic_vector(31 downto 0);
	
	signal uart_write_data           : std_logic;
	
BEGIN
	
    mem_enable		<=	'1' when sys_rst_i = '0' AND dmem_o.ena_o = '1' AND dmem_o.adr_o(31 downto 28) /= x"2" else '0';
    sel_o			<= dmem_o.sel_o WHEN dmem_o.we_o = '1' ELSE (OTHERS => '0');
    dmem_i.ena_i	<= '1';
    
    --DMA/data bus mux              
	dram.addr		<= dma_mem_address(31 downto 2) when dma_active_sig = '1' else dmem_adr;
	dram.en			<= dma_enable_internal_ram when dma_active_sig = '1'   else mem_enable;
	dram.we			<= dma_mem_write_byte_enable when dma_active_sig = '1' else std_logic_vector(sel_o);
	dram.dataw		<= dma_mem_data_write when dma_active_sig = '1'else std_logic_vector(dmem_o.dat_o);
	dmem_o_data		<= dram.datar;
	mem_data_read	<= std_logic_vector(dram.datar);

	ram: entity work.ram_mblite
		port map(
			clk             => sys_clk_i,
			--data/dma memory bus
			address_a       => dram.addr,
			enable_a        => dram.en,
			wbe_a           => dram.we,
			data_write_a    => dram.dataw,
			data_read_a     => data_read_a,
			
			--instruction memory bus
			address_b       => imem_adr,
			enable_b        => imem_o.ena_o,
			wbe_b           => (others => '0'),
			data_write_b    => (others => '0'),
			data_read_b     => data_read_b
		);
	
	
	--assertion just to deal with std_logic to std_ulogic conversion
	imem_i.dat_i	<= std_ulogic_vector(data_read_b);
	dram.datar		<= std_ulogic_vector(data_read_a);	

	imem_adr <= 	std_logic_vector(imem_o.adr_o(rom_size - 1 DOWNTO 2)) when imem_o.adr_o = x"00000010" else 
					std_logic_vector(imem_o.adr_o(rom_size - 1 DOWNTO 16)) & current_page_reg(15 downto 14) & std_logic_vector(imem_o.adr_o(13 DOWNTO 2)) when imem_o.adr_o(rom_size - 1 DOWNTO 16) = "0000000000000000" else
					std_logic_vector(imem_o.adr_o(rom_size - 1 DOWNTO 2));
	
	dmem_adr <=		std_logic_vector(dmem_o.adr_o(ram_size - 1 DOWNTO 2)) when imem_o.adr_o = x"00000010" else
					std_logic_vector(dmem_o.adr_o(ram_size - 1 DOWNTO 16)) & current_page_reg(15 downto 14) & std_logic_vector(dmem_o.adr_o(13 DOWNTO 2)) when dmem_o.adr_o(ram_size - 1 DOWNTO 14) = "000000000000000000" else
					std_logic_vector(dmem_o.adr_o(ram_size - 1 DOWNTO 2));
								
	imem_adr_32_debug <= std_logic_vector(imem_o.adr_o(rom_size - 1 downto 2));
	page_debug <= current_page_reg(15 downto 14);

    core0 : entity work.core PORT MAP
    (
        imem_o => imem_o,
        dmem_o => dmem_o,
        imem_i => imem_i,
        dmem_i => dmem_i,
        int_i  => irq,
        rst_i  => sys_rst_i,
        clk_i  => sys_clk_i
    );
    
	UartFile: entity work.UartFile
	generic map (log_file => log_file)
	port map(
		reset			=> sys_rst_i,                         
		data_av			=> uart_write_data,          
		data_in			=> std_logic_vector(dmem_reg.dat_o)
	);
	uart_write_data     <= '1' when dmem_reg.adr_o = DEBUG and dmem_reg.ena_o = '1' else '0';
	
	process(sys_clk_i)
	begin
		if sys_rst_i = '1' then
			dmem_reg.adr_o	<= (others => '0');
			dmem_reg.dat_o	<= (others => '0');
			dmem_reg.ena_o	<= '0';
			dmem_reg.we_o	<= '0';
			current_page_reg<= (others => '0');
			next_page_reg	<= (others => '0');
			time_slice		<= (others => '0');
			irq_mask_reg	<= (others => '0');
			sys_call_reg	<= (others => '0');
			tick_counter	<= (others => '0');
			rtid <= '0';
		elsif sys_clk_i'event and sys_clk_i='1' then
			dmem_reg.adr_o	<= dmem_o.adr_o;
			dmem_reg.dat_o	<= dmem_o.dat_o;
			dmem_reg.ena_o	<= dmem_o.ena_o;
			dmem_reg.we_o	<= dmem_o.we_o;
			
			if imem_o.adr_o = x"00000010" then
				current_page_reg <= (others => '0');
			--kernel writes the new page value
			elsif dmem_o.adr_o = CURRENT_PAGE and dmem_o.we_o = '1' then
				current_page_reg <= std_logic_vector(dmem_o.dat_o);
			elsif imem_i.dat_i = x"b62e0000" then
				current_page_reg <= next_page_reg;
			end if;
			      			
			if imem_i.dat_i = x"b62e0000" then
				rtid <= '1';
			else
				rtid <= '0';
			end if;
			
			if imem_o.adr_o = x"00000010" then
				enter_int <= '1';
			else
				enter_int <= '0';
			end if;
			
			if imem_o.adr_o = x"00000008" then
				enter_exception <= '1';
			else
				enter_exception <= '0';
			end if;
			
			if imem_o.adr_o = x"00000260" then
				return_exception <= '1';
			else
				return_exception <= '0';
			end if;
			
			if current_page_reg /= x"00" then
				time_slice <= time_slice + 1;
	       	end if;

			--set the irq_mask_reg
			set_register(irq_mask_reg,dmem_o,IRQ_MASK);
			--kernel sets the next page to return from a interrupt call in a task
			set_register(next_page_reg, dmem_o, NEXT_PAGE);
	       	-- reinitialize counter
	       	set_register(time_slice,dmem_o,COUNTER);
	       	-- sys_call register
	       	set_register(sys_call_reg,dmem_o,SYS_CALL);
	       	tick_counter <= tick_counter + 1;
		end if;
	end process;
	
	--irq_status
	irq_reg			<= "0000000000000000000000000" & sys_call_reg(0) & ni_intr & '0' & time_slice(14) & "000";
	irq				<= '1' when (irq_reg and irq_mask_reg) /= x"00" else '0';
	
	
	dmem_i.dat_i <= std_ulogic_vector(irq_reg)							when dmem_reg.adr_o = IRQ_STATUS and dmem_reg.ena_o = '1'  else
					((0) => ni_read_av, others => '0')					when dmem_reg.adr_o = NI_STATUS_READ and dmem_reg.ena_o = '1' else
					((0) => ni_send_av, others => '0')					when dmem_reg.adr_o = NI_STATUS_SEND and dmem_reg.ena_o = '1' else
					std_ulogic_vector(ni_data_read)						when dmem_reg.adr_o = NI_READ and dmem_reg.ena_o = '1' else
					std_ulogic_vector(ni_config)						when dmem_reg.adr_o = NI_CONFIGURATION and dmem_reg.ena_o = '1' else
					((0) => dma_active_sig, others => '0')				when dmem_reg.adr_o = DMA_ACTIVE and dmem_reg.ena_o = '1' else
					std_ulogic_vector(irq_mask_reg)						when dmem_reg.adr_o = IRQ_MASK and dmem_reg.ena_o = '1' else
					std_ulogic_vector(tick_counter)						when dmem_reg.adr_o = TICK_COUNTER_ADDR and dmem_reg.ena_o = '1' else
					dmem_o_data;
	
	ni: entity work.network_interface
	generic map ( address_router => address_router)
	port map(
		clock		=> sys_clk_i,
		reset		=> sys_rst_i,
		
		-- NoC interface
		clock_tx	=> clock_tx,
		tx		=> tx,
		data_out	=> data_out,
		credit_i	=> credit_i,
		clock_rx	=> clock_rx,
		rx		=> rx,
		data_in		=> data_in,
		credit_o	=> credit_o,
		
		-- CPU/DMA interface
		hold		=> plasma_hold,
		send_av		=> ni_send_av,
		read_av		=> ni_read_av,
		intr		=> ni_intr,
		send_data	=> ni_send_data,
		read_data	=> ni_read_data,
		data_write	=> ni_data_write,
		data_read	=> ni_data_read,
		config		=> ni_config
	);
	
	
	
	-- CPU -> NI
	cpu_read_data		<= '1' when dmem_reg.adr_o = NI_READ and dmem_reg.ena_o = '1' else '0';
	cpu_send_data		<= '1' when dmem_reg.adr_o = NI_WRITE and dmem_reg.we_o = '1' else '0';
	cpu_packet_ack		<= '1' when dmem_reg.adr_o = NI_ACK and dmem_reg.we_o = '1' else '0';
	cpu_packet_nack		<= '1' when dmem_reg.adr_o = NI_NACK and dmem_reg.we_o = '1' else '0';
	cpu_packet_end		<= '1' when dmem_reg.adr_o = NI_END and dmem_reg.ena_o = '1' else '0';
	
	-- NI inputs with DMA
	ni_send_data			<= dma_send_data or cpu_send_data;
	ni_read_data			<= dma_read_data or cpu_read_data;
	MUX_NI: ni_data_write	<= dma_data_write when dma_send_data = '1' else std_logic_vector(dmem_reg.dat_o);
		
	dma: entity work.dma
	port map(
		clock 			=> sys_clk_i,
		reset      		=> sys_rst_i,

		-- NI interface
		read_av			=> ni_read_av,
		read_data      	=> dma_read_data,
		send_av			=> ni_send_av,
		send_data      	=> dma_send_data,

		-- Configuration pins
		set_address		=> cpu_set_address,
		set_size		=> cpu_set_size,
		set_op			=> cpu_set_op,
		start			=> cpu_start,

		-- Input/Output data ports
		data_read      	=> dma_data_read,
		data_write     	=> dma_data_write,
		active			=> dma_active_sig,

		-- Interrupt management
		intr           	=> dma_intr,        
		intr_ack		=> cpu_ack,

		-- Memory interface
		mem_address    	=> dma_mem_address, 
		mem_data_write	=> dma_mem_data_write,
		mem_data_read  	=> dma_mem_data_read,
		mem_byte_we    	=> dma_mem_write_byte_enable      
	);
	
	-- CPU -> DMA
	cpu_set_size        <= '1' when dmem_reg.adr_o = DMA_SIZE and dmem_reg.ena_o = '1' else '0';
	cpu_set_address     <= '1' when dmem_reg.adr_o = DMA_ADDR and dmem_reg.ena_o = '1' else '0';
	cpu_set_op          <= '1' when dmem_reg.adr_o = DMA_OP and dmem_reg.ena_o = '1' else '0';
	cpu_start           <= '1' when dmem_reg.adr_o = START_DMA and dmem_reg.ena_o = '1' else '0';
	cpu_ack             <= '1' when dmem_reg.adr_o = DMA_ACK and dmem_reg.ena_o = '1' else '0';
   	
   	dma_enable_internal_ram 	<= '1' when dma_mem_address(30 downto 28) = "000" else '0';     
   	   
	-- DMA inputs
	MUX_DMA1: dma_data_read   	<= std_logic_vector(dmem_reg.dat_o) when cpu_set_op = '1' or cpu_set_size = '1' or cpu_set_address = '1' else ni_data_read;
	MUX_DMA2: dma_mem_data_read	<= mem_data_read when dma_enable_internal_ram = '1' else data_read;   

END arch;
