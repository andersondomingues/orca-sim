-- TITLE: Plasma (CPU core with memory)
-- AUTHOR: Steve Rhoads (rhoadss@yahoo.com)
--         Ismael Augusto Grehs (grehs@inf.pucrs.br)
-- DATE CREATED: 6/4/02
-- FILENAME: plasma.vhd
-- PROJECT: Plasma CPU core
-- COPYRIGHT: Software placed into the public domain by the author.
--    Software 'as is' without warranty.  Author liable for nothing.
-- DESCRIPTION:
--    This entity combines the CPU core with memory and a debug.
--
-- Memory Map:
--   0x00000000 - 0x0000ffff   Internal RAM (64KB)
--   0x10000000 - 0x100fffff   External RAM (1MB)
--   Access all Misc registers with 32-bit accesses
--   0x20000000  debug Write (will pause CPU if busy)
--   0x20000000  debug Read
--   0x20000010  IRQ Mask
--   0x20000020  IRQ Status
--   0x20000030 
--   0x20000050 
--   0x20000060  Counter 

--   0x20000100 - NI Status Reading
--   0x20000110 - NI Status Sending
--   0x20000120 - NI Read Data
--   0x20000130 - NI Write Data
--   0x20000140 - NI Configuration
--   0x20000150 - NI Packet ACK
--   0x20000160 - NI Packet NACK
--   0x20000170 - NI Packet END

-- Mappings only for the slave CPU
--   0x20000200 - Set DMA Size
--   0x20000210 - Set DMA Address
--   0x20000220 - Set DMA Operation
--   0x20000230 - Start DMA
--   0x20000240 - DMA ACK
--   0x20000250 - DMA_AVAILABLE


--   0x20000300 - Tick Counter

--   IRQ bits:
--      7   
--      6   
--      5   NoC
--      4   DMA (slave only)
--      3   Counter(18)
--      2  ^Counter(18)
--      1  ^debugBufferFull
--      0   debugDataAvailable
--
--  Re-structurated for adding DMA and NI modules.
---------------------------------------------------------------------

library ieee;
use work.mlite_pack.all;                
use work.HeMPS_defaults.all;

use ieee.std_logic_1164.all;
use ieee.std_logic_misc.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_textio.all;
use ieee.std_logic_unsigned.all;

use std.textio.all;
--library unisim;
--use unisim.vcomponents.all;

entity plasma is
	generic (
	    memory_type			: string := "XIL"; -- "TRI_PORT_X"
	    processor_type		: string := "sla";
	    mlite_description	: string := "RTL";
	    ram_description		: string := "RTL";
		log_file			: string := "output.txt";
		address_router		: regmetadeflit:= (others=>'0')
	);
  	port (  
		clock						: in  std_logic;
		reset    					: in  std_logic;
				
		-- NoC Interface		
		tx0      					: out std_logic;
		data_out0					: out regflitwoCRC;
		eop_out0					: out std_logic;
		credit_i0					: in  std_logic;        
		rx0      					: in  std_logic;
		data_in0 					: in  regflitwoCRC;
		credit_o0					: out std_logic;
		eop_in0						: in std_logic;
				
		tx1      					: out std_logic;
		data_out1					: out regflitwoCRC;
		eop_out1					: out std_logic;
		credit_i1					: in  std_logic;        
		rx1      					: in  std_logic;
		data_in1 					: in  regflitwoCRC;
		credit_o1					: out std_logic;
		eop_in1						: in std_logic;
       
        out_seek            		: out regNBit_seek;
        in_ack_seek         		: in  std_logic;
        out_clear           		: out std_logic;
        in_ack_clear        		: in  std_logic;
        out_source          		: out regNsource_target;
        out_target          		: out regNsource_target;
        out_hop_counter     		: out regNhop;
				
		-- External Memory		
		address   					: out std_logic_vector(31 downto 2);
		read_req					: out std_logic;
		data_write					: out std_logic_vector(31 downto 0);
		data_read		 			: in  std_logic_vector(31 downto 0);
		write_byte_enable 			: out std_logic_vector(3 downto 0); 
		data_valid					: in  std_logic;
		
		-- Debug MC
		write_enable_debug			: out  std_logic;
		data_out_debug				: out  std_logic_vector(31 downto 0);
		busy_debug					: in std_logic;
		
		ack_task					: out  std_logic;
		req_task					: in  std_logic_vector(31 downto 0);
		NI_failed_reception 		: in std_logic;

        seek_unreachable_interrupt  : in std_logic;
		seek_resend_interrupt  		: in std_logic;

        seek_unreachable_target     : in regNsource_target

	);
end entity plasma;

architecture structural of plasma is

	-- Memory map constants.
	constant DEBUG				: std_logic_vector(31 downto 0):=x"20000000";
	constant IRQ_MASK			: std_logic_vector(31 downto 0):=x"20000010";
	constant IRQ_STATUS_ADDR	: std_logic_vector(31 downto 0):=x"20000020";
	constant COUNTER			: std_logic_vector(31 downto 0):=x"20000060";
	constant FIFO_AVAIL			: std_logic_vector(31 downto 0):=x"20000040";
	constant END_SIM			: std_logic_vector(31 downto 0):=x"20000080";
		
	-- Netwok interface mapping.
	constant NI_STATUS_READ		: std_logic_vector(31 downto 0):=x"20000100";
	constant NI_STATUS_SEND		: std_logic_vector(31 downto 0):=x"20000110";
	constant NI_READ			: std_logic_vector(31 downto 0):=x"20000120";
	constant NI_WRITE			: std_logic_vector(31 downto 0):=x"20000130";
	constant NI_CONFIGURATION	: std_logic_vector(31 downto 0):=x"20000140";
	constant NI_ACK				: std_logic_vector(31 downto 0):=x"20000150";
	constant NI_NACK			: std_logic_vector(31 downto 0):=x"20000160";
	constant NI_END				: std_logic_vector(31 downto 0):=x"20000170";
	
	-- DMA mapping.
	constant DMA_SIZE			: std_logic_vector(31 downto 0):=x"20000200";
	constant DMA_ADDR			: std_logic_vector(31 downto 0):=x"20000210";
	constant DMA_OP				: std_logic_vector(31 downto 0):=x"20000220";
	constant START_DMA			: std_logic_vector(31 downto 0):=x"20000230";
	constant DMA_ACK			: std_logic_vector(31 downto 0):=x"20000240";
	constant DMA_ACTIVE			: std_logic_vector(31 downto 0):=x"20000250";
	--signal TASK_ALLOCATIN ON DMA_MASTER
	constant DMA_TARGET 		: std_logic_vector(31 downto 0):=x"20000390";
	constant DMA_TASK_ID		: std_logic_vector(31 downto 0):=x"20000400";
	
    constant NI_SEEK			: std_logic_vector(31 downto 0):=x"20000260";
	constant FAILED_RCV_REG  	: std_logic_vector(31 downto 0):=x"20000270";
	constant RELEASE_NI_REG  	: std_logic_vector(31 downto 0):=x"20000280";    
	
	constant TICK_COUNTER_ADDR	: std_logic_vector(31 downto 0):=x"20000300";
	
	constant REQ_TASK_REG		: std_logic_vector(31 downto 0):=x"20000350";
	constant ACK_TASK_REG		: std_logic_vector(31 downto 0):=x"20000360";
	constant SEEK_REG			: std_logic_vector(31 downto 0):=x"20000370";
	constant CLEAR_REG			: std_logic_vector(31 downto 0):=x"20000380";
	
	signal cpu_mem_address_reg               : std_logic_vector(31 downto 0);
	signal cpu_mem_data_write_reg            : std_logic_vector(31 downto 0);
	signal cpu_mem_write_byte_enable_reg     : std_logic_vector(3 downto 0); 
	signal irq_mask_reg              : std_logic_vector(7 downto 0);
	signal irq_status                : std_logic_vector(7 downto 0); 
	signal irq                       : std_logic;
	signal time_slice               : std_logic_vector(31 downto 0);
	signal write_enable              : std_logic; 
	signal tick_counter              : std_logic_vector(31 downto 0);       
	signal current_page              : std_logic_vector(7 downto 0); 
	
	--cpu
	signal cpu_mem_address           : std_logic_vector(31 downto 0);
	signal cpu_mem_data_write        : std_logic_vector(31 downto 0);
	signal cpu_mem_data_read         : std_logic_vector(31 downto 0);
	signal cpu_mem_write_byte_enable : std_logic_vector(3 downto 0);
	signal cpu_mem_pause             : std_logic;
	
	signal cpu_enable_external_ram   : std_logic;
	signal cpu_enable_internal_ram   : std_logic;
	signal cpu_read_data             : std_logic;
	signal cpu_send_data             : std_logic;
	signal cpu_packet_ack            : std_logic;
	signal cpu_packet_nack           : std_logic;
	signal cpu_packet_end            : std_logic;
	signal cpu_set_size              : std_logic;
	signal cpu_set_address           : std_logic;
	signal cpu_set_op                : std_logic;
	signal cpu_start                 : std_logic;
	signal cpu_ack                   : std_logic;

	signal cpu_set_pe_target		 : std_logic;
	signal cpu_set_task_id			 : std_logic;
	
	--ram
	signal data_read_ram             : std_logic_vector(31 downto 0);
	signal mem_data_read             : std_logic_vector(31 downto 0);
	
	--mc debug 
	signal debug_busy                 : std_logic;
	signal debug_write_data           : std_logic;
	signal debug_write_busy           : std_logic;
	signal debug_data_avail           : std_logic;   
	signal data_read_debug            : std_logic_vector(7 downto 0);
	
	--network interface
	signal ni_send_av               : std_logic;
	signal ni_read_av               : std_logic;
	signal ni_intr                  : std_logic;
	signal ni_send_data             : std_logic;
	signal ni_read_data             : std_logic;
	signal ni_data_write            : std_logic_vector(31 downto 0);
	signal ni_data_read             : std_logic_vector(31 downto 0);
	signal ni_config                : std_logic_vector(31 downto 0);
	
	--dma    
	signal dma_read_data             : std_logic;
	signal dma_send_data             : std_logic;
	signal dma_intr                  : std_logic;
	signal dma_mem_address           : std_logic_vector( 31 downto 0);
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
	
	--emulated uart log_file
	signal uart_write_data           : std_logic;
	
	-- pipelined signals (required by timing constraints) --add by wachter
	signal address_mux		        : std_logic_vector(31 downto 2);
	signal cpu_mem_address_reg1	    : std_logic_vector(31 downto 0);
	
    signal end_sim_reg              : std_logic_vector(31 downto 0);
    
    --seek signals
    signal need_seek                : std_logic;
    signal need_clear               : std_logic;
    
    --cpu seek -> ni signals
    signal ni_send_seek             : std_logic;
    signal ni_seek_data             : std_logic_vector(31 downto 0);
    
    --ni -> cpu
    signal failed_reception         : std_logic;
    signal failed_buffer	        : std_logic;
    signal aux	        : std_logic;
    
    signal release_ni               : std_logic;
    signal cpu_release_ni           : std_logic;
    signal dma_release_ni           : std_logic;

    signal int_seek_unreachable_target	: regNsource_target;
	signal int_seek_unreachable			: std_logic;
	signal int_seek_resend				: std_logic;

	signal NI_failed_reception_rst		: std_logic;
	signal NI_failed_reception_stepdet	: std_logic;
	signal NI_failed_reception_rst_reg	: std_logic;
begin

    u1_cpu: entity work.mlite_cpu 
	      port map (
	         clk          => clock,                      
	         reset_in     => reset,                      
	         intr_in      => irq,                        
	                                                     
	         mem_address  => cpu_mem_address,               
	         mem_data_w   => cpu_mem_data_write,             
	         mem_data_r   => cpu_mem_data_read,          
	         mem_byte_we  => cpu_mem_write_byte_enable,  
	         mem_pause    => cpu_mem_pause,
	         current_page => current_page);

	MASTER_RAM: if processor_type = "mas" generate
			u2_ram: entity work.ram_master
			port map (
				clk		=> clock,                       

				enable_a    	=> cpu_enable_internal_ram,         
				wbe_a       	=> cpu_mem_write_byte_enable,           
				address_a   	=> cpu_mem_address(31 downto 2),
				data_write_a	=> cpu_mem_data_write,          
				data_read_a 	=> data_read_ram,

				enable_b    	=> dma_enable_internal_ram,         
				wbe_b       	=> dma_mem_write_byte_enable,           
				address_b   	=> dma_mem_address(31 downto 2),
				data_write_b	=> dma_mem_data_write,          
				data_read_b		=> mem_data_read
			);
	end generate MASTER_RAM;
	
	SLAVE_RAM: if processor_type = "sla" generate
			u2_ram: entity work.ram_plasma
			port map (
				clk         	=> clock,                       

				enable_a    	=> cpu_enable_internal_ram,         
				wbe_a       	=> cpu_mem_write_byte_enable,           
				address_a   	=> cpu_mem_address(31 downto 2),
				data_write_a	=> cpu_mem_data_write,          
				data_read_a 	=> data_read_ram,

				enable_b    	=> dma_enable_internal_ram,         
				wbe_b       	=> dma_mem_write_byte_enable,           
				address_b   	=> dma_mem_address(31 downto 2),
				data_write_b	=> dma_mem_data_write,          
				data_read_b 	=> mem_data_read
			);
	end generate SLAVE_RAM;

	 MASTER_DEBUG: if processor_type = "mas" generate
		write_enable_debug	<= '1' when cpu_mem_address_reg = DEBUG and write_enable = '1' else '0';
		data_out_debug 		<= cpu_mem_data_write_reg;
		debug_write_busy	<= busy_debug;
		debug_busy			<= '1' when cpu_mem_address_reg = DEBUG and write_enable = '1' and busy_debug = '1' else '0';
	end generate MASTER_DEBUG;
	
	SLAVE_DEBUG: if processor_type = "sla" generate
		u3_UartFile: entity work.UartFile
		generic map (log_file => log_file)
		port map(
			reset			=> reset,                         
			data_av			=> uart_write_data,          
			data_in			=> cpu_mem_data_write_reg
		);
		uart_write_data     <= '1' when cpu_mem_address_reg = DEBUG and write_enable = '1' else '0';
	
		debug_busy		<= '0';
		debug_write_busy	<= '0';
		debug_data_avail	<= '0';
	end generate SLAVE_DEBUG;
	

        u4_ni: entity work.Network_Interface
		generic map ( 
			address_router => address_router,
			buffer_type => ram_description
		)
		port map(
			clock		        => clock,          
			reset               => reset,          

			-- NoC interface (Local port 0)  
			tx0                 			=> tx0,             
			--data_out0           => "0000" & data_out0(15 downto 0),       
			data_out0           => data_out0,       
			eop_out0	        => eop_out0,
			credit_i0           => credit_i0,       
			rx0                 => rx0,             
			data_in0			 => data_in0,        
			eop_in0		        => eop_in0,
			credit_o0	        => credit_o0,

			-- NoC interface (Local port 1) 
			tx1                 => tx1,             
			--data_out1           => "0000" & data_out1(15 downto 0),       
			data_out1           => data_out1,       
			eop_out1	        => eop_out1,
			credit_i1           => credit_i1,       
			rx1                 => rx1,             
			data_in1            => data_in1,        
			eop_in1		        => eop_in1,
			credit_o1	        => credit_o1,
            
            -- SEEK interface 
            receive_seek        => ni_send_seek,
            seek_data           => ni_seek_data,
            release_ni          => release_ni,
            
			-- CPU/DMA interface 
			send_av             => ni_send_av,    
			read_av             => ni_read_av,    
			intr                => ni_intr,       
			send_data           => ni_send_data,  
			read_data           => ni_read_data,   
			data_write          => ni_data_write, 
			data_read           => ni_data_read,  
			config              => ni_config
		);
	
	MASTER_DMA: if processor_type = "mas" generate         
	u5_dma: entity work.dma_master
		port map(
			clock 			=> clock,           
			reset      		=> reset,                    
			
			-- NI interface
			read_av			=> ni_read_av,
			read_data      	=> dma_read_data,
			send_av			=> ni_send_av,
			send_data      	=> dma_send_data,
			
			-- Configuration pins
			set_address	    => cpu_set_address,
			set_size		=> cpu_set_size,
			set_op			=> cpu_set_op,
			start			=> cpu_start,
			set_pe_target	=> cpu_set_pe_target,
			set_task_id  	=> cpu_set_task_id,
			
			-- Input/Output data ports
			data_read      	=> dma_data_read,
			data_write     	=> dma_data_write,
			active			=> dma_active_sig,
			
			-- Interrupt management
			intr           	=> dma_intr,        
			intr_ack		=> cpu_ack,
			
			-- Memory interface
			data_valid	    => data_valid,
			mem_address	    => dma_mem_address, 
			mem_addr_ddr	=> dma_mem_addr_ddr,
			mem_ddr_read_req=> dma_mem_ddr_read_req,
			mem_ddr_access	=> mem_ddr_access,
			mem_data_write	=> dma_mem_data_write,
			mem_data_read  	=> dma_mem_data_read,
			mem_byte_we    	=> dma_mem_write_byte_enable      
		);
    end generate;

	SLAVE_DMA: if processor_type = "sla" generate	
	u5_dma: entity work.dma
		port map(
			clock 			=> clock,           
			reset      		=> reset,                    
			
			-- NI interface
			read_av			=> ni_read_av,
			read_data      	=> dma_read_data,
			send_av			=> ni_send_av,
			send_data      	=> dma_send_data,
            failed_reception=> NI_failed_reception,
            release_ni      => dma_release_ni,
			
			-- Configuration pins
			set_address	    => cpu_set_address,
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
		mem_hold <= '0';
    end generate;
   	-- CPU inputs

    process(clock, reset)
    begin
        if reset = '1' then
                out_source      				<= (others => '0');
                out_target      				<= (others => '0');
                out_hop_counter 				<= (others => '0');
                out_seek        				<= "00";
                need_seek       				<= '0';
                out_clear       				<= '0';
                need_clear      				<= '0';

                int_seek_unreachable			<= '0';
                int_seek_resend					<= '0';
                int_seek_unreachable_target		<= (others => '0');
                NI_failed_reception_rst_reg		<= '0';
      	elsif rising_edge(clock) then
            NI_failed_reception_rst_reg			<= NI_failed_reception_rst;

      		
            --seek
             if cpu_mem_address_reg = SEEK_REG and write_enable = '1' then
               if int_seek_unreachable = '1' then--set interrupt
               
						out_seek        <= "01";
						report "seek from " & CONV_STRING_8BITS(address_router) & " to " & CONV_STRING_8BITS(cpu_mem_data_write_reg(7 downto 0)) severity note;
						out_source      <= address_router;
						out_target      <= cpu_mem_data_write_reg(7 downto 0);
						out_hop_counter <= "000000";

						if cpu_mem_data_write_reg(7 downto 0) = int_seek_unreachable_target then--free the interruption
							int_seek_unreachable	<= '0';						
						end if;				
                end if;   
                
                
                                                    
            else
                out_seek   <= "00";
				if seek_unreachable_interrupt = '1' then--set interrupt
					int_seek_unreachable			<= '1';
					int_seek_unreachable_target		<= seek_unreachable_target;
				    --report "SEEK_UNREACHABLE FROM " & CONV_STRING_8BITS(address_router) & " to " & CONV_STRING_8BITS(cpu_mem_data_write_reg(7 downto 0)) severity note;
				end if;

				if seek_resend_interrupt = '1' then--set interrupt
					int_seek_resend			<= '1';
  					int_seek_unreachable_target		<= seek_unreachable_target;
					--report "SEEK_RESEND FROM      " & CONV_STRING_8BITS(address_router) & " to " & CONV_STRING_8BITS(cpu_mem_data_write_reg(7 downto 0)) severity note;
				end if;			
            end if;
  
		   if (cpu_mem_address_reg = CLEAR_REG and write_enable = '1')  then
                out_clear       <= '1';                
                report "clear from " & CONV_STRING_8BITS(address_router) & " to " & CONV_STRING_8BITS(cpu_mem_data_write_reg(7 downto 0)) severity note;
                out_source      <= address_router;
                out_target      <= cpu_mem_data_write_reg(7 downto 0);
                out_hop_counter <= "000000";   

                if   int_seek_resend = '1' then
					  int_seek_resend			<= '0';
				end if;		
                                                    
            else
                out_clear       <= '0';
            end if;
            
        end if;
    end process;
	
    stepdet: entity work.stepdet
    port map(
        ck          => clock,
        rst         => NI_failed_reception_rst_reg,
        in_sig      => NI_failed_reception,
        out_sig     => NI_failed_reception_stepdet
    );
    NI_failed_reception_rst <= '1' when cpu_mem_address_reg = FAILED_RCV_REG else '0';


	 MUX_CPU: cpu_mem_data_read <=	data_read_reg                           			when cpu_mem_address_reg(30 downto 28) = "001" else	-- External RAM
		   							--ZERO(31 downto 8) & data_read_debug   			when cpu_mem_address_reg = DEBUG else
				   					ZERO(31 downto 8) & irq_mask_reg        			when cpu_mem_address_reg = IRQ_MASK else
				   					ZERO(31 downto 8) & irq_status          			when cpu_mem_address_reg = IRQ_STATUS_ADDR else
									time_slice                              			when cpu_mem_address_reg = COUNTER else
									ZERO(31 downto 1) & ni_read_av          			when cpu_mem_address_reg = NI_STATUS_READ else
									ZERO(31 downto 1) & ni_send_av	        			when cpu_mem_address_reg = NI_STATUS_SEND else
									ni_data_read                            			when cpu_mem_address_reg = NI_READ else
									ni_config                               			when cpu_mem_address_reg = NI_CONFIGURATION else
									tick_counter                            			when cpu_mem_address_reg = TICK_COUNTER_ADDR else
									req_task                                			when cpu_mem_address_reg = REQ_TASK_REG else
									ZERO(31 downto 1) & dma_active_sig      			when cpu_mem_address_reg = DMA_ACTIVE else
									ZERO(31 downto 1) & empty_out           			when cpu_mem_address_reg = FIFO_AVAIL else
                                    ZERO(31 downto 1) & NI_failed_reception_stepdet		when cpu_mem_address_reg = FAILED_RCV_REG else
                                    ZERO(31 downto 8) & int_seek_unreachable_target		when cpu_mem_address_reg = SEEK_REG else
									data_read_ram;
	
	
	cpu_mem_pause		<= debug_busy or mem_hold;
    irq					<= '1' when (irq_status and irq_mask_reg) /= x"00" else '0';
	
	-- CPU -> NI
	cpu_read_data		<= '1' when cpu_mem_address_reg = NI_READ and write_enable = '0' else '0';
	cpu_send_data       <= '1' when cpu_mem_address_reg = NI_WRITE and write_enable = '1' else '0';
	cpu_packet_ack      <= '1' when cpu_mem_address_reg = NI_ACK and write_enable = '1' else '0';
	cpu_packet_nack     <= '1' when cpu_mem_address_reg = NI_NACK and write_enable = '1' else '0';
	cpu_packet_end      <= '1' when cpu_mem_address_reg = NI_END and write_enable = '1' else '0';      
    end_sim_reg         <= x"00000000" when cpu_mem_address_reg = END_SIM and write_enable = '1' else x"00000001";
    
    --CPU -> NI SEEK
    ni_send_seek        <= '1' when cpu_mem_address_reg = NI_SEEK and write_enable = '1' else '0';
    ni_seek_data        <= cpu_mem_data_write_reg when ni_send_seek = '1';
    cpu_release_ni      <= '1' when cpu_mem_address_reg = RELEASE_NI_REG and write_enable = '1' else '0';
    
    release_ni          <= cpu_release_ni or dma_release_ni;
    
	-- CPU -> DMA
	cpu_set_size        <= '1' when cpu_mem_address_reg = DMA_SIZE and write_enable = '1' else '0';
	cpu_set_address     <= '1' when cpu_mem_address_reg = DMA_ADDR and write_enable = '1' else '0';
	cpu_set_op          <= '1' when cpu_mem_address_reg = DMA_OP and write_enable = '1' else '0';
	cpu_start           <= '1' when cpu_mem_address_reg = START_DMA and write_enable = '1' else '0';
	cpu_ack             <= '1' when cpu_mem_address_reg = DMA_ACK and write_enable = '1' else '0';
	cpu_set_pe_target	<= '1' when cpu_mem_address_reg = DMA_TARGET and write_enable = '1' else '0';
	cpu_set_task_id		<= '1' when cpu_mem_address_reg = DMA_TASK_ID and write_enable = '1' else '0';

	
	-- NI inputs  
	ni_send_data        		<= dma_send_data or cpu_send_data;
	ni_read_data        		<= dma_read_data or cpu_read_data;   
	MUX_NI: ni_data_write 		<= dma_data_write when dma_send_data = '1' else cpu_mem_data_write_reg;
   	   
	-- DMA inputs
	MUX_DMA1: dma_data_read   	<= cpu_mem_data_write_reg when cpu_set_op = '1' or cpu_set_size = '1' or cpu_set_address = '1' or cpu_set_pe_target = '1' or cpu_set_task_id = '1' else ni_data_read;
	MUX_DMA2: dma_mem_data_read	<= mem_data_read when dma_enable_internal_ram = '1' else data_read;   
	
	
	-- External Memory
	write_byte_enable			<= cpu_mem_write_byte_enable_reg when cpu_enable_internal_ram = '0' else
			                       dma_mem_write_byte_enable when dma_enable_internal_ram = '0' else
			                       "0000";
	
	MUX_DATA_W: data_write 		<= cpu_mem_data_write_reg     when cpu_enable_internal_ram = '0' else
                           	  	   dma_mem_data_write when dma_enable_internal_ram = '0' else
                           	  	   ZERO;   
   
 		-- Others
	write_enable            	<= '1' when cpu_mem_write_byte_enable_reg /= "0000" else '0';
	cpu_enable_internal_ram 	<= '1' when cpu_mem_address(30 downto 28) = "000" else '0';      
	--cpu_enable_external_ram 	<= '1' when cpu_mem_address_reg(30 downto 28) = "001" else '0';
	dma_enable_internal_ram 	<= '1' when dma_mem_address(30 downto 28) = "000" else '0';
	
	irq_status              	<= ( int_seek_unreachable & '0' & ni_intr & dma_intr & time_slice(14) & not time_slice(14) & not debug_write_busy &  int_seek_resend); 	
	
	process(clock, reset)
	begin            
    	if reset = '1' then
        	cpu_mem_address_reg <= ZERO;
	        cpu_mem_data_write_reg <= ZERO;
	        cpu_mem_write_byte_enable_reg <= ZERO(3 downto 0);
	        irq_mask_reg <= ZERO(7 downto 0);
	        time_slice <= ZERO;
	        tick_counter <= ZERO;
			ack_task <= '0';
		
      	elsif rising_edge(clock) then
        	if cpu_mem_pause = '0' then
				cpu_mem_address_reg <= cpu_mem_address;
            	cpu_mem_data_write_reg <= cpu_mem_data_write;
            	cpu_mem_write_byte_enable_reg <= cpu_mem_write_byte_enable;
            
            	if cpu_mem_address_reg = IRQ_MASK and write_enable = '1' then
					irq_mask_reg <= cpu_mem_data_write_reg(7 downto 0);
				end if;
				
				if current_page /= x"00" then
	        		time_slice <= time_slice + 1;
	       		end if;
         	end if;
			
			if cpu_mem_address_reg = ACK_TASK_REG then
				ack_task <= '1';
			elsif req_task(31) = '0' then 
				ack_task <= '0';
			end if;
			
			-- reinitialize counter
	     	if cpu_mem_address_reg = COUNTER and write_enable = '1' then
	        	time_slice <= cpu_mem_data_write_reg;
	       	end if;       	
	       
	       	tick_counter <= tick_counter + 1;
		end if;
	end process;
	
	MEM_HOLD_MASTER:if processor_type = "mas" generate
		u5_access_repository: entity work.access_repository
			port map(
				clock			=> clock,
				reset			=> reset,
				--access to repository in ddr2
				read_req		=> read_req,
				address			=> address,
				data_valid		=> data_valid,
				data_read		=> data_read,
				--dma acess
				mem_ddr_access		=> mem_ddr_access,
				dma_mem_addr_ddr	=> dma_mem_addr_ddr,
				dma_mem_ddr_read_req	=> dma_mem_ddr_read_req,
				--plasma interface
				cpu_mem_address		=> cpu_mem_address,
				cpu_mem_address_reg	=> cpu_mem_address_reg,
				mem_hold		=> mem_hold,
				data_read_reg		=> data_read_reg
			);
	end generate;
	
end architecture structural;
