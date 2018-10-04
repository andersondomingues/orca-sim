------------------------------------------------------------------
---
--- HEMPS 4.0  - October 25 2011
---
--- File function: creates the processing element, including the hermes router and the PE (Plasma or MBlite)
---
--- Responsibles: Eduardo Wachter, Marcelo Mandelli, Carlo Lucas, Fernando Moraes
--- 
--- Contact: fernando.moraes@pucrs.br
---
--- Para compilar código, executar o comando "hempslocal", ir para a pasta "/Documents/scenarios/hemps_seek_disturbing2" e executar o comando "make all"
------------------------------------------------------------------

library ieee;
use work.HeMPS_defaults.all;
use ieee.std_logic_1164.all;

entity processing_element is
        generic(
                memory_type             : string := "XIL";
                processor_type          : string := "sla";
                core_type               : string := "plasma";
                mlite_description       : string := "RTL";
                ram_description         : string := "RTL";
                router_description      : string := "RTL";
                log_file                : string := "UNUSED";
                router_address          : regmetadeflit
                );
        port(
                -- Noc Ports
                clock                   : in  std_logic;
                reset                   : in  std_logic;

                rx			    : in  regNport_neighbor;
                eop_in		            : in  regNport_neighbor;
                data_in		            : in  arrayNport_regflit_neighbor; 
                credit_out	            : out regNport_neighbor;
                
                tx			    : out regNport_neighbor;
                eop_out		            : out regNport_neighbor;
                data_out	            : out arrayNport_regflit_neighbor;
                credit_in	            : in  regNport_neighbor;
                
                --seek in/out
                reset_seek              : in  std_logic;				       
                
                in_seek                 : in  regNport_seek_neighbor;          
                out_ack_seek            : out regNport_seek_neighbor;          
                in_clear                : in  regNport_seek_neighbor;          
                out_ack_clear           : out regNport_seek_neighbor;          
                
                in_source               : in  regNportNsource_target_neighbor; 
                in_target               : in  regNportNsource_target_neighbor; 
                
                in_hop_counter          : in  regNportNhop_neighbor;           
                
                out_seek                : out regNport_seek_neighbor;          
                in_ack_seek             : in  regNport_seek_neighbor;          
                out_clear               : out regNport_seek_neighbor;          
                in_ack_clear            : in  regNport_seek_neighbor;          
                                                                               
                out_source              : out regNportNsource_target_neighbor; 
                out_target              : out regNportNsource_target_neighbor; 
                                                                               
                out_hop_counter         : out regNportNhop_neighbor;           

                -- External Memory
                address                 : out std_logic_vector(31 downto 2);   
                read_req                : out std_logic; 					   
                data_write              : out std_logic_vector(31 downto 0);   
                data_read               : in  std_logic_vector(31 downto 0);   
                write_byte_enable       : out std_logic_vector(3 downto 0);    
                data_valid              : in  std_logic;                       
                
                -- Debug MC
                write_enable_debug      : out  std_logic;                      
                data_out_debug          : out  std_logic_vector(31 downto 0);   
                busy_debug              : in   std_logic;                    
                ack_task                : out  std_logic;                      
                req_task                : in   std_logic_vector(31 downto 0)
		
		-- Wrapper
		--scan_in				: in  std_logic;
		--scan_out			: out std_logic;
		--normal_mode			: in  std_logic;
		--shift				: in  std_logic;
		--normal_mode_Mem_Debug_wrapper		: in  std_logic;
		--shift_Mem_Debug_Mem_Debug_wrapper	: in  std_logic
        );
end processing_element;

architecture processing_element of processing_element is
		
	type arrayNport_scan is array((NPORT_SEEK-2) downto 0) of std_logic_vector (30 downto 0);

        -- NoC Interface
        signal rx_router                : regNport;
        signal eop_in_router            : regNport;
        signal data_in_router           : arrayNport_regflit;
        signal credit_out_router        : regNport;
            
        signal tx_router                : regNport;
        signal eop_out_router           : regNport;
        signal data_out_router          : arrayNport_regflit;
        signal credit_in_router         : regNport;
            
        signal tx0_pe                   : std_logic;
        signal data_out0_pe             : regflit;
        signal eop_out0_pe              : std_logic;
        signal credit_i0_pe             : std_logic; 
        signal rx0_pe                   : std_logic;
        signal data_in0_pe              : regflit;
        signal eop_in0_pe               : std_logic;
        signal credit_o0_pe             : std_logic;
                
        signal tx1_pe                   : std_logic;
        signal data_out1_pe             : regflit;
        signal eop_out1_pe              : std_logic;
        signal credit_i1_pe             : std_logic; 
        signal rx1_pe                   : std_logic;
        signal data_in1_pe              : regflit;
        signal eop_in1_pe               : std_logic;
        signal credit_o1_pe             : std_logic;
            
        --signal reset_seek               : std_logic;
        
        signal in_seek_router           : regNport_seek;
        signal out_ack_seek_router      : regNport_seek;
        signal in_clear_router          : regNport_seek;
        signal out_ack_clear_router     : regNport_seek;
        signal in_source_router         : regNportNsource_target;
        signal in_target_router         : regNportNsource_target;
        signal in_hop_counter_router    : regNportNhop;
        signal out_seek_router          : regNport_seek;
        signal in_ack_seek_router       : regNport_seek;
        signal out_clear_router         : regNport_seek;
        signal in_ack_clear_router      : regNport_seek;
        signal out_source_router        : regNportNsource_target;
        signal out_target_router        : regNportNsource_target;
        signal out_hop_counter_router   : regNportNhop;
        
        signal in_seek_pe               : std_logic;        
        signal out_ack_seek_pe          : std_logic;        
        signal in_clear_pe              : std_logic;        
        signal out_ack_clear_pe         : std_logic;        
        signal in_source_pe             : regNsource_target;
        signal in_target_pe             : regNsource_target;
        signal in_hop_counter_pe        : regNhop;          
        signal out_seek_pe              : std_logic;        
        signal in_ack_seek_pe           : std_logic;        
        signal out_clear_pe             : std_logic;        
        signal in_ack_clear_pe          : std_logic;        
        signal out_source_pe            : regNsource_target;
        signal out_target_pe            : regNsource_target;
        signal out_hop_counter_pe       : regNhop;          
        
        signal sys_int_i                : std_logic;
                
        --not reset
        signal reset_n                  : std_logic;
                                                                                
        --Signals for wrapper cell of address
        signal   func_out_address      : std_logic_vector (31 downto 2);
        signal   scan_out_address      : std_logic;
	signal   address_signal        : std_logic_vector (31 downto 2);

        --Signals for wrapper cell of read_req
        signal   func_out_read_req      : std_logic;
        signal   scan_out_read_req      : std_logic;
        signal   read_req_signal        : std_logic;
        
        --Signals for wrapper cell of data_write
        signal   func_out_data_write      : std_logic_vector (31 downto 0);
        signal   scan_out_data_write      : std_logic;
        signal   data_write_signal		  : std_logic_vector (31 downto 0);

        --Signals for wrapper cell of data_read
        signal   func_out_data_read      : std_logic_vector (31 downto 0);
        signal   scan_out_data_read      : std_logic;

        --Signals for wrapper cell of write_byte_enable
        signal   func_out_write_byte_enable      : std_logic_vector (3 downto 0);
        signal   scan_out_write_byte_enable      : std_logic;
        signal   write_byte_enable_signal        : std_logic_vector (3 downto 0);
        
        --Signals for wrapper cell of data_valid
        signal   func_out_data_valid      : std_logic;
        signal   scan_out_data_valid      : std_logic;
	        
        --Signals for wrapper cell of write_enable_debug
        signal func_out_write_enable_debug      : std_logic;
        signal scan_out_write_enable_debug      : std_logic;
        signal write_enable_debug_signal        : std_logic;
        
        --Signals for wrapper cell of data_out_debug
        signal func_out_data_out_debug      : std_logic_vector (31 downto 0);
        signal scan_out_data_out_debug      : std_logic;
        signal data_out_debug_signal        : std_logic_vector (31 downto 0);
        
        --Signals for wrapper cell of busy_debug
        signal func_out_busy_debug        : std_logic;
        signal scan_out_busy_debug        : std_logic;
        
        --Signals for wrapper cell of ack_task 
        signal func_out_ack_task          : std_logic;
        signal scan_out_ack_task          : std_logic;
        signal ack_task_signal            : std_logic;

        --Signals for wrapper cell of req_task
        signal func_out_req_task          : std_logic_vector (31 downto 0);
        signal scan_out_req_task          : std_logic;
        
        --Signals for wrapper of External Memory and Debug MC
        signal shift_Memory_Debug              : std_logic;
        signal normal_mode_Memory_Debug        : std_logic;
        signal faulty_port_Memory_Debug        : std_logic;
        signal scan_in_Memory_Debug		: std_logic := '1';
        signal scan_out_Memory_Debug		: std_logic;
                
         --Signals for wrapper cell of in_seek
        signal func_out_in_seek      : regNport_seek_neighbor;
        signal scan_out_in_seek      : std_logic;
        
        --Signals for wrapper cell of out_ack_seek
        signal func_out_out_ack_seek      : regNport_seek_neighbor;
        signal scan_out_out_ack_seek      : std_logic;
        
        --Signals for wrapper cell of in_clear
        signal func_out_in_clear      : regNport_seek_neighbor;
        signal scan_out_in_clear      : std_logic;
       
        --Signals for wrapper cell of out_ack_clear
        signal func_out_out_ack_clear     : regNport_seek_neighbor;
        signal scan_out_out_ack_clear     : std_logic;
        
        --Signals for wrapper cell of in_source         
        signal  func_out_in_source      : regNportNsource_target_neighbor;
        signal  scan_out_in_source      : std_logic;
        
        --Signals for wrapper cell of in_target
        signal func_out_in_target      : regNportNsource_target_neighbor;
        signal scan_out_in_target      : std_logic; 
        
        --Signals for wrapper cell of in_hop_counter        
        signal func_out_in_hop_counter      : regNportNhop_neighbor;
        signal scan_out_in_hop_counter      : std_logic;
                
        --Signals for wrapper cell of out_seek
        signal func_out_out_seek      : regNport_seek_neighbor;
        signal scan_out_out_seek      : std_logic;
        
        --Signals for wrapper cell of in_ack_seek
        signal func_out_in_ack_seek      : regNport_seek_neighbor;
        signal scan_out_in_ack_seek      : std_logic;
        
        --Signals for wrapper cell of out_clear
        signal func_out_out_clear      : regNport_seek_neighbor;
        signal scan_out_out_clear      : std_logic;
        
        --Signals for wrapper cell of in_ack_clear,
        signal func_out_in_ack_clear      : regNport_seek_neighbor;
        signal scan_out_in_ack_clear      : std_logic;
        
        --Signals for wrapper cell of out_source
        signal func_out_out_source      : regNportNsource_target_neighbor;
        signal scan_out_out_source      : std_logic;
        
        --Signals for wrapper cell of out_target
        signal func_out_out_target      : regNportNsource_target_neighbor;
        signal scan_out_out_target      : std_logic;
          
        --Signals for wrapper cell of out_hop_counter
        signal func_out_out_hop_counter      : regNportNhop_neighbor;
        signal scan_out_out_hop_counter      : std_logic;
            
        --Signals for wrapper of seek in/out
        signal scan_in_seek     : std_logic := '1';
        signal scan_out_seek	 : std_logic;
        
        --Signals for wrapper cell of rx
        signal func_out_rx            : regNport_neighbor;
        signal scan_out_rx            : std_logic;
        
        --Signals for wrapper of eop_in
        signal func_out_eop_in            : regNport_neighbor;
        signal scan_out_eop_in            : std_logic;
        
        --Signals for wrapper of data_in
        signal func_out_data_in            : arrayNport_regflit_neighbor;
        signal scan_out_data_in            : std_logic;
        
        --Signals for wrapper of credit_in
        signal func_out_credit_in            : regNport_neighbor;
        signal scan_out_credit_in            : std_logic;
        
        --Signals for wrapper of credit_out
        signal func_out_credit_out            : regNport_neighbor;
        signal scan_out_credit_out            : std_logic;
        
        --Signals for wrapper of tx
        signal func_out_tx            : regNport_neighbor;
        signal scan_out_tx            : std_logic;
        
        --Signals for wrapper of eop_out
        signal func_out_eop_out            : regNport_neighbor;
        signal scan_out_eop_out            : std_logic;
        
        --Signals for wrapper of data_out
        signal func_out_data_out            : arrayNport_regflit_neighbor;
        signal scan_out_data_out            : std_logic;
                
        --Signals for wrapper of NOC
        signal scan_in_NOC	 : std_logic := '1';
        signal scan_out_NOC	 : std_logic;
                           
        --Signals for wrappers seek in/out and NOC 
	signal shift_wrapper         : std_logic;
	signal normal_mode_wrapper   : std_logic;
	signal faulty_port           : std_logic_vector (3 downto 0) := "0000";
	signal scan_internal : arrayNport_scan;


	signal scan_in				: std_logic;
	signal scan_out			:  std_logic;
	signal normal_mode			:  std_logic;
	signal shift				:  std_logic;
	signal normal_mode_Mem_Debug_wrapper		:  std_logic;
	signal shift_Mem_Debug_Mem_Debug_wrapper	:  std_logic;
    
    signal failed_reception     : std_logic;

begin 

------------------
------------------
--Wrapper cells --
------------------
------------------

       shift       <= '1';
       normal_mode <= '1';

       shift_Mem_Debug_Mem_Debug_wrapper  <= '1';
       normal_mode_Mem_Debug_wrapper	  <= '1';      

       scan_in <= '1';
--faulty_port <= "0000";

--shift_Memory_Debug       <= '1';
--normal_mode_Memory_Debug <= '1';
--faulty_port_Memory_Debug <= '0';

--scan_internal(0)(0) <= '1'; -- scan_in
--scan_internal(1)(0) <= '1'; -- scan_in
--scan_internal(2)(0) <= '1'; -- scan_in
--scan_internal(3)(0) <= '1'; -- scan_in

scan_internal(0)(0) <= scan_in;			
-- POSSIVEL MULTIPLO DRIVER ... VAI DAR PAU NA SINTESE LOGICA !!!!
scan_out <= scan_internal(0)(30);

scan_internal(1)(0) <= scan_in;			
-- POSSIVEL MULTIPLO DRIVER ... VAI DAR PAU NA SINTESE LOGICA !!!!
scan_out <= scan_internal(1)(30);			

scan_internal(2)(0) <= scan_in;			
-- POSSIVEL MULTIPLO DRIVER ... VAI DAR PAU NA SINTESE LOGICA !!!!
scan_out <= scan_internal(2)(30);

scan_internal(3)(0) <= scan_in;			
-- POSSIVEL MULTIPLO DRIVER ... VAI DAR PAU NA SINTESE LOGICA !!!!
scan_out <= scan_internal(3)(30);

shift_wrapper <= shift;	
normal_mode_wrapper  <= normal_mode;		
--faulty_port  <= "1111";			

shift_Memory_Debug       <= shift_Mem_Debug_Mem_Debug_wrapper;
normal_mode_Memory_Debug <= normal_mode_Mem_Debug_wrapper;

--Genration of Wrapper
generation_of_wrapper: for i in 0 to 3 generate
	wrapper_cell_rx_generation_1:  Entity work.wrapper_cell
		port map(
			func_in     => rx(i*2),
			scan_in     => scan_internal(i)(0), 
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_rx((i*2)),
			scan_out    => scan_internal(i)(1)
		);

	wrapper_cell_rx_generation_2:  Entity work.wrapper_cell
		port map(
			func_in     => rx((i*2)+1),
			scan_in     => scan_internal(i)(1),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_rx((i*2)+1),
			scan_out    => scan_internal(i)(2)
		);

	wrapper_cell_eop_in_generation_1:  Entity work.wrapper_cell
		port map(
			func_in     => eop_in(i*2),
			scan_in     => scan_internal(i)(2),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_eop_in(i*2),
			scan_out    => scan_internal(i)(3)  
		);

	wrapper_cell_eop_in_generation_2:  Entity work.wrapper_cell
		port map(
			func_in     => eop_in((i*2)+1),
			scan_in     => scan_internal(i)(3),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_eop_in((i*2)+1),
			scan_out    => scan_internal(i)(4)  
		);

	wrapper_cell_data_in_generation_1:  Entity work.wrapper_cell_N
		generic map(
			N => FLIT_WIDTH
			)		
		port map(
			func_in     => data_in(i*2),
			scan_in     => scan_internal(i)(4),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_data_in(i*2),
			scan_out    => scan_internal(i)(5)  
		);

	wrapper_cell_data_in_generation_2:  Entity work.wrapper_cell_N
		generic map(
			N => FLIT_WIDTH
			)
		port map(
			func_in     => data_in((i*2)+1),
			scan_in     => scan_internal(i)(5),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_data_in((i*2)+1),
			scan_out    => scan_internal(i)(6)  
		);


	wrapper_cell_credit_in_generation_1:  Entity work.wrapper_cell
		generic map(
			faulty_value => '1'
		)
		port map(
			func_in     => credit_in(i*2),
			scan_in     => scan_internal(i)(6),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_credit_in(i*2),
			scan_out    => scan_internal(i)(7) 
		);

	wrapper_cell_credit_in_generation_2:  Entity work.wrapper_cell
		generic map(
			faulty_value => '1'
		)
		port map(
			func_in     => credit_in((i*2)+1),
			scan_in     => scan_internal(i)(7),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_credit_in((i*2)+1),
			scan_out    => scan_internal(i)(8) 
		);

	wrapper_cell_in_seek_generation:  Entity work.wrapper_cell
		port map(
			func_in     => in_seek(i),
			scan_in     => scan_internal(i)(8),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_in_seek(i),
			scan_out    => scan_internal(i)(9) 
		);

	wrapper_cell_in_clear_generation:  Entity work.wrapper_cell
		port map(
			func_in     => in_clear(i),
			scan_in     => scan_internal(i)(9),
			clock       => clock,
			shift       => shift_wrapper,     
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_in_clear(i),
			scan_out    => scan_internal(i)(10)
			);

	wrapper_cell_in_source_generation:  Entity work.wrapper_cell_N
		generic map(
			N => SOURCE_TARGET_SIZE
			     )
		port map(
			func_in     => in_source(i),
			scan_in     => scan_internal(i)(10),
			clock       => clock,
			shift       => shift_wrapper,     
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_in_source(i),
			scan_out    => scan_internal(i)(11) 
			);

	wrapper_cell_in_target_generation:  Entity work.wrapper_cell_N
		generic map(
			N => SOURCE_TARGET_SIZE
			     )
		port map(
			func_in     => in_target(i),
			scan_in     => scan_internal(i)(11),
			clock       => clock,
			shift       => shift_wrapper,     
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_in_target(i),
			scan_out    => scan_internal(i)(12) 
			);
 
	wrapper_cell_in_hop_counter_generation:  Entity work.wrapper_cell_N
		generic map(
			N => HOP_SIZE
			     )
		port map(
			func_in     => in_hop_counter(i),
			scan_in     => scan_internal(i)(12),
			clock       => clock,
			shift       => shift_wrapper,     
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_in_hop_counter(i),
			scan_out    => scan_internal(i)(13) 
			);

	wrapper_cell_in_ack_seek_generation:  Entity work.wrapper_cell
		port map(
			func_in     => in_ack_seek(i),
			scan_in     => scan_internal(i)(13),
			clock       => clock,
			shift       => shift_wrapper,     
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_in_ack_seek(i),
			scan_out    => scan_internal(i)(14)
			);
	
	wrapper_cell_in_ack_clear_generation:  Entity work.wrapper_cell
		port map(
			func_in     => in_ack_clear(i),
			scan_in     => scan_internal(i)(14),
			clock       => clock,
			shift       => shift_wrapper,     
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_in_ack_clear(i),
			scan_out    => scan_internal(i)(15)
			);

	wrapper_cell_credit_out_generation_1:  Entity work.wrapper_cell
		generic map(
			faulty_value => '1'
		)
		port map(
			func_in     => credit_out_router(i*2),
			scan_in     => scan_internal(i)(15),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_credit_out(i*2),
			scan_out    => scan_internal(i)(16)  
		);

	wrapper_cell_credit_out_generation_2:  Entity work.wrapper_cell
		generic map(
			faulty_value => '1'
		)
		port map(
			func_in     => credit_out_router((i*2)+1),
			scan_in     => scan_internal(i)(16),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_credit_out((i*2)+1),
			scan_out    => scan_internal(i)(17)  
		);

	wrapper_cell_tx_generation_1:  Entity work.wrapper_cell
		port map(
			func_in     => tx_router(i*2),
			scan_in     => scan_internal(i)(17),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_tx(i*2),
			scan_out    => scan_internal(i)(18)   
		);


	wrapper_cell_tx_generation_2:  Entity work.wrapper_cell
		port map(
			func_in     => tx_router((i*2)+1),
			scan_in     => scan_internal(i)(18),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_tx((i*2)+1),
			scan_out    => scan_internal(i)(19) 
		);

	wrapper_cell_eop_out_generation_1:  Entity work.wrapper_cell
		port map(
			func_in     => eop_out_router(i*2),
			scan_in     => scan_internal(i)(19),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_eop_out(i*2),
			scan_out    => scan_internal(i)(20) 
		);

	wrapper_cell_eop_out_generation_2:  Entity work.wrapper_cell
		port map(
			func_in     => eop_out_router((i*2)+1),
			scan_in     => scan_internal(i)(20),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_eop_out((i*2)+1),
			scan_out    => scan_internal(i)(21) 
		);
	
	wrapper_cell_data_out_generation_1:  Entity work.wrapper_cell_N
		generic map(
			N => FLIT_WIDTH
			     )		
		port map(
			func_in     => data_out_router(i*2),
			scan_in     => scan_internal(i)(21),
			clock       => clock,
			shift	    => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_data_out(i*2),
			scan_out    => scan_internal(i)(22)  
		);
 		
	wrapper_cell_data_out_generation_2:  Entity work.wrapper_cell_N
		generic map(
			N => FLIT_WIDTH
			     )
		port map(
			func_in     => data_out_router((i*2)+1),
			scan_in     => scan_internal(i)(22),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_data_out((i*2)+1),
			scan_out    => scan_internal(i)(23)  
		);

	wrapper_cell_out_ack_seek_generation:  Entity work.wrapper_cell
		port map(
			func_in     => out_ack_seek_router(i),
			scan_in     => scan_internal(i)(23),
			clock       => clock,
			shift       => shift_wrapper,      
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_out_ack_seek(i),
			scan_out    => scan_internal(i)(24) 
			);		
	
	wrapper_cell_out_ack_clear_generation:  Entity work.wrapper_cell
		port map(
			func_in     => out_ack_clear_router(i),
			scan_in     => scan_internal(i)(24),
			clock       => clock,
			shift       => shift_wrapper,    
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_out_ack_clear(i),
			scan_out    => scan_internal(i)(25)  
			);
			
	wrapper_cell_out_seek_generation_1:  Entity work.wrapper_cell
		port map(
			func_in     => out_seek_router(i),
			scan_in     => scan_internal(i)(25),
			clock       => clock,
			shift       => shift_wrapper,     
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_out_seek(i),
			scan_out    => scan_internal(i)(26) 
			);
	
	wrapper_cell_out_clear_generation:  Entity work.wrapper_cell
		port map(
			func_in     => out_clear_router(i),
			scan_in     => scan_internal(i)(26),
			clock       => clock,
			shift       => shift_wrapper,     
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_out_clear(i),
			scan_out    => scan_internal(i)(27) 
			);

	wrapper_cell_out_source_generation:  Entity work.wrapper_cell_N
		generic map(
			N => SOURCE_TARGET_SIZE
			   )
		port map(
			func_in     => out_source_router(i),
			scan_in     => scan_internal(i)(27),
			clock       => clock,
			shift       => shift_wrapper,     
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_out_source(i),
			scan_out    => scan_internal(i)(28) 
			);

	wrapper_cell_out_target_generation:  Entity work.wrapper_cell_N
		generic map(
			N => SOURCE_TARGET_SIZE
			   )
		port map(
			func_in     => out_target_router(i),
			scan_in     => scan_internal(i)(28),
			clock       => clock,
			shift       => shift_wrapper,     
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_out_target(i),
			scan_out    => scan_internal(i)(29) 
			);

	wrapper_cell_out_hop_counter_generation:  Entity work.wrapper_cell_N
		generic map(
			N => HOP_SIZE
			   )
		port map(
			func_in     => out_hop_counter_router(i),
			scan_in     => scan_internal(i)(29),
			clock       => clock,
			shift       => shift_wrapper,     
			normal_mode => normal_mode_wrapper,
			faulty_port => faulty_port(i),
			func_out    => func_out_out_hop_counter(i),
			scan_out    => scan_internal(i)(30)
			);
end generate;

--Generation of Wrapper for Memory External and Debug MC
wrapper_cell_address_generation:  Entity work.wrapper_cell_N
		generic map(
			N => 30
		)
		port map(
			func_in     => address_signal,
			scan_in     => scan_out_req_task, 
			clock       => clock,
			shift       => shift_Memory_Debug,      
			normal_mode => normal_mode_Memory_Debug,
			faulty_port => faulty_port_Memory_Debug,
			func_out    => func_out_address,
			scan_out    => scan_out_address
		);

address <= func_out_address;
					
wrapper_cell_read_req_generation:  Entity work.wrapper_cell
		port map(
			func_in     => read_req_signal,
			scan_in     => scan_out_address,
			clock       => clock,
			shift       => shift_Memory_Debug,      
			normal_mode => normal_mode_Memory_Debug,
			faulty_port => faulty_port_Memory_Debug,
			func_out    => func_out_read_req,
			scan_out    => scan_out_read_req
		);

read_req <= func_out_read_req;
		
wrapper_cell_data_write_generation:  Entity work.wrapper_cell_N
		generic map(
			N => 32
		)
		port map(
			func_in     => data_write_signal,
			scan_in     => scan_out_read_req,
			clock       => clock,
			shift       => shift_Memory_Debug,      
			normal_mode => normal_mode_Memory_Debug,
			faulty_port => faulty_port_Memory_Debug,
			func_out    => func_out_data_write,
			scan_out    => scan_out_data_write
			);

data_write <= func_out_data_write;
			
wrapper_cell_data_read_generation:  Entity work.wrapper_cell_N
		generic map(
			N => 32
		)
		port map(
			func_in     => data_read,
			scan_in     => scan_in_Memory_Debug,
			clock       => clock,
			shift       => shift_Memory_Debug,      
			normal_mode => normal_mode_Memory_Debug,
			faulty_port => faulty_port_Memory_Debug,
			func_out    => func_out_data_read,
			scan_out    => scan_out_data_read
			);
		
wrapper_cell_write_byte_enable_generation:  Entity work.wrapper_cell_N
		generic map(
			N => 4
		)
		port map(
			func_in     => write_byte_enable_signal,
			scan_in     => scan_out_data_write,
			clock       => clock,
			shift       => shift_Memory_Debug,      
			normal_mode => normal_mode_Memory_Debug,
			faulty_port => faulty_port_Memory_Debug,
			func_out    => func_out_write_byte_enable,
			scan_out    => scan_out_write_byte_enable
			);

write_byte_enable <= func_out_write_byte_enable;
			
wrapper_cell_data_valid_generation:  Entity work.wrapper_cell
		port map(
			func_in     => data_valid,
			scan_in     => scan_out_data_read,
			clock       => clock,
			shift       => shift_Memory_Debug,      
			normal_mode => normal_mode_Memory_Debug,
			faulty_port => faulty_port_Memory_Debug,
			func_out    => func_out_data_valid,
			scan_out    => scan_out_data_valid
			);
			
wrapper_cell_write_enable_debug_generation:  Entity work.wrapper_cell
		port map(
			func_in     => write_enable_debug_signal,
			scan_in     => scan_out_write_byte_enable,
			clock       => clock,
			shift       => shift_Memory_Debug,      
			normal_mode => normal_mode_Memory_Debug,
			faulty_port => faulty_port_Memory_Debug,
			func_out    => func_out_write_enable_debug,
			scan_out    => scan_out_write_enable_debug
			);
			
write_enable_debug <= func_out_write_enable_debug;
			
wrapper_cell_data_out_debug_generation:  Entity work.wrapper_cell_N
		generic map(
			N => 32
		)
		port map(
			func_in     => data_out_debug_signal,
			scan_in     => scan_out_write_enable_debug,
			
			clock       => clock,
			shift       => shift_Memory_Debug,      
			normal_mode => normal_mode_Memory_Debug,
			faulty_port => faulty_port_Memory_Debug,
			func_out    => func_out_data_out_debug,
			scan_out    => scan_out_data_out_debug
			);

data_out_debug <= func_out_data_out_debug;
			
wrapper_cell_busy_debug_generation:  Entity work.wrapper_cell
		port map(
			func_in     => busy_debug,
			scan_in     => scan_out_data_valid,
			clock       => clock,
			shift       => shift_Memory_Debug,      
			normal_mode => normal_mode_Memory_Debug,
			faulty_port => faulty_port_Memory_Debug,
			func_out    => func_out_busy_debug,
			scan_out    => scan_out_busy_debug
			);
			
wrapper_cell_ack_task_generation:  Entity work.wrapper_cell
		port map(
			func_in     => ack_task_signal,
			scan_in     => scan_out_data_out_debug,
			clock       => clock,
			shift       => shift_Memory_Debug,      
			normal_mode => normal_mode_Memory_Debug,
			faulty_port => faulty_port_Memory_Debug,
			func_out    => func_out_ack_task,
			scan_out    => scan_out_Memory_Debug
			);
			
ack_task <= func_out_ack_task;

wrapper_cell_req_task_generation:  Entity work.wrapper_cell_N
		generic map(
			N => 32
		)
		port map(
			func_in     => req_task,
			scan_in     => scan_out_busy_debug,
			clock       => clock,
			shift       => shift_Memory_Debug,      
			normal_mode => normal_mode_Memory_Debug,
			faulty_port => faulty_port_Memory_Debug,
			func_out    => func_out_req_task,
			scan_out    => scan_out_req_task
			);
					
-------------------
-------------------
--begin Router CC--
-------------------
-------------------
        reset_n <= not reset;
                
        Router: Entity work.Router
		generic map( 
			address 		=> router_address,
			input_buffers	=> "SIM"
		)
		port map(
			clock 		            => clock,
			reset 		            => reset,
            
			rx 			    => rx_router,
			eop_in 		            => eop_in_router,
			data_in 	            => data_in_router,
			credit_out 	            => credit_out_router,
            
			tx 			    => tx_router,
			eop_out 	            => eop_out_router,
			data_out 	            => data_out_router,
			credit_in 	            => credit_in_router,
            
            reset_seek              => reset_seek,
            
            in_seek                 => in_seek_router,
            out_ack_seek            => out_ack_seek_router,
            in_clear                => in_clear_router,
            out_ack_clear           => out_ack_clear_router,
            in_source               => in_source_router,
            in_target               => in_target_router,
            in_hop_counter          => in_hop_counter_router,
            
            out_seek                => out_seek_router,
            in_ack_seek             => in_ack_seek_router,
            out_clear               => out_clear_router,
            in_ack_clear            => in_ack_clear_router,
            out_source              => out_source_router,
            out_target              => out_target_router,
            out_hop_counter         => out_hop_counter_router,
            failed_reception        => failed_reception
		);

-------------------
-------------------
--end  Router  CC--
-------------------
-------------------

        connection_neighbor: for i in 0 to 7 generate
            rx_router(i)                <= func_out_rx(i);
            eop_in_router(i)            <= func_out_eop_in(i);
            data_in_router(i)           <= func_out_data_in(i); 
            credit_out(i)               <= func_out_credit_out(i);
            tx(i)                       <= func_out_tx(i);
            eop_out(i)                  <= func_out_eop_out(i);
            data_out(i)                 <= func_out_data_out(i); 
            credit_in_router(i)         <= func_out_credit_in(i);
        end generate;
        
        connection_seek_neighbor: for i in 0 to 3 generate
            in_seek_router(i)           <= func_out_in_seek(i);
            out_ack_seek(i)             <= func_out_out_ack_seek(i);
            in_clear_router(i)          <= func_out_in_clear(i);
            out_ack_clear(i)            <= func_out_out_ack_clear(i);
            in_source_router(i)         <= func_out_in_source(i);
            in_target_router(i)         <= func_out_in_target(i);
            in_hop_counter_router(i)    <= func_out_in_hop_counter(i);
                
            out_seek(i)                 <= func_out_out_seek(i);
            in_ack_seek_router(i)       <= func_out_in_ack_seek(i);
            out_clear(i)                <= func_out_out_clear(i);
            in_ack_clear_router(i)      <= func_out_in_ack_clear(i);
            out_source(i)               <= func_out_out_source(i);
            out_target(i)               <= func_out_out_target(i);
            out_hop_counter(i)          <= func_out_out_hop_counter(i);
        end generate;
        
        rx_router(8)                <= tx0_pe;
        eop_in_router(8)            <= eop_out0_pe;
        data_in_router(8)           <= data_out0_pe;
        credit_i0_pe                <= credit_out_router(8);

        rx0_pe                      <= tx_router(8);
        eop_in0_pe                  <= eop_out_router(8);
        data_in0_pe                 <= data_out_router(8);
        credit_in_router(8)         <= credit_o0_pe;
        
        rx_router(9)                <= tx1_pe;
        eop_in_router(9)            <= eop_out1_pe;
        data_in_router(9)           <= data_out1_pe;
        credit_i1_pe                <= credit_out_router(9);

        rx1_pe                      <= tx_router(9);
        eop_in1_pe                  <= eop_out_router(9);
        data_in1_pe                 <= data_out_router(9);
        credit_in_router(9)         <= credit_o1_pe;
        
        --in_seek_pe                  <= out_seek_router(4);
        --in_ack_seek_router(4)       <= out_ack_seek_pe;
        --in_clear_pe                 <= out_clear_router(4);
        --in_ack_clear_router(4)      <= out_ack_clear_pe;
        --in_source_pe                <= out_source_router(4);
        --in_target_pe                <= out_target_router(4);
        --in_hop_counter_pe           <= out_hop_counter_router(4);
        
        
        in_seek_router(4)           <= out_seek_pe;
        in_ack_seek_pe              <= out_ack_seek_router(4);
        in_clear_router(4)          <= out_clear_pe;
        in_ack_clear_pe             <= out_ack_clear_router(4);
        in_source_router(4)         <= out_source_pe;
        in_target_router(4)         <= out_target_pe;
        in_hop_counter_router(4)    <= out_hop_counter_pe;
        
-------------------
-------------------
--begin  Plasma  --
-------------------
-------------------
         
        PE_PLASMA: if core_type = "plasma" generate
                plasma: entity work.plasma
                        generic map (
                                memory_type             => "TRI",
                                processor_type          => processor_type,
                                mlite_description       => mlite_description,
                                ram_description         => ram_description,
                                log_file                => log_file,
                                address_router          => router_address
                        )
                        port map(
                                clock                           => clock,
                                reset                           => reset,
                                
                                -- NoC interface (Local port 0)  
                                tx0                             => tx0_pe,
                                data_out0                       => data_out0_pe,
                                eop_out0	                    => eop_out0_pe,
                                credit_i0                       => credit_i0_pe,
                                rx0                             => rx0_pe,
                                data_in0                        => data_in0_pe,
                                eop_in0		                    => eop_in0_pe,
                                credit_o0	                    => credit_o0_pe,

                                -- NoC interface (Local port 1) 
                                tx1                             => tx1_pe,
                                data_out1                       => data_out1_pe,
                                eop_out1	                    => eop_out1_pe,
                                credit_i1                       => credit_i1_pe,
                                rx1                             => rx1_pe,
                                data_in1                        => data_in1_pe,
                                eop_in1		                    => eop_in1_pe,
                                credit_o1	                    => credit_o1_pe,
        
                                --seek in/out
                                --in_seek                         => in_seek_pe,
                                --out_ack_seek                    => out_ack_seek_pe,
                                --in_clear                        => in_clear_pe,
                                --out_ack_clear                   => out_ack_clear_pe,
                                --in_source                       => in_source_pe,
                                --in_target                       => in_target_pe,
                                --in_hop_counter                  => in_hop_counter_pe,
                                
                                out_seek                        => out_seek_pe,
                                in_ack_seek                     => in_ack_seek_pe,
                                out_clear                       => out_clear_pe,
                                in_ack_clear                    => in_ack_clear_pe,
                                out_source                      => out_source_pe,
                                out_target                      => out_target_pe,
                                out_hop_counter                 => out_hop_counter_pe,
        
                                address                         => address_signal,
                                read_req                        => read_req_signal, 
                                data_write                      => data_write_signal,
                                data_read                       => func_out_data_read,
                                write_byte_enable               => write_byte_enable_signal,
                                data_valid                      => func_out_data_valid,

                                write_enable_debug              => write_enable_debug_signal,
                                data_out_debug                  => data_out_debug_signal,
                                busy_debug                      => func_out_busy_debug,
                                
                                ack_task                        => ack_task_signal,
                                req_task                        => func_out_req_task,
                                
                                NI_failed_reception             => failed_reception
                        );
        end generate;
        
-------------------
-------------------
--  end  Plasma  --
-------------------
-------------------

-------------------
-------------------
--begin  mb-lite --
-------------------
---------------------
        --PE_MBLITE: if core_type = "mblite" generate
                --mblite: entity work.mblite_soc
                        --generic map (
                                --log_file                => log_file,
                                --address_router          => router_address
                        --)
                        --port map(
                                --sys_clk_i               => clock,
                                --sys_rst_i               => reset,
                                --clock_tx                => clock_tx_pe,
                                --tx                      => tx_pe,
                                --data_out                => data_out_pe,
                                --credit_i                => credit_i_pe,
                                --clock_rx                => clock_rx_pe,
                                --rx                      => rx_pe,
                                --data_in                 => data_in_pe,
                                --credit_o                => credit_o_pe
                        --);
        --end generate;

-------------------
-------------------
--  end  mb-lite --
-------------------
-------------------

end architecture processing_element;
