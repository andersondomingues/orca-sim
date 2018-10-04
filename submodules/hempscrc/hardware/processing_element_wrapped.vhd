library ieee;
use work.HeMPS_defaults.all;
use ieee.std_logic_1164.all;

entity processing_element_wrapped is
        generic(
                memory_type             : string := "XIL";
                processor_type          : string := "sla";
                core_type               : string := "plasma";
                mlite_description       : string := "RTL";
                ram_description         : string := "RTL";
                router_description      : string := "RTL";
                log_file				: string := "UNUSED";
                router_address          : regmetadeflit 
        );
        port(
                -- Noc Ports
                clock                   : in  std_logic;
                reset                   : in  std_logic;

                rx			            : in  regNport_neighbor;
                eop_in		            : in  regNport_neighbor;
                data_in		            : in  arrayNport_regflit_neighbor;
                credit_out	            : out regNport_neighbor;
                
                tx			            : out regNport_neighbor;
                eop_out		            : out regNport_neighbor;
                data_out	            : out arrayNport_regflit_neighbor;
                credit_in	            : in  regNport_neighbor;
                
                --seek in/out
                reset_seek              : in  std_logic;
                
                in_seek                 : in  regNport_seek_neighbor_NBit;
                out_ack_seek            : out regNport_seek_neighbor;
                in_clear                : in  regNport_seek_neighbor;
                out_ack_clear           : out regNport_seek_neighbor;
                in_source               : in  regNportNsource_target_neighbor;
                in_target               : in  regNportNsource_target_neighbor;
                in_hop_counter          : in  regNportNhop_neighbor;
                
                out_seek                : out regNport_seek_neighbor_NBit;
                in_ack_seek             : in  regNport_seek_neighbor;
                out_clear               : out regNport_seek_neighbor;
                in_ack_clear            : in  regNport_seek_neighbor;
                out_source              : out regNportNsource_target_neighbor;
                out_target              : out regNportNsource_target_neighbor;
                out_hop_counter         : out regNportNhop_neighbor;

                --failed_port
                out_failed_port         : out regNport_neighbor;
                in_failed_port          : in  regNport_neighbor;
                
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
                busy_debug              : in std_logic;
                
                ack_task                : out  std_logic;
                req_task                : in  std_logic_vector(31 downto 0)
        );
end processing_element_wrapped;

architecture processing_element_wrapped of processing_element_wrapped is

-- Noc Ports
signal credit_out_signal	            :regNport_neighbor;
signal tx_signal			            :regNport_neighbor;
signal eop_out_signal		            :regNport_neighbor;
signal data_out_signal	            	:arrayNport_regflit_neighbor;
 
--seek in/out
signal in_seek_signal                 : regNport_seek_neighbor;
signal out_ack_seek_signal            : regNport_seek_neighbor;
signal in_clear_signal                : regNport_seek_neighbor;
signal out_ack_clear_signal           : regNport_seek_neighbor;
signal in_source_signal               : regNportNsource_target_neighbor;
signal in_target_signal               : regNportNsource_target_neighbor;
signal in_hop_counter_signal          : regNportNhop_neighbor;
signal out_seek_signal                : regNport_seek_neighbor_NBit;
signal in_ack_seek_signal             : regNport_seek_neighbor;
signal out_clear_signal               : regNport_seek_neighbor;
signal in_ack_clear_signal            : regNport_seek_neighbor;
signal out_source_signal              : regNportNsource_target_neighbor;
signal out_target_signal              : regNportNsource_target_neighbor;
signal out_hop_counter_signal         : regNportNhop_neighbor;

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
signal func_out_in_seek      : regNport_seek_neighbor_NBit;
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
signal func_out_out_seek      : regNport_seek_neighbor_NBit;
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
type arrayNport_scan is array(3 downto 0) of std_logic_vector (22 downto 0);

signal shift_wrapper         : std_logic;
signal normal_mode_wrapper   : std_logic;
signal faulty_port           : std_logic_vector (7 downto 0) := "00000000";
signal out_failed_test_port  : std_logic_vector (7 downto 0) := "00000000"; -- fochi
signal faulty_port_seek_in   : std_logic_vector (3 downto 0) := "0000";
signal faulty_port_seek_out  : std_logic_vector (3 downto 0) := "0000";
signal scan_internal 		  : arrayNport_scan;

signal scan_in				  :  std_logic;
signal scan_out			  :  std_logic;
signal normal_mode			  :  std_logic;
signal shift				  :  std_logic;
signal scan_Memory_Debug_in  :  std_logic;
signal scan_Memory_Debug_out :  std_logic;
signal scan				  :  std_logic_vector (3 downto 0);
signal normal_mode_Mem_Debug_wrapper		:  std_logic;
signal shift_Mem_Debug_Mem_Debug_wrapper	:  std_logic;

signal credit_out_router : regNport;

begin

	PE: entity work.processing_element
		generic map(
			memory_type           => memory_type,       
			processor_type        => processor_type,   
			core_type             => core_type,         
			mlite_description     => mlite_description, 
			ram_description       => ram_description,   
			router_description    => router_description,  
			log_file			  => log_file, 
			router_address        => router_address   
		)
		port map(
			clock                   =>  clock,                    
			reset          			=>  reset,          		
			
			rx			            =>  func_out_rx,			      
			eop_in		            =>  func_out_eop_in,		      
			data_in		            =>  func_out_data_in,
			credit_out	            =>  credit_out_signal,
			-- credit_out	            =>  credit_out,
			
			tx			            =>  tx,			      
			eop_out		            =>  eop_out,		      
			data_out	            =>  data_out,	      
			credit_in	            =>  credit_in,
			
			reset_seek              =>  reset_seek,        
			
			in_seek                 =>  func_out_in_seek,           
			out_ack_seek            =>  out_ack_seek_signal,      
			in_clear                =>  func_out_in_clear,          
			out_ack_clear           =>  out_ack_clear_signal,     
			in_source               =>  func_out_in_source,         
			in_target               =>  func_out_in_target,         
			in_hop_counter          =>  func_out_in_hop_counter,    
			
			out_seek                =>  out_seek_signal,          
			in_ack_seek             =>  func_out_in_ack_seek,      
			out_clear               =>  out_clear_signal,         
			in_ack_clear            =>  func_out_in_ack_clear,      
			out_source              =>  out_source_signal,        
			out_target              =>  out_target_signal,                       
			out_hop_counter         =>  out_hop_counter_signal,   
            
            in_failed_port          =>  in_failed_port,
            out_failed_port         =>  faulty_port,--sinal de erro vindo do crc para ativar o wrapper -- fochi
			out_failed_test_port 	=> 	out_failed_test_port, --fochi 	
			
			address                 =>  address_signal,           
			read_req                =>  read_req_signal,          
			data_write              =>  data_write_signal,        
			data_read               =>  data_read,         
			write_byte_enable       =>  write_byte_enable_signal, 
			data_valid              =>  data_valid,        
			
			write_enable_debug      =>  write_enable_debug_signal,
			data_out_debug          =>  data_out_debug_signal,    
			busy_debug              =>  busy_debug,        
			ack_task                =>  ack_task_signal,          
			req_task                =>  req_task
		);
	
------------------
------------------
--Wrapper cells --
------------------
------------------

shift       <= '1';
normal_mode <= '1';

shift_Mem_Debug_Mem_Debug_wrapper <= '1';
normal_mode_Mem_Debug_wrapper	  <= '1';      

scan_in <= '1';
       
scan_internal(0)(0) <= scan_in;			
scan(0) <= scan_internal(0)(22);

scan_internal(1)(0) <= scan(0);			
scan(1) <= scan_internal(1)(22);			

scan_internal(2)(0) <= scan(1);			
scan(2) <= scan_internal(2)(22);

scan_internal(3)(0) <= scan(2);			
scan(3) <= scan_internal(3)(22);

scan_out <= scan(3);

scan_out_req_task <= scan_in_Memory_Debug;

shift_wrapper <= shift;	
normal_mode_wrapper  <= normal_mode;					

shift_Memory_Debug       <= shift_Mem_Debug_Mem_Debug_wrapper;
normal_mode_Memory_Debug <= normal_mode_Mem_Debug_wrapper;


faulty_port_seek_in(0) <= faulty_port(0) or faulty_port(1);
faulty_port_seek_in(1) <= faulty_port(2) or faulty_port(3);
faulty_port_seek_in(2) <= faulty_port(4) or faulty_port(5);
faulty_port_seek_in(3) <= faulty_port(6) or faulty_port(7);

faulty_port_seek_out(0) <= faulty_port(0) or faulty_port(1);
faulty_port_seek_out(1) <= faulty_port(2) or faulty_port(3);
faulty_port_seek_out(2) <= faulty_port(4) or faulty_port(5);
faulty_port_seek_out(3) <= faulty_port(6) or faulty_port(7);


--output failed port added by wachter
out_failed_port <= faulty_port;


generation_of_wrapper: for i in 0 to 3 generate

--Genration of Wrapper	
				wrapper_cell_in_seek_generation:  Entity work.wrapper_cell_N
			generic map(
				N => SEEK_BIT_SIZE
					)
			port map(
				func_in     => in_seek(i),
				scan_in     => scan_internal(i)(0),
				clock       => clock,
				shift       => shift_wrapper,      
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port_seek_in(i),
				out_failed_test_port	=> '0',
				func_out    => func_out_in_seek(i),
				scan_out    => scan_internal(i)(1) 
			);
			
		wrapper_cell_in_clear_generation:  Entity work.wrapper_cell
			port map(
				func_in     => in_clear(i),
				scan_in     => scan_internal(i)(1),
				clock       => clock,
				shift       => shift_wrapper,     
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port_seek_in(i),
				out_failed_test_port	=> '0',
				func_out    => func_out_in_clear(i),
				scan_out    => scan_internal(i)(2)
				);
	
		wrapper_cell_in_source_generation:  Entity work.wrapper_cell_N
			generic map(
				N => SOURCE_TARGET_SIZE
					)
			port map(
				func_in     => in_source(i),
				scan_in     => scan_internal(i)(2),
				clock       => clock,
				shift       => shift_wrapper,     
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port_seek_in(i),
				out_failed_test_port	=> '0',
				func_out    => func_out_in_source(i),
				scan_out    => scan_internal(i)(3) 
				);
	
		wrapper_cell_in_target_generation:  Entity work.wrapper_cell_N
			generic map(
				N => SOURCE_TARGET_SIZE
					)
			port map(
				func_in     => in_target(i),
				scan_in     => scan_internal(i)(3),
				clock       => clock,
				shift       => shift_wrapper,     
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port_seek_in(i),
				out_failed_test_port	=> '0',
				func_out    => func_out_in_target(i),
				scan_out    => scan_internal(i)(4) 
				);
	
		wrapper_cell_in_hop_counter_generation:  Entity work.wrapper_cell_N
			generic map(
				N => HOP_SIZE
					)
			port map(
				func_in     => in_hop_counter(i),
				scan_in     => scan_internal(i)(4),
				clock       => clock,
				shift       => shift_wrapper,     
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port_seek_in(i),
				out_failed_test_port	=> '0',
				func_out    => func_out_in_hop_counter(i),
				scan_out    => scan_internal(i)(5) 
				);
	
		wrapper_cell_in_ack_seek_generation:  Entity work.wrapper_cell
			port map(
				func_in     => in_ack_seek(i),
				scan_in     => scan_internal(i)(5),
				clock       => clock,
				shift       => shift_wrapper,     
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port_seek_in(i),
				out_failed_test_port	=> '0',
				func_out    => func_out_in_ack_seek(i),
				scan_out    => scan_internal(i)(6)
				);
		
		wrapper_cell_in_ack_clear_generation:  Entity work.wrapper_cell
			port map(
				func_in     => in_ack_clear(i),
				scan_in     => scan_internal(i)(6),
				clock       => clock,
				shift       => shift_wrapper,     
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port_seek_in(i),
				out_failed_test_port	=> '0',
				func_out    => func_out_in_ack_clear(i),
				scan_out    => scan_internal(i)(7)
				);
	
		wrapper_cell_credit_in_generation_1:  Entity work.wrapper_cell
			generic map(
				faulty_value => '1'
			)
			port map(
				func_in     => credit_out_signal(i*2),
				scan_in     => scan_internal(i)(7),
				clock       => clock,
				shift       => shift_wrapper,      
				normal_mode => normal_mode_wrapper,
				out_failed_test_port	=> '0',
				faulty_port => faulty_port(i*2),
				func_out    => credit_out(i*2),
				scan_out    => scan_internal(i)(8)  
			);
	
		wrapper_cell_credit_in_generation_2:  Entity work.wrapper_cell
			generic map(
				faulty_value => '1'
			)
			port map(
				func_in     => credit_out_signal((i*2)+1),
				scan_in     => scan_internal(i)(8),
				clock       => clock,
				shift       => shift_wrapper,      
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port((i*2)+1),
				out_failed_test_port	=> '0',
				func_out    => credit_out((i*2)+1),
				scan_out    => scan_internal(i)(9)  
			);
				
		wrapper_cell_rx_generation_1:  Entity work.wrapper_cell
			port map(
				func_in     => rx(i*2),
				scan_in     => scan_internal(i)(9),
				clock       => clock,
				shift       => shift_wrapper,      
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port(i*2),
			--	out_failed_test_port	=> '0',
				out_failed_test_port	=> out_failed_test_port(i*2),
				func_out    => func_out_rx(i*2),
				scan_out    => scan_internal(i)(10)   
			);

		wrapper_cell_rx_generation_2:  Entity work.wrapper_cell
			port map(
				func_in     => rx((i*2)+1),
				scan_in     => scan_internal(i)(10),
				clock       => clock,
				shift       => shift_wrapper,      
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port((i*2)+1),
				--out_failed_test_port	=> '0',
				out_failed_test_port	=> out_failed_test_port((i*2)+1),
				func_out    => func_out_rx((i*2)+1),
				scan_out    => scan_internal(i)(11) 
			);
	
		wrapper_cell_eop_in_generation_1:  Entity work.wrapper_cell
			port map(
				func_in     => eop_in(i*2),
				scan_in     => scan_internal(i)(11),
				clock       => clock,
				shift       => shift_wrapper,      
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port(i*2),
				out_failed_test_port	=> out_failed_test_port(i*2),
				func_out    => func_out_eop_in(i*2),
				scan_out    => scan_internal(i)(12) 
			);
	
		wrapper_cell_eop_in_generation_2:  Entity work.wrapper_cell
			port map(
				func_in     => eop_in((i*2)+1),
				scan_in     => scan_internal(i)(12),
				clock       => clock,
				shift       => shift_wrapper,      
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port((i*2)+1),
				out_failed_test_port	=>out_failed_test_port((i*2)+1),
				func_out    => func_out_eop_in((i*2)+1),
				scan_out    => scan_internal(i)(13)
			);

		wrapper_cell_data_in_generation_1:  Entity work.wrapper_cell_N
			generic map(
				N => FLIT_WIDTH
					)		
			port map(
				func_in     			=> data_in(i*2),
				scan_in     			=> scan_internal(i)(13),
				clock       			=> clock,
				shift	    			=> shift_wrapper,      
				normal_mode 			=> normal_mode_wrapper,
				faulty_port 			=> faulty_port(i*2), 
				out_failed_test_port	=> out_failed_test_port(i*2),--fochi			
				func_out    			=> func_out_data_in(i*2),
				scan_out    			=> scan_internal(i)(14)  
			);
			
		wrapper_cell_data_in_generation_2:  Entity work.wrapper_cell_N
			generic map(
				N => FLIT_WIDTH
					)
			port map(
				func_in    				=> data_in((i*2)+1),
				scan_in    				=> scan_internal(i)(14),
				clock      				=> clock,
				shift      				=> shift_wrapper,      
				normal_mode				=> normal_mode_wrapper,
				faulty_port				=> faulty_port((i*2)+1),
				out_failed_test_port	=> out_failed_test_port((i*2)+1),--fochi				
				func_out   				=> func_out_data_in((i*2)+1),
				scan_out   				=> scan_internal(i)(15)  
			);

		wrapper_cell_out_ack_seek_generation:  Entity work.wrapper_cell
			port map(
				func_in     => out_ack_seek_signal(i),
				scan_in     => scan_internal(i)(15),
				clock       => clock,
				shift       => shift_wrapper,      
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port_seek_out(i),
				out_failed_test_port	=> '0',
				func_out    => func_out_out_ack_seek(i),
				scan_out    => scan_internal(i)(16) 
				);		
		
		out_ack_seek <= func_out_out_ack_seek;
		
		wrapper_cell_out_ack_clear_generation:  Entity work.wrapper_cell
			port map(
				func_in     => out_ack_clear_signal(i),
				scan_in     => scan_internal(i)(16),
				clock       => clock,
				shift       => shift_wrapper,    
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port_seek_out(i),
				out_failed_test_port	=> '0',
				func_out    => func_out_out_ack_clear(i),
				scan_out    => scan_internal(i)(17)  
				);
				
		out_ack_clear <= func_out_out_ack_clear;
				
		wrapper_cell_out_seek_generation:  Entity work.wrapper_cell_N
			generic map(
				N => SEEK_BIT_SIZE
					)
			port map(
				func_in     => out_seek_signal(i),
				scan_in     => scan_internal(i)(17),
				clock       => clock,
				shift       => shift_wrapper,     
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port_seek_out(i),
				out_failed_test_port	=> '0',
				func_out    => func_out_out_seek(i),
				scan_out    => scan_internal(i)(18) 
				);
				
		out_seek <= func_out_out_seek;
		
		wrapper_cell_out_clear_generation:  Entity work.wrapper_cell
			port map(
				func_in     => out_clear_signal(i),
				scan_in     => scan_internal(i)(18),
				clock       => clock,
				shift       => shift_wrapper,     
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port_seek_out(i),
				out_failed_test_port	=> '0',
				func_out    => func_out_out_clear(i),
				scan_out    => scan_internal(i)(19) 
				);
				
		out_clear <= func_out_out_clear;
	
		wrapper_cell_out_source_generation:  Entity work.wrapper_cell_N
			generic map(
				N => SOURCE_TARGET_SIZE
				)
			port map(
				func_in     => out_source_signal(i),
				scan_in     => scan_internal(i)(19),
				clock       => clock,
				shift       => shift_wrapper,     
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port_seek_out(i),
				out_failed_test_port	=> '0',
				func_out    => func_out_out_source(i),
				scan_out    => scan_internal(i)(20) 
				);

		out_source <= func_out_out_source;
	
		wrapper_cell_out_target_generation:  Entity work.wrapper_cell_N
			generic map(
				N => SOURCE_TARGET_SIZE
				)
			port map(
				func_in     => out_target_signal(i),
				scan_in     => scan_internal(i)(20),
				clock       => clock,
				shift       => shift_wrapper,     
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port_seek_out(i),
				out_failed_test_port	=> '0',
				func_out    => func_out_out_target(i),
				scan_out    => scan_internal(i)(21) 
				);

		out_target <= func_out_out_target;
	
		wrapper_cell_out_hop_counter_generation:  Entity work.wrapper_cell_N
			generic map(
				N => HOP_SIZE
				)
			port map(
				func_in     => out_hop_counter_signal(i),
				scan_in     => scan_internal(i)(21),
				clock       => clock,
				shift       => shift_wrapper,     
				normal_mode => normal_mode_wrapper,
				faulty_port => faulty_port_seek_out(i),
				out_failed_test_port	=> '0',
				func_out    => func_out_out_hop_counter(i),
				scan_out    => scan_internal(i)(22)
				);
				
		out_hop_counter <= func_out_out_hop_counter;
				
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
				out_failed_test_port	=> '0',
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
				out_failed_test_port	=> '0',
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
				out_failed_test_port	=> '0',
				func_out    => func_out_data_write,
				scan_out    => scan_out_data_write
				);
	
	data_write <= func_out_data_write;
			
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
				out_failed_test_port	=> '0',
				func_out    => func_out_write_byte_enable,
				scan_out    => scan_out_write_byte_enable
				);
	
	write_byte_enable <= func_out_write_byte_enable;
				
	wrapper_cell_write_enable_debug_generation:  Entity work.wrapper_cell
			port map(
				func_in     => write_enable_debug_signal,
				scan_in     => scan_out_write_byte_enable,
				clock       => clock,
				shift       => shift_Memory_Debug,      
				normal_mode => normal_mode_Memory_Debug,
				faulty_port => faulty_port_Memory_Debug,
				out_failed_test_port	=> '0',
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
				out_failed_test_port	=> '0',
				func_out    => func_out_data_out_debug,
				scan_out    => scan_out_data_out_debug
				);
	
	data_out_debug <= func_out_data_out_debug;
				
	wrapper_cell_ack_task_generation:  Entity work.wrapper_cell
			port map(
				func_in     => ack_task_signal,
				scan_in     => scan_out_data_out_debug,
				clock       => clock,
				shift       => shift_Memory_Debug,      
				normal_mode => normal_mode_Memory_Debug,
				faulty_port => faulty_port_Memory_Debug,
				out_failed_test_port	=> '0',
				func_out    => func_out_ack_task,
				scan_out    => scan_out_Memory_Debug
				);
				
	ack_task <= func_out_ack_task;
	
end processing_element_wrapped;
	
