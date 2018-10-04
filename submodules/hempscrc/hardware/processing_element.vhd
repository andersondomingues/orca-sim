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
                in_failed_port          : in  regNport_neighbor;
                out_failed_port         : out regNport_neighbor;
                out_failed_test_port 	: out regNport_neighbor; -- fochi test_port_receive

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
end processing_element;

architecture processing_element of processing_element is

        -- NoC Interface
        
        signal rx_router                : regNport;
        signal eop_in_router            : regNport;
        signal data_in_router           : arrayNport_regflit;
        signal credit_out_router        : regNport;
            
        signal tx_router                : regNport;
        signal eop_out_router           : regNport;
        signal eop_out_router_test      : regNport;
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
        
        signal in_seek_router           : regNport_seek_NBit;
        signal out_ack_seek_router      : regNport_seek;
        signal in_clear_router          : regNport_seek;
        signal out_ack_clear_router     : regNport_seek;
        signal in_source_router         : regNportNsource_target;
        signal in_target_router         : regNportNsource_target;
        signal in_hop_counter_router    : regNportNhop;
        signal out_seek_router          : regNport_seek_NBit;
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
        signal out_seek_pe              : regNBit_seek;        
        signal in_ack_seek_pe           : std_logic;        
        signal out_clear_pe             : std_logic;        
        signal in_ack_clear_pe          : std_logic;        
        signal out_source_pe            : regNsource_target;
        signal out_target_pe            : regNsource_target;
        signal out_hop_counter_pe       : regNhop;
        
        --detect fail
        signal out_source_fail_detect       : regNport_neighborNsource_target_neighbor; 
        signal out_target_fail_detect       : regNport_neighborNsource_target_neighbor;
        signal out_seek_fail_detect         : regNport_regNBit_seek;  
        
        --seek manager signals
        signal out_seek_manager             : regNBit_seek;
        signal in_ack_seek_manager          : std_logic;
        signal out_clear_manager            : std_logic;
        signal in_ack_clear_manager         : std_logic;
        signal out_source_seek_manager      : regNport_neighbor;
        signal out_target_seek_manager      : regNport_neighbor;
		
		-- test_port signals
		signal out_test_port		        : regNport_neighbor; -- fochi 31/10
		signal out_test_port_recv	        : regNport_neighbor; -- fochi 05/11
		signal out_failed_port_int_router	: regNport_neighbor;
		signal in_failed_port_test			: regNport_neighbor;
		signal out_reset_stepdet			: regNport_neighbor;
		signal data_out_test             	: regflit;
		signal eop_out_test            		: std_logic;
		
        signal sys_int_i                : std_logic;
        
        --not reset
        signal reset_n                  : std_logic;
        signal 	rx_out					:std_logic;
        signal failed_reception         : std_logic;

        signal seek_unreachable_interrupt      : std_logic;
        signal seek_unreachable_target         : regNsource_target;
		signal seek_resend_interrupt  		   :  std_logic;

        signal ack_seek_fail_detect     : regNport_neighbor;
begin

-------------------
-------------------
--  fail detect  --
-------------------
-------------------
	generation_of_fail_detect: for i in 0 to 7 generate
		HeMPS_fail_detect: Entity work.fail_detect
			port map(
				clock          	        => clock,        
				reset          	        => reset,
				        
				in_failed_port 	        => in_failed_port(i),
				tx             	        => tx_router(i),
				in_eop_in      	        => eop_out_router(i),
				in_data_in		        => data_out_router(i),
                in_ack_seek_fail_detect => ack_seek_fail_detect(i),
				
				out_source	  			=> out_source_fail_detect(i), 
				out_target	            => out_target_fail_detect(i),  
				out_seek_fail_detect    => out_seek_fail_detect(i) 				
			);
	end generate;

	
--------------------
--------------------
--  seek manager  --
--------------------
--------------------
		HeMPS_seek_manager: Entity work.seek_manager
			port map(
                clock       			=> clock, 
                reset					=> reset,

                in_seek_pe				=> out_seek_pe,
                out_ack_seek_pe         => in_ack_seek_pe,
                in_clear_pe             => out_clear_pe,
                out_ack_clear_pe        => in_ack_clear_pe,
                in_source_pe   			=> out_source_pe,
                in_target_pe   			=> out_target_pe,

                in_seek_fail_detect   	=> out_seek_fail_detect,
                in_source_fail_detect   => out_source_fail_detect,
                in_target_fail_detect   => out_target_fail_detect,
                out_ack_seek_fail_detect=> ack_seek_fail_detect,

                out_seek_manager        => out_seek_manager,
                in_ack_seek_manager     => in_ack_seek_manager,
                out_clear_manager       => out_clear_manager,
                in_ack_clear_manager    => in_ack_clear_manager,
                out_source_seek_manager => out_source_seek_manager,
                out_target_seek_manager => out_target_seek_manager

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
			clock                       => clock,
			reset                       => reset,
   
			rx                          => rx_router,
			eop_in                      => eop_in_router,
			data_in                     => data_in_router,
			credit_out                  => credit_out_router,

			tx                          => tx_router,
			eop_out                     => eop_out_router,
			data_out                    => data_out_router,
			credit_in                   => credit_in_router,

            reset_seek                  => reset_seek,
            
            in_seek                     => in_seek_router,
            out_ack_seek                => out_ack_seek_router,
            in_clear                    => in_clear_router,
            out_ack_clear               => out_ack_clear_router,
            in_source                   => in_source_router,
            in_target                   => in_target_router,
            in_hop_counter              => in_hop_counter_router,

            seek_unreachable_interrupt  => seek_unreachable_interrupt,
            seek_unreachable_target     => seek_unreachable_target,
            seek_resend_interrupt 		=> seek_resend_interrupt,
            
            out_seek                    => out_seek_router,
            in_ack_seek                 => in_ack_seek_router,
            out_clear                   => out_clear_router,
            in_ack_clear                => in_ack_clear_router,
            out_source                  => out_source_router,
            out_target                  => out_target_router,
            out_hop_counter             => out_hop_counter_router,
            failed_reception            => failed_reception,
            --out_failed_port             => out_failed_port,
            out_failed_port_test        => in_failed_port_test,
            out_failed_port             => out_failed_port_int_router, -- fochi sinal out_failed_port_int_router dentro do PE. vai para o test_port_receive	
            out_reset_stepdet			=> out_reset_stepdet,
            in_failed_port         		=> in_failed_port,
            eop_out_test_recv			=> eop_out_router_test
		);

		
-------------------
-------------------
--end  Router  CC--
-------------------
-------------------

--------------------
--------------------
--  test port  --
--------------------
--------------------
	HeMPS_test_port: Entity work.test_port
			port map(
				clock          	        => clock,        
				reset          	        => reset,
				data_out_test           => data_out_test,
				eop_out_test            => eop_out_test,
				in_failed_port 	        => in_failed_port,
				out_test_port			=> out_test_port,
				rx_out					=> rx_out
			);
			
-----------------------
-----------------------
--  test port receive --
-----------------------
-----------------------
	HeMPS_test_port_recv_East0: Entity work.test_port_receive
			port map(
				clock          	        => clock,        
				reset          	        => reset,
				data_in                 => data_in_router(EAST0),
				in_failed_port_test		=> in_failed_port_test(EAST0),
				eop_in_test_recv      	=> eop_out_router_test(EAST0),
				out_reset_stepdet		=> out_reset_stepdet(EAST0),
				in_failed_port 	        => out_failed_port_int_router(EAST0),
				out_test_port_recv		=> out_failed_test_port(EAST0) --vai para o PE_W -> e os wrapper_cells
			);

	HeMPS_test_port_recv_East1: Entity work.test_port_receive
			port map(
				clock          	        => clock,        
				reset          	        => reset,
				data_in                 => data_in_router(EAST1),
				in_failed_port_test		=> in_failed_port_test(EAST1),
				eop_in_test_recv      	=> eop_out_router_test(EAST1),
				out_reset_stepdet		=> out_reset_stepdet(EAST1),
				in_failed_port 	        => out_failed_port_int_router(EAST1),
				out_test_port_recv		=> out_failed_test_port(EAST1) --vai para o PE_W -> e os wrapper_cells
			);

	HeMPS_test_port_recv_West1: Entity work.test_port_receive
			port map(
				clock          	        => clock,        
				reset          	        => reset,
				data_in                 => data_in_router(WEST1),
				in_failed_port_test		=> in_failed_port_test(WEST1),
				eop_in_test_recv      	=> eop_out_router_test(WEST1),
				out_reset_stepdet		=> out_reset_stepdet(WEST1),
				in_failed_port 	        => out_failed_port_int_router(WEST1),
				out_test_port_recv		=> out_failed_test_port(WEST1) --vai para o PE_W -> e os wrapper_cells
			);

	HeMPS_test_port_recv_West0: Entity work.test_port_receive
			port map(
				clock          	        => clock,        
				reset          	        => reset,
				data_in                 => data_in_router(WEST0),
				in_failed_port_test		=> in_failed_port_test(WEST0),
				eop_in_test_recv      	=> eop_out_router_test(WEST0),
				out_reset_stepdet		=> out_reset_stepdet(WEST0),
				in_failed_port 	        => out_failed_port_int_router(WEST0),
				out_test_port_recv		=> out_failed_test_port(WEST0) --vai para o PE_W -> e os wrapper_cells
			);	


	HeMPS_test_port_recv_North0: Entity work.test_port_receive
			port map(
				clock          	        => clock,        
				reset          	        => reset,
				data_in                 => data_in_router(NORTH0),
				in_failed_port_test		=> in_failed_port_test(NORTH0),
				eop_in_test_recv      	=> eop_out_router_test(NORTH0),
				out_reset_stepdet		=> out_reset_stepdet(NORTH0),
				in_failed_port 	        => out_failed_port_int_router(NORTH0),
				out_test_port_recv		=> out_failed_test_port(NORTH0) --vai para o PE_W -> e os wrapper_cells
			);

	HeMPS_test_port_recv_North1: Entity work.test_port_receive
			port map(
				clock          	        => clock,        
				reset          	        => reset,
				data_in                 => data_in_router(NORTH1),
				in_failed_port_test		=> in_failed_port_test(NORTH1),
				eop_in_test_recv      	=> eop_out_router_test(NORTH1),
				out_reset_stepdet		=> out_reset_stepdet(NORTH1),
				in_failed_port 	        => out_failed_port_int_router(NORTH1),
				out_test_port_recv		=> out_failed_test_port(NORTH1) --vai para o PE_W -> e os wrapper_cells
			);

	HeMPS_test_port_recv_South1: Entity work.test_port_receive
			port map(
				clock          	        => clock,        
				reset          	        => reset,
				data_in                 => data_in_router(SOUTH1),
				in_failed_port_test		=> in_failed_port_test(SOUTH1),
				eop_in_test_recv      	=> eop_in(SOUTH1),
				out_reset_stepdet		=> out_reset_stepdet(SOUTH1),
				in_failed_port 	        => out_failed_port_int_router(SOUTH1),
				out_test_port_recv		=> out_failed_test_port(SOUTH1) --vai para o PE_W -> e os wrapper_cells
			);

	HeMPS_test_port_recv_South0: Entity work.test_port_receive
			port map(
				clock          	        => clock,        
				reset          	        => reset,
				data_in                 => data_in_router(SOUTH0),
				in_failed_port_test		=> in_failed_port_test(SOUTH0),
				eop_in_test_recv      	=> eop_out_router_test(SOUTH0),
				out_reset_stepdet		=> out_reset_stepdet(SOUTH0),
				in_failed_port 	        => out_failed_port_int_router(SOUTH0),
				out_test_port_recv		=> out_failed_test_port(SOUTH0) --vai para o PE_W -> e os wrapper_cells
			);	
		
			
		out_failed_port <= out_failed_port_int_router; -- out_failed_port do PE recebe o sinal do router.

        connection_neighbor: for i in 0 to 7 generate
            rx_router(i)                <= rx(i);-- test_port	
            eop_in_router(i)            <= eop_in(i);
            data_in_router(i)           <= data_in(i);
            credit_out(i)               <= credit_out_router(i);
                                        
            tx(i)                       <= tx_router(i) when out_test_port(i)= '0' else rx_out;
            eop_out(i)                  <= eop_out_router(i) when out_test_port(i)= '0' else eop_out_test; -- test_port		
            data_out(i)                 <= data_out_router(i) when out_test_port(i)= '0' else data_out_test; -- test_port
            credit_in_router(i)         <= credit_in(i);            
        end generate;
        
        connection_seek_neighbor: for i in 0 to 3 generate
            in_seek_router(i)           <= in_seek(i);
            out_ack_seek(i)             <= out_ack_seek_router(i);
            in_clear_router(i)          <= in_clear(i);
            out_ack_clear(i)            <= out_ack_clear_router(i);
            in_source_router(i)         <= in_source(i);
            in_target_router(i)         <= in_target(i);
            in_hop_counter_router(i)    <= in_hop_counter(i);
                
            out_seek(i)                 <= out_seek_router(i);
            in_ack_seek_router(i)       <= in_ack_seek(i);
            out_clear(i)                <= out_clear_router(i);
            in_ack_clear_router(i)      <= in_ack_clear(i);
            out_source(i)               <= out_source_router(i);
            out_target(i)               <= out_target_router(i);
            out_hop_counter(i)          <= out_hop_counter_router(i);
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
        
        
        in_seek_router(4)           <= out_seek_manager;        
		in_ack_seek_manager         <= out_ack_seek_router(4);
        in_clear_router(4)          <= out_clear_manager;
        in_ack_clear_manager        <= out_ack_clear_router(4);
        in_source_router(4)         <= out_source_seek_manager;
        in_target_router(4)         <= out_target_seek_manager;
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
                                data_out0                       => data_out0_pe(15 downto 0),
                                eop_out0	                    => eop_out0_pe,
                                credit_i0                       => credit_i0_pe,
                                rx0                             => rx0_pe,
                                data_in0                        => data_in0_pe(15 downto 0),
                                eop_in0		                    => eop_in0_pe,
                                credit_o0	                    => credit_o0_pe,

                                -- NoC interface (Local port 1) 
                                tx1                             => tx1_pe,
                                data_out1                       => data_out1_pe(15 downto 0),
                                eop_out1	                    => eop_out1_pe,
                                credit_i1                       => credit_i1_pe,
                                rx1                             => rx1_pe,
                                data_in1                        => data_in1_pe(15 downto 0),
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
        
                                address                         => address,
                                read_req                        => read_req, 
                                data_write                      => data_write,
                                data_read                       => data_read,
                                write_byte_enable               => write_byte_enable,
                                data_valid                      => data_valid,

                                write_enable_debug              => write_enable_debug,
                                data_out_debug                  => data_out_debug,
                                busy_debug                      => busy_debug,
                                
                                ack_task                        => ack_task,
                                req_task                        => req_task,
                                NI_failed_reception             => failed_reception,

                                seek_unreachable_interrupt      => seek_unreachable_interrupt,
                                seek_resend_interrupt 			=> seek_resend_interrupt,
                                seek_unreachable_target         => seek_unreachable_target
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
-------------------
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
