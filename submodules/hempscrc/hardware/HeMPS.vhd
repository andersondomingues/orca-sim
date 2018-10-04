------------------------------------------------------------------
---
--- HEMPS 4.0  - October 25 2011
---
--- File function: generates a HeMPS instance according to parameters defined in HeMPS_PKG
---
--- Responsibles: Eduardo Wachter, Marcelo Mandelli, Carlo Lucas, Fernando Moraes
--- 
--- Contact: fernando.moraes@pucrs.br
---
------------------------------------------------------------------

library IEEE;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use work.HeMPS_PKG.all;
use work.HeMPS_defaults.all;

entity HeMPS is
        generic (
            mlite_description   : string := "RTL";
            router_description  : string := "RTL";
            ram_description     : string := "RTL";
            router_address		: regmetadeflit := "00010000"
        );
        port (
                clock                           : in std_logic;
                reset                           : in std_logic;

                -- Tasks repository interface
                read_req                        : out std_logic;
                mem_addr                        : out std_logic_vector(29 downto 0);
                data_read                       : in  std_logic_vector(31 downto 0);
                data_valid                      : in  std_logic;

                -- Debug interface
                write_enable_debug              : out std_logic;
                data_out_debug                  : out std_logic_vector(31 downto 0);
                busy_debug                      : in  std_logic;
                
                ack_task                        : out std_logic;
                req_task                        : in  std_logic_vector(31 downto 0);
                
                -------------router seek_pulse
                
                in_source_router_seek_local   		:	in	regNsource_target;
                in_target_router_seek_local   		:	in	regNsource_target;
                in_hop_router_seek_local			:	in	regNhop;
                in_service_router_seek_local		:	in 	seek_bitN_service;
                in_req_router_seek_local			:	in	std_logic;
               
                in_source_router_seek_local2   		:	in	regNsource_target;
                in_target_router_seek_local2   		:	in	regNsource_target;
                in_hop_router_seek_local2			:	in	regNhop;
                in_service_router_seek_local2		:	in 	seek_bitN_service;
                in_req_router_seek_local2			:	in	std_logic;
                
                in_source_router_seek_local3   		:	in	regNsource_target;
                in_target_router_seek_local3   		:	in	regNsource_target;
                in_hop_router_seek_local3			:	in	regNhop;
                in_service_router_seek_local3		:	in 	seek_bitN_service;
                in_req_router_seek_local3			:	in	std_logic;
                
                in_source_router_seek_local4   		:	in	regNsource_target;
                in_target_router_seek_local4   		:	in	regNsource_target;
                in_hop_router_seek_local4			:	in	regNhop;
                in_service_router_seek_local4		:	in 	seek_bitN_service;
                in_req_router_seek_local4			:	in	std_logic
                
                              
                
        );                                       
end;

architecture HeMPS of HeMPS is  

        -- Interconnection signals 
        type regNport_neighbor_Nrot is array(NUMBER_PROCESSORS-1 downto 0) of regNport_neighbor;
        type arrayNport_regflit_neighbor_Nrot is array(NUMBER_PROCESSORS-1 downto 0) of arrayNport_regflit_neighbor;
        
        signal rx			            : regNport_neighbor_Nrot;
        signal eop_in		            : regNport_neighbor_Nrot;
        signal data_in		            : arrayNport_regflit_neighbor_Nrot;
        signal credit_out	            : regNport_neighbor_Nrot;
        
        signal tx			            : regNport_neighbor_Nrot;
        signal eop_out		            : regNport_neighbor_Nrot;
        signal data_out	                : arrayNport_regflit_neighbor_Nrot;
        signal credit_in	            : regNport_neighbor_Nrot;
        
        type regNport_seek_neighbor_Nrot is array(NUMBER_PROCESSORS-1 downto 0) of regNport_seek_neighbor;
        type regNport_seek_NBit_neighbor_Nrot is array(NUMBER_PROCESSORS-1 downto 0) of regNport_seek_neighbor_NBit;
        type regNportNhop_neighbor_Nrot is array(NUMBER_PROCESSORS-1 downto 0) of regNportNhop_neighbor;
        type regNportNsource_target_neighbor_Nrot is array(NUMBER_PROCESSORS-1 downto 0) of regNportNsource_target_neighbor;
        
        -- new seek
      
        --
        
        signal in_seek                  : regNport_seek_NBit_neighbor_Nrot;
        signal out_ack_seek             : regNport_seek_neighbor_Nrot;
        signal out_nack_seek             : regNport_seek_neighbor_Nrot;
        signal in_clear                 : regNport_seek_neighbor_Nrot;
        signal out_ack_clear            : regNport_seek_neighbor_Nrot;
        signal in_source                : regNportNsource_target_neighbor_Nrot;
        signal in_target                : regNportNsource_target_neighbor_Nrot;
        signal in_hop_counter           : regNportNhop_neighbor_Nrot;
        
        signal out_seek                 : regNport_seek_NBit_neighbor_Nrot;
        signal in_ack_seek              : regNport_seek_neighbor_Nrot;
        signal in_nack_seek             : regNport_seek_neighbor_Nrot;        
        signal out_clear                : regNport_seek_neighbor_Nrot;
        signal in_ack_clear             : regNport_seek_neighbor_Nrot;
        signal out_source               : regNportNsource_target_neighbor_Nrot;
        signal out_target               : regNportNsource_target_neighbor_Nrot;
        signal out_hop_counter          : regNportNhop_neighbor_Nrot;
        
          
         
	   
       
       signal in_source_router_seek	: regNportNsource_target_neighbor_Nrot_new;
       signal in_target_router_seek 	: regNportNsource_target_neighbor_Nrot_new;
       signal out_source_router_seek	: regNportNsource_target_neighbor_Nrot_new;
       signal out_target_router_seek	: regNportNsource_target_neighbor_Nrot_new;
       signal in_hop_router_seek		: regNportNhop_neighbor_Nrot_new;
       signal in_service_router_seek	: regNportNservice_Nrot_new;
       signal in_req_router_seek		: service_req_seek;
       signal in_ack_router_seek		: service_ack_seek;
       signal in_nack_router_seek		: service_ack_seek;
       signal in_fail_router_seek		: service_ack_seek;
       signal out_fail_router_seek		: service_ack_seek;
       signal out_req_router_seek		: service_req_seek;
       signal out_ack_router_seek		: service_ack_seek;
       signal out_nack_router_seek		: service_ack_seek;
       signal out_service_router_seek	: regNportNservice_Nrot_new;
       signal out_hop_router_seek		: regNportNhop_neighbor_Nrot_new;
       



        signal   address_router : std_logic_vector(7 downto 0);
        
        type router_position is array(NUMBER_PROCESSORS-1 downto 0) of integer range 0 to TR;
        signal position : router_position;
        
        signal reset_PE                 : std_logic_vector(NUMBER_PROCESSORS-1 downto 0);
        signal reset_router_seek                 : std_logic_vector(NUMBER_PROCESSORS-1 downto 0);
        
        --failed port signals added by wachter
        signal out_failed_port          : regNport_neighbor_Nrot;
        signal in_failed_port           : regNport_neighbor_Nrot;
        
        begin
        
        core_type_gen:   for i in 0 to NUMBER_PROCESSORS-1 generate
                position(i) <= RouterPosition(i);
                reset_PE(i) <= reset;
                reset_router_seek(i) <= reset;
        end generate core_type_gen;
        
        
        proc: for i in 0 to NUMBER_PROCESSORS-1 generate
                --mas:if (MASTER_ADDRESS = i) generate     -- MASTER_ADDRESS defined in the package
                        --master: entity work.processing_element_wrapped
                        --generic map (
                                --memory_type             => "TRI",
                                --router_address          => RouterAddress(i),
                                --processor_type          => "mas",
                                --core_type               => core_type(i),
                                --mlite_description       => mlite_description,
                                --router_description      => router_description,
                                --ram_description         => ram_description
                                --)
                        --port map(
                                --clock                   => clock,
                                --reset                   => reset_PE(i),
                                
                                --rx		                => rx(i),
                                --eop_in		            => eop_in(i),
                                --data_in		            => data_in(i),
                                --credit_out	            => credit_out(i),
                                                        
                                --tx			            => tx(i),
                                --eop_out		            => eop_out(i),
                                --data_out	            => data_out(i),
                                --credit_in	            => credit_in(i),
                                
                                --reset_seek              => reset_PE(i),
                                
                                --in_seek                 => in_seek(i),
                                --out_ack_seek            => out_ack_seek(i),
                                --in_clear                => in_clear(i),
                                --out_ack_clear           => out_ack_clear(i),
                                --in_source               => in_source(i),
                                --in_target               => in_target(i),
                                --in_hop_counter          => in_hop_counter(i),
                                
                                --out_seek                => out_seek(i),
                                --in_ack_seek             => in_ack_seek(i),
                                --out_clear               => out_clear(i),
                                --in_ack_clear            => in_ack_clear(i),
                                --out_source              => out_source(i),
                                --out_target              => out_target(i),
                                --out_hop_counter         => out_hop_counter(i),
                                                                
                                --out_failed_port         => out_failed_port(i),
                                --in_failed_port          => in_failed_port(i),

                                --address                 => mem_addr,
                                --read_req                => read_req,
                                --data_write              => open,
                                --data_read               => data_read,
                                --write_byte_enable       => open,
                                --data_valid              => data_valid,

                                --write_enable_debug      => write_enable_debug,
                                --data_out_debug          => data_out_debug,
                                --busy_debug              => busy_debug,
                                
                                --ack_task                => ack_task,
                                --req_task                => req_task
                        --);         
                --end generate mas;

                --slav:if (i /= MASTER_ADDRESS) generate
                        --slave: entity work.processing_element_wrapped
                        --generic map (
                                --memory_type             => "TRI",
                                --router_address          => RouterAddress(i),
                                --processor_type          => "sla",
                                --core_type               => core_type(i),
                                --mlite_description       => mlite_description,
                                --router_description      => router_description,
                                --ram_description         => ram_description,
                                --log_file                => log_filename(i)
                                --)
                        --port map(
                                --clock                   => clock,
                                --reset                   => reset_PE(i),
                                
                                --rx		                => rx(i),
                                --eop_in		            => eop_in(i),
                                --data_in		            => data_in(i),
                                --credit_out	            => credit_out(i),
                                                        
                                --tx			            => tx(i),
                                --eop_out		            => eop_out(i),
                                --data_out	            => data_out(i),
                                --credit_in	            => credit_in(i),

                                --reset_seek              => reset_PE(i),
                                
                                --in_seek                 => in_seek(i),
                                --out_ack_seek            => out_ack_seek(i),
                                --in_clear                => in_clear(i),
                                --out_ack_clear           => out_ack_clear(i),
                                --in_source               => in_source(i),
                                --in_target               => in_target(i),
                                --in_hop_counter          => in_hop_counter(i),
                                
                                --out_seek                => out_seek(i),
                                --in_ack_seek             => in_ack_seek(i),
                                --out_clear               => out_clear(i),
                                --in_ack_clear            => in_ack_clear(i),
                                --out_source              => out_source(i),
                                --out_target              => out_target(i),
                                --out_hop_counter         => out_hop_counter(i),
                                                                
                                --out_failed_port         => out_failed_port(i),
                                --in_failed_port          => in_failed_port(i),
                                
                                --address                 => open,
                                --data_write              => open,
                                --data_read               => (others => '0'),
                                --write_byte_enable       => open,
                                --data_valid              => '0',

                                --write_enable_debug      => open,
                                --data_out_debug          => open,
                                --busy_debug              => '0',
                                
                                --ack_task                => open,
                                --req_task                => (others=>'0')
                        --);
                --end generate slav;       
               
               seek:if ((i /= MASTER_ADDRESS) or (MASTER_ADDRESS = i))  generate 
               
                        seek: entity work.router_seek
						generic map (
                                router_address          => RouterAddress(i)
                                    )
                        port map(
                        
                        clock						=>	clock,
						reset						=>	reset_router_seek(i),
						in_source_router_seek		=>	in_source_router_seek(i),--
						in_target_router_seek  		=>	in_target_router_seek(i),--
						in_hop_router_seek			=>	in_hop_router_seek(i),	--	
						in_service_router_seek		=>	in_service_router_seek(i),--	
						in_req_router_seek			=>	in_req_router_seek(i),		
						in_ack_router_seek			=>	in_ack_router_seek(i),	
						in_nack_router_seek			=>	in_nack_router_seek(i),							
						in_fail_router_seek			=>	in_fail_router_seek(i),						
						out_req_router_seek			=>	out_req_router_seek(i),		
						out_ack_router_seek			=>	out_ack_router_seek(i),	
						out_nack_router_seek		=>	out_nack_router_seek(i),	
						out_service_router_seek		=>	out_service_router_seek(i),--	
						out_source_router_seek 		=>  out_source_router_seek(i),-- 	
						out_target_router_seek 		=>  out_target_router_seek(i), --	
						out_hop_router_seek			=>	out_hop_router_seek(i)	--	
                               
                        );
                        
                        
				end generate seek;             
---
--- 
--- interconnects 

--0 to 49
--21 to 58
--71 to 48
--90 to 59

---Conexões do test_bench para as portas locais dos roteadores. Conforme o indice.
in_source_router_seek(0)(LOCAL)	<= in_source_router_seek_local ;
in_target_router_seek(0)(LOCAL)	<= in_target_router_seek_local ;
in_hop_router_seek(0)(LOCAL)	<= in_hop_router_seek_local	   ;
in_service_router_seek(0)(LOCAL)<= in_service_router_seek_local;
in_req_router_seek(0)(LOCAL)	<= in_req_router_seek_local	   ;

in_source_router_seek(3)(LOCAL)	<= in_source_router_seek_local4;
in_target_router_seek(3)(LOCAL)	<= in_target_router_seek_local4;
in_hop_router_seek(3)(LOCAL)	<= in_hop_router_seek_local4   ;
in_service_router_seek(3)(LOCAL)<= in_service_router_seek_local4;
in_req_router_seek(3)(LOCAL)	<= in_req_router_seek_local4	   ;

in_source_router_seek(12)(LOCAL)	<= in_source_router_seek_local2;
in_target_router_seek(12)(LOCAL)	<= in_target_router_seek_local2;
in_hop_router_seek(12)(LOCAL)		<= in_hop_router_seek_local2   ;
in_service_router_seek(12)(LOCAL)	<= in_service_router_seek_local2;
in_req_router_seek(12)(LOCAL)		<= in_req_router_seek_local2	   ;

in_source_router_seek(15)(LOCAL)	<= in_source_router_seek_local3;
in_target_router_seek(15)(LOCAL)	<= in_target_router_seek_local3;
in_hop_router_seek(15)(LOCAL)	<= in_hop_router_seek_local3   ;
in_service_router_seek(15)(LOCAL)<= in_service_router_seek_local3;
in_req_router_seek(15)(LOCAL)	<= in_req_router_seek_local3	;

in_ack_router_seek(0)(LOCAL) <= '1' when out_req_router_seek(0)(LOCAL) ='1' else '0' ;
in_ack_router_seek(3)(LOCAL) <= '1' when out_req_router_seek(3)(LOCAL) ='1' else '0' ;
in_ack_router_seek(12)(LOCAL) <= '1' when out_req_router_seek(12)(LOCAL) ='1' else '0' ;
in_ack_router_seek(15)(LOCAL) <= '1' when out_req_router_seek(15)(LOCAL) ='1' else '0' ;

--in_req_router_seek(0)(LOCAL) <= '0';
in_req_router_seek(1)(LOCAL) <= '0';
in_req_router_seek(2)(LOCAL) <= '0';
--in_req_router_seek(3)(LOCAL) <= '0';
in_req_router_seek(4)(LOCAL) <= '0';
in_req_router_seek(4)(LOCAL) <= '0';
in_req_router_seek(5)(LOCAL) <= '0';
in_req_router_seek(6)(LOCAL) <= '0';
in_req_router_seek(7)(LOCAL) <= '0';
in_req_router_seek(8)(LOCAL) <= '0';
in_req_router_seek(9)(LOCAL) <= '0';
in_req_router_seek(10)(LOCAL) <= '0';
in_req_router_seek(11)(LOCAL) <= '0';
--in_req_router_seek(12)(LOCAL) <= '0';
in_req_router_seek(13)(LOCAL) <= '0';
in_req_router_seek(14)(LOCAL) <= '0';
--in_req_router_seek(15)(LOCAL) <= '0';

--in_fail_router_seek(i) <= (others => '0');
in_fail_router_seek(i) <= "11111" when i=9 or i=10 or i=11  else (others => '0');

-------------------------------------------------------------------------------------------- para noc 10x10

--in_source_router_seek(0)(LOCAL)	<= in_source_router_seek_local ;
--in_target_router_seek(0)(LOCAL)	<= in_target_router_seek_local ;
--in_hop_router_seek(0)(LOCAL)	<= in_hop_router_seek_local	   ;
--in_service_router_seek(0)(LOCAL)<= in_service_router_seek_local;
--in_req_router_seek(0)(LOCAL)	<= in_req_router_seek_local	   ;
--
--in_source_router_seek(21)(LOCAL)	<= in_source_router_seek_local2;
--in_target_router_seek(21)(LOCAL)	<= in_target_router_seek_local2;
--in_hop_router_seek(21)(LOCAL)	<= in_hop_router_seek_local2   ;
--in_service_router_seek(21)(LOCAL)<= in_service_router_seek_local2;
--in_req_router_seek(21)(LOCAL)	<= in_req_router_seek_local2	   ;
--
--in_source_router_seek(71)(LOCAL)	<= in_source_router_seek_local3;
--in_target_router_seek(71)(LOCAL)	<= in_target_router_seek_local3;
--in_hop_router_seek(71)(LOCAL)	<= in_hop_router_seek_local3   ;
--in_service_router_seek(71)(LOCAL)<= in_service_router_seek_local3;
--in_req_router_seek(71)(LOCAL)	<= in_req_router_seek_local3	   ;
--
--in_source_router_seek(90)(LOCAL)	<= in_source_router_seek_local4;
--in_target_router_seek(90)(LOCAL)	<= in_target_router_seek_local4;
--in_hop_router_seek(90)(LOCAL)	<= in_hop_router_seek_local4   ;
--in_service_router_seek(90)(LOCAL)<= in_service_router_seek_local4;
--in_req_router_seek(90)(LOCAL)	<= in_req_router_seek_local4	   ;
--
--in_ack_router_seek(0)(LOCAL) <= '1' when out_req_router_seek(0)(LOCAL) ='1' else '0' ;
--in_ack_router_seek(21)(LOCAL) <= '1' when out_req_router_seek(21)(LOCAL) ='1' else '0' ;
--in_ack_router_seek(71)(LOCAL) <= '1' when out_req_router_seek(71)(LOCAL) ='1' else '0' ;
--in_ack_router_seek(90)(LOCAL) <= '1' when out_req_router_seek(90)(LOCAL) ='1' else '0' ;
--
----in_req_router_seek(0)(LOCAL) <= '0';
--in_req_router_seek(1)(LOCAL) <= '0';
--in_req_router_seek(2)(LOCAL) <= '0';
--in_req_router_seek(3)(LOCAL) <= '0';
--in_req_router_seek(4)(LOCAL) <= '0';
--in_req_router_seek(4)(LOCAL) <= '0';
--in_req_router_seek(5)(LOCAL) <= '0';
--in_req_router_seek(6)(LOCAL) <= '0';
--in_req_router_seek(7)(LOCAL) <= '0';
--in_req_router_seek(8)(LOCAL) <= '0';
--in_req_router_seek(9)(LOCAL) <= '0';
--in_req_router_seek(10)(LOCAL) <= '0';
--in_req_router_seek(11)(LOCAL) <= '0';
--in_req_router_seek(12)(LOCAL) <= '0';
--in_req_router_seek(13)(LOCAL) <= '0';
--in_req_router_seek(14)(LOCAL) <= '0';
--in_req_router_seek(15)(LOCAL) <= '0';
--in_req_router_seek(16)(LOCAL) <= '0';
--in_req_router_seek(17)(LOCAL) <= '0';
--in_req_router_seek(18)(LOCAL) <= '0';
--in_req_router_seek(19)(LOCAL) <= '0';
--in_req_router_seek(20)(LOCAL) <= '0';
----in_req_router_seek(21)(LOCAL) <= '0';
--in_req_router_seek(22)(LOCAL) <= '0';
--in_req_router_seek(23)(LOCAL) <= '0';
--in_req_router_seek(24)(LOCAL) <= '0';
--in_req_router_seek(25)(LOCAL) <= '0';
--in_req_router_seek(26)(LOCAL) <= '0';
--in_req_router_seek(27)(LOCAL) <= '0';
--in_req_router_seek(28)(LOCAL) <= '0';
--in_req_router_seek(29)(LOCAL) <= '0';
--in_req_router_seek(30)(LOCAL) <= '0';
--in_req_router_seek(31)(LOCAL) <= '0';
--in_req_router_seek(32)(LOCAL) <= '0';
--in_req_router_seek(33)(LOCAL) <= '0';
--in_req_router_seek(34)(LOCAL) <= '0';
--in_req_router_seek(35)(LOCAL) <= '0';
--in_req_router_seek(36)(LOCAL) <= '0';
--in_req_router_seek(37)(LOCAL) <= '0';
--in_req_router_seek(38)(LOCAL) <= '0';
--in_req_router_seek(39)(LOCAL) <= '0';
--in_req_router_seek(40)(LOCAL) <= '0';
--in_req_router_seek(41)(LOCAL) <= '0';
--in_req_router_seek(42)(LOCAL) <= '0';
--in_req_router_seek(43)(LOCAL) <= '0';
--in_req_router_seek(44)(LOCAL) <= '0';
--in_req_router_seek(45)(LOCAL) <= '0';
--in_req_router_seek(46)(LOCAL) <= '0';
--in_req_router_seek(47)(LOCAL) <= '0';
--in_req_router_seek(48)(LOCAL) <= '0';
--in_req_router_seek(49)(LOCAL) <= '0';
--in_req_router_seek(50)(LOCAL) <= '0';
--in_req_router_seek(51)(LOCAL) <= '0';
--in_req_router_seek(52)(LOCAL) <= '0';
--in_req_router_seek(53)(LOCAL) <= '0';
--in_req_router_seek(54)(LOCAL) <= '0';
--in_req_router_seek(55)(LOCAL) <= '0';
--in_req_router_seek(56)(LOCAL) <= '0';
--in_req_router_seek(57)(LOCAL) <= '0';
--in_req_router_seek(58)(LOCAL) <= '0';
--in_req_router_seek(59)(LOCAL) <= '0';
--in_req_router_seek(60)(LOCAL) <= '0';
--in_req_router_seek(61)(LOCAL) <= '0';
--in_req_router_seek(62)(LOCAL) <= '0';
--in_req_router_seek(63)(LOCAL) <= '0';
--in_req_router_seek(64)(LOCAL) <= '0';
--in_req_router_seek(65)(LOCAL) <= '0';
--in_req_router_seek(66)(LOCAL) <= '0';
--in_req_router_seek(67)(LOCAL) <= '0';
--in_req_router_seek(68)(LOCAL) <= '0';
--in_req_router_seek(69)(LOCAL) <= '0';
--in_req_router_seek(70)(LOCAL) <= '0';
----in_req_router_seek(71)(LOCAL) <= '0';
--in_req_router_seek(72)(LOCAL) <= '0';
--in_req_router_seek(73)(LOCAL) <= '0';
--in_req_router_seek(74)(LOCAL) <= '0';
--in_req_router_seek(75)(LOCAL) <= '0';
--in_req_router_seek(76)(LOCAL) <= '0';
--in_req_router_seek(77)(LOCAL) <= '0';
--in_req_router_seek(78)(LOCAL) <= '0';
--in_req_router_seek(79)(LOCAL) <= '0';
--in_req_router_seek(80)(LOCAL) <= '0';
--in_req_router_seek(81)(LOCAL) <= '0';
--in_req_router_seek(82)(LOCAL) <= '0';
--in_req_router_seek(83)(LOCAL) <= '0';
--in_req_router_seek(84)(LOCAL) <= '0';
--in_req_router_seek(85)(LOCAL) <= '0';
--in_req_router_seek(86)(LOCAL) <= '0';
--in_req_router_seek(87)(LOCAL) <= '0';
--in_req_router_seek(88)(LOCAL) <= '0';
--in_req_router_seek(89)(LOCAL) <= '0';
----in_req_router_seek(90)(LOCAL) <= '0';
--in_req_router_seek(91)(LOCAL) <= '0';
--in_req_router_seek(92)(LOCAL) <= '0';
--in_req_router_seek(93)(LOCAL) <= '0';
--in_req_router_seek(94)(LOCAL) <= '0';
--in_req_router_seek(95)(LOCAL) <= '0';
--in_req_router_seek(96)(LOCAL) <= '0';
--in_req_router_seek(97)(LOCAL) <= '0';
--in_req_router_seek(98)(LOCAL) <= '0';
--in_req_router_seek(99)(LOCAL) <= '0';
--
----in_fail_router_seek(i) <= (others => '0');
--in_fail_router_seek(i) <= "11111" when i=2 or i=12 or i=22 or i=32 or i=52 or i=62 or i=72 or i=82 or i=92 or i=14 or i=24 or i=34 or i=44 or i=54 or i=64 or i=74 or i=84 or i=15 or i=85 or i=16 or i=36 or i=46 or i=56 or i=66 or i=86 or i=17 or i=37 or i=87 or i=18 or i=38 or i=68 or i=88 or i=39 or i=69   else "00000";

		-- Interconnects the routers
		EAST_connection: if RouterPosition(i) = BL or RouterPosition(i) = CL or RouterPosition(i) = TL  or RouterPosition(i) = BC or RouterPosition(i) = TC or RouterPosition(i) = CC generate
			---seek
			in_source_router_seek(i)(EAST)		<= out_source_router_seek(i+1)(WEST);
            in_target_router_seek(i)(EAST)		<= out_target_router_seek(i+1)(WEST);
            in_hop_router_seek(i)(EAST)		<= out_hop_router_seek(i+1)(WEST);
            in_service_router_seek(i)(EAST)	<= out_service_router_seek(i+1)(WEST);
            in_req_router_seek(i)(EAST)		<= out_req_router_seek(i+1)(WEST);
            in_ack_router_seek(i)(EAST)		<= out_ack_router_seek(i+1)(WEST);
            in_nack_router_seek(i)(EAST)	<= out_nack_router_seek(i+1)(WEST);
            
			---
			
			
			rx(i)(EAST0)		        <= tx(i+1)(WEST0);
			eop_in(i)(EAST0)	        <= eop_out(i+1)(WEST0);
			data_in(i)(EAST0)	        <= data_out(i+1)(WEST0);
			credit_in(i)(EAST0)	        <= credit_out(i+1)(WEST0);
                    
			rx(i)(EAST1)		        <= tx(i+1)(WEST1);
			eop_in(i)(EAST1)	        <= eop_out(i+1)(WEST1);
			data_in(i)(EAST1)	        <= data_out(i+1)(WEST1);
			credit_in(i)(EAST1)	        <= credit_out(i+1)(WEST1);
            
            in_seek(i)(EAST)            <= out_seek(i+1)(WEST);
            in_ack_seek(i)(EAST)        <= out_ack_seek(i+1)(WEST);
            in_clear(i)(EAST)           <= out_clear(i+1)(WEST);
            in_ack_clear(i)(EAST)       <= out_ack_clear(i+1)(WEST);
            in_source(i)(EAST)          <= out_source(i+1)(WEST);
            in_target(i)(EAST)          <= out_target(i+1)(WEST);
            in_hop_counter(i)(EAST)     <= out_hop_counter(i+1)(WEST);
            
            in_failed_port(i)(EAST0)     <= out_failed_port(i+1)(WEST0);
            in_failed_port(i)(EAST1)     <= out_failed_port(i+1)(WEST1);
		end generate;
		
		WEST_connection: if RouterPosition(i) = BR or RouterPosition(i) = CRX or RouterPosition(i) = TR or  RouterPosition(i) = BC or RouterPosition(i) = TC or RouterPosition(i) = CC generate
			in_source_router_seek(i)(WEST)		<= out_source_router_seek(i-1)(EAST);
            in_target_router_seek (i)(WEST)	<= out_target_router_seek(i-1)(EAST);
            in_hop_router_seek(i)(WEST)		<= out_hop_router_seek(i-1)(EAST);
            in_service_router_seek(i)(WEST)	<= out_service_router_seek	(i-1)(EAST);
            in_req_router_seek(i)(WEST)		<= out_req_router_seek	(i-1)(EAST);
            in_ack_router_seek(i)(WEST)		<= out_ack_router_seek(i-1)(EAST);
            in_nack_router_seek(i)(WEST)	<= out_nack_router_seek(i-1)(EAST);    
			
			
			rx(i)(WEST0)		        <= tx(i-1)(EAST0);
			eop_in(i)(WEST0)	        <= eop_out(i-1)(EAST0);
			data_in(i)(WEST0)	        <= data_out(i-1)(EAST0);
			credit_in(i)(WEST0)	        <= credit_out(i-1)(EAST0);
                    
			rx(i)(WEST1)		        <= tx(i-1)(EAST1);
			eop_in(i)(WEST1)	        <= eop_out(i-1)(EAST1);
			data_in(i)(WEST1)	        <= data_out(i-1)(EAST1);
			credit_in(i)(WEST1)	        <= credit_out(i-1)(EAST1);
            
            in_seek(i)(WEST)            <= out_seek(i-1)(EAST);
            in_ack_seek(i)(WEST)        <= out_ack_seek(i-1)(EAST);
            in_clear(i)(WEST)           <= out_clear(i-1)(EAST);
            in_ack_clear(i)(WEST)       <= out_ack_clear(i-1)(EAST);
            in_source(i)(WEST)          <= out_source(i-1)(EAST);
            in_target(i)(WEST)          <= out_target(i-1)(EAST);
            in_hop_counter(i)(WEST)     <= out_hop_counter(i-1)(EAST);
            
            in_failed_port(i)(WEST0)     <= out_failed_port(i-1)(EAST0);
            in_failed_port(i)(WEST1)     <= out_failed_port(i-1)(EAST1);
		end generate;
		
		NORTH_connection: if RouterPosition(i) = BL or RouterPosition(i) = BC or RouterPosition(i) = BR or RouterPosition(i) = CL or RouterPosition(i) = CRX or RouterPosition(i) = CC generate
			in_source_router_seek(i)(NORTH)		<= out_source_router_seek(i+NUMBER_PROCESSORS_X)(SOUTH);
            in_target_router_seek(i)(NORTH)		<= out_target_router_seek(i+NUMBER_PROCESSORS_X)(SOUTH);
            in_hop_router_seek(i)(NORTH)			<= out_hop_router_seek(i+NUMBER_PROCESSORS_X)(SOUTH);
			in_service_router_seek(i)(NORTH)		<= out_service_router_seek	(i+NUMBER_PROCESSORS_X)(SOUTH);
			in_req_router_seek(i)(NORTH)			<= out_req_router_seek(i+NUMBER_PROCESSORS_X)(SOUTH);
            in_ack_router_seek(i)(NORTH)			<= out_ack_router_seek(i+NUMBER_PROCESSORS_X)(SOUTH);
            in_nack_router_seek(i)(NORTH)			<= out_nack_router_seek(i+NUMBER_PROCESSORS_X)(SOUTH);            
			
			
			rx(i)(NORTH0)		            <= tx(i+NUMBER_PROCESSORS_X)(SOUTH0);
			eop_in(i)(NORTH0)	            <= eop_out(i+NUMBER_PROCESSORS_X)(SOUTH0);
			data_in(i)(NORTH0)	            <= data_out(i+NUMBER_PROCESSORS_X)(SOUTH0);
			credit_in(i)(NORTH0)            <= credit_out(i+NUMBER_PROCESSORS_X)(SOUTH0);
                        
			rx(i)(NORTH1)		            <= tx(i+NUMBER_PROCESSORS_X)(SOUTH1);
			eop_in(i)(NORTH1)	            <= eop_out(i+NUMBER_PROCESSORS_X)(SOUTH1);
			data_in(i)(NORTH1)	            <= data_out(i+NUMBER_PROCESSORS_X)(SOUTH1);
			credit_in(i)(NORTH1)            <= credit_out(i+NUMBER_PROCESSORS_X)(SOUTH1);
            
            in_seek(i)(NORTH)               <= out_seek(i+NUMBER_PROCESSORS_X)(SOUTH);
            in_ack_seek(i)(NORTH)           <= out_ack_seek(i+NUMBER_PROCESSORS_X)(SOUTH);
            in_clear(i)(NORTH)              <= out_clear(i+NUMBER_PROCESSORS_X)(SOUTH);
            in_ack_clear(i)(NORTH)          <= out_ack_clear(i+NUMBER_PROCESSORS_X)(SOUTH);
            in_source(i)(NORTH)             <= out_source(i+NUMBER_PROCESSORS_X)(SOUTH);
            in_target(i)(NORTH)             <= out_target(i+NUMBER_PROCESSORS_X)(SOUTH);
            in_hop_counter(i)(NORTH)        <= out_hop_counter(i+NUMBER_PROCESSORS_X)(SOUTH);
            
            in_failed_port(i)(NORTH0)        <= out_failed_port(i+NUMBER_PROCESSORS_X)(SOUTH0);
            in_failed_port(i)(NORTH1)        <= out_failed_port(i+NUMBER_PROCESSORS_X)(SOUTH1);
		end generate;
		
		SOUTH_connection: if RouterPosition(i) = TL or RouterPosition(i) = TC or RouterPosition(i) = TR or RouterPosition(i) = CL or RouterPosition(i) = CRX or RouterPosition(i) = CC generate
			in_source_router_seek(i)(SOUTH)		<= out_source_router_seek(i-NUMBER_PROCESSORS_X)(NORTH);
            in_target_router_seek(i)(SOUTH)		<= out_target_router_seek(i-NUMBER_PROCESSORS_X)(NORTH);
            in_hop_router_seek(i)(SOUTH)			<= out_hop_router_seek(i-NUMBER_PROCESSORS_X)(NORTH);
			in_service_router_seek	(i)(SOUTH)		<= out_service_router_seek(i-NUMBER_PROCESSORS_X)(NORTH);
            in_req_router_seek(i)(SOUTH)			<= out_req_router_seek(i-NUMBER_PROCESSORS_X)(NORTH);
            in_ack_router_seek(i)(SOUTH)			<= out_ack_router_seek(i-NUMBER_PROCESSORS_X)(NORTH);
            in_nack_router_seek(i)(SOUTH)			<= out_nack_router_seek(i-NUMBER_PROCESSORS_X)(NORTH);            
			
			rx(i)(SOUTH0)		            <= tx(i-NUMBER_PROCESSORS_X)(NORTH0);
			eop_in(i)(SOUTH0)	            <= eop_out(i-NUMBER_PROCESSORS_X)(NORTH0);
			data_in(i)(SOUTH0)	            <= data_out(i-NUMBER_PROCESSORS_X)(NORTH0);
			credit_in(i)(SOUTH0)            <= credit_out(i-NUMBER_PROCESSORS_X)(NORTH0);
                        
			rx(i)(SOUTH1)		            <= tx(i-NUMBER_PROCESSORS_X)(NORTH1);
			eop_in(i)(SOUTH1)	            <= eop_out(i-NUMBER_PROCESSORS_X)(NORTH1);
			data_in(i)(SOUTH1)	            <= data_out(i-NUMBER_PROCESSORS_X)(NORTH1);
			credit_in(i)(SOUTH1)            <= credit_out(i-NUMBER_PROCESSORS_X)(NORTH1);
            
            in_seek(i)(SOUTH)               <= out_seek(i-NUMBER_PROCESSORS_X)(NORTH);
            in_ack_seek(i)(SOUTH)           <= out_ack_seek(i-NUMBER_PROCESSORS_X)(NORTH);
            in_clear(i)(SOUTH)              <= out_clear(i-NUMBER_PROCESSORS_X)(NORTH);
            in_ack_clear(i)(SOUTH)          <= out_ack_clear(i-NUMBER_PROCESSORS_X)(NORTH);
            in_source(i)(SOUTH)             <= out_source(i-NUMBER_PROCESSORS_X)(NORTH);
            in_target(i)(SOUTH)             <= out_target(i-NUMBER_PROCESSORS_X)(NORTH);
            in_hop_counter(i)(SOUTH)        <= out_hop_counter(i-NUMBER_PROCESSORS_X)(NORTH);
            
            in_failed_port(i)(SOUTH0)        <= out_failed_port(i-NUMBER_PROCESSORS_X)(NORTH0);
            in_failed_port(i)(SOUTH1)        <= out_failed_port(i-NUMBER_PROCESSORS_X)(NORTH1);
		end generate;
		
		
		EAST_grounding: if RouterPosition(i) = BR or RouterPosition(i) = CRX or RouterPosition(i) = TR generate
			
			in_source_router_seek(i)(EAST)			<= (others => '0');
			in_target_router_seek(i)(EAST)			<= (others => '0');
			in_hop_router_seek(i)(EAST)			<= (others => '0');
			in_service_router_seek(i)(EAST)		<= (others => '0');
			in_req_router_seek(i)(EAST)			<= '0';
			in_ack_router_seek(i)(EAST)			<= '1';
			
			
			
			rx(i)(EAST0)		        <= '0';
			eop_in(i)(EAST0)	        <= '0';
			credit_in(i)(EAST0)	        <= '0';
                    
			rx(i)(EAST1)		        <= '0';
			eop_in(i)(EAST1)	        <= '0';
			credit_in(i)(EAST1)	        <= '0';
            
            in_seek(i)(EAST)            <= NOT_SEEK;
            in_ack_seek(i)(EAST)        <= '0';
            in_clear(i)(EAST)           <= '0';
            in_ack_clear(i)(EAST)       <= '0';
            in_source(i)(EAST)          <= (others => '0');
            in_target(i)(EAST)          <= (others => '0');
            in_hop_counter(i)(EAST)     <= (others => '0');
            
            in_failed_port(i)(EAST0)    <= '0';
            in_failed_port(i)(EAST1)    <= '0';
		end generate;
		
		WEST_grounding: if RouterPosition(i) = BL or RouterPosition(i) = CL or RouterPosition(i) = TL generate
			in_source_router_seek(i)(WEST)			<= (others => '0');
			in_target_router_seek(i)(WEST)			<= (others => '0');
			in_hop_router_seek(i)(WEST)				<= (others => '0');
			in_service_router_seek(i)(WEST)			<= (others => '0');
			in_req_router_seek(i)(WEST)				<= '0';
			in_ack_router_seek(i)(WEST)				<= '1';
			
			rx(i)(WEST0)		        <= '0';
			eop_in(i)(WEST0)	        <= '0';
			credit_in(i)(WEST0)	        <= '0';
                    
			rx(i)(WEST1)		        <= '0';
			eop_in(i)(WEST1)	        <= '0';
			credit_in(i)(WEST1)	        <= '0';
            
            in_seek(i)(WEST)            <= NOT_SEEK;
            in_ack_seek(i)(WEST)        <= '0';
            in_clear(i)(WEST)           <= '0';
            in_ack_clear(i)(WEST)       <= '0';
            in_source(i)(WEST)          <= (others => '0');
            in_target(i)(WEST)          <= (others => '0');
            in_hop_counter(i)(WEST)     <= (others => '0');
            
            in_failed_port(i)(WEST0)    <= '0';
            in_failed_port(i)(WEST1)    <= '0';
		end generate;
		
		NORTH_grounding: if RouterPosition(i) = TL or RouterPosition(i) = TC or RouterPosition(i) = TR generate
			in_source_router_seek(i)(NORTH)			<= (others => '0');
			in_target_router_seek(i)(NORTH)			<= (others => '0');
			in_hop_router_seek(i)(NORTH)				<= (others => '0');
			in_service_router_seek(i)(NORTH)			<= (others => '0');
			in_req_router_seek(i)(NORTH)				<= '0';
			in_ack_router_seek(i)(NORTH)				<= '1';
			
			rx(i)(NORTH0)		        <= '0';
			eop_in(i)(NORTH0)	        <= '0';
			credit_in(i)(NORTH0)        <= '0';
        
			rx(i)(NORTH1)		        <= '0';
			eop_in(i)(NORTH1)	        <= '0';
			credit_in(i)(NORTH1)        <= '0';
            
            in_seek(i)(NORTH)           <= NOT_SEEK;
            in_ack_seek(i)(NORTH)       <= '0';
            in_clear(i)(NORTH)          <= '0';
            in_ack_clear(i)(NORTH)      <= '0';
            in_source(i)(NORTH)         <= (others => '0');
            in_target(i)(NORTH)         <= (others => '0');
            in_hop_counter(i)(NORTH)    <= (others => '0');
            
            in_failed_port(i)(NORTH0)    <= '0';
            in_failed_port(i)(NORTH1)    <= '0';
		end generate;
		
		SOUTH_grounding: if RouterPosition(i) = BL or RouterPosition(i) = BC or RouterPosition(i) = BR generate
			in_source_router_seek(i)(SOUTH)			<= (others => '0');
			in_target_router_seek(i)(SOUTH)			<= (others => '0');
			in_hop_router_seek(i)(SOUTH)				<= (others => '0');
			in_service_router_seek(i)(SOUTH)			<= (others => '0');
			in_req_router_seek(i)(SOUTH)				<= '0';
			in_ack_router_seek(i)(SOUTH)				<= '1';
			
			rx(i)(SOUTH0)		        <= '0';
			eop_in(i)(SOUTH0)	        <= '0';
			credit_in(i)(SOUTH0)        <= '0';
    
			rx(i)(SOUTH1)		        <= '0';
			eop_in(i)(SOUTH1)	        <= '0';
			credit_in(i)(SOUTH1)        <= '0';
            
            in_seek(i)(SOUTH)           <= NOT_SEEK;
            in_ack_seek(i)(SOUTH)       <= '0';
            in_clear(i)(SOUTH)          <= '0';
            in_ack_clear(i)(SOUTH)      <= '0';
            in_source(i)(SOUTH)         <= (others => '0');
            in_target(i)(SOUTH)         <= (others => '0');
            in_hop_counter(i)(SOUTH)    <= (others => '0');
            
            in_failed_port(i)(SOUTH0)    <= '0';
            in_failed_port(i)(SOUTH1)    <= '0';
		end generate;
        end generate proc;
           
end architecture;
