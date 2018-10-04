---------------------------------------------------------------------------------------------------
--
-- Title       : Router
-- Design      : QoS
-- Company     : GAPH
--
---------------------------------------------------------------------------------------------------
--
-- File        : Router.vhd
-- Generated   : Thu Mar  6 17:08:38 2008
-- From        : interface description file
-- By          : Itf2Vhdl ver. 1.20
--
---------------------------------------------------------------------------------------------------
--							   
-- Description : Implements a NoC router based on its mesh position. 
-- 	Connects the router components: Input Buffers, Crossbar and Switch Control.
--
---------------------------------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use work.HeMPS_defaults.all;

entity Router is
generic( 
	address			: regHalfFlit:= (others=>'0');
	input_buffers	: string := "RTL");
port(
    clock		        : in  std_logic;
    reset		        : in  std_logic;   
    rx			        : in  regNport;
    eop_in		        : in  regNport;
    data_in		        : in  arrayNport_regflit;
	credit_out	        : out regNport;       
    tx			        : out regNport;
    eop_out		        : out regNport;
    data_out	        : out arrayNport_regflit;
	credit_in	        : in  regNport;
    --seek in/out
    reset_seek          : in  std_logic;
    in_seek             : in  regNport_seek_NBit;
    out_ack_seek        : out regNport_seek;
    in_clear            : in  regNport_seek;
    out_ack_clear       : out regNport_seek;
    in_source           : in  regNportNsource_target;
    in_target           : in  regNportNsource_target;
    in_hop_counter      : in  regNportNhop;
    out_seek            : out regNport_seek_NBit;
    in_ack_seek         : in  regNport_seek;
    out_clear           : out regNport_seek;
    in_ack_clear        : in  regNport_seek;
    out_source          : out regNportNsource_target;
    out_target          : out regNportNsource_target;
    out_hop_counter     : out regNportNhop;
    failed_reception    : out std_logic;

    seek_unreachable_interrupt  : out std_logic;
    seek_resend_interrupt  		: out std_logic;
    seek_unreachable_target     : out regNsource_target;
    --error_c						: out regNport;
    out_failed_port      : out  regNport_neighbor;
    out_failed_port_test : out  regNport_neighbor; --fochi
    in_failed_port       : in  	regNport_neighbor;
    out_reset_stepdet	 : in 	regNport_neighbor; --fochi
    eop_out_test_recv		 : out  regNport --fochi
    
    );	
end Router;

architecture Router of Router is

signal req_routing, ack_routing, tx_buffer, sender, credit_buffer, eop_buffer, next_flit: regNport;
signal data_buffer: arrayNport_regflit;
signal table: matrixNportNport_std_logic;

--MUX signals
signal mux_seek_ctrl        : std_logic;
signal aaaaaa : std_logic;
signal data_out_mux         : regflit;
signal data_in_test_router  : arrayNport_regflit;
signal rx_out_mux           : std_logic;
signal eop_out_mux          : std_logic;

signal credit_out_buffer    : std_logic;

signal data_in_seek         : regflit;
signal rx_seek              : std_logic;
signal eop_seek             : std_logic;
signal credit_seek          : std_logic;

signal L0_free              : std_logic;

signal req_backtrack        : std_logic;
signal ack_backtrack        : std_logic;
signal source               : regNsource_target;
signal mask                 : regflit;
signal seek_resend_sr_router: std_logic;
signal flit_num             : std_logic_vector(2 downto 0);
signal port_dir             : std_logic_vector(1 downto 0);
signal flit_pos             : std_logic_vector(2 downto 0);

--signals of mux from buffer to crossbar
signal out_mux_buffer       : arrayNport_regflit;
signal mux_buffer_ctrl      : regNport;
signal enable_mux           : regNport;
signal enable_shift         : regNport;

--signal to signalize to PE in case of fault
signal fail_detect          : regNport;


signal data_out_monitor     : arrayNport_regflit;
signal tx_monitor           : regNport;
signal eop_monitor          : regNport;
signal credit_monitor       : regNport;

signal error				: regNport;
signal error_out			: regNport;
signal error_buffer			: regNport;
signal in_error_stepdet		: regNport;

signal erro_out_test_port	: regNport;
signal reset_stepdet		: regNport;



begin

    --MUX LOCAL0 PORT
    data_out_mux            <=  data_in(SEEK_PORT)     when mux_seek_ctrl = '0' else
                                data_in_seek;
    rx_out_mux              <=  rx(SEEK_PORT)          when mux_seek_ctrl = '0' else
                                rx_seek;
    eop_out_mux             <=  eop_in(SEEK_PORT)      when mux_seek_ctrl = '0' else
                                eop_seek;
    credit_monitor(SEEK_PORT)   <=  credit_out_buffer   when mux_seek_ctrl = '0' else
                            '0';
                            
    reset_stepdet(EAST0)  <= '1' when out_reset_stepdet(EAST0) = '0'   else '0';--fochi    
    reset_stepdet(EAST1)  <= '1' when out_reset_stepdet(EAST1) = '0'   else '0';--fochi    
    reset_stepdet(WEST0)  <= '1' when out_reset_stepdet(WEST0) = '0'   else '0';--fochi    
    reset_stepdet(WEST1)  <= '1' when out_reset_stepdet(WEST1) = '0'   else '0';--fochi    
    reset_stepdet(NORTH0)  <= '1' when out_reset_stepdet(NORTH0) = '0' else '0';--fochi    
    reset_stepdet(NORTH1)  <= '1' when out_reset_stepdet(NORTH1) = '0' else '0';--fochi    
    reset_stepdet(SOUTH0)  <= '1' when out_reset_stepdet(SOUTH0) = '0' else '0';--fochi    
    reset_stepdet(SOUTH1)  <= '1' when out_reset_stepdet(SOUTH1) = '0' else '0';--fochi 
    
	eop_out_test_recv(EAST0)  <= eop_buffer(EAST0) ; -- eop buffer que vai para o test_port_receive
	eop_out_test_recv(EAST1)  <= eop_buffer(EAST1) ; 
	eop_out_test_recv(WEST0)  <= eop_buffer(WEST0) ; 
	eop_out_test_recv(WEST1)  <= eop_buffer(WEST1) ; 
	eop_out_test_recv(NORTH0) <= eop_buffer(NORTH0); 
	eop_out_test_recv(NORTH1) <= eop_buffer(NORTH1); 
	eop_out_test_recv(SOUTH0) <= eop_buffer(SOUTH0); 
	eop_out_test_recv(SOUTH1) <= eop_buffer(SOUTH1); 
	
           
    out_failed_port_test(EAST0)		<= error_buffer(EAST0);
    out_failed_port_test(EAST1)		<= error_buffer(EAST1);
    out_failed_port_test(WEST0)		<= error_buffer(WEST0);
    out_failed_port_test(WEST1)		<= error_buffer(WEST1);
    out_failed_port_test(NORTH0)	<= error_buffer(NORTH0);		
    out_failed_port_test(NORTH1)	<= error_buffer(NORTH1);		
    out_failed_port_test(SOUTH1)	<= error_buffer(SOUTH1);	
    out_failed_port_test(SOUTH0)	<= error_buffer(SOUTH0);    
    
	out_failed_port(EAST0) <= error_out(EAST0);            
	out_failed_port(EAST1) <= error_out(EAST1);               
	out_failed_port(WEST0) <= error_out(WEST0);               
	out_failed_port(WEST1) <= error_out(WEST1);   
	out_failed_port(SOUTH0) <= error_out(SOUTH0);  
	out_failed_port(SOUTH1) <= error_out(SOUTH1);	
	out_failed_port(NORTH0) <= error_out(NORTH0);   
	out_failed_port(NORTH1) <= error_out(NORTH1);
	
	in_error_stepdet(NORTH0) <=	 error_buffer(NORTH0);  
    in_error_stepdet(NORTH1) <=	 error_buffer(NORTH1); 
    in_error_stepdet(EAST0)  <=	error_buffer(EAST0);  
    in_error_stepdet(EAST1)  <=	error_buffer(EAST1);  
    in_error_stepdet(WEST0)  <=	error_buffer(WEST0);  
    in_error_stepdet(WEST1)  <=	error_buffer(WEST1);  
    in_error_stepdet(SOUTH0) <=	 error_buffer(SOUTH0);  
    in_error_stepdet(SOUTH1) <=	 error_buffer(SOUTH1);  
    
    --out_failed_port_test(EAST0)		<= error(EAST0);
    --out_failed_port_test(EAST1)		<= error(EAST1);
    --out_failed_port_test(WEST0)		<= error(WEST0);
    --out_failed_port_test(WEST1)		<= error(WEST1);
    --out_failed_port_test(NORTH0)	<= error(NORTH0);		
    --out_failed_port_test(NORTH1)	<= error(NORTH1);		
    --out_failed_port_test(SOUTH1)	<= error(SOUTH1);	
    --out_failed_port_test(SOUTH0)	<= error(SOUTH0);	
    
    
	--out_failed_port(EAST0) <= error_out(EAST0);            
	--out_failed_port(EAST1) <= error_out(EAST1);               
	--out_failed_port(WEST0) <= error_out(WEST0);               
	--out_failed_port(WEST1) <= error_out(WEST1);   
	--out_failed_port(SOUTH0) <= error_out(SOUTH0);  
	--out_failed_port(SOUTH1) <= error_out(SOUTH1);	
	--out_failed_port(NORTH0) <= error_out(NORTH0);   
	--out_failed_port(NORTH1) <= error_out(NORTH1);
	
	--in_error_stepdet(NORTH0) <=	error(NORTH0) or error_buffer(NORTH0);  
    --in_error_stepdet(NORTH1) <=	error(NORTH1) or error_buffer(NORTH1); 
    --in_error_stepdet(EAST0)  <=	error(EAST0) or error_buffer(EAST0);  
    --in_error_stepdet(EAST1)  <=	error(EAST1) or error_buffer(EAST1);  
    --in_error_stepdet(WEST0)  <=	error(WEST0) or error_buffer(WEST0);  
    --in_error_stepdet(WEST1)  <=	error(WEST1) or error_buffer(WEST1);  
    --in_error_stepdet(SOUTH0) <=	error(SOUTH0) or error_buffer(SOUTH0);  
    --in_error_stepdet(SOUTH1) <=	error(SOUTH1) or error_buffer(SOUTH1);  
     
     error_out(LOCAL0) <= '0';
     error_out(LOCAL1) <= '0';
 
    process(reset,rx_out_mux,eop_out_mux)
    begin
        if reset = '1' then
            L0_free <= '1';
        elsif rx_out_mux = '1' then
            if eop_out_mux = '1' then
                L0_free <= '1';
            else
                L0_free <= '0';
            end if;
        end if;
    end process;

    seek:entity work.seek
    generic map(
        seek_address            => address
    )
    port map(
    --sinais do seek. // sinais de outro lugar
        clock                   => clock,
        reset                   => reset_seek,
        in_seek                 => in_seek,
        out_ack_seek            => out_ack_seek,
        in_clear                => in_clear,
        out_ack_clear           => out_ack_clear,
        in_source               => in_source,
        in_target               => in_target,
        in_hop_counter          => in_hop_counter,
        
        out_seek                => out_seek,
        in_ack_seek             => in_ack_seek,
        out_clear               => out_clear,
        in_ack_clear            => in_ack_clear,
        out_source              => out_source,
        out_target              => out_target,
        out_hop_counter         => out_hop_counter,
        
        req_backtrack           => req_backtrack,
        ack_backtrack           => ack_backtrack,
        source                  => source,
        flit_num                => flit_num,
        flit_pos                => flit_pos,
        port_dir                => port_dir,
        
        seek_unreachable_interrupt  => seek_unreachable_interrupt,
        seek_resend_interrupt 		=> seek_resend_interrupt,
        seek_unreachable_target     => seek_unreachable_target,
            
        --local port interface
        data_out                => data_in_seek, -- fochi
        rx_out                  => rx_seek,
        eop_out                 => eop_seek,
        credit_in               => credit_out_buffer,
        port_ctrl               => mux_seek_ctrl,
        free_port               => L0_free--,
        --rx_seek_port            => rx(SEEK_PORT)
    );
    
	Buffer_Local0: Entity work.input_buffer
	generic	map (buffer_type => input_buffers)
	port map(
		clock 		=> clock,
		reset 		=> reset,
		data_in 	=> data_in(LOCAL0),        --data_out_mux, 
		rx 			=> rx(LOCAL0),             --rx_out_mux,   
		eop_in 		=> eop_in(LOCAL0),         --eop_out_mux,  
		req_routing => req_routing(LOCAL0),
		ack_routing => ack_routing(LOCAL0),	 
		tx 			=> tx_buffer(LOCAL0),
		eop_out 	=> eop_buffer(LOCAL0),
		data_out	=> data_buffer(LOCAL0),
		sender		=> sender(LOCAL0), 		
		credit_in	=> credit_buffer(LOCAL0),
		next_flit 	=> next_flit(LOCAL0),
		credit_out 	=> credit_monitor(LOCAL0),--credit_out_buffer,
        fail_detect => fail_detect(LOCAL0),
        error		=> '0'
	);
	
	Buffer_Local1: Entity work.input_buffer
		generic	map (buffer_type => input_buffers)
		port map(
			clock 		=> clock,
			reset 		=> reset,
			data_in 	=> data_out_mux, --data_in(LOCAL1),    
			rx 			=> rx_out_mux,   --rx(LOCAL1),         
			eop_in 		=> eop_out_mux,  --eop_in(LOCAL1),     
			req_routing => req_routing(LOCAL1),
			ack_routing => ack_routing(LOCAL1),	 
			tx 			=> tx_buffer(LOCAL1),
			eop_out 	=> eop_buffer(LOCAL1),
			data_out	=> data_buffer(LOCAL1),
			sender		=> sender(LOCAL1), 		
			credit_in	=> credit_buffer(LOCAL1),
			next_flit 	=> next_flit(LOCAL1),
			credit_out 	=> credit_out_buffer,--credit_out(LOCAL1),
            -- credit_out  => credit_out(LOCAL1),
            fail_detect => fail_detect(LOCAL1),
            error		=> '0'
	);

    Buffer_East0: Entity work.input_buffer
    generic	map (buffer_type => input_buffers)
    port map(
        clock 		=> clock,
        reset 		=> reset,
        data_in 	=> data_in(EAST0),
        rx 			=> rx(EAST0),
        eop_in 		=> eop_in(EAST0),
        req_routing => req_routing(EAST0),
        ack_routing => ack_routing(EAST0),	 
        tx 			=> tx_buffer(EAST0),
        eop_out 	=> eop_buffer(EAST0),
        data_out	=> data_buffer(EAST0),
        sender		=> sender(EAST0),	
        credit_in 	=> credit_buffer(EAST0),
        next_flit 	=> next_flit(EAST0),
        credit_out 	=> credit_monitor(EAST0),
        fail_detect => fail_detect(EAST0),
        error 		=> error_out(EAST0)
    );	  	
    
    DECEAST0_BUFFER: entity work.deccrc
		port map(
			input => data_buffer(EAST0)(15 downto 0),
			crc_in => data_buffer(EAST0)(19 downto 16),
			rx => rx(EAST0),
			error => error_buffer(EAST0));
    
    --DECEast0: entity work.deccrc
		--port map(
			--input => data_in(EAST0)(15 downto 0),
			--crc_in => data_in(EAST0)(19 downto 16),
			--rx => rx(EAST0),
			--error => error(EAST0));
			
	DECEast0_stepdet: entity work.stepdet
        port map(
            ck          => clock,
            rst         => reset_stepdet(EAST0),
            in_sig      => in_error_stepdet(EAST0),
            out_sig     => error_out(EAST0)            
        );	
        
       
        
    Buffer_East1: Entity work.input_buffer
    generic	map (buffer_type => input_buffers)
    port map(
        clock 		=> clock,
        reset 		=> reset,
        data_in 	=> data_in(EAST1),
        rx 			=> rx(EAST1),
        eop_in 		=> eop_in(EAST1),
        req_routing => req_routing(EAST1),
        ack_routing => ack_routing(EAST1),	 
        tx 			=> tx_buffer(EAST1),
        eop_out 	=> eop_buffer(EAST1),
        data_out	=> data_buffer(EAST1),
        sender		=> sender(EAST1),	
        credit_in 	=> credit_buffer(EAST1),
        next_flit 	=> next_flit(EAST1),
        credit_out 	=> credit_monitor(EAST1),
        fail_detect => fail_detect(EAST1),
        error 		=> error_out(EAST1)
    );
   
    DECEAST1_BUFFER: entity work.deccrc
		port map(
			input => data_buffer(EAST1)(15 downto 0),
			crc_in => data_buffer(EAST1)(19 downto 16),
			rx => rx(EAST1),
			error => error_buffer(EAST1));
    
    --DECEast1: entity work.deccrc
		--port map(
			--input => data_in(EAST1)(15 downto 0),
			--crc_in => data_in(EAST1)(19 downto 16),
			--rx => rx(EAST1),
			--error => error(EAST1));
			
	DECEast1_stepdet: entity work.stepdet
        port map(
            ck          => clock,
            rst         => reset_stepdet(EAST1),
            in_sig      => in_error_stepdet(EAST1),
            out_sig     => error_out(EAST1) 
        );	
    
    
    Buffer_West0: Entity work.input_buffer 
    generic	map (buffer_type => input_buffers)
    port map(
        clock 		=> clock,
        reset 		=> reset,
        data_in 	=> data_in(WEST0),
        rx 			=> rx(WEST0),
        eop_in 		=> eop_in(WEST0),
        req_routing => req_routing(WEST0),
        ack_routing => ack_routing(WEST0),	 
        tx 			=> tx_buffer(WEST0),
        eop_out 	=> eop_buffer(WEST0),
        data_out	=> data_buffer(WEST0),
        sender		=> sender(WEST0), 		
        credit_in 	=> credit_buffer(WEST0),
        next_flit 	=> next_flit(WEST0),
        credit_out 	=> credit_monitor(WEST0),
        fail_detect => fail_detect(WEST0),
        error 		=> error_out(WEST0)
    );	   			
    
    DECWEST0_BUFFER: entity work.deccrc
		port map(
			input => data_buffer(WEST0)(15 downto 0),
			crc_in => data_buffer(WEST0)(19 downto 16),
			rx => rx(WEST0),
			error => error_buffer(WEST0));
			
    --DECWEST0: entity work.deccrc
		--port map(
			--input => data_in(WEST0)(15 downto 0),
			--crc_in => data_in(WEST0)(19 downto 16),
			--rx => rx(WEST0),
			--error => error(WEST0));
			
	DECWest0_stepdet: entity work.stepdet
        port map(
            ck          => clock,
            rst         => reset_stepdet(WEST0),
            in_sig      => in_error_stepdet(WEST0),
            out_sig     => error_out(WEST0) 
        );
     
        
    Buffer_West1: Entity work.input_buffer 
    generic	map (buffer_type => input_buffers)
    port map(
        clock 		=> clock,
        reset 		=> reset,
        data_in 	=> data_in(WEST1),
        rx 			=> rx(WEST1),
        eop_in 		=> eop_in(WEST1),
        req_routing => req_routing(WEST1),
        ack_routing => ack_routing(WEST1),	 
        tx 			=> tx_buffer(WEST1),
        eop_out 	=> eop_buffer(WEST1),
        data_out	=> data_buffer(WEST1),
        sender		=> sender(WEST1), 		
        credit_in 	=> credit_buffer(WEST1),
        next_flit 	=> next_flit(WEST1),
        credit_out 	=> credit_monitor(WEST1),
        fail_detect => fail_detect(WEST1),
        error 		=> error_out(WEST1)
    );
    
    DECWEST1_BUFFER: entity work.deccrc
		port map(
			input => data_buffer(WEST1)(15 downto 0),
			crc_in => data_buffer(WEST1)(19 downto 16),
			rx => rx(WEST1),
			error => error_buffer(WEST1));
			
    --DECWEST1: entity work.deccrc
		--port map(
			--input => data_in(WEST1)(15 downto 0),
			--crc_in => data_in(WEST1)(19 downto 16),
			--rx => rx(WEST1),
			--error => error(WEST1));
			
	DECWest1_stepdet: entity work.stepdet
        port map(
            ck          => clock,
            rst         => reset_stepdet(WEST1),
            in_sig      => in_error_stepdet(WEST1),
            out_sig     => error_out(WEST1)
        );
     
    
    Buffer_South0: Entity work.input_buffer	
    generic	map (buffer_type => input_buffers)
    port map(
        clock 		=> clock,
        reset 		=> reset,
        data_in 	=> data_in(SOUTH0),
        rx 			=> rx(SOUTH0),
        eop_in 		=> eop_in(SOUTH0),
        req_routing => req_routing(SOUTH0),
        ack_routing => ack_routing(SOUTH0),		  
        tx 			=> tx_buffer(SOUTH0),
        eop_out 	=> eop_buffer(SOUTH0),
        data_out	=> data_buffer(SOUTH0),
        sender		=> sender(SOUTH0),		
        credit_in 	=> credit_buffer(SOUTH0), 
        next_flit 	=> next_flit(SOUTH0),	
        credit_out 	=> credit_monitor(SOUTH0),
        fail_detect => fail_detect(SOUTH0),
        error 		=> error_out(SOUTH0)
    );
	
	DECSOUTH0_BUFFER: entity work.deccrc
		port map(
			input => data_buffer(SOUTH0)(15 downto 0),
			crc_in => data_buffer(SOUTH0)(19 downto 16),
			rx => rx(SOUTH0),
			error => error_buffer(SOUTH0));
    
    --DECSOUTH0: entity work.deccrc
		--port map(
			--input => data_in(SOUTH0)(15 downto 0),
			--crc_in => data_in(SOUTH0)(19 downto 16),
			--rx => rx(SOUTH0),
			--error => error(SOUTH0));
			
	DECSouth0_stepdet: entity work.stepdet
        port map(
            ck          => clock,
            rst         => reset_stepdet(SOUTH0),
            in_sig      => in_error_stepdet(SOUTH0),
            out_sig     => error_out(SOUTH0)
        );
     
    
    Buffer_South1: Entity work.input_buffer	
    generic	map (buffer_type => input_buffers)
    port map(
        clock 		=> clock,
        reset 		=> reset,
        data_in 	=> data_in(SOUTH1),
        rx 			=> rx(SOUTH1),
        eop_in 		=> eop_in(SOUTH1),
        req_routing => req_routing(SOUTH1),
        ack_routing => ack_routing(SOUTH1),		  
        tx 			=> tx_buffer(SOUTH1),
        eop_out 	=> eop_buffer(SOUTH1),
        data_out	=> data_buffer(SOUTH1),
        sender		=> sender(SOUTH1),		
        credit_in 	=> credit_buffer(SOUTH1), 
        next_flit 	=> next_flit(SOUTH1),	
        credit_out 	=> credit_monitor(SOUTH1),
        fail_detect => fail_detect(SOUTH1),
        error 		=> error_out(SOUTH1)
    );
    
    DECSOUTH1_BUFFER: entity work.deccrc
		port map(
			input => data_buffer(SOUTH1)(15 downto 0),
			crc_in => data_buffer(SOUTH1)(19 downto 16),
			rx => rx(SOUTH1),
			error => error_buffer(SOUTH1));
    
    --DECSOUTH1: entity work.deccrc
		--port map(
			--input => data_in(SOUTH1)(15 downto 0),
			--crc_in => data_in(SOUTH1)(19 downto 16),
			--rx => rx(SOUTH1),
			--error => error(SOUTH1));
			
		
	DECSouth1_stepdet: entity work.stepdet
        port map(
            ck          => clock,
            rst         => reset_stepdet(SOUTH1),
            in_sig      => in_error_stepdet(SOUTH1),
            out_sig     => error_out(SOUTH1)
        );	
    
    Buffer_North0: Entity work.input_buffer
    generic	map (buffer_type => input_buffers)
    port map(
        clock 		=> clock,
        reset 		=> reset,
        data_in 	=> data_in(NORTH0),
        rx 			=> rx(NORTH0),
        eop_in 		=> eop_in(NORTH0),
        req_routing => req_routing(NORTH0),
        ack_routing => ack_routing(NORTH0),	
        tx 			=> tx_buffer(NORTH0),
        eop_out 	=> eop_buffer(NORTH0),
        data_out	=> data_buffer(NORTH0),
        sender		=> sender(NORTH0),	  
        credit_in 	=> credit_buffer(NORTH0),
        next_flit 	=> next_flit(NORTH0),	
        credit_out 	=> credit_monitor(NORTH0),
        fail_detect => fail_detect(NORTH0),
        error 		=> error_out(NORTH0)
    );	
    
    DECNORTH0_BUFFER: entity work.deccrc
		port map(
			input => data_buffer(NORTH0)(15 downto 0),
			crc_in => data_buffer(NORTH0)(19 downto 16),
			rx => rx(NORTH0),
			error => error_buffer(NORTH0));
    
    --DECNORTH0: entity work.deccrc
		--port map(
			--input => data_in(NORTH0)(15 downto 0),
			--crc_in => data_in(NORTH0)(19 downto 16),
			--rx => rx(NORTH0),	
			--error => error(NORTH0));
			
				
	DECNorth0_stepdet: entity work.stepdet
        port map(
            ck          => clock,
            rst         => reset_stepdet(NORTH0),
            in_sig      => in_error_stepdet(NORTH0),
            out_sig     => error_out(NORTH0)
        );	
   
    Buffer_North1: Entity work.input_buffer
    generic	map (buffer_type => input_buffers)
    port map(
        clock 		=> clock,
        reset 		=> reset,
        data_in 	=> data_in(NORTH1),
        rx 			=> rx(NORTH1),
        eop_in 		=> eop_in(NORTH1),
        req_routing => req_routing(NORTH1),
        ack_routing => ack_routing(NORTH1),	
        tx 			=> tx_buffer(NORTH1),
        eop_out 	=> eop_buffer(NORTH1),
        data_out	=> data_buffer(NORTH1),
        sender		=> sender(NORTH1),	  
        credit_in 	=> credit_buffer(NORTH1),
        next_flit 	=> next_flit(NORTH1),	
        credit_out 	=> credit_monitor(NORTH1),
        fail_detect => fail_detect(NORTH1),
        error 		=> error_out(NORTH1)
    );
    
    DECNORTH1_BUFFER: entity work.deccrc
		port map(
			input => data_buffer(NORTH1)(15 downto 0),
			crc_in => data_buffer(NORTH1)(19 downto 16),
			rx => rx(NORTH1),
			error => error_buffer(NORTH1));
			
	--DECNORTH1: entity work.deccrc
		--port map(
			--input => data_in(NORTH1)(15 downto 0),
			--crc_in => data_in(NORTH1)(19 downto 16),
			--rx => rx(NORTH1),
			--error => error(NORTH1));		
		
	DECNorth1_stepdet: entity work.stepdet
        port map(
            ck          => clock,
            rst         => reset_stepdet(NORTH1),
            in_sig      => in_error_stepdet(NORTH1),
            out_sig     => error_out(NORTH1)
        );
       
    mux_control: entity work.mux_control
    port map(
        clock           => clock,
        reset		    => reset,
        in_buffer       => data_buffer,
        out_buffer      => out_mux_buffer,
        mux_buffer_ctrl => mux_buffer_ctrl,
        tx_buffer       => tx_buffer,
        credit_buffer   => credit_buffer,
        ack_backtrack   => ack_backtrack,
        seek_resend_sr	=> seek_resend_sr_router,--fochi
        flit_num        => flit_num,
        enable_mux      => enable_mux,
        enable_shift    => enable_shift,
        mask            => mask
    );    
	
	Switch_Control: Entity work.Switch_Control 
	generic map( 
		address 		=> address
	)
	port map(
	    clock 		        => clock,
	    reset 		        => reset,
	    req_routing         => req_routing,
	    ack_routing         => ack_routing,	  
	    data 		        => data_buffer,
	    sender 		        => sender,
		next_flit	        => next_flit,	 
		table 		        => table,
        req_backtrack       => req_backtrack,
        ack_backtrack       => ack_backtrack,
        source              => source,
        mask                => mask,
        seek_resend_sr		=> seek_resend_sr_router,--fochi
        flit_pos            => flit_pos,
        mux_buffer_ctrl     => mux_buffer_ctrl,
        enable_mux          => enable_mux,
        enable_shift        => enable_shift,
        port_dir            => port_dir,
        in_failed_port		=> in_failed_port
	);

    Crossbar: Entity work.CrossbarCC 
    port map (
        table		=> table,
        data		=> out_mux_buffer,
        eop_in		=> eop_buffer,
        data_av		=> tx_buffer,
        data_ack	=> credit_buffer,
        data_out	=> data_out_monitor,
        tx			=> tx_monitor,
        eop_out		=> eop_monitor,
        error_c 	=> error_out,
        credit_i	=> credit_in
    );
    

    -- data_out_monitor, eop_monitor and tx_monitor are intermidiate signals for monitoring purposes
    -- since we can't read an output port
    data_out    <= data_out_monitor;
    tx          <= tx_monitor;
    eop_out     <= eop_monitor;
    credit_out  <= credit_monitor;

    failed_reception    <= fail_detect(EAST0)   when table(EAST0)(LOCAL0) = '1' or table(EAST0)(LOCAL1) = '1' else
                           fail_detect(EAST1)   when table(EAST1)(LOCAL0) = '1' or table(EAST1)(LOCAL1) = '1' else
                           fail_detect(WEST0)   when table(WEST0)(LOCAL0) = '1' or table(WEST0)(LOCAL1) = '1' else
                           fail_detect(WEST1)   when table(WEST1)(LOCAL0) = '1' or table(WEST1)(LOCAL1) = '1' else
                           fail_detect(NORTH0)  when table(NORTH0)(LOCAL0) = '1' or table(NORTH0)(LOCAL1) = '1' else
                           fail_detect(NORTH1)  when table(NORTH1)(LOCAL0) = '1' or table(NORTH1)(LOCAL1) = '1' else
                           fail_detect(SOUTH0)  when table(SOUTH0)(LOCAL0) = '1' or table(SOUTH0)(LOCAL1) = '1' else
                           fail_detect(SOUTH1)  when table(SOUTH1)(LOCAL0) = '1' or table(SOUTH1)(LOCAL1) = '1' else
                           '0';

    -- Monitors local 1
    Monitor_TX1 : entity work.local_monitor
    generic map
    (
        FILENAME => "mon_local_" & CONV_STRING_8BITS(address) & ".txt"
    )
    port map
    (
        clock => clock,
        reset => reset,
        data  => data_in(LOCAL1),
        ack   => rx(LOCAL1),
        eop   => eop_in(LOCAL1),
        credit => credit_monitor(LOCAL1)
    );
    
    Monitor_RX1 : entity work.local_monitor
    generic map
    (
        FILENAME => "mon_local_" & CONV_STRING_8BITS(address) & ".txt"
    )
    port map
    (
        clock => clock,
        reset => reset,
        data  => data_out_monitor(LOCAL1),
        ack   => tx_monitor(LOCAL1),
        eop   => eop_monitor(LOCAL1),
        credit => credit_in(LOCAL1)
    );

    -- Monitors local 0
    Monitor_TX0 : entity work.local_monitor
    generic map
    (
        FILENAME => "mon_local_" & CONV_STRING_8BITS(address) & ".txt"
    )
    port map
    (
        clock => clock,
        reset => reset,
        data  => data_in(LOCAL0),
        ack   => rx(LOCAL0),
        eop   => eop_in(LOCAL0),
        credit => credit_monitor(LOCAL0)
    );
    
    Monitor_RX0 : entity work.local_monitor
    generic map
    (
        FILENAME => "mon_local_" & CONV_STRING_8BITS(address) & ".txt"
    )
    port map
    (
        clock => clock,
        reset => reset,
        data  => data_out_monitor(LOCAL0),
        ack   => tx_monitor(LOCAL0),
        eop   => eop_monitor(LOCAL0),
        credit => credit_in(LOCAL0)
    );

end Router;
