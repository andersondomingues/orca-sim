---------------------------------------------------------------------------------------------------
--
-- Title       : Switch Control
-- Design      : QoS
-- Company     : GAPH
--
---------------------------------------------------------------------------------------------------
--
-- File        : Switch_Control.vhd
-- Generated   : Thu Mar  6 17:08:38 2008
-- From        : interface description file
-- By          : Itf2Vhdl ver. 1.20
--
---------------------------------------------------------------------------------------------------
--							   
-- Description : Serves the port requests and controls the ports switching table.
--					
-- Circuit Switching: Connections established only by channels 0 (High priority channel)
---------------------------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_unsigned.all;
use ieee.numeric_std.all;
use work.HeMPS_defaults.all;
							   
entity Switch_Control is
generic( 
	address		: regHalfFlit := (others=>'0')
);
port(
    clock 		    : in  std_logic;
    reset 		    : in  std_logic;  
	req_routing     : in  regNport;
    ack_routing     : out regNport;		
    data 		    : in  arrayNport_regflit;
    sender 		    : in  regNport;	
	next_flit	    : out regNport; 
	table		    : out matrixNportNport_std_logic;
    req_backtrack   : out std_logic;
    ack_backtrack   : in  std_logic;
    source          : out regNsource_target;
    enable_mux      : out regNport;
    enable_shift    : out regNport;
    mask            : out regflit;
    seek_resend_sr  : out std_logic;--fochi
    flit_pos        : in  std_logic_vector(2 downto 0);
    port_dir        : in  std_logic_vector(1 downto 0);
    mux_buffer_ctrl : in  regNport;
    in_failed_port  : in  regNport_neighbor
    );
end Switch_Control;

architecture Switch_Control of Switch_Control is

type state is (S0,S1,S2,S2a,S3,S_WAIT_BACKTRACK,S_WAIT_FLIT);
signal EA: state;

-- sinais do arbitro
signal ask: std_logic;
signal sel,prox: integer range 0 to (NPORT-1);
signal header : regflit;

-- sinais do controle
signal dirx,diry: integer range 0 to (NPORT-1);
signal lx,ly,tx,ty: regQuarterFlit := (others=> '0');
signal free_port: regNport;	 
signal rot_table: matrixNportNport_std_logic;
signal try_again: boolean;
signal even_line: boolean;
signal sr_channel_1: std_logic;


--alias flit_type:std_logic_vector(3 downto 0)    is header(FLIT_WIDTH-1 downto FLIT_WIDTH-4);
--alias target:regHalfFlit                        is header(HALF_FLIT-1 downto 0);
--alias priority:std_logic                        is header(FLIT_WIDTH-8);
--alias routing:std_logic                         is header(FLIT_WIDTH-6);

alias flit_type:std_logic_vector(3 downto 0)    is header(FLIT_WIDTH-5 downto FLIT_WIDTH-8);--fochi
alias target:regHalfFlit                        is header(HALF_FLIT-1 downto 0);
alias priority:std_logic                        is header(FLIT_WIDTH-12);
alias routing:std_logic                         is header(FLIT_WIDTH-10);

-- signals to comunicate with seek module
signal backtrack_path_found : std_logic;

--source routing
alias sr_valid_0:std_logic                           is header(11);
alias sr_channel_0:std_logic                        is header(10);
alias sr_port_0:std_logic_vector(1 downto 0)        is header(9 downto 8);

begin

    ask <=	req_routing(LOCAL0) or req_routing(LOCAL1) or req_routing(EAST0) or req_routing(WEST0) or 
    		req_routing(NORTH0) or req_routing(SOUTH0) or req_routing(EAST1) or req_routing(WEST1) or 
    		req_routing(NORTH1) or req_routing(SOUTH1);
    
    sr_channel_1 <= not(sr_channel_0);
    
	-- Pega o header do pacote selecionado pelo Round Robin
	header <= data(sel);
            
    lx <= address((HALF_FLIT - 1) downto QUARTER_FLIT);
    ly <= address((QUARTER_FLIT - 1) downto 0);

    tx <= header((HALF_FLIT - 1) downto QUARTER_FLIT);
    ty <= header((QUARTER_FLIT - 1) downto 0);
    
    --lx <= address((HALF_FLIT - 3) downto QUARTER_FLIT-1);
    --ly <= address((QUARTER_FLIT - 2) downto 0);

    --tx <= header((HALF_FLIT - 3) downto QUARTER_FLIT-1);
    --ty <= header((QUARTER_FLIT - 2) downto 0);

    dirx <= WEST0 when lx > tx else EAST0;
    diry <= NORTH0 when ly < ty else SOUTH0;
    
    -- Round Robin: seleciona uma das portas de entrada para o roteamento (prox).	
	process(sel,req_routing)
    begin
        case sel is
            when LOCAL0 => 
				if req_routing(LOCAL1)='1' then prox<=LOCAL1;
				elsif req_routing(EAST0)='1' then prox<=EAST0;
				elsif req_routing(EAST1)='1' then prox<=EAST1;
                elsif req_routing(WEST0)='1' then  prox<=WEST0;
				elsif req_routing(WEST1)='1' then  prox<=WEST1;
                elsif req_routing(NORTH0)='1' then prox<=NORTH0; 
				elsif req_routing(NORTH1)='1' then prox<=NORTH1;
                elsif req_routing(SOUTH0)='1' then prox<=SOUTH0;
				elsif req_routing(SOUTH1)='1' then prox<=SOUTH1;				
                else prox<=LOCAL0; end if;
                
            when LOCAL1 => 
				if req_routing(EAST0)='1' then prox<=EAST0;
				elsif req_routing(EAST1)='1' then prox<=EAST1;
			    elsif req_routing(WEST0)='1' then  prox<=WEST0;
				elsif req_routing(WEST1)='1' then  prox<=WEST1;
			    elsif req_routing(NORTH0)='1' then prox<=NORTH0; 
				elsif req_routing(NORTH1)='1' then prox<=NORTH1;
			    elsif req_routing(SOUTH0)='1' then prox<=SOUTH0;
				elsif req_routing(SOUTH1)='1' then prox<=SOUTH1;
				elsif req_routing(LOCAL0)='1' then prox<=LOCAL0;
                else prox<=LOCAL1; end if;
								
            when EAST0 => 
				if req_routing(EAST1)='1' then prox<=EAST1;
				elsif req_routing(WEST0)='1' then  prox<=WEST0;
				elsif req_routing(WEST1)='1' then  prox<=WEST1;
                elsif req_routing(NORTH0)='1' then prox<=NORTH0; 
				elsif req_routing(NORTH1)='1' then prox<=NORTH1;
                elsif req_routing(SOUTH0)='1' then prox<=SOUTH0;
				elsif req_routing(SOUTH1)='1' then prox<=SOUTH1;
				elsif req_routing(LOCAL0)='1' then prox<=LOCAL0;
				elsif req_routing(LOCAL1)='1' then prox<=LOCAL1;
				else prox<=EAST0; end if;
					
			when EAST1 => 
				if req_routing(WEST0)='1' then  prox<=WEST0;
				elsif req_routing(WEST1)='1' then  prox<=WEST1;
                elsif req_routing(NORTH0)='1' then prox<=NORTH0; 
				elsif req_routing(NORTH1)='1' then prox<=NORTH1;
                elsif req_routing(SOUTH0)='1' then prox<=SOUTH0;
				elsif req_routing(SOUTH1)='1' then prox<=SOUTH1;
				elsif req_routing(LOCAL0)='1' then prox<=LOCAL0;
				elsif req_routing(LOCAL1)='1' then prox<=LOCAL1;
				elsif req_routing(EAST0)='1' then prox<=EAST0;				
				else prox<=EAST1; end if;
							
            when WEST0 => 
				if req_routing(WEST1)='1' then  prox<=WEST1;
				elsif req_routing(NORTH0)='1' then prox<=NORTH0; 
				elsif req_routing(NORTH1)='1' then prox<=NORTH1;
                elsif req_routing(SOUTH0)='1' then prox<=SOUTH0;
				elsif req_routing(SOUTH1)='1' then prox<=SOUTH1;
				elsif req_routing(LOCAL0)='1' then prox<=LOCAL0;
				elsif req_routing(LOCAL1)='1' then prox<=LOCAL1;
				elsif req_routing(EAST0)='1' then prox<=EAST0;
				elsif req_routing(EAST1)='1' then prox<=EAST1;
				else prox<=WEST0; end if;
					
			when WEST1 => 
				if req_routing(NORTH0)='1' then prox<=NORTH0; 
				elsif req_routing(NORTH1)='1' then prox<=NORTH1;
                elsif req_routing(SOUTH0)='1' then prox<=SOUTH0;
				elsif req_routing(SOUTH1)='1' then prox<=SOUTH1;
				elsif req_routing(LOCAL0)='1' then prox<=LOCAL0;
				elsif req_routing(LOCAL1)='1' then prox<=LOCAL1;
				elsif req_routing(EAST0)='1' then prox<=EAST0;
				elsif req_routing(EAST1)='1' then prox<=EAST1;
				elsif req_routing(WEST0)='1' then  prox<=WEST0;				
				else prox<=WEST1; end if;
							
            when NORTH0 =>
				if req_routing(NORTH1)='1' then prox<=NORTH1;
                elsif req_routing(SOUTH0)='1' then prox<=SOUTH0;
				elsif req_routing(SOUTH1)='1' then prox<=SOUTH1;
				elsif req_routing(LOCAL0)='1' then prox<=LOCAL0;
				elsif req_routing(LOCAL1)='1' then prox<=LOCAL1;
				elsif req_routing(EAST0)='1' then prox<=EAST0;
				elsif req_routing(EAST1)='1' then prox<=EAST1;
				elsif req_routing(WEST0)='1' then  prox<=WEST0;
				elsif req_routing(WEST1)='1' then  prox<=WEST1;
				else prox<=NORTH0; end if;
					
			when NORTH1 =>
				if req_routing(SOUTH0)='1' then prox<=SOUTH0;
				elsif req_routing(SOUTH1)='1' then prox<=SOUTH1;
				elsif req_routing(LOCAL0)='1' then prox<=LOCAL0;
				elsif req_routing(LOCAL1)='1' then prox<=LOCAL1;
				elsif req_routing(EAST0)='1' then prox<=EAST0;
				elsif req_routing(EAST1)='1' then prox<=EAST1;
				elsif req_routing(WEST0)='1' then  prox<=WEST0;
				elsif req_routing(WEST1)='1' then  prox<=WEST1;
				elsif req_routing(NORTH0)='1' then prox<=NORTH0;                
				else prox<=NORTH1; end if;
						   
             when SOUTH0 => 
				if req_routing(SOUTH1)='1' then prox<=SOUTH1;
				elsif req_routing(LOCAL0)='1' then prox<=LOCAL0;
				elsif req_routing(LOCAL1)='1' then prox<=LOCAL1;
				elsif req_routing(EAST0)='1' then prox<=EAST0;
				elsif req_routing(EAST1)='1' then prox<=EAST1;
				elsif req_routing(WEST0)='1' then  prox<=WEST0;
				elsif req_routing(WEST1)='1' then  prox<=WEST1;
				elsif req_routing(NORTH0)='1' then prox<=NORTH0; 
				elsif req_routing(NORTH1)='1' then prox<=NORTH1;
				else prox<=SOUTH0; end if;
					
			when SOUTH1 => 
				if req_routing(LOCAL0)='1' then prox<=LOCAL0;
				elsif req_routing(LOCAL1)='1' then prox<=LOCAL1;
				elsif req_routing(EAST0)='1' then prox<=EAST0;
				elsif req_routing(EAST1)='1' then prox<=EAST1;
				elsif req_routing(WEST0)='1' then  prox<=WEST0;
				elsif req_routing(WEST1)='1' then  prox<=WEST1;
				elsif req_routing(NORTH0)='1' then prox<=NORTH0; 
				elsif req_routing(NORTH1)='1' then prox<=NORTH1;
				elsif req_routing(SOUTH0)='1' then prox<=SOUTH0;				
				else prox<=SOUTH1; end if;
					
			when others =>
        end case;
    end process; 
     	 
			
	process(clock,reset)
	variable packet_toward_high_label : boolean;
    begin		  	   
		if reset = '1' then
			sel             <= LOCAL0;
            ack_routing     <= (others => '0');                  
			rot_table       <= (others=>(others=>'0'));	
			next_flit       <= (others => '0');
			try_again       <= false;
			EA              <= S0;
            req_backtrack   <= '0';
            seek_resend_sr	<= '0';
            source          <= (others => '0');
            enable_mux      <= (others=>'0');
            mask			<= (others=>'0');
            backtrack_path_found <= '0';
            
                 
		elsif rising_edge(clock) then
	        case EA is
	            -- Takes the port selected by the Round Robin.
				when S0 =>
					-- Wait for a port request.
					if ask = '1' then						
						if try_again and sel /= prox and req_routing(sel) = '1' then
							try_again <= false;
						else
							sel <= prox;
							try_again <= true;
						end if;
						
						EA <= S1;
					else
						EA <= S0;
					end if;
					
					-- Updates the switch table.
					for i in 0 to NPORT-1 loop
						if sender(i) = '0' then
							rot_table(i) <= (others=>'0');
						end if;
					end loop;
                    enable_mux <= (others=>'0');
                    enable_shift <= (others=>'0');
                    backtrack_path_found <= '0';
                    seek_resend_sr	<= '0';
									
				-- Executes the Hamiltonian routing algorithm.
				when S1 =>
                    -- source routing
                    if flit_type /= SOURCE_ROUTING and flit_type /= PACKET_SWITCHING and flit_type /= SOURCE_ROUTING  then -- add by fochi 20/10
                    EA <= S0;    
                    end if;
                    
                    if flit_type = SOURCE_ROUTING then
                        if sr_valid_0 = '0' then --this is a valid flit
                            --if sel (input port) is the same that sr_port_0 (output port) means that it should be routed to local port
                            if (((sel = EAST0 or sel = EAST1) and sr_port_0 = "00") or ((sel = WEST0 or sel = WEST1) and sr_port_0 = "01") or ((sel = NORTH0 or sel = NORTH1) and sr_port_0 = "10") or ((sel = SOUTH0 or sel = SOUTH1) and sr_port_0 = "11")) then
                                if free_port(LOCAL*2+to_integer(unsigned'("" & sr_channel_0))) = '1' then
                                	--report "SOURCE ROUTING PACKET ARRIVED IN DESTINATION:" & CONV_STRING_8BITS(address) severity note;
	                                ack_routing(sel) <= '1';
	                                rot_table(sel)(LOCAL*2+to_integer(unsigned'("" & sr_channel_0))) <= '1';
									EA <= S3;
--										seek_resend_sr	<= '0';
								else 
                                	EA <= S0;
                                end if;
                            elsif free_port(to_integer(unsigned(sr_port_0))*2+to_integer(unsigned'("" & sr_channel_0))) = '1' then--else, try to route to port given by sr_port_0
                                ack_routing(sel) <= '1';
                                rot_table(sel)(to_integer(unsigned(sr_port_0))*2+to_integer(unsigned'("" & sr_channel_0))) <= '1';
								EA <= S3;
								seek_resend_sr	<= '0';
                                enable_shift(sel)<= '1';
                           elsif free_port(to_integer(unsigned(sr_port_0))*2+to_integer(unsigned'("" & sr_channel_1))) = '1' then--else, try to route to port given by sr_port_0
                                ack_routing(sel) <= '1';
                                rot_table(sel)(to_integer(unsigned(sr_port_0))*2+to_integer(unsigned'("" & sr_channel_1))) <= '1';
								EA <= S3;
								seek_resend_sr	<= '1';
                                enable_shift(sel)<= '1';
                                
                            else
                                EA <= S0;
                            end if;
                        else --all sr_valid fields are NOT valid, discart the first flit
                            next_flit(sel) <= '1';
                            EA <= S2;
                    end if;
                        
                    -- seek packet
					elsif flit_type = BACKTRACK_PATH then
                        if target = address then
                            backtrack_path_found <= '1';
                        end if;
                        EA                  <= S_WAIT_BACKTRACK;
                        source              <= target;
                        req_backtrack       <= '1';
                    end if;
                
                    -- Executes the distributed XY routing algorithm
                    if flit_type = PACKET_SWITCHING then
					-- Packet achieved the target
                        if target = address then
                            if free_port(LOCAL1)='1' then
                                rot_table(sel)(LOCAL1) <= '1';
                                ack_routing(sel) <= '1';
                                EA <= S3;                                                                                                    
                            elsif free_port(LOCAL0)='1'then
                                rot_table(sel)(LOCAL0) <= '1';
                                ack_routing(sel) <= '1';
                                EA <= S3;
                            else
                                EA <= S0;
                            end if;
                            
                        -- Packet is switched to EAST or WEST 
                        elsif lx /= tx then
                            if free_port(dirx+1) = '1' then
                                ack_routing(sel) <= '1';
                                rot_table(sel)(dirx+1) <= '1';
                                EA <= S3;
                            elsif free_port(dirx) = '1' then
                                ack_routing(sel) <= '1';
                                rot_table(sel)(dirx) <= '1';
                                EA <= S3;                                
                            -- No free channel
                            elsif free_port(dirx+1) = '0' and free_port(dirx) = '0' then -- add by fochi - se os dois canais estiverem ocupados
								if diry = WEST0 and in_failed_port(WEST0)= '1' and in_failed_port(WEST1) = '1' then -- se for roteado para o west e o enlace estiver falho vai para o ralo
									ack_routing(sel) <= '1';
									rot_table(sel)(dirx) <= '1';
									EA <= S3; 								
								elsif diry = EAST0 and in_failed_port(EAST0)= '1' and in_failed_port(EAST1)= '1'then -- se for roteado para o east e o enlace estiver falho vai para o ralo
									ack_routing(sel) <= '1';
									rot_table(sel)(dirx) <= '1';
									EA <= S3; 
                                else
									EA <= S0; 
								end if;    
                            else 
                                EA <= S0; 
                            end if;
                                          
                        -- Packet is switched to NORTH or SOUTH 
                        -- Verifies if the channel 1 is free
                        elsif free_port(diry+1) = '1' then
                            ack_routing(sel) <= '1';
                            rot_table(sel)(diry+1) <= '1'; 
                            EA <= S3;
                        elsif free_port(diry) = '1' then		
                            ack_routing(sel) <= '1';
                            rot_table(sel)(diry) <= '1'; 
                            EA <= S3;
                         elsif free_port(diry+1) = '0' and free_port(diry) = '0' then -- add by fochi -- se os dois canais estiverem ocupados
								if diry = NORTH0 and in_failed_port(NORTH0) = '1' and in_failed_port(NORTH1) = '1' then -- se for roteado para o norte e o enlace estiver falho vai para o ralo
									ack_routing(sel) <= '1';
									rot_table(sel)(diry) <= '1';
									EA <= S3; 								
								elsif diry = SOUTH0 and in_failed_port(SOUTH0)= '1' and in_failed_port(SOUTH1)= '1' then -- se for roteado para o sul e o enlace estiver falho vai para o ralo
									ack_routing(sel) <= '1';
									rot_table(sel)(diry) <= '1';
									EA <= S3; 
                                else
									EA <= S0; 
								end if; 				
                        else 
                            EA <= S0; 
                        end if;
                        
                     
                   end if;
                    
				-- Time for Input Buffer removes the first flit.
				when S2 =>
					next_flit(sel)<= '0'; 
					EA <= S2a;
					
				-- Waits the request routing for the next multicast destination.
				when S2a =>
					if req_routing(sel) = '1' then
						EA <= S1;
					else
				--							EA <= S1;
					EA <= S2a;
					end if;
				
				-- Time for Input Buffer to low the 'req_routing' signal.
				when S3 =>
					ack_routing(sel)<='0';
					EA <= S0;
                
                -- Waits the ack of backtrack
				when S_WAIT_BACKTRACK =>
                    if ack_backtrack = '1' and ((free_port(to_integer(unsigned(port_dir))*2+1) = '1' and backtrack_path_found = '0') or (free_port(LOCAL1) = '1' and backtrack_path_found = '1')) then
                        if sel = SEEK_PORT then
                            EA <= S3;
                        else
                            EA                      <= S_WAIT_FLIT;
                            enable_mux(sel)         <= '1';
                            case flit_pos is
                                when "000" =>
                                    mask            <= "0000" & std_logic_vector(to_unsigned(sel/2, 2)) & "00000000000000";
                                when "001" =>                                                
                                    mask            <= "000000" & std_logic_vector(to_unsigned(sel/2, 2)) & "000000000000";
                                when "010" =>                                                
                                    mask            <= "00000000" & std_logic_vector(to_unsigned(sel/2, 2)) & "0000000000";
                                when "011" =>                                                
                                    mask            <= "0000000000" & std_logic_vector(to_unsigned(sel/2, 2)) & "00000000";
                                when "100" =>                                                
                                    mask            <= "000000000000" & std_logic_vector(to_unsigned(sel/2, 2)) & "000000";
                                when "101" =>                                                
                                    mask            <= "00000000000000" & std_logic_vector(to_unsigned(sel/2, 2)) & "0000";
                                when "110" =>                                                
                                    mask            <= "0000000000000000" & std_logic_vector(to_unsigned(sel/2, 2)) & "00";
                                when others =>                                               
                                    mask            <= "000000000000000000" & std_logic_vector(to_unsigned(sel/2, 2));     
                            end case;
                            
                        end if;
                        
                        if backtrack_path_found = '0' then
                            --pick the port addressed by port_dir + 1 to pick always the low priority channel
                            rot_table(sel)(to_integer(unsigned(port_dir))*2+1)  <= '1';
                        else
                            rot_table(sel)(LOCAL1)  <= '1';
                        end if;
                        ack_routing(sel)                                    <= '1';
                        req_backtrack                                       <= '0';
                    else
                        EA <= S_WAIT_BACKTRACK;
                        -- Updates the switch table.
                        for i in 0 to NPORT-1 loop
                            if sender(i) = '0' then
                                rot_table(i) <= (others=>'0');
                            end if;
                        end loop;
                    end if;
                    
				when S_WAIT_FLIT =>
                    if mux_buffer_ctrl(sel) = '1' then
                        EA <= S0;
                        ack_routing(sel)<='0';
                    else
                        EA <= S_WAIT_FLIT;
                    end if;
                    enable_mux(sel) <= '1';
                
				when others =>
					EA <= S0;
				
	        end case;  
		end if;
    end process;
    				  
	table <= rot_table;
			
	-- Mantém atualizada a tabela de portas livres.
	free_port(LOCAL0) <= not (rot_table(EAST0)(LOCAL0) or rot_table(EAST1)(LOCAL0) or rot_table(SOUTH0)(LOCAL0) or rot_table(SOUTH1)(LOCAL0) or rot_table(WEST0)(LOCAL0) or rot_table(WEST1)(LOCAL0) or rot_table(NORTH0)(LOCAL0) or rot_table(NORTH1)(LOCAL0)); 
	free_port(LOCAL1) <= not (rot_table(EAST0)(LOCAL1) or rot_table(EAST1)(LOCAL1) or rot_table(SOUTH0)(LOCAL1) or rot_table(SOUTH1)(LOCAL1) or rot_table(WEST0)(LOCAL1) or rot_table(WEST1)(LOCAL1) or rot_table(NORTH0)(LOCAL1) or rot_table(NORTH1)(LOCAL1));
	free_port(EAST0)  <= not (rot_table(LOCAL0)(EAST0) or rot_table(LOCAL1)(EAST0) or rot_table(WEST0)(EAST0) or rot_table(WEST1)(EAST0) or rot_table(NORTH0)(EAST0) or rot_table(NORTH1)(EAST0) or rot_table(SOUTH0)(EAST0) or rot_table(SOUTH1)(EAST0) or in_failed_port(EAST0));
	free_port(EAST1)  <= not (rot_table(LOCAL0)(EAST1) or rot_table(LOCAL1)(EAST1) or rot_table(WEST0)(EAST1) or rot_table(WEST1)(EAST1) or rot_table(NORTH0)(EAST1) or rot_table(NORTH1)(EAST1) or rot_table(SOUTH0)(EAST1) or rot_table(SOUTH1)(EAST1) or in_failed_port(EAST1));
	free_port(WEST0)  <= not (rot_table(LOCAL0)(WEST0) or rot_table(LOCAL1)(WEST0) or rot_table(EAST0)(WEST0) or rot_table(EAST1)(WEST0) or rot_table(SOUTH0)(WEST0) or rot_table(SOUTH1)(WEST0) or rot_table(NORTH0)(WEST0) or rot_table(NORTH1)(WEST0)or in_failed_port(WEST0)); 
	free_port(WEST1)  <= not (rot_table(LOCAL0)(WEST1) or rot_table(LOCAL1)(WEST1) or rot_table(EAST0)(WEST1) or rot_table(EAST1)(WEST1) or rot_table(SOUTH0)(WEST1) or rot_table(SOUTH1)(WEST1) or rot_table(NORTH0)(WEST1) or rot_table(NORTH1)(WEST1)or in_failed_port(WEST1));
	free_port(SOUTH0) <= not (rot_table(LOCAL0)(SOUTH0) OR rot_table(LOCAL1)(SOUTH0) or rot_table(EAST0)(SOUTH0) OR rot_table(EAST1)(SOUTH0) OR rot_table(WEST0)(SOUTH0) OR rot_table(WEST1)(SOUTH0) OR rot_table(NORTH0)(SOUTH0) OR rot_table(NORTH1)(SOUTH0)or in_failed_port(SOUTH0));
	free_port(SOUTH1) <= not (rot_table(LOCAL0)(SOUTH1) OR rot_table(LOCAL1)(SOUTH1) or rot_table(EAST0)(SOUTH1) OR rot_table(EAST1)(SOUTH1) OR rot_table(WEST0)(SOUTH1) OR rot_table(WEST1)(SOUTH1) OR rot_table(NORTH0)(SOUTH1) OR rot_table(NORTH1)(SOUTH1)or in_failed_port(SOUTH1));
	free_port(NORTH0) <= not (rot_table(LOCAL0)(NORTH0) OR rot_table(LOCAL1)(NORTH0) or rot_table(EAST0)(NORTH0) OR rot_table(EAST1)(NORTH0) OR rot_table(WEST0)(NORTH0) OR rot_table(WEST1)(NORTH0) OR rot_table(SOUTH0)(NORTH0) OR rot_table(SOUTH1)(NORTH0)or in_failed_port(NORTH0));
	free_port(NORTH1) <= not (rot_table(LOCAL0)(NORTH1) OR rot_table(LOCAL1)(NORTH1) or rot_table(EAST0)(NORTH1) OR rot_table(EAST1)(NORTH1) OR rot_table(WEST0)(NORTH1) OR rot_table(WEST1)(NORTH1) OR rot_table(SOUTH0)(NORTH1) OR rot_table(SOUTH1)(NORTH1)or in_failed_port(NORTH1));
	
end Switch_Control;	 
