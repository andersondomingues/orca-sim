------------------------------------------------------------------
---
--- seek router module 17/07/2015
---
--- File function: searches the network for a possible path between a pair of source - target
---
--- Responsibles: Vinicius M. Fochi, Eduardo Wachter, Luciano L. Caimi
--- 
--- Contact: vinicius.fochi@acad.pucrs.br
----
--- 1 fio:  seek
--- 1 fio:  clear
--- 16 fios:  para origem/destino
--- 6 bits: hop counter
--- 3 bits: service
----
------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use IEEE.STD_LOGIC_unsigned.all;
use ieee.numeric_std.all;
use work.HeMPS_defaults.all;

entity router_seek is
		generic (
                router_address        			: regmetadeflit 
        );
		port(
				clock							: in	std_logic;
				reset   						: in	std_logic;		
				in_source_router_seek           : in	regNportNsource_target_neighbor_new;
				in_target_router_seek           : in	regNportNsource_target_neighbor_new;
				in_hop_router_seek				: in	regNportNhop_neighbor_new;
				in_service_router_seek			: in 	regNport_seek_bitN_service;
				in_req_router_seek				: in	std_logic_vector(NPORT_SEEK-1 downto 0);
				in_ack_router_seek				: in	std_logic_vector(NPORT_SEEK-1 downto 0);
				in_nack_router_seek				: in	std_logic_vector(NPORT_SEEK-1 downto 0);
				in_fail_router_seek				: in	std_logic_vector(NPORT_SEEK-1 downto 0);
													                             
				out_req_router_seek				: out	std_logic_vector(NPORT_SEEK-1 downto 0); 
				out_ack_router_seek				: out	std_logic_vector(NPORT_SEEK-1 downto 0);
				out_nack_router_seek			: out	std_logic_vector(NPORT_SEEK-1 downto 0);				
				out_service_router_seek			: out	regNport_seek_bitN_service;
				out_source_router_seek        	: out	regNportNsource_target_neighbor_new;
				out_target_router_seek        	: out	regNportNsource_target_neighbor_new;
				out_hop_router_seek				: out	regNportNhop_neighbor_new				
		    );
end entity;  

architecture router_seek of router_seek is

type T_ea_manager   is (S_INIT, ARBITRATION, TEST_SERVICE, SERVICE_BACKTRACK, BACKTRACK_PROPAGATE, 
						INIT_BACKTRACK, BACKTRACK_WAIT_NEXT, PREPARE_NEXT, 
                        BACKTRACK_MOUNT, CLEAR_TABLE, COMPARE_TARGET, SEND_LOCAL, PROPAGATE, WAIT_ACK_PORTS, INIT_CLEAR, 
                        END_BACKTRACK);

signal EA_manager, PE_manager  : T_ea_manager; 

type T_ea_manager_input   is (S_INIT_INPUT, ARBITRATION_INPUT, LOOK_TABLE_INPUT, TEST_SPACE_AVAIL, SERVICE_INPUT, TABLE_WRITE_INPUT, WRITE_BACKTRACK_INPUT, WRITE_CLEAR_INPUT, WAIT_REQ_DOWN, SEND_NACK);
signal EA_manager_input, PE_manager_input  : T_ea_manager_input;

constant REG_BACKTRACK_SIZE						: integer := 96;

type	source_target_table_type					     			is array (TABLE_SEEK_LENGHT-1 downto 0) of regNsource_target;
type	service_table_type              					 		is array (TABLE_SEEK_LENGHT-1 downto 0) of seek_bitN_service;
type	hop_table_type               								is array (TABLE_SEEK_LENGHT-1 downto 0) of regNhop;
type	backtrack_port_table_type    								is array (TABLE_SEEK_LENGHT-1 downto 0) of std_logic_vector(2 downto 0);
type	source_router_port_table_type    							is array (TABLE_SEEK_LENGHT-1 downto 0) of std_logic_vector(1 downto 0);

signal sel_port,next_port, j										: integer range 0 to 4;
--signal sel_port_1,sel_port_2, sel_port_3, sel_port_4				: integer range 0 to 4; 
signal sel,prox	, free_index, source_index 							: integer range 0 to 7;

signal	int_in_req_router_seek, int_out_ack_router_seek				: std_logic_vector(NPORT_SEEK-1 downto 0);
signal	vector_ack_ports											: std_logic_vector(NPORT_SEEK-2 downto 0);
signal	reg_backtrack												: std_logic_vector(REG_BACKTRACK_SIZE-1 downto 0);			
signal	compare_is_source											: std_logic_vector(TABLE_SEEK_LENGHT-1 downto 0);
signal	pending_table, task	,used_table								: std_logic_vector(TABLE_SEEK_LENGHT-1 downto 0);
signal	compare_bactrack_pending_in_table							: std_logic_vector(TABLE_SEEK_LENGHT-1 downto 0);
signal	backtrack_port												: std_logic_vector(1 downto 0);
signal  req_task, req_int											: std_logic;
signal	in_the_table, space_aval_in_the_table						: std_logic;
signal	is_my_turn_send_backtrack, backtrack_pending_in_table		: std_logic;
signal	source_table	 											: source_target_table_type;
signal	target_table	 											: source_target_table_type;
signal	service_table	 											: service_table_type;
signal	hop_table	 												: hop_table_type;
signal	my_hop_table 												: hop_table_type;
signal	backtrack_port_table										: backtrack_port_table_type; -- porta que chegou a requisição
signal	source_router_port_table									: source_router_port_table_type;

  
begin	

	OUTPUT_PORT_WRITE: for j in 0 to (NPORT_SEEK-1) generate  
		 out_source_router_seek(j)	<=	source_table(sel);  
		 out_target_router_seek(j)	<=	target_table(sel);
		 out_service_router_seek(j)	<=	service_table(sel);	
		 out_hop_router_seek(j)		<=  hop_table(sel);
	end generate;   
   
   COMPARE_SERVICE: for i in 0 to TABLE_SEEK_LENGHT-1 generate
        
		compare_is_source(i) <= '1' when source_table(i) = in_source_router_seek(sel_port) and used_table(i)='1'
									    else '0';
								    
		compare_bactrack_pending_in_table(i) <= '1' when service_table(i) = BACKTRACK_SERVICE and source_table(i) = in_source_router_seek(sel_port) and pending_table(i) = '1' and used_table(i)='1'
									    else '0';	-- precisa para saber se tem um backtrack pendente na tabela nesse caso não pode entrar nenhum outro backtrack							    
    end generate;    
   
	--compare_is_clear <= '1' when in_service_router_seek(sel_port) = CLEAR_SERVICE  else '0';
	--compare_is_bactrack <= '1' when in_service_router_seek(sel_port) = BACKTRACK_SERVICE  else '0';
	req_int <= int_in_req_router_seek(EAST) or int_in_req_router_seek(WEST) or int_in_req_router_seek(NORTH) or int_in_req_router_seek(SOUTH) or int_in_req_router_seek(LOCAL); -- or bit a bit da requisição das portas EAST,SOUTH, NORTH, WEST e LOCAL
	task <= used_table and pending_table;
	is_my_turn_send_backtrack <= '1' when (hop_table(sel) - my_hop_table(sel)) = 1 else '0'; 
	
	--No feasible entries for infix operator "rem".
-- fail wrapper
     int_in_req_router_seek <= in_req_router_seek and not(in_fail_router_seek);
     out_ack_router_seek <= int_out_ack_router_seek or in_fail_router_seek;
	

-- Dependent code: VHDL 2008 -> directive -2008 to compile     
      in_the_table 	<=  or compare_is_source; 
      req_task		<=	or task;
      backtrack_pending_in_table <= or compare_bactrack_pending_in_table; -- precisa para saber se tem um backtrack pendente na tabela nesse caso não pode entrar nenhum outro backtrack							    
	  space_aval_in_the_table <=  nand used_table;	
 	      

-- Table lenght dependent 
    ----  só usa este índice quando space_aval_in_the_table for '1' indica indice da posição livre na tabela
    free_index <= 	0   when used_table(0)='0'  else
					1   when used_table(1)='0'  else
					2   when used_table(2)='0'  else
					3   when used_table(3)='0'  else
					4   when used_table(4)='0'  else
					5   when used_table(5)='0'  else
					6   when used_table(6)='0'  else
					7   ;

---- indice do source encontrado na tabela 
    source_index <= 0   when compare_is_source(0)='1'  else
					1   when compare_is_source(1)='1'  else
					2   when compare_is_source(2)='1'  else
					3   when compare_is_source(3)='1'  else
					4   when compare_is_source(4)='1'  else
					5   when compare_is_source(5)='1'  else
					6   when compare_is_source(6)='1'  else
					7   ; 
					
process(reset, clock)--processo que trata a logica do proximo estado do FSM manager
begin
			   if reset = '1' then
					EA_manager_input  <= S_INIT_INPUT;				
			   elsif clock'event and clock='1' then     
					EA_manager_input <= PE_manager_input;
		       end if;
end process;

----
----  INPUT TABLE DATA PATH  - lógica sequencial do circuito de inserção na tabela
----
process(clock, reset)--processo que gerencia cada estado manager
begin
		if reset = '1' then
			int_out_ack_router_seek 	<= (others => '0');
			sel_port             		<= LOCAL;

			-- 07/10
			source_table	 			<= (others => (others => '0'));
			target_table	 			<= (others => (others => '0'));
			service_table	 			<= (others => (others => '0'));
			hop_table	 				<= (others => (others => '0'));
			backtrack_port_table		<= (others => (others => '0'));
			source_router_port_table	<= (others => (others => '0'));
 			pending_table				<= (others => '0');
 			used_table					<= (others => '0');	
 			my_hop_table		 		<= (others => (others => '0'));	
			int_out_ack_router_seek(sel_port) <= '0';
			out_nack_router_seek(sel_port) 	  <= '0';		
					
		elsif clock'event and clock='1' then   
				case EA_manager_input is
					when S_INIT_INPUT =>	
						int_out_ack_router_seek(sel_port) <= '0';
					    out_nack_router_seek(sel_port) <= '0';
					
					when WAIT_REQ_DOWN	=>
						
					when ARBITRATION_INPUT =>
						sel_port <= next_port;				
						
					when LOOK_TABLE_INPUT =>	

					when TEST_SPACE_AVAIL =>
						if in_the_table = '1' and in_service_router_seek(sel_port) /= BACKTRACK_SERVICE then
							int_out_ack_router_seek(sel_port) <= '1';	
						elsif in_the_table = '0' and in_service_router_seek(sel_port) = CLEAR_SERVICE then
							int_out_ack_router_seek(sel_port) <= '1';
						end if;
									
					
					when TABLE_WRITE_INPUT =>			
						int_out_ack_router_seek(sel_port) <= '1';					
						
						source_table(free_index) 				<=	in_source_router_seek(sel_port);   	
						target_table(free_index) 				<=	in_target_router_seek(sel_port);   	
						service_table(free_index) 				<=  in_service_router_seek(sel_port);
						hop_table(free_index) 					<=  in_hop_router_seek(sel_port)+1;
						backtrack_port_table(free_index)		<=	std_logic_vector(to_unsigned(sel_port, 3));
						pending_table(free_index)				<= '1';
						used_table(free_index)					<= '1';												
										
					when WRITE_CLEAR_INPUT =>	
						service_table(source_index) 			<=  CLEAR_SERVICE;	
						pending_table(source_index)				<= '1';
						int_out_ack_router_seek(sel_port) 		<= '1';
						
					when WRITE_BACKTRACK_INPUT =>						
						int_out_ack_router_seek(sel_port) <= '1';
						service_table(source_index) 				<= BACKTRACK_SERVICE;
						target_table(source_index)					<= in_target_router_seek(sel_port);
						hop_table(source_index) 					<= in_hop_router_seek(sel_port);
						pending_table(source_index)					<= '1';
						my_hop_table(source_index)					<= hop_table(source_index);
						source_router_port_table(source_index)		<= std_logic_vector(to_unsigned(sel_port, 2));
						
					when SEND_NACK =>
						out_nack_router_seek(sel_port) <= '1';	
						
					when others => 
					
				end case;	

---------------------------------------------------------------------------------------------------------------
				case  EA_manager is             
				    when PROPAGATE =>   
			            pending_table(sel)	<=  '0';
			
				    when CLEAR_TABLE =>   
			            pending_table(sel)		<=  '0';
						used_table(sel)			<=  '0';	

					when SEND_LOCAL =>									
			            pending_table(sel)		<=  '0';

					when INIT_BACKTRACK =>	
						target_table(sel) 		<= (SOURCE_TARGET_SIZE-1 downto 2 => '0') & backtrack_port_table(sel)(1 DOWNTO 0);   	
						service_table(sel) 		<= BACKTRACK_SERVICE;	
						pending_table(sel)		<= '1';									
					
					when PREPARE_NEXT => 
						--if	is_my_turn_send_backtrack = '0' then
							--pending_table(sel)		<=  '0';
							
						--else
							--pending_table(sel)		<= '1';
						--	report "PREPARE_NEXT ------------------------------------" 	severity failure; 
						--end if;
						pending_table(sel)		<=  is_my_turn_send_backtrack;  -- veja como funciona acima
						target_table(sel)		<=  (SOURCE_TARGET_SIZE-1 downto 2 => '0') & source_router_port_table(sel);
						hop_table(sel) 			<=  my_hop_table(sel);				
						 
					when BACKTRACK_MOUNT =>	
			            pending_table(sel)		<=  '0';
			            hop_table(sel) 			<=  my_hop_table(sel);		            
			            
			        when INIT_CLEAR =>
						service_table(sel) 		<=  CLEAR_SERVICE;	
						pending_table(sel)		<= '1';

					when others => null;

				end case;
		end if;
end process;			

----
----  INPUT TABLE FSM process  2 - computa o próximo estado a partir dos valores atuais de estado e sinais de controle - lógica combinacional
----
--process(EA_manage, todos os sinais de if)
process(EA_manager_input, req_int, in_the_table, space_aval_in_the_table, backtrack_pending_in_table,in_req_router_seek)
begin
		case EA_manager_input is		
		
		
		
			when S_INIT_INPUT =>			
				if req_int = '1' then 
					PE_manager_input	<= 	ARBITRATION_INPUT;						
				else
					PE_manager_input	<=	S_INIT_INPUT;	 
				end if;				

								
			when ARBITRATION_INPUT =>				
				PE_manager_input <= LOOK_TABLE_INPUT;
				

			when LOOK_TABLE_INPUT =>			
				PE_manager_input <= TEST_SPACE_AVAIL;
			
			when TEST_SPACE_AVAIL =>
				if space_aval_in_the_table = '1' then 								
					PE_manager_input <= SERVICE_INPUT;								
				else 																
					PE_manager_input <= WAIT_REQ_DOWN;							
				end if;
				
			when SERVICE_INPUT =>	
				if in_service_router_seek(sel_port) = BACKTRACK_SERVICE  and backtrack_pending_in_table = '0'  then
					PE_manager_input <= WRITE_BACKTRACK_INPUT;
				elsif in_service_router_seek(sel_port) = BACKTRACK_SERVICE  and backtrack_pending_in_table = '1'  then
					PE_manager_input <= SEND_NACK;			
				elsif in_service_router_seek(sel_port) = CLEAR_SERVICE and in_the_table = '1' then 
					PE_manager_input <= WRITE_CLEAR_INPUT; 
				elsif in_service_router_seek(sel_port) /= CLEAR_SERVICE  and in_the_table = '0' then -- não é um serviço de clear e ele não está na tabela // escreve ele na tabela
					PE_manager_input <= TABLE_WRITE_INPUT;				
				else 
					PE_manager_input <= WAIT_REQ_DOWN;
				end if;				
				
			when SEND_NACK =>
					PE_manager_input <= WAIT_REQ_DOWN;
				
			when WAIT_REQ_DOWN	=>
				if in_req_router_seek(sel_port) = '1' then 
					PE_manager_input	<= 	WAIT_REQ_DOWN;						
				else
					PE_manager_input	<=	S_INIT_INPUT;	 
				end if;
				
			when others =>
				PE_manager_input	<=	WAIT_REQ_DOWN;			
		end case;
end process;		

----
----  PROCESSING DATA PATH  - lógica sequencial - contém todos dos registradores necessários ao circuito 
----
process(clock, reset)--processo que gerencia cada estado manager
	variable 	aux_backtrack_port		: integer range 0 to 3;

begin
		if reset = '1' then
					sel             				<= ROW0;
					vector_ack_ports			    <= (others => '0');
					reg_backtrack					<= (others => '0');
					
		elsif clock'event and clock='1' then   
				case EA_manager is
					when S_INIT =>	
						out_req_router_seek 					<= (others => '0');
						vector_ack_ports			    		<= (others => '0');

					when ARBITRATION =>
							sel <= prox;						
											     	                        
					when TEST_SERVICE=>    					
						backtrack_port <= backtrack_port_table(sel)(1 DOWNTO 0);

					when BACKTRACK_MOUNT =>			
						reg_backtrack <= reg_backtrack(REG_BACKTRACK_SIZE - 3 downto 0) & target_table(sel)(1 downto 0);		            					
					
					when END_BACKTRACK =>	
						reg_backtrack <= reg_backtrack(REG_BACKTRACK_SIZE - 3 downto 0) & source_router_port_table(sel);
						
					when BACKTRACK_PROPAGATE =>							
						out_req_router_seek(to_integer(unsigned(backtrack_port)))	<= '1';
										
					when PREPARE_NEXT =>
						out_req_router_seek(to_integer(unsigned(backtrack_port))) 	<= '0';						
						
					when SEND_LOCAL =>
						out_req_router_seek(LOCAL)     	<= '1';
						
					when PROPAGATE =>							
						for j in 0 to (NPORT_SEEK-2) loop  
							if (j /= to_integer(unsigned((backtrack_port_table(sel)))) )  then                       
								out_req_router_seek(j)	<= '1';
							end if;       
						end loop;	
					
					when WAIT_ACK_PORTS =>						
						if sel /= LOCAL then
							vector_ack_ports(to_integer(unsigned(backtrack_port_table(sel)(1 DOWNTO 0)))) <= '1';
						end if;
						for j in 0 to (NPORT_SEEK-2) loop  
							if (in_ack_router_seek(j) = '1') then
								vector_ack_ports(j) <= '1';
								out_req_router_seek(j) <= '0';
							end if;
						end loop;
						
					when others => null;				
				end case;
		end if;
end process;

----
----  PROCESSING DATA FSM process 1 - atribui o proximo estado calculado ao estado atual - lógica sequencial
----
process(reset, clock)--processo que trata a logica do proximo estado do FSM manager
begin
			   if reset = '1' then
					EA_manager  <= S_INIT;				
			   elsif clock'event and clock='1' then     
					EA_manager <= PE_manager;
		       end if;
end process;

----
---- PROCESSING DATA FSM process  2 - computa o próximo estado a partir dos valores atuais de estado e sinais de controle - lógica compinacional
----
--process(EA_manage, todos os sinais de if)
process(EA_manager, req_task , source_table, in_the_table, space_aval_in_the_table, in_ack_router_seek, in_nack_router_seek, vector_ack_ports, int_in_req_router_seek, service_table, target_table, hop_table, is_my_turn_send_backtrack)
begin
		case EA_manager is
			when S_INIT =>			
				if req_task = '1' then 
					PE_manager	<= 	ARBITRATION;						
				else
					PE_manager	<=	S_INIT;	 
				end if;	
				
			when ARBITRATION =>
				PE_manager <= TEST_SERVICE;	
				
					 
			when TEST_SERVICE=>
				if service_table(sel) = CLEAR_SERVICE then
					PE_manager <= CLEAR_TABLE;
				elsif service_table(sel) = BACKTRACK_SERVICE then
					PE_manager <= SERVICE_BACKTRACK;
				else
					PE_manager <= COMPARE_TARGET;
				end if;
	 
			when CLEAR_TABLE =>		
				PE_manager <= PROPAGATE;
				
			when COMPARE_TARGET =>
				if target_table(sel) /= router_address then   
					PE_manager <= PROPAGATE;
				elsif service_table(sel) = SEARCHPATH_SERVICE and target_table(sel) = router_address  then -- é o target e tem que disparar o backtrack
					PE_manager <= INIT_BACKTRACK;
				else
					PE_manager <= SEND_LOCAL;
				end if;	

			when INIT_BACKTRACK =>
				PE_manager <= S_INIT;
				report "INIT_BACKTRACK  : " & CONV_STRING_8BITS(in_source_router_seek(sel_port)) & " to " & CONV_STRING_8BITS(in_target_router_seek(sel_port));
							
			when SERVICE_BACKTRACK =>
				if  source_table(sel) = router_address then
					PE_manager <= BACKTRACK_MOUNT;
				else
					PE_manager <= BACKTRACK_PROPAGATE;
				end if;					
	
			when BACKTRACK_MOUNT =>			
				if hop_table(sel) = "000010"   then
					PE_manager <= END_BACKTRACK;	
				else
					--report "BACKTRACK_MOUNT" severity note;		
					PE_manager <= S_INIT;
				end if;	
				
			when END_BACKTRACK =>	
					PE_manager <= INIT_CLEAR;
								
			when INIT_CLEAR =>
				PE_manager <= S_INIT;
			
				report "CLEAR: " & CONV_STRING_32BITS(reg_backtrack(63 downto 32)) & CONV_STRING_32BITS(reg_backtrack(31 downto 0))& " " & CONV_STRING_4BITS(router_address(7 downto 4)) & " "  & CONV_STRING_4BITS(router_address(3 downto 0)) & lf;
								
			when BACKTRACK_PROPAGATE =>				 
				if in_ack_router_seek(to_integer(unsigned(backtrack_port))) = '1' then
					PE_manager <= PREPARE_NEXT;					
				elsif in_nack_router_seek(to_integer(unsigned(backtrack_port))) = '1' then
					PE_manager <= S_INIT;
					--report "BACKTRACK NACK" ;
				else 
					PE_manager <= BACKTRACK_PROPAGATE; 
				end if;				
												
			when PREPARE_NEXT => 
					PE_manager <= S_INIT;		 						
		
			when SEND_LOCAL =>
				if (in_ack_router_seek(LOCAL) = '1' ) then
					PE_manager <= S_INIT;
					report "SEND LOCAL: " & integer'image(to_integer(unsigned(vector_ack_ports)));			
				else 
					PE_manager <= SEND_LOCAL;
				end if;
				
			when PROPAGATE =>	
				PE_manager <= WAIT_ACK_PORTS;	
				
			when WAIT_ACK_PORTS =>
				if (vector_ack_ports = "1111") then  	
					PE_manager <= S_INIT;
				else
					PE_manager <= WAIT_ACK_PORTS;
				end if;
				
			when others => 
				PE_manager <= S_INIT;
		end case;
end process;

	--sel_port_1 <= (sel_port+1) rem 5;
	--sel_port_2 <= (sel_port+2) rem 5;
	--sel_port_3 <= (sel_port+3) rem 5;
	--sel_port_4 <= (sel_port+4) rem 5;

--next_port <=  sel_port_1 when int_in_req_router_seek(sel_port_1)='1' else
              --sel_port_2 when int_in_req_router_seek(sel_port_2)='1' else
              --sel_port_3 when int_in_req_router_seek(sel_port_3)='1' else
              --sel_port_4 when int_in_req_router_seek(sel_port_4)='1' else
             --sel_port;

process(sel_port, int_in_req_router_seek ) -- ROUND ROBIN
	begin
		case sel_port is        
				when EAST => 
						if 	  int_in_req_router_seek(WEST)='1' then  next_port<=WEST;
						elsif int_in_req_router_seek(NORTH)='1' then next_port<=NORTH; 
						elsif int_in_req_router_seek(SOUTH)='1' then next_port<=SOUTH;
						elsif int_in_req_router_seek(LOCAL)='1' then next_port<=LOCAL;
						else next_port<=EAST; end if;					
												
				when WEST => 
						if 	  int_in_req_router_seek(NORTH)='1' then next_port<=NORTH;
						elsif int_in_req_router_seek(SOUTH)='1' then next_port<=SOUTH;
						elsif int_in_req_router_seek(LOCAL)='1' then next_port<=LOCAL;
						elsif int_in_req_router_seek(EAST)='1' then  next_port<=EAST;
						else next_port<=WEST; end if;				
												
				when NORTH =>				
						if    int_in_req_router_seek(SOUTH)='1' then next_port<=SOUTH;
						elsif int_in_req_router_seek(LOCAL)='1' then next_port<=LOCAL;
						elsif int_in_req_router_seek(EAST)='1' then  next_port<=EAST;
						elsif int_in_req_router_seek(WEST)='1' then  next_port<=WEST;
						else next_port<=NORTH; end if;			
										
				when SOUTH => 
						if    int_in_req_router_seek(LOCAL)='1' then next_port<=LOCAL;
						elsif int_in_req_router_seek(EAST)='1' then  next_port<=EAST; 
						elsif int_in_req_router_seek(WEST)='1' then  next_port<=WEST;
						elsif int_in_req_router_seek(NORTH)='1' then next_port<=NORTH;
						else next_port<=SOUTH; end if;
						
				when LOCAL => 
						if    int_in_req_router_seek(EAST)='1' then  next_port<=EAST; 
						elsif int_in_req_router_seek(WEST)='1' then  next_port<=WEST;
						elsif int_in_req_router_seek(NORTH)='1' then next_port<=NORTH;
						elsif int_in_req_router_seek(SOUTH)='1' then next_port<=SOUTH;
						else next_port<=LOCAL; end if;	
							
				when others => null;
		end case;
end process;

process(sel,task)-- Round robin for table
begin
		case sel is
            when ROW0 => 
				if task(ROW1)='1' then    prox<=ROW1;
				elsif task(ROW2)='1' then prox<=ROW2;
				elsif task(ROW3)='1' then prox<=ROW3;
                elsif task(ROW4)='1' then prox<=ROW4;
				elsif task(ROW5)='1' then prox<=ROW5;
                elsif task(ROW6)='1' then prox<=ROW6; 
				elsif task(ROW7)='1' then prox<=ROW7;
                else prox<=ROW0; end if;
                
            when ROW1 => 
				if task(ROW2)='1' then prox<=ROW2;
				elsif task(ROW3)='1' then prox<=ROW3;
			    elsif task(ROW4)='1' then  prox<=ROW4;
				elsif task(ROW5)='1' then  prox<=ROW5;
			    elsif task(ROW6)='1' then prox<=ROW6; 
				elsif task(ROW7)='1' then prox<=ROW7;
				elsif task(ROW0)='1' then prox<=ROW0;
                else prox<=ROW1; end if;
								
            when ROW2 => 
				if    task(ROW3)='1' then prox<=ROW3;
				elsif task(ROW4)='1' then  prox<=ROW4;
				elsif task(ROW5)='1' then  prox<=ROW5;
                elsif task(ROW6)='1' then prox<=ROW6; 
				elsif task(ROW7)='1' then prox<=ROW7;
				elsif task(ROW0)='1' then prox<=ROW0;
				elsif task(ROW1)='1' then prox<=ROW1;
				else prox<=ROW2; end if;
					
			when ROW3 => 
				if    task(ROW4)='1' then  prox<=ROW4;
				elsif task(ROW5)='1' then  prox<=ROW5;
                elsif task(ROW6)='1' then prox<=ROW6; 
				elsif task(ROW7)='1' then prox<=ROW7;
				elsif task(ROW0)='1' then prox<=ROW0;
				elsif task(ROW1)='1' then prox<=ROW1;
				elsif task(ROW2)='1' then prox<=ROW2;				
				else prox<=ROW3; end if;
							
            when ROW4 => 
				if    task(ROW5)='1' then  prox<=ROW5;
				elsif task(ROW6)='1' then prox<=ROW6; 
				elsif task(ROW7)='1' then prox<=ROW7;
				elsif task(ROW0)='1' then prox<=ROW0;
				elsif task(ROW1)='1' then prox<=ROW1;
				elsif task(ROW2)='1' then prox<=ROW2;
				elsif task(ROW3)='1' then prox<=ROW3;
				else prox<=ROW4; end if;
					
			when ROW5 => 
				if    task(ROW6)='1' then prox<=ROW6; 
				elsif task(ROW7)='1' then prox<=ROW7;
				elsif task(ROW0)='1' then prox<=ROW0;
				elsif task(ROW1)='1' then prox<=ROW1;
				elsif task(ROW2)='1' then prox<=ROW2;
				elsif task(ROW3)='1' then prox<=ROW3;
				elsif task(ROW4)='1' then  prox<=ROW4;				
				else prox<=ROW5; end if;
							
            when ROW6 =>
				if 	  task(ROW7)='1' then prox<=ROW7;
				elsif task(ROW0)='1' then prox<=ROW0;
				elsif task(ROW1)='1' then prox<=ROW1;
				elsif task(ROW2)='1' then prox<=ROW2;
				elsif task(ROW3)='1' then prox<=ROW3;
				elsif task(ROW4)='1' then  prox<=ROW4;
				elsif task(ROW5)='1' then  prox<=ROW5;
				else prox<=ROW6; end if;
					
			when ROW7 =>
				if    task(ROW0)='1' then prox<=ROW0;
				elsif task(ROW1)='1' then prox<=ROW1;
				elsif task(ROW2)='1' then prox<=ROW2;
				elsif task(ROW3)='1' then prox<=ROW3;
				elsif task(ROW4)='1' then  prox<=ROW4;
				elsif task(ROW5)='1' then  prox<=ROW5;
				elsif task(ROW6)='1' then prox<=ROW6;                
				else prox<=ROW7; end if;
			           					
			when others => NULL;
        end case;
 end process;       			
end router_seek;
