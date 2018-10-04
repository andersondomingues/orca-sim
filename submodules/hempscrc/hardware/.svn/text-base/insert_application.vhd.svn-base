------------------------------------------------------------------
---
--- HEMPS 4.0  - October 25 2011
---
--- File function: dinamically insert new tasks in the repository - called bu the test bench
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
use work.memory_pack.all;

entity insert_application is
        port (
                clock                   : in std_logic;
                reset                   : in std_logic;
                new_app                 : in std_logic;
                app                             : in ram;
                app_allocated   : out std_logic
        );
end;

architecture insert_application of insert_application is

        
        signal task_cont                : std_logic_vector(31 downto 0);
        signal extra_tasks              : std_logic_vector(31 downto 0);
        signal task_code_size   : std_logic_vector(31 downto 0);        
        signal task_id                  : std_logic_vector(31 downto 0);
        signal i                                : std_logic_vector(31 downto 0);
        signal j                                : std_logic_vector(31 downto 0);
        
        type state_type is (wait_new_app, get_pointers, task_header, write_header1, write_header2, write_code, check, its_over); --write_header2
    signal PE, EA: state_type;

begin

                process(clock, reset)
                begin
                        if (reset='1') then
                                EA <= wait_new_app;
                        elsif (clock'event and clock='1') then
                                EA <= PE;
                        end if;
                end process;
                
                process(EA, PE, new_app, task_cont, extra_tasks, task_code_size, i, j)
                begin
                
                        case EA is
                        
                                when wait_new_app => --espera o sinal new_app que significa que tem uma nova aplicacao a ser inserida
                                        if new_app = '1' then 
                                                PE <= get_pointers; 
                                        else
                                                PE <= wait_new_app;
                                        end if;
                                
                                when get_pointers => --armazena os ponteiros de fim do header da memoria e inicio da area de cod objetos
                                        PE <= task_header; 
                                        
                                when task_header => --obtem as informacoes do header tarefa a ser inserida
                                        PE <= write_header1; --vai pro estado de escrita do header
                                
                                when write_header1 =>
                                        PE <= write_header2; 
                                
                                when write_header2 =>
                                        if      CONV_INTEGER(j)<20 then 
                                                PE <= write_header2; --continua escrevendo o header
                                        else
                                                PE <= write_code; --vai pro estado de escrita do código objeto
                                        end if;
                                        
                                when write_code =>
                                        if((task_code_size-1) = i) then --se terminou escrita do cod objeto
                                                PE <= check; --vai pro estado que checa se acabou de inserir todas tarefas
                                        else --senão
                                                PE <= write_code; --continua escrevendo o código objeto
                                        end if;
                                
                                when check => --verifica se ainda tem tarefas a serem inseridas
                                        if (task_cont=extra_tasks) then --se terminou de inserir as tarefas
                                                PE <= its_over; --vai pro estado final
                                        else --senão
                                                PE <= task_header; -- inclui nova tarefa
                                        end if;
                                                
                                when its_over =>
                                        PE <= wait_new_app; --vai pro início
                        end case;
        
                end process;
                
                process(clock, reset)
                
                        variable mem_tasks : integer := -1;
                        variable mem_header_address : integer := -1;
                        variable mem_code_address : integer := -1;
                        variable task_code_address : integer := -1;
                
                begin
                        
                        if (reset='1') then
                        
                                task_cont <= (others=>'0');
                                mem_tasks := -1;
                                extra_tasks <= (others=>'0');
                                mem_header_address := -1;
                                mem_code_address := -1;
                                task_code_address := -1;
                                app_allocated <= '0';
                                task_code_size <= (others=>'0');
                                task_id <= (others=>'0');
                                i <= (others=>'0');
                                j <= (others=>'0');
                                
                        elsif (clock'event and clock='1') then
                        
                                if  EA=wait_new_app then
                                        app_allocated <= '0';
                                end if;
                        
                                if EA=get_pointers then
                                        mem_tasks := CONV_INTEGER(memory(0)); -- némero de tarefas na memoria
                                        mem_header_address := (mem_tasks*24)+1; --endereço de início do header da memória
                                        mem_code_address :=  CONV_INTEGER(memory((mem_tasks*24)-20))/4; -- endereço de início da área de cód. objeto da memória
                                        extra_tasks <= app(0)-'1'; --número de tarefas extras a serem inseridas
                                end if;
                                
                                if EA=task_header then
                                        task_id <= app((CONV_INTEGER(task_cont)*23)+11); --id da tarefa
                                        task_code_size <= app((CONV_INTEGER(task_cont)*23)+12); --tamanho do código objeto da tarefa extra
                                        task_code_address := (CONV_INTEGER(app((CONV_INTEGER(task_cont)*23)+13))/4); --endereço do código objeto da tarefa na memória extra
                                end if;
                                
                                if EA=write_header1 then
                                        memory(CONV_INTEGER(mem_header_address)+(CONV_INTEGER(task_cont)*24))   <= task_id; -- insere o id da tarefa
                                        memory(CONV_INTEGER(mem_header_address)+(CONV_INTEGER(task_cont)*24)+1) <= task_code_size; --insere o tamanho do cód. objeto da tarefa
                                        memory(CONV_INTEGER(mem_header_address)+(CONV_INTEGER(task_cont)*24)+2) <= x"ffffffff"; --insere o proc. da tarefa
                                        memory(CONV_INTEGER(mem_header_address)+(CONV_INTEGER(task_cont)*24)+3) <= CONV_STD_LOGIC_VECTOR((mem_code_address - CONV_INTEGER(task_code_size)) * 4, 32); --insere o end. do cód. objeto da tarefa
                                        j <= (others=>'0'); --variavel auxiliar
                                end if;
                                
                                if EA=write_header2 then
                                        memory(CONV_INTEGER(mem_header_address)+(CONV_INTEGER(task_cont)*24)+4+CONV_INTEGER(j)) <= app((CONV_INTEGER(task_cont)*23)+14+CONV_INTEGER(j));
                                        j <= j + '1'; --incrementa variável auxiliar
                                        i <= (others=>'0'); --variavel auxiliar
                                end if;
                                
                                if EA=write_code then
                                        memory((mem_code_address - CONV_INTEGER(task_code_size)) + CONV_INTEGER(i) ) <= app(task_code_address + CONV_INTEGER(i)); --escreve cod. objeto
                                        i <= i + '1'; --incrementa variável auxiliar
                                end if;
                                
                                if EA=check then
                                        task_cont <= task_cont + '1'; --atualiza contador de tarefas incluídas
                                        mem_code_address := mem_code_address - CONV_INTEGER(task_code_size); --atualiza posição do ponteiro do cód. obj. da memória
                                end if;
                                
                                if  EA=its_over then
                                        memory(0)<= memory(0) + app(0);
                                        app_allocated <= '1';
                                        -- its over!
                                end if; 
                                
                        end if;
        
                end process;


end insert_application;