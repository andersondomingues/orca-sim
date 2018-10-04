library IEEE;
    use IEEE.std_logic_1164.all;
    use ieee.std_logic_arith.all;
    use std.textio.all;
    use ieee.std_logic_textio.all;
    use work.HeMPS_defaults.all;
     
entity local_monitor is
    generic 
    (
        FILENAME : string        
    );
    port 
    (
        clock   : in std_logic;
        reset   : in std_logic;
        data    : in regflit;
        ack     : in std_logic;
        eop     : in std_logic;
        credit  : in std_logic
    );
end entity;

architecture local_monitor of local_monitor is
    -- Counters
    signal clock_counter : integer := 0;
       
    -- Packet information
    type packet_manager is (START, FLIT_ADDRESS, FLIT_SIZE, FLIT_DATA, SERVICE1, SERVICE0, SOURCE1, SOURCE0, SOURCE_ROUTING_PACKET, SEEK);
    signal current_state : packet_manager;    
    
    function print_service(service : std_logic_vector(19 downto 0))
        return string is
    begin
        case service is
            when x"00010"    => return "MESSAGE_REQUEST";
            when x"00020"    => return "MESSAGE_DELIVERY";
            when x"00030"    => return "LOCATION_ANSWER";
            when x"00040"    => return "TASK_ALLOCATION";
            when x"00050"    => return "TASK_ALLOCATED";
            when x"00060"    => return "TASK_REQUEST";
            when x"00070"    => return "TASK_TERMINATED";
            when x"00080"    => return "TASK_DEALLOCATED";
            when x"00100"    => return "DEBUG_MESSAGE";
            when x"00120"    => return "LOCATION_REQUEST";
            when x"00130"    => return "NEW_TASK";
            when others     => return "UNKNOWN SERVICE";
        end case;
    end print_service;
begin

    -- Clock Counter
    process(clock, reset)
    begin
        if reset = '1' then
            clock_counter <= 0;
        elsif rising_edge(clock) then
            clock_counter <= clock_counter + 1;
        end if;
    end process;
    
    -- Updates the state
    -- Reads the packet
    process(clock)
        file     outfile : text is out FILENAME;
        variable outline : line;
        variable source_routing: std_logic;
    begin    
        if reset = '1' then
            current_state <= START;
            source_routing := '0';
        elsif rising_edge(clock) then
            case current_state is
                when START =>
                    current_state <= FLIT_ADDRESS;
                    
                when FLIT_ADDRESS =>
                    if ack = '1' and credit = '1' then
                        write(outline, clock_counter);
                        write(outline, string'(" | "));

                        --if flit type is source routing (6), keep printing
                        if( data(15 downto 12) = x"7") then
                            hwrite(outline, data);
                            write(outline, string'(" "));
                            current_state <= SOURCE_ROUTING_PACKET;
                        elsif( data(15 downto 12) = x"6") then
                            write(outline, string'("SEEK "));
                            hwrite(outline, data);
                            write(outline, string'(" "));
                            current_state <= SEEK;
                        else
                            hwrite(outline, data);
                            write(outline, string'(" "));
                            current_state <= FLIT_SIZE;
                        end if;
                    end if;
                    
                when FLIT_SIZE =>
                    if ack = '1' and credit = '1' then
                        current_state <= SERVICE1;
                    end if;
                
                when SEEK =>
                    if ack = '1' and credit = '1' then
                        hwrite(outline, data);
                        write(outline, string'(" "));
                        
                        if eop = '1' then
                            write(outline, string'("| "));
                            write(outline, clock_counter);
                            writeline(outfile, outline);
                            current_state <= FLIT_ADDRESS;
                        end if;
                    end if;

                when SOURCE_ROUTING_PACKET =>
                    --if it is not source_routing anymore, the header is finished
                    if ack = '1' and credit = '1' then 
                        if( data(15 downto 12) /= x"7") then
                            current_state <= SERVICE1;
                        end if;
                        
                        hwrite(outline, data);
                        write(outline, string'(" "));
                    end if;

                when SERVICE1 =>
                    if ack = '1' and credit = '1' then
                        -- hwrite(outline, data);
                        -- write(outline, string'(" "));
                        current_state <= SERVICE0;
                    end if ;

                when SERVICE0 =>
                    if ack = '1' and credit = '1' then
                        write(outline, print_service(data));
                        write(outline, string'(" "));
                        current_state <= SOURCE1;
                    end if ;

                when SOURCE1 =>
                    if ack = '1' and credit = '1' then
                        -- hwrite(outline, data);
                        -- write(outline, string'(" "));
                        current_state <= SOURCE0;
                    end if ;

                when SOURCE0 =>
                    if ack = '1' and credit = '1' then
                        hwrite(outline, data);
                        write(outline, string'(" "));
                        current_state <= FLIT_DATA;
                    end if ;

                when FLIT_DATA =>
                    if ack = '1' and credit = '1' then
                        -- hwrite(outline, data);
                        -- write(outline, string'(" "));
                        
                        if eop = '1' then
                            write(outline, string'("| "));
                            write(outline, clock_counter);
                            writeline(outfile, outline);
                            current_state <= FLIT_ADDRESS;
                        end if;
                    end if;
            end case;
        end if;
    end process;
end architecture;