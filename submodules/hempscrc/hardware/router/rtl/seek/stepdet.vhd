--------------------------------------------------------------------------
-----------------------------------------------------------------------
-- FSM for the transition detector - gera um pulso de largura de um pulso
-- de clock em função de entrada assíncrona
--------------------------------------------------------------------------
-------------------------------------------------------------------------
library ieee;
use IEEE.std_logic_1164.all;

entity stepdet is
  port(ck,rst: in std_logic;
        in_sig:  in  std_logic;
        out_sig: out std_logic
      );
end stepdet;

architecture stepdet of stepdet is
  type type_state  is ( Swait1, Spulse);--, Swait0);
  signal EA :  type_state;
begin
  --out_sig     <= '1' when EA=Spulse  else '0';

    process(rst, ck)
    begin
        if rst='1' then
            EA          <= Swait1;
            out_sig     <= '0';
        elsif ck'event and ck='1' then
            case EA is
                when Swait1 =>
                    if in_sig='1' then
                        EA <= Spulse;
                    end if;
                    out_sig     <= '0';
                when Spulse =>
                    EA          <= Spulse;
                    out_sig     <= '1';
                --when Swait0 =>      if in_sig='0' then      EA <= Swait1; end if;
            end case;
        end if;
    end process;
end stepdet;
