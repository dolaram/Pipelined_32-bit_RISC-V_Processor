
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;


entity PC is
    port(
        clk,rst,en: in std_logic;
        PC_in : in std_logic_vector(31 downto 0);
        PC_out : out std_logic_vector(31 downto 0));
end PC;

architecture Behavioral of PC is

begin
process(clk)
begin
    if(rising_edge(clk)) then
        if(rst = '1') then
            PC_out <= (others => '0');
        elsif(en='1') then
            PC_out <= PC_in;
        end if;
    end if;
end process;
end Behavioral;
