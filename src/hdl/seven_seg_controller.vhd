library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity seven_seg_controller is
    Port (
        clk          : in  STD_LOGIC;
        number_in    : in  STD_LOGIC_VECTOR (5 downto 0); -- Cambiar aquí también
        segments     : out STD_LOGIC_VECTOR (6 downto 0);
        anodes       : out STD_LOGIC_VECTOR (3 downto 0)
    );
end seven_seg_controller;

architecture Behavioral of seven_seg_controller is
    signal refresh_counter : unsigned(17 downto 0) := (others => '0');
    signal digit_select    : std_logic_vector(1 downto 0);
    signal current_digit   : integer range 0 to 9;
begin

    process(clk)
    begin
        if rising_edge(clk) then
            refresh_counter <= refresh_counter + 1;
        end if;
    end process;

    digit_select <= std_logic_vector(refresh_counter(17 downto 16));

    -- Lógica para separar Decenas y Unidades del número de satélite
    process(digit_select, number_in)
        variable full_val : integer;
    begin
        full_val := to_integer(unsigned(number_in));
        case digit_select is
            when "00" => -- Unidades
                current_digit <= full_val rem 10;
                anodes <= "1110"; 
            when "01" => -- Decenas
                current_digit <= full_val / 10;
                anodes <= "1101";
            when others =>
                current_digit <= 0;
                anodes <= "1111"; -- Apagar el resto
        end case;
    end process;

    -- Nuevo mapeo: segments(0)=a, (1)=b, (2)=c, (3)=d, (4)=e, (5)=f, (6)=g
    -- Basys 3: '0' enciende, '1' apaga
    process(current_digit)
    begin
        case current_digit is
            --                 gfedcba
            when 0 => segments <= "1000000"; -- Correcto
            when 1 => segments <= "1111001"; -- Solo b y c encendidos (bits 1 y 2 en '0')
            when 2 => segments <= "0100100"; 
            when 3 => segments <= "0110000"; 
            when 4 => segments <= "0011001"; 
            when 5 => segments <= "0010010"; 
            when 6 => segments <= "0000010"; 
            when 7 => segments <= "1111000"; 
            when 8 => segments <= "0000000"; 
            when 9 => segments <= "0010000"; 
            when others => segments <= "1111111";
        end case;
    end process;

end Behavioral;