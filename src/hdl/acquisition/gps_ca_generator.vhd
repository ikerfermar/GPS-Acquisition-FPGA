library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- gps_ca_generator.vhd
--
-- Generador de codigo C/A GPS L1 segun IS-GPS-200.
-- Registros G1 y G2 de 10 bits con realimentacion Gold.
-- Salida: ca_out = G1(10) xor G2(tap_a) xor G2(tap_b)
--
-- epoch_out='1' se emite en chip[0] del nuevo epoch (1 ciclo
-- despues de chip[1022]) -> el fft_controller puede capturar
-- desde chip[0] obteniendo el pico de correlacion en la fase
-- correcta.
-- =============================================================

entity gps_ca_generator is
    Port (
        clk       : in  STD_LOGIC;
        clk_en    : in  STD_LOGIC;
        reset     : in  STD_LOGIC;
        prn_num   : in  STD_LOGIC_VECTOR(4 downto 0);
        ca_out    : out STD_LOGIC;
        epoch_out : out STD_LOGIC
    );
end gps_ca_generator;

architecture Behavioral of gps_ca_generator is

    signal g1          : std_logic_vector(1 to 10) := (others => '1');
    signal g2          : std_logic_vector(1 to 10) := (others => '1');
    signal bit_counter : integer range 0 to 1022 := 0;
    signal epoch_next  : std_logic := '0';
    signal epoch_out_r : std_logic := '0';

    -- Tablas de taps G2 (IS-GPS-200, PRNs 1-32 -> indices 0-31)
    function get_tap_a(prn : std_logic_vector(4 downto 0)) return integer is
    begin
        case to_integer(unsigned(prn)) is
            when 0  => return 2;  when 1  => return 3;
            when 2  => return 4;  when 3  => return 5;
            when 4  => return 1;  when 5  => return 2;
            when 6  => return 1;  when 7  => return 2;
            when 8  => return 3;  when 9  => return 2;
            when 10 => return 3;  when 11 => return 5;
            when 12 => return 6;  when 13 => return 7;
            when 14 => return 8;  when 15 => return 9;
            when 16 => return 1;  when 17 => return 2;
            when 18 => return 3;  when 19 => return 4;
            when 20 => return 5;  when 21 => return 6;
            when 22 => return 1;  when 23 => return 4;
            when 24 => return 5;  when 25 => return 6;
            when 26 => return 7;  when 27 => return 8;
            when 28 => return 1;  when 29 => return 2;
            when 30 => return 3;  when 31 => return 4;
            when others => return 2;
        end case;
    end function;

    function get_tap_b(prn : std_logic_vector(4 downto 0)) return integer is
    begin
        case to_integer(unsigned(prn)) is
            when 0  => return 6;  when 1  => return 7;
            when 2  => return 8;  when 3  => return 9;
            when 4  => return 9;  when 5  => return 10;
            when 6  => return 8;  when 7  => return 9;
            when 8  => return 10; when 9  => return 3;
            when 10 => return 4;  when 11 => return 6;
            when 12 => return 7;  when 13 => return 8;
            when 14 => return 9;  when 15 => return 10;
            when 16 => return 4;  when 17 => return 5;
            when 18 => return 6;  when 19 => return 7;
            when 20 => return 8;  when 21 => return 9;
            when 22 => return 3;  when 23 => return 6;
            when 24 => return 7;  when 25 => return 8;
            when 26 => return 9;  when 27 => return 10;
            when 28 => return 6;  when 29 => return 7;
            when 30 => return 8;  when 31 => return 9;
            when others => return 6;
        end case;
    end function;

begin

    epoch_out <= epoch_out_r;

    process(clk)
        variable g1_fb, g2_fb, g2_out : std_logic;
    begin
        if rising_edge(clk) then
            if reset = '1' then
                g1          <= (others => '1');
                g2          <= (others => '1');
                bit_counter <= 0;
                epoch_next  <= '0';
                epoch_out_r <= '0';
            else
                if clk_en = '1' then
                    g1_fb  := g1(3) xor g1(10);
                    g1     <= g1_fb & g1(1 to 9);
                    g2_fb  := g2(2) xor g2(3) xor g2(6) xor g2(8) xor g2(9) xor g2(10);
                    g2_out := g2(get_tap_a(prn_num)) xor g2(get_tap_b(prn_num));
                    g2     <= g2_fb & g2(1 to 9);
                    ca_out <= g1(10) xor g2_out;

                    epoch_out_r <= epoch_next;
                    if bit_counter = 1022 then
                        bit_counter <= 0;
                        epoch_next  <= '1';
                    else
                        bit_counter <= bit_counter + 1;
                        epoch_next  <= '0';
                    end if;
                else
                    epoch_out_r <= epoch_out_r;
                    epoch_next <= epoch_next;
                end if;
            end if;
        end if;
    end process;

end Behavioral;