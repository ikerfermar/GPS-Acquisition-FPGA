library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.gps_config_pkg.all;

-- =============================================================
-- multi_sat_rx_gen.vhd
--
-- Generador sintetico de senal GPS L1 C/A, replica el formato
-- del MAX2769C (Device State 2): I-only sign-magnitude (I1/I0),
-- IF=4.092 MHz,
-- Fs=16.368 Msps.
--
-- Hasta 3 satelites en paralelo. Cada satelite genera:
--   BPSK( CA_N(t - phase_N), IF + doppler_N )
-- La senal compuesta se cuantiza en 2 bits sign-magnitude:
--   I1 = signo (0:+, 1:-), I0 = magnitud (0:debil, 1:fuerte).
--
-- SINCRONISMO:
--   clk_en      : avance del CA (= clk_en_fe del top)
--   clk_en_samp : avance del NCO portadora (= sample_en_samp del top)
--
-- NCO portadora por satelite (avanza en clk_en_samp = sample_en_samp ~16.368 MHz):
--   inc_N = IF_INC + doppler_bins_N * DOP_FACTOR
--   El NCO avanza con clk_en_samp='1' -> cada ciclo de 100 MHz (igual que el mixer)
--   IF_INC     = round(4.092e6 / 100e6 * 2^24) = 686524  [igual que doppler_mixer]
--   DOP_FACTOR = round(320 / 100e6 * 2^24) = 54           [pero valor en codigo: 54]
-- =============================================================

entity multi_sat_rx_gen is
    Port (
        clk          : in  STD_LOGIC;
        clk_en       : in  STD_LOGIC;
        clk_en_samp  : in  STD_LOGIC;
        reset        : in  STD_LOGIC;

        sat1_prn     : in  STD_LOGIC_VECTOR(4 downto 0);
        sat1_doppler : in  STD_LOGIC_VECTOR(7 downto 0);
        sat1_phase   : in  STD_LOGIC_VECTOR(9 downto 0);
        sat1_gain    : in  STD_LOGIC_VECTOR(2 downto 0);
        sat1_en      : in  STD_LOGIC;

        sat2_prn     : in  STD_LOGIC_VECTOR(4 downto 0);
        sat2_doppler : in  STD_LOGIC_VECTOR(7 downto 0);
        sat2_phase   : in  STD_LOGIC_VECTOR(9 downto 0);
        sat2_gain    : in  STD_LOGIC_VECTOR(2 downto 0);
        sat2_en      : in  STD_LOGIC;

        sat3_prn     : in  STD_LOGIC_VECTOR(4 downto 0);
        sat3_doppler : in  STD_LOGIC_VECTOR(7 downto 0);
        sat3_phase   : in  STD_LOGIC_VECTOR(9 downto 0);
        sat3_gain    : in  STD_LOGIC_VECTOR(2 downto 0);
        sat3_en      : in  STD_LOGIC;

        i1_out    : out STD_LOGIC;
        i0_out    : out STD_LOGIC;
        rx_out    : out STD_LOGIC;
        epoch_out : out STD_LOGIC
    );
end multi_sat_rx_gen;

architecture Behavioral of multi_sat_rx_gen is

    component gps_ca_generator
        port (clk, clk_en, reset : in std_logic;
              prn_num             : in  std_logic_vector(4 downto 0);
              ca_out, epoch_out   : out std_logic);
    end component;

    signal ca1, ca2, ca3 : std_logic := '0';
    signal ep1           : std_logic := '0';

    signal en1, en2, en3          : std_logic := '0';
    signal cnt1, cnt2, cnt3       : unsigned(9 downto 0) := (others => '0');
    signal pdone1, pdone2, pdone3 : std_logic := '0';
    signal sat1_en_d, sat2_en_d, sat3_en_d : std_logic := '0';

    -- NCO portadora avanza con clk_en_samp='1' -> muestra a muestra a ~16.368 MHz.
    -- IF_INC CORREGIDO: round(4.092e6 / 100e6 * 2^24) = round(686523.68) = 686524
    -- Igual que doppler_mixer.vhd -> la portadora del generador se cancela exactamente
    -- con el NCO del mezclador (ambos avanzan al mismo ritmo en modo sintetico).
    -- DOP_FACTOR = 54: 1 bin Doppler = 320 Hz a 100 MHz (320/100e6 * 2^24 = 53.7 ~ 54)
    constant IF_INC    : unsigned(23 downto 0) := to_unsigned(CFG_IF_NCO_INC, 24);

    signal ph1, ph2, ph3             : unsigned(23 downto 0) := (others => '0');
    signal carrier1, carrier2, carrier3 : std_logic := '0';

    signal bpsk1, bpsk2, bpsk3 : std_logic := '0';
    signal rx_sign              : std_logic := '0';
    signal rx_mag               : std_logic := '0';

    -- x*54 = x*(32+16+4+2), implementado con shifts para evitar mult entero.
    function mul_doppler54(x : signed(7 downto 0)) return signed is
        variable x24 : signed(23 downto 0);
        variable y24 : signed(23 downto 0);
    begin
        x24 := resize(x, 24);
        y24 := shift_left(x24, 5) + shift_left(x24, 4) + shift_left(x24, 2) + shift_left(x24, 1);
        return y24;
    end function;

begin

    ca_gen1 : gps_ca_generator
        port map (clk => clk, clk_en => en1, reset => reset,
                  prn_num => sat1_prn, ca_out => ca1, epoch_out => ep1);

    ca_gen2 : gps_ca_generator
        port map (clk => clk, clk_en => en2, reset => reset,
                  prn_num => sat2_prn, ca_out => ca2, epoch_out => open);

    ca_gen3 : gps_ca_generator
        port map (clk => clk, clk_en => en3, reset => reset,
                  prn_num => sat3_prn, ca_out => ca3, epoch_out => open);

    epoch_out <= ep1;

    -- Retardo de fase inicial (chip rate)
    process(clk)
        variable sat1_phase_u : unsigned(9 downto 0);
        variable sat2_phase_u : unsigned(9 downto 0);
        variable sat3_phase_u : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            sat1_phase_u := unsigned(sat1_phase);
            sat2_phase_u := unsigned(sat2_phase);
            sat3_phase_u := unsigned(sat3_phase);
            if reset = '1' then
                cnt1 <= (others => '0'); pdone1 <= '0';
                cnt2 <= (others => '0'); pdone2 <= '0';
                cnt3 <= (others => '0'); pdone3 <= '0';
                sat1_en_d <= '0';
                sat2_en_d <= '0';
                sat3_en_d <= '0';
                if sat1_phase_u = 0 then pdone1 <= '1'; end if;
                if sat2_phase_u = 0 then pdone2 <= '1'; end if;
                if sat3_phase_u = 0 then pdone3 <= '1'; end if;
            else
                -- Si un satelite se activa en caliente, reaplicar su retardo de fase.
                if sat1_en_d = '0' and sat1_en = '1' then
                    cnt1 <= (others => '0');
                    if sat1_phase_u = 0 then pdone1 <= '1'; else pdone1 <= '0'; end if;
                end if;
                if sat2_en_d = '0' and sat2_en = '1' then
                    cnt2 <= (others => '0');
                    if sat2_phase_u = 0 then pdone2 <= '1'; else pdone2 <= '0'; end if;
                end if;
                if sat3_en_d = '0' and sat3_en = '1' then
                    cnt3 <= (others => '0');
                    if sat3_phase_u = 0 then pdone3 <= '1'; else pdone3 <= '0'; end if;
                end if;

                if clk_en = '1' then
                    if pdone1 = '0' and sat1_en_d = '1' then
                        -- phase=0 implica sin retardo; evitar underflow en (phase-1)
                        if sat1_phase_u = 0 or cnt1 >= sat1_phase_u - 1 then pdone1 <= '1';
                        else cnt1 <= cnt1 + 1; end if;
                    end if;
                    if pdone2 = '0' and sat2_en_d = '1' then
                        if sat2_phase_u = 0 or cnt2 >= sat2_phase_u - 1 then pdone2 <= '1';
                        else cnt2 <= cnt2 + 1; end if;
                    end if;
                    if pdone3 = '0' and sat3_en_d = '1' then
                        if sat3_phase_u = 0 or cnt3 >= sat3_phase_u - 1 then pdone3 <= '1';
                        else cnt3 <= cnt3 + 1; end if;
                    end if;
                end if;
                sat1_en_d <= sat1_en;
                sat2_en_d <= sat2_en;
                sat3_en_d <= sat3_en;
            end if;
        end if;
    end process;

    en1 <= clk_en and pdone1 and sat1_en;
    en2 <= clk_en and pdone2 and sat2_en;
    en3 <= clk_en and pdone3 and sat3_en;

    -- NCOs de portadora IF+Doppler (avanzan en clk_en_samp)
    process(clk)
        variable inc1_v, inc2_v, inc3_v : signed(23 downto 0);
        variable next1, next2, next3    : unsigned(24 downto 0);
    begin
        if rising_edge(clk) then
            if reset = '1' then
                ph1 <= (others => '0');
                ph2 <= (others => '0');
                ph3 <= (others => '0');
                carrier1 <= '0'; carrier2 <= '0'; carrier3 <= '0';
            elsif clk_en_samp = '1' then
                inc1_v := mul_doppler54(signed(sat1_doppler));
                inc2_v := mul_doppler54(signed(sat2_doppler));
                inc3_v := mul_doppler54(signed(sat3_doppler));

                next1 := ('0' & ph1) + ('0' & IF_INC) + unsigned(resize(inc1_v, 25));
                next2 := ('0' & ph2) + ('0' & IF_INC) + unsigned(resize(inc2_v, 25));
                next3 := ('0' & ph3) + ('0' & IF_INC) + unsigned(resize(inc3_v, 25));

                ph1 <= next1(23 downto 0);
                ph2 <= next2(23 downto 0);
                ph3 <= next3(23 downto 0);

                carrier1 <= next1(23);
                carrier2 <= next2(23);
                carrier3 <= next3(23);
            end if;
        end if;
    end process;

    -- BPSK: CA xor portadora (ca=0 -> +1 sin invertir, ca=1 -> -1 invierte)
    bpsk1 <= ca1 xor carrier1;
    bpsk2 <= ca2 xor carrier2;
    bpsk3 <= ca3 xor carrier3;

    -- Composicion ponderada: signo de suma de satelites activos.
    -- Cada satelite aporta +/-GAIN segun su bit BPSK para modelar
    -- desbalance de potencias real (evita cancelacion fuerte en 1-bit).
    process(clk)
        variable acc   : integer range -63 to 63;
        variable g1, g2, g3 : integer range 0 to 7;
        variable max_g : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            if reset = '1' then
                rx_sign <= '0';
                rx_mag  <= '0';
            else
                acc := 0;
                g1 := to_integer(unsigned(sat1_gain));
                g2 := to_integer(unsigned(sat2_gain));
                g3 := to_integer(unsigned(sat3_gain));

                if sat1_en = '1' then
                    if bpsk1 = '1' then acc := acc + g1; else acc := acc - g1; end if;
                end if;
                if sat2_en = '1' then
                    if bpsk2 = '1' then acc := acc + g2; else acc := acc - g2; end if;
                end if;
                if sat3_en = '1' then
                    if bpsk3 = '1' then acc := acc + g3; else acc := acc - g3; end if;
                end if;

                -- Umbral de magnitud fuerte: abs(acc) > max_gain_activo (estrictamente mayor).
                -- Con '>=' (bug anterior): cuando todos los satelites tienen la misma ganancia
                -- (ej. g1=g2=g3=5), todas las combinaciones de +-g dan |acc| in {5,15},
                -- ambas >= 5 => 100% de muestras strong (I0=1 siempre). AGC reportado: 100%.
                -- Con '>' (correcto): solo |acc|=15 (todos con mismo signo, 2/8 casos) > 5
                -- => ~25% strong. Aproxima el ~33% del front-end real (GAINREF=170, MAX2769C).
                -- Esto hace que la senal sintetica sea una copia fiel del frente de radiofrecuencia
                -- para depuracion (el bloque sintetico reemplaza al frente en sw_test_mode).
                if (sat1_en = '0' and sat2_en = '0' and sat3_en = '0') then
                    rx_sign <= '0';
                    rx_mag  <= '0';
                else
                    if acc < 0 then
                        rx_sign <= '1';
                    else
                        rx_sign <= '0';
                    end if;

                    -- max_gain_active: mayor ganancia entre satelites habilitados
                    max_g := 0;
                    if sat1_en = '1' and g1 > max_g then max_g := g1; end if;
                    if sat2_en = '1' and g2 > max_g then max_g := g2; end if;
                    if sat3_en = '1' and g3 > max_g then max_g := g3; end if;
                    if max_g = 0 then max_g := 1; end if;
                    if abs(acc) > max_g then
                        rx_mag <= '1';
                    else
                        rx_mag <= '0';
                    end if;
                end if;
            end if;
        end if;
    end process;

    i1_out <= rx_sign;
    i0_out <= rx_mag;
    rx_out <= not rx_sign;

end Behavioral;