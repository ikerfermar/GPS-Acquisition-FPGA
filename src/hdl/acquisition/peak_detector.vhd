library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- peak_detector.vhd
--
-- Recibe las 1024 salidas de la IFFT (magnitud aproximada via
-- |re| + |im|), encuentra el pico y calcula el nivel de ruido:
--   noise_floor = (suma_total - peak_val) >> 10
--               = media de las 1023 muestras no-pico
-- El clamp a minimo=1 se aplica en acquisition_controller para
-- aliviar timing en esta ruta critica.
-- =============================================================

entity peak_detector is
    Port (
        clk           : in  STD_LOGIC;
        reset_n       : in  STD_LOGIC;
        din_valid     : in  STD_LOGIC;
        din_data      : in  STD_LOGIC_VECTOR(31 downto 0);
        peak_pos      : out STD_LOGIC_VECTOR(9 downto 0);
        peak_val      : out STD_LOGIC_VECTOR(15 downto 0);
        noise_floor   : out STD_LOGIC_VECTOR(15 downto 0); -- NUEVO: media del ruido
        new_peak_ready: out STD_LOGIC
    );
end peak_detector;

architecture Behavioral of peak_detector is
    signal re, im : signed(15 downto 0);

    signal current_max  : unsigned(15 downto 0) := (others => '0');
    signal current_pos  : unsigned(9 downto 0)  := (others => '0');
    signal sample_cnt   : unsigned(9 downto 0)  := (others => '0');
    -- Maximo teorico: 1024 * 65534 = 67,106,816 (< 2^26), por lo que
    -- 26 bits bastan y reducen logica en la ruta critica de ruido.
    signal sum_accum    : unsigned(25 downto 0) := (others => '0'); -- acumulador suma total

    signal max_pos_reg    : std_logic_vector(9 downto 0)  := (others => '0');
    signal max_val_reg    : std_logic_vector(15 downto 0) := (others => '0');
    signal noise_floor_reg: std_logic_vector(15 downto 0) := (others => '0');
begin

    re <= signed(din_data(15 downto 0));
    im <= signed(din_data(31 downto 16));

    process(clk)
        variable v_re_abs   : unsigned(15 downto 0);
        variable v_im_abs   : unsigned(15 downto 0);
        variable v_mag      : unsigned(15 downto 0);
        variable v_max      : unsigned(15 downto 0);
        variable v_sum      : unsigned(25 downto 0);
        variable noise_sum  : unsigned(25 downto 0);
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                current_max     <= (others => '0');
                current_pos     <= (others => '0');
                sample_cnt      <= (others => '0');
                sum_accum       <= (others => '0');
                new_peak_ready  <= '0';
                max_val_reg     <= (others => '0');
                max_pos_reg     <= (others => '0');
                noise_floor_reg <= (others => '0');
            elsif din_valid = '1' then
                -- FIX: abs(signed(-32768,16)) overflows back to -32768 in NUMERIC_STD.
                -- Saturate x"8000" to x"7FFF" (32767) so the sum never wraps.
                -- All other negative values are negated correctly via 2's complement.
                if re = "1000000000000000" then
                    v_re_abs := to_unsigned(32767, 16);
                elsif re(15) = '1' then
                    v_re_abs := unsigned(not std_logic_vector(re)) + 1;
                else
                    v_re_abs := unsigned(std_logic_vector(re));
                end if;
                if im = "1000000000000000" then
                    v_im_abs := to_unsigned(32767, 16);
                elsif im(15) = '1' then
                    v_im_abs := unsigned(not std_logic_vector(im)) + 1;
                else
                    v_im_abs := unsigned(std_logic_vector(im));
                end if;
                v_mag := v_re_abs + v_im_abs;  -- max 32767+32767=65534, no overflow
                v_max := current_max;
                v_sum := sum_accum + v_mag;

                if v_mag > v_max then
                    v_max       := v_mag;
                    current_pos <= sample_cnt;
                end if;

                if sample_cnt = 1023 then
                    -- Pico final
                    max_val_reg <= std_logic_vector(v_max);
                    max_pos_reg <= std_logic_vector(current_pos);
                    if v_mag > current_max then
                        max_pos_reg <= std_logic_vector(sample_cnt);
                    end if;

                    -- noise_floor = (suma_total - peak_val) / 1024
                    noise_sum := v_sum - v_max;
                    noise_floor_reg <= std_logic_vector(noise_sum(25 downto 10));

                    -- Reset para el siguiente frame
                    current_max    <= (others => '0');
                    sample_cnt     <= (others => '0');
                    sum_accum      <= (others => '0');
                    new_peak_ready <= '1';
                else
                    current_max    <= v_max;
                    sample_cnt     <= sample_cnt + 1;
                    sum_accum      <= v_sum;
                    new_peak_ready <= '0';
                end if;
            else
                new_peak_ready <= '0';
            end if;
        end if;
    end process;

    peak_pos    <= max_pos_reg;
    peak_val    <= max_val_reg;
    noise_floor <= noise_floor_reg;

end Behavioral;