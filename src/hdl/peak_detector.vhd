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
-- noise_floor tiene un minimo de 1 para evitar que K*noise=0
-- deje pasar cualquier señal en el CFAR.
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
    signal sum_accum    : unsigned(31 downto 0) := (others => '0'); -- acumulador suma total

    signal max_pos_reg    : std_logic_vector(9 downto 0)  := (others => '0');
    signal max_val_reg    : std_logic_vector(15 downto 0) := (others => '0');
    signal noise_floor_reg: std_logic_vector(15 downto 0) := (others => '0');
begin

    re <= signed(din_data(15 downto 0));
    im <= signed(din_data(31 downto 16));

    process(clk)
        variable v_mag      : unsigned(15 downto 0);
        variable v_max      : unsigned(15 downto 0);
        variable v_sum      : unsigned(31 downto 0);
        variable noise_sum  : unsigned(31 downto 0);
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
                v_mag := unsigned(abs(re)) + unsigned(abs(im));
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
                    -- Mínimo forzado a 1: si noise_sum=0 (pico perfecto),
                    -- noise_floor=0 haría que K*noise_floor=0 y cualquier
                    -- señal pasara el CFAR. Con mínimo=1 se evita.
                    noise_sum := v_sum - v_max;
                    if noise_sum(25 downto 10) = x"0000" then
                        noise_floor_reg <= x"0001";
                    else
                        noise_floor_reg <= std_logic_vector(noise_sum(25 downto 10));
                    end if;

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