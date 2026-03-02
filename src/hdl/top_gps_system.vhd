library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

library work;
use work.gps_config_pkg.all;

-- =============================================================
-- top_gps_system.vhd
--
-- Receptor GPS L1 C/A de adquisicion en Basys-3 (Artix-7).
--
-- SEÑAL DE ENTRADA:
--   Real      (sw[5]=0): i1_real del MAX2769C, 1 bit, IF=4.092 MHz
--   Sintetica (sw[5]=1): multi_sat_rx_gen, mismo formato y pipeline
--
-- PIPELINE (gobernado por clk_en_fe ~1.023 MHz):
--
--   i1_real / i1_synth
--       |
--       +-- doble FF (anti-metaestabilidad)
--       |
--       +-- NCO maestro 16.368 MHz (sample_en_samp)
--       |       +-- decimador x16 --> i1_dec + clk_en_fe
--       |
--       +-- doppler_mixer (NCO IF+Doppler, 100 MHz) --> rx_I / rx_Q
--       |
--       +-- fft_controller (FFT-RX, FFT-CA, IFFT correlacion)
--       |
--       +-- peak_detector  --> peak_val, peak_pos, noise_floor
--       |
--       +-- acquisition_controller --> RAM --> uart_reporter
--
-- SWITCHES:
--   sw[4:0]  : PRN manual
--   sw[5]    : 1=señal sintetica, 0=señal real
--   sw[13:6] : offset Doppler manual / fino en modo auto
--   sw[14]   : 1=modo auto (barrido), 0=modo manual
--   sw[15]   : display: 0=bin/PRN, 1=fase del pico
--
-- LEDS:
--   led[0] : señal sintetica activa (sw[5])
--   led[1] : epoch (~1 Hz parpadeo)
--   led[2] : senal adquirida (auto) / pico fuerte (manual)
-- =============================================================

entity top_gps_system is
    Port (
        clk         : in  STD_LOGIC;
        sw          : in  STD_LOGIC_VECTOR(15 downto 0);
        led         : out STD_LOGIC_VECTOR(2 downto 0);
        seg         : out STD_LOGIC_VECTOR(6 downto 0);
        an          : out STD_LOGIC_VECTOR(3 downto 0);
        uart_tx_pin : out STD_LOGIC;
        i1_real     : in  STD_LOGIC;
        i0_real     : in  STD_LOGIC
    );
end top_gps_system;

architecture Structural of top_gps_system is

    signal clk_100MHz  : std_logic;
    signal locked      : std_logic;
    signal sys_reset   : std_logic;
    signal sys_reset_n : std_logic;

    -- NCO maestro 16.368 MHz
    -- INC = round(16.368e6 / 100e6 * 2^16) = 10727
    signal nco_samp_accum : unsigned(15 downto 0) := (others => '0');
    signal sample_en_samp : std_logic := '0';
    constant NCO_SAMP_INC : unsigned(15 downto 0) := to_unsigned(10727, 16);

    -- Señal del front-end y doble FF
    signal i1_synth  : std_logic := '0';
    signal i1_fe_raw : std_logic := '0';
    signal i1_ff1    : std_logic := '0';
    signal i1_ff2    : std_logic := '0';

    -- Decimador x16 → clk_en_fe (~1.023 MHz)
    -- Unico enable de todo el pipeline de correlacion
    signal dec_cnt   : unsigned(3 downto 0) := (others => '0');
    signal i1_dec    : std_logic := '0';
    signal clk_en_fe : std_logic := '0';

    -- CA local
    signal ca_sig_local : std_logic;
    signal epoch_signal : std_logic;

    -- doppler_mixer
    signal doppler_sel_s      : std_logic_vector(7 downto 0);
    signal doppler_offset_mux : std_logic_vector(7 downto 0);
    signal doppler_offset     : std_logic_vector(7 downto 0);
    signal rx_I_mixed         : std_logic_vector(15 downto 0);
    signal rx_Q_mixed         : std_logic_vector(15 downto 0);
    signal doppler_sel_auto   : std_logic_vector(7 downto 0);

    -- FFT / peak_detector
    signal correlation_data  : std_logic_vector(31 downto 0);
    signal correlation_valid : std_logic;
    signal peak_ready_s      : std_logic;
    signal pos_pico          : std_logic_vector(9 downto 0);
    signal val_pico          : std_logic_vector(15 downto 0);
    signal noise_floor_s     : std_logic_vector(15 downto 0);

    -- acquisition_controller
    signal prn_out         : std_logic_vector(4 downto 0);
    signal prn_sel         : std_logic_vector(4 downto 0);
    signal prn_local_sel   : std_logic_vector(4 downto 0);
    signal acq_start       : std_logic := '0';
    signal acq_done        : std_logic;
    signal acq_valid_s     : std_logic;
    signal acq_reset_n     : std_logic;
    signal sweep_bin       : std_logic_vector(7 downto 0);
    signal sweep_prn       : std_logic_vector(4 downto 0);
    signal sweep_running   : std_logic;
    signal sweep_restart_s : std_logic;

    -- Control
    signal sw_test_mode : std_logic;
    signal sw_auto_mode : std_logic;
    signal sw_display   : std_logic;
    signal sw14_prev    : std_logic := '0';
    signal sw14_pp      : std_logic := '0';

    -- UART
    signal uart_reporting       : std_logic;
    signal reporter_force_reset : std_logic;
    signal ram_we         : std_logic;
    signal ram_wr_addr    : std_logic_vector(4 downto 0);
    signal ram_wr_acq     : std_logic;
    signal ram_wr_doppler : std_logic_vector(7 downto 0);
    signal ram_wr_phase   : std_logic_vector(9 downto 0);
    signal ram_wr_snr_s   : std_logic_vector(15 downto 0);

    -- Display
    signal display_timer  : unsigned(24 downto 0) := (others => '0');
    signal pos_pico_latch : std_logic_vector(9 downto 0) := (others => '0');
    signal display_input  : std_logic_vector(5 downto 0);
    signal t1, t2         : integer := 0;

    -- Enables satelites sinteticos (VHDL-93)
    signal sat1_en_s : std_logic;
    signal sat2_en_s : std_logic;
    signal sat3_en_s : std_logic;

    -- ── Componentes ───────────────────────────────────────────────────────
    component clk_wiz_0
        port (clk_in1  : in  std_logic;
              clk_out1 : out std_logic;
              locked   : out std_logic);
    end component;

    component seven_seg_controller
        port (clk       : in  std_logic;
              number_in : in  std_logic_vector(5 downto 0);
              segments  : out std_logic_vector(6 downto 0);
              anodes    : out std_logic_vector(3 downto 0));
    end component;

    component gps_ca_generator
        port (clk, clk_en, reset : in  std_logic;
              prn_num             : in  std_logic_vector(4 downto 0);
              ca_out, epoch_out   : out std_logic);
    end component;

    component doppler_mixer
        port (clk            : in  std_logic;
              reset_n        : in  std_logic;
              clk_en         : in  std_logic;
              i1_in          : in  std_logic;
              doppler_sel    : in  std_logic_vector(7 downto 0);
              doppler_offset : in  std_logic_vector(7 downto 0);
              rx_I           : out std_logic_vector(15 downto 0);
              rx_Q           : out std_logic_vector(15 downto 0));
    end component;

    component fft_controller
        port (clk, clk_en, reset_n : in  std_logic;
              rx_I         : in  std_logic_vector(15 downto 0);
              rx_Q         : in  std_logic_vector(15 downto 0);
              ca_bit_local : in  std_logic;
              ca_epoch     : in  std_logic;
              fft_ready    : out std_logic;
              fft_data_out : out std_logic_vector(31 downto 0));
    end component;

    component peak_detector
        port (clk, reset_n, din_valid : in  std_logic;
              din_data                : in  std_logic_vector(31 downto 0);
              peak_pos                : out std_logic_vector(9 downto 0);
              peak_val                : out std_logic_vector(15 downto 0);
              noise_floor             : out std_logic_vector(15 downto 0);
              new_peak_ready          : out std_logic);
    end component;

    component multi_sat_rx_gen
        port (clk          : in  std_logic;
              clk_en       : in  std_logic;
              clk_en_samp  : in  std_logic;
              reset        : in  std_logic;
              sat1_prn     : in  std_logic_vector(4 downto 0);
              sat1_doppler : in  std_logic_vector(7 downto 0);
              sat1_phase   : in  std_logic_vector(9 downto 0);
              sat1_en      : in  std_logic;
              sat2_prn     : in  std_logic_vector(4 downto 0);
              sat2_doppler : in  std_logic_vector(7 downto 0);
              sat2_phase   : in  std_logic_vector(9 downto 0);
              sat2_en      : in  std_logic;
              sat3_prn     : in  std_logic_vector(4 downto 0);
              sat3_doppler : in  std_logic_vector(7 downto 0);
              sat3_phase   : in  std_logic_vector(9 downto 0);
              sat3_en      : in  std_logic;
              i1_out       : out std_logic;
              i0_out       : out std_logic;
              rx_out       : out std_logic;
              epoch_out    : out std_logic);
    end component;

    component acquisition_controller
        generic (PEAK_THRESHOLD   : integer := 200;
                 K_CFAR           : integer := 4;
                 NUM_PRNS         : integer := 32;
                 N_INT            : integer := 5;
                 BIN_RANGE        : integer := 64;
                 CONTINUOUS_SWEEP : boolean  := true);
        port (clk            : in  std_logic;
              reset_n        : in  std_logic;
              start          : in  std_logic;
              ca_epoch       : in  std_logic;
              peak_ready     : in  std_logic;
              peak_val       : in  std_logic_vector(15 downto 0);
              peak_pos       : in  std_logic_vector(9 downto 0);
              noise_floor    : in  std_logic_vector(15 downto 0);
              doppler_sel    : out std_logic_vector(7 downto 0);
              prn_out        : out std_logic_vector(4 downto 0);
              ram_we         : out std_logic;
              ram_wr_addr    : out std_logic_vector(4 downto 0);
              ram_wr_acq     : out std_logic;
              ram_wr_doppler : out std_logic_vector(7 downto 0);
              ram_wr_phase   : out std_logic_vector(9 downto 0);
              ram_wr_snr     : out std_logic_vector(15 downto 0);
              acq_done       : out std_logic;
              acq_valid      : out std_logic;
              sat_count      : out std_logic_vector(5 downto 0);
              best_prn       : out std_logic_vector(4 downto 0);
              best_doppler   : out std_logic_vector(7 downto 0);
              best_pos       : out std_logic_vector(9 downto 0);
              best_val       : out std_logic_vector(15 downto 0);
              sweep_bin      : out std_logic_vector(7 downto 0);
              sweep_prn      : out std_logic_vector(4 downto 0);
              sweep_running  : out std_logic;
              sweep_restart  : out std_logic);
    end component;

    component uart_reporter
        generic (CLK_FREQ  : integer := 100_000_000;
                 BAUD_RATE : integer := 115_200;
                 NUM_PRNS  : integer := 32);
        port (clk          : in  std_logic;
              reset_n      : in  std_logic;
              force_reset  : in  std_logic;
              wr_en        : in  std_logic;
              wr_addr      : in  std_logic_vector(4 downto 0);
              wr_acq       : in  std_logic;
              wr_doppler   : in  std_logic_vector(7 downto 0);
              wr_phase     : in  std_logic_vector(9 downto 0);
              wr_snr       : in  std_logic_vector(15 downto 0);
              sweep_start  : in  std_logic;
              report_start : in  std_logic;
              uart_tx_pin  : out std_logic;
              reporting    : out std_logic);
    end component;

begin

    prn_sel        <= sw(4 downto 0);
    sw_test_mode   <= sw(5);
    doppler_offset <= sw(13 downto 6);
    sw_auto_mode   <= sw(14);
    sw_display     <= sw(15);

    sys_reset   <= not locked;
    sys_reset_n <= locked;

    acq_reset_n          <= sys_reset_n AND sw_auto_mode;
    reporter_force_reset <= NOT sw_auto_mode;

    process(sw_test_mode)
    begin
        if CFG_SAT1_EN then sat1_en_s <= sw_test_mode; else sat1_en_s <= '0'; end if;
        if CFG_SAT2_EN then sat2_en_s <= sw_test_mode; else sat2_en_s <= '0'; end if;
        if CFG_SAT3_EN then sat3_en_s <= sw_test_mode; else sat3_en_s <= '0'; end if;
    end process;

    i1_fe_raw <= i1_synth when sw_test_mode = '1' else i1_real;

    -- Doble FF anti-metaestabilidad (real y sintetica pasan por aqui)
    process(clk_100MHz)
    begin
        if rising_edge(clk_100MHz) then
            if sys_reset_n = '0' then
                i1_ff1 <= '0'; i1_ff2 <= '0';
            else
                i1_ff1 <= i1_fe_raw;
                i1_ff2 <= i1_ff1;
            end if;
        end if;
    end process;

    -- NCO maestro 16.368 MHz
    process(clk_100MHz)
        variable next_s : unsigned(16 downto 0);
    begin
        if rising_edge(clk_100MHz) then
            next_s         := ('0' & nco_samp_accum) + ('0' & NCO_SAMP_INC);
            sample_en_samp <= next_s(16);
            nco_samp_accum <= next_s(15 downto 0);
        end if;
    end process;

    -- Decimador x16: captura 1 de cada 16 pulsos del NCO maestro
    -- clk_en_fe es el unico enable del pipeline de correlacion
    process(clk_100MHz)
    begin
        if rising_edge(clk_100MHz) then
            if sys_reset_n = '0' then
                dec_cnt   <= (others => '0');
                i1_dec    <= '0';
                clk_en_fe <= '0';
            else
                clk_en_fe <= '0';
                if sample_en_samp = '1' then
                    if dec_cnt = 15 then
                        dec_cnt <= (others => '0');
                    else
                        dec_cnt <= dec_cnt + 1;
                    end if;
                    if dec_cnt = 0 then
                        i1_dec    <= i1_ff2;
                        clk_en_fe <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process;

    prn_local_sel      <= prn_out          when sw_auto_mode = '1' else prn_sel;
    doppler_sel_s      <= doppler_sel_auto when sw_auto_mode = '1' else sw(13 downto 6);
    doppler_offset_mux <= doppler_offset   when sw_auto_mode = '1' else x"00";

    -- Deteccion flanco sw[14] para disparar barrido
    process(clk_100MHz)
    begin
        if rising_edge(clk_100MHz) then
            if sys_reset_n = '0' then
                sw14_prev <= '0'; sw14_pp <= '0'; acq_start <= '0';
            else
                sw14_prev <= sw_auto_mode;
                sw14_pp   <= sw14_prev;
                acq_start <= '0';
                if sw_auto_mode = '1' and sw14_pp = '0' then
                    acq_start <= '1';
                end if;
            end if;
        end if;
    end process;

    -- Display (refresco cada 250 ms)
    process(clk_100MHz)
    begin
        if rising_edge(clk_100MHz) then
            if sys_reset_n = '0' then
                display_timer  <= (others => '0');
                pos_pico_latch <= (others => '0');
            else
                if display_timer >= 25000000 then
                    pos_pico_latch <= pos_pico;
                    display_timer  <= (others => '0');
                else
                    display_timer <= display_timer + 1;
                end if;
            end if;
        end if;
    end process;

    display_input <=
        sweep_bin(7 downto 2)                            when sw_auto_mode='1' and sw_display='0' else
        std_logic_vector(unsigned('0' & sweep_prn) + 1)  when sw_auto_mode='1' and sw_display='1' else
        pos_pico_latch(5 downto 0)                       when sw_display='1' else
        std_logic_vector(unsigned('0' & prn_sel) + 1);

    -- LEDs
    process(clk_100MHz)
    begin
        if rising_edge(clk_100MHz) then
            if sys_reset_n = '0' then
                t1 <= 0; t2 <= 0;
            else
                if epoch_signal = '1' then t1 <= 5000000;
                elsif t1 > 0 then t1 <= t1 - 1; end if;

                if sw_auto_mode = '1' then
                    if acq_valid_s = '1' then t2 <= 5000000; else t2 <= 0; end if;
                else
                    if peak_ready_s = '1' then
                        if unsigned(val_pico) > 1000 then t2 <= 5000000;
                        else t2 <= 0; end if;
                    elsif t2 > 0 then t2 <= t2 - 1;
                    end if;
                end if;

                led(0) <= sw_test_mode;
                if t1 > 0 then led(1) <= '1'; else led(1) <= '0'; end if;
                if t2 > 0 then led(2) <= '1'; else led(2) <= '0'; end if;
            end if;
        end if;
    end process;

    -- ================================================================
    -- INSTANCIAS
    -- ================================================================

    reloj_inst : clk_wiz_0
        port map (clk_in1  => clk,
                  clk_out1 => clk_100MHz,
                  locked   => locked);

    display_inst : seven_seg_controller
        port map (clk       => clk_100MHz,
                  number_in => display_input,
                  segments  => seg,
                  anodes    => an);

    ca_gen_local : gps_ca_generator
        port map (clk       => clk_100MHz,
                  clk_en    => clk_en_fe,
                  reset     => sys_reset,
                  prn_num   => prn_local_sel,
                  ca_out    => ca_sig_local,
                  epoch_out => epoch_signal);

    -- El generador sintetico usa clk=clk_100MHz y clk_en_samp='1'.
    -- Con esto su NCO portadora avanza cada ciclo de 100 MHz (igual que el doppler_mixer),
    -- usando IF_INC=686421 → portadora de 4.092 MHz en ambos → correlacion maxima.
    -- No se necesita clk_16MHz ni sample_en_samp para este modulo.
    multi_sat_inst : multi_sat_rx_gen
        port map (clk          => clk_100MHz,
                  clk_en       => clk_en_fe,
                  clk_en_samp  => '1',
                  reset        => sys_reset,
                  sat1_prn     => std_logic_vector(to_unsigned(CFG_SAT1_PRN - 1, 5)),
                  sat1_doppler => std_logic_vector(to_signed(CFG_SAT1_DOPPLER, 8)),
                  sat1_phase   => std_logic_vector(to_unsigned(CFG_SAT1_PHASE, 10)),
                  sat1_en      => sat1_en_s,
                  sat2_prn     => std_logic_vector(to_unsigned(CFG_SAT2_PRN - 1, 5)),
                  sat2_doppler => std_logic_vector(to_signed(CFG_SAT2_DOPPLER, 8)),
                  sat2_phase   => std_logic_vector(to_unsigned(CFG_SAT2_PHASE, 10)),
                  sat2_en      => sat2_en_s,
                  sat3_prn     => std_logic_vector(to_unsigned(CFG_SAT3_PRN - 1, 5)),
                  sat3_doppler => std_logic_vector(to_signed(CFG_SAT3_DOPPLER, 8)),
                  sat3_phase   => std_logic_vector(to_unsigned(CFG_SAT3_PHASE, 10)),
                  sat3_en      => sat3_en_s,
                  i1_out       => i1_synth,
                  i0_out       => open,
                  rx_out       => open,
                  epoch_out    => open);

    mixer_inst : doppler_mixer
        port map (clk            => clk_100MHz,
                  reset_n        => sys_reset_n,
                  clk_en         => clk_en_fe,
                  i1_in          => i1_dec,
                  doppler_sel    => doppler_sel_s,
                  doppler_offset => doppler_offset_mux,
                  rx_I           => rx_I_mixed,
                  rx_Q           => rx_Q_mixed);

    fft_ctrl_inst : fft_controller
        port map (clk          => clk_100MHz,
                  clk_en       => clk_en_fe,
                  reset_n      => sys_reset_n,
                  rx_I         => rx_I_mixed,
                  rx_Q         => rx_Q_mixed,
                  ca_bit_local => ca_sig_local,
                  ca_epoch     => epoch_signal,
                  fft_ready    => correlation_valid,
                  fft_data_out => correlation_data);

    peak_inst : peak_detector
        port map (clk            => clk_100MHz,
                  reset_n        => sys_reset_n,
                  din_valid      => correlation_valid,
                  din_data       => correlation_data,
                  peak_pos       => pos_pico,
                  peak_val       => val_pico,
                  noise_floor    => noise_floor_s,
                  new_peak_ready => peak_ready_s);

    acq_ctrl_inst : acquisition_controller
        generic map (PEAK_THRESHOLD   => CFG_PEAK_THRESHOLD,
                     K_CFAR           => CFG_K_CFAR,
                     NUM_PRNS         => CFG_NUM_PRNS,
                     N_INT            => CFG_N_INT,
                     BIN_RANGE        => CFG_BIN_RANGE,
                     CONTINUOUS_SWEEP => CFG_CONTINUOUS)
        port map (clk            => clk_100MHz,
                  reset_n        => acq_reset_n,
                  start          => acq_start,
                  ca_epoch       => epoch_signal,
                  peak_ready     => peak_ready_s,
                  peak_val       => val_pico,
                  peak_pos       => pos_pico,
                  noise_floor    => noise_floor_s,
                  doppler_sel    => doppler_sel_auto,
                  prn_out        => prn_out,
                  ram_we         => ram_we,
                  ram_wr_addr    => ram_wr_addr,
                  ram_wr_acq     => ram_wr_acq,
                  ram_wr_doppler => ram_wr_doppler,
                  ram_wr_phase   => ram_wr_phase,
                  ram_wr_snr     => ram_wr_snr_s,
                  acq_done       => acq_done,
                  acq_valid      => acq_valid_s,
                  sat_count      => open,
                  best_prn       => open,
                  best_doppler   => open,
                  best_pos       => open,
                  best_val       => open,
                  sweep_bin      => sweep_bin,
                  sweep_prn      => sweep_prn,
                  sweep_running  => sweep_running,
                  sweep_restart  => sweep_restart_s);

    uart_reporter_inst : uart_reporter
        generic map (CLK_FREQ  => CFG_CLK_FREQ,
                     BAUD_RATE => CFG_BAUD_RATE,
                     NUM_PRNS  => CFG_NUM_PRNS)
        port map (clk          => clk_100MHz,
                  reset_n      => sys_reset_n,
                  force_reset  => reporter_force_reset,
                  wr_en        => ram_we,
                  wr_addr      => ram_wr_addr,
                  wr_acq       => ram_wr_acq,
                  wr_doppler   => ram_wr_doppler,
                  wr_phase     => ram_wr_phase,
                  wr_snr       => ram_wr_snr_s,
                  sweep_start  => sweep_restart_s,
                  report_start => acq_done,
                  uart_tx_pin  => uart_tx_pin,
                  reporting    => uart_reporting);

end Structural;