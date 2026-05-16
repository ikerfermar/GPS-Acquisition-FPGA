library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

use work.gps_config_pkg.all;

-- =============================================================
-- top_gps_system.vhd
--
-- Receptor GPS L1 C/A de adquisicion en Basys-3 (Artix-7).
--
-- SENAL DE ENTRADA:
--   Real      (sw_5=0): i1/i0_real sign-magnitude del MAX2769C, IF=4.092 MHz
--   Sintetica (sw_5=1): multi_sat_rx_gen, mismo formato y pipeline
--
-- PIPELINE (gobernado por clk_en_fe ~1.023 MHz):
--
--   i1/i0_real o i1/i0_synth (sign-magnitude)
--       |
--       +-- doble FF (anti-metaestabilidad)
--       |
--       +-- CLKOUT 16.368 MHz (JA7:H1) -> 2-FF sync -> sample_en_samp
--       |       +-- decimador x16 --> i1_dec/i0_dec + clk_en_fe
--       |
--       +-- doppler_mixer (NCO IF+Doppler, 100 MHz) --> rx_I / rx_Q
--       |
--       +-- fft_controller (FFT-RX, FFT-CA, IFFT correlacion)
--       |
--       +-- peak_detector  --> peak_val, peak_pos, noise_floor
--       |
--       +-- acquisition_controller --> RAM --> uart_reporter
--
-- CLKOUT (MAX2769C pin 18):
--   Cable coaxial. Centro=senal (JA7:H1), blindaje=GND (JA8, sin declarar).
--   Se sincroniza al dominio 100 MHz mediante dos flip-flops; el flanco
--   de subida detectado reemplaza el antiguo NCO 16.368 MHz. Esto elimina
--   la deriva entre el NCO interno y el oscilador real del front-end.
--
-- SWITCHES:
--   sw_5    : 1=senal sintetica, 0=senal real
--
-- LEDS:
--   led(0)      : senal sintetica activa (sw_5)
--   led(1)      : epoch (~1 Hz parpadeo)
--   led(2)      : senal adquirida
--   led(3)..(14): sondas de depuracion (estado de I1/I0 y relojes)
--   led(15)     : pulso acq_done (fin de barrido)
-- =============================================================

entity top_gps_system is
    Port (
        clk         : in  STD_LOGIC;
        sw_5        : in  STD_LOGIC;
        led         : out STD_LOGIC_VECTOR(15 downto 0);
        uart_tx_pin : out STD_LOGIC;
        i1_real     : in  STD_LOGIC;
        i0_real     : in  STD_LOGIC;
        fe_clkout   : in  STD_LOGIC   -- MAX2769C CLKOUT 16.368 MHz, JA7:H1
    );
end top_gps_system;

architecture Structural of top_gps_system is
    attribute ASYNC_REG : string;

    signal clk_100MHz  : std_logic;
    signal locked      : std_logic;
    signal sys_reset   : std_logic;
    signal sys_reset_n : std_logic;

    -- Sincronizacion de CLKOUT del MAX2769C al dominio 100 MHz.
    -- CLKOUT (16.368 MHz, JA7:H1) sincronizado con dos FF; la deteccion
    -- de flanco de subida genera sample_en_samp, referenciando el muestreo
    -- al oscilador hardware real del ADC (elimina deriva del antiguo NCO).
    signal fe_clk_sync1   : std_logic := '0';
    signal fe_clk_sync2   : std_logic := '0';
    signal fe_clk_prev    : std_logic := '0';
    signal sample_en_samp : std_logic := '0';
    attribute ASYNC_REG of fe_clk_sync1 : signal is "TRUE";
    attribute ASYNC_REG of fe_clk_sync2 : signal is "TRUE";

    -- DDS interno para replay/sintetico: genera exactamente 16368 enables por
    -- cada 100000 ciclos (= 16.368 MHz). Permite replay y modo sintetico sin
    -- necesidad del clock fisico MAX2769C (sample_en_samp=0 cuando no conectado).
    -- acc += 16368 cada ciclo; cuando acc >= 83632 (=100000-16368) substraemos 100000.
    signal int_sample_acc : unsigned(16 downto 0) := (others => '0');
    signal int_sample_en  : std_logic := '0';
    signal sample_en_drv  : std_logic := '0';  -- mux: DDS en replay/synth, real en RF

    -- Senal del front-end y doble FF
    signal i1_synth    : std_logic := '0';
    signal i0_synth    : std_logic := '1';
    signal fe_i1_raw   : std_logic := '0';
    signal fe_i0_raw   : std_logic := '0';
    signal i1_fe_raw   : std_logic := '0';
    signal i0_fe_raw   : std_logic := '0';
    signal i1_ff1    : std_logic := '0';
    signal i1_ff2    : std_logic := '0';
    signal i0_ff1    : std_logic := '0';
    signal i0_ff2    : std_logic := '0';
    attribute ASYNC_REG of i1_ff1 : signal is "TRUE";
    attribute ASYNC_REG of i1_ff2 : signal is "TRUE";
    attribute ASYNC_REG of i0_ff1 : signal is "TRUE";
    attribute ASYNC_REG of i0_ff2 : signal is "TRUE";

    -- Decimador x16 -> clk_en_fe (~1.023 MHz)
    -- Unico enable de todo el pipeline de correlacion
    signal dec_cnt   : unsigned(3 downto 0) := (others => '0');
    signal i1_dec    : std_logic := '0';
    signal i0_dec    : std_logic := '0';
    signal clk_en_fe : std_logic := '0';

    -- CA local
    signal ca_sig_local : std_logic;
    signal epoch_signal : std_logic;

    -- doppler_mixer
    signal doppler_offset_mux : std_logic_vector(7 downto 0);
    signal rx_I_mixed         : std_logic_vector(15 downto 0);
    signal rx_Q_mixed         : std_logic_vector(15 downto 0);
    signal doppler_sel_auto   : std_logic_vector(7 downto 0);

    -- FFT / peak_detector
    signal correlation_data  : std_logic_vector(31 downto 0);
    signal correlation_valid : std_logic;
    signal correlation_ready : std_logic;
    signal peak_ready_s      : std_logic;
    signal pos_pico          : std_logic_vector(9 downto 0);
    signal val_pico          : std_logic_vector(15 downto 0);
    signal noise_floor_s     : std_logic_vector(15 downto 0);
    signal peak_ready_acq    : std_logic := '0';
    signal pos_pico_acq      : std_logic_vector(9 downto 0) := (others => '0');
    signal val_pico_acq      : std_logic_vector(15 downto 0) := (others => '0');
    signal noise_floor_acq   : std_logic_vector(15 downto 0) := (others => '0');

    -- Registro pipeline para romper ruta critica fft_ctrl -> peak_inst
    -- (WNS=0.175ns en N=1024: 11 niveles logicos, 9.31ns de 10ns por routing largo).
    -- Anadir 1 ciclo de latencia separa el routing cross-chip del calculo de magnitud.
    signal correlation_data_r  : std_logic_vector(31 downto 0) := (others => '0');
    signal correlation_valid_r : std_logic := '0';

    -- acquisition_controller
    signal prn_out         : std_logic_vector(4 downto 0);
    signal acq_valid_s     : std_logic;
    signal sweep_running   : std_logic;
    signal sweep_restart_s : std_logic;
    signal sweep_time_cycles_s : std_logic_vector(31 downto 0);
    signal candidate_count_s   : std_logic_vector(7 downto 0);
    signal acq_done            : std_logic := '0';
    signal fft_coherent_sel_s : std_logic_vector(1 downto 0);
    -- FBA control signals (acq_controller -> peak_detector)
    signal fba_clear_s        : std_logic := '0';
    signal fba_last_block_s   : std_logic := '0';

    -- Control
    signal sw_5_ff1   : std_logic := '0';
    signal sw_5_sync  : std_logic := '0';
    attribute ASYNC_REG of sw_5_ff1  : signal is "TRUE";
    attribute ASYNC_REG of sw_5_sync : signal is "TRUE";
    signal sw_test_mode : std_logic;
    signal ca_local_reset      : std_logic;

    -- UART
    signal ram_we         : std_logic := '0';
    signal ram_wr_addr    : std_logic_vector(4 downto 0) := (others => '0');
    signal ram_wr_acq     : std_logic := '0';
    signal ram_wr_doppler : std_logic_vector(7 downto 0) := (others => '0');
    signal ram_wr_phase   : std_logic_vector(9 downto 0) := (others => '0');
    signal ram_wr_doppler_rep : std_logic_vector(7 downto 0) := (others => '0');
    signal ram_wr_snr     : std_logic_vector(15 downto 0) := (others => '0');
    signal ram_wr_noise   : std_logic_vector(15 downto 0) := (others => '0');
    signal ram_wr_margin  : std_logic_vector(15 downto 0) := (others => '0');
    signal ram_wr_flags   : std_logic_vector(7 downto 0) := (others => '0');
    signal diag_iq_valid_s: std_logic_vector(31 downto 0) := (others => '0');
    signal diag_iq_err_s  : std_logic_vector(31 downto 0) := (others => '0');
    signal diag_i1_prev_s : std_logic := '0';
    signal diag_i0_prev_s : std_logic := '0';
    signal diag_prev_valid_s : std_logic := '0';
    signal agc_strong_count_s : std_logic_vector(31 downto 0) := (others => '0');
    signal agc_total_count_s  : std_logic_vector(31 downto 0) := (others => '0');

    constant C_LED_HOLD_CYCLES       : integer := CFG_LED_HOLD_CYCLES;

    signal t1, t2     : integer := 0;

    -- Enables satelites sinteticos (VHDL-93)
    signal sat1_en_s : std_logic;
    signal sat2_en_s : std_logic;
    signal sat3_en_s : std_logic;
    constant C_SAT1_GAIN_SLV : std_logic_vector(2 downto 0) := std_logic_vector(to_unsigned(CFG_SAT1_GAIN, 3));
    constant C_SAT2_GAIN_SLV : std_logic_vector(2 downto 0) := std_logic_vector(to_unsigned(CFG_SAT2_GAIN, 3));
    constant C_SAT3_GAIN_SLV : std_logic_vector(2 downto 0) := std_logic_vector(to_unsigned(CFG_SAT3_GAIN, 3));

    -- Precompute static SAT parameters once (avoid repeated casts in multiple blocks)
    constant C_SAT1_PRN_IDX_SLV : std_logic_vector(4 downto 0) := std_logic_vector(to_unsigned(CFG_SAT1_PRN - 1, 5));
    constant C_SAT2_PRN_IDX_SLV : std_logic_vector(4 downto 0) := std_logic_vector(to_unsigned(CFG_SAT2_PRN - 1, 5));
    constant C_SAT3_PRN_IDX_SLV : std_logic_vector(4 downto 0) := std_logic_vector(to_unsigned(CFG_SAT3_PRN - 1, 5));
    constant C_SAT1_DOPPLER_SLV : std_logic_vector(7 downto 0) := std_logic_vector(to_signed(CFG_SAT1_DOPPLER, 8));
    constant C_SAT2_DOPPLER_SLV : std_logic_vector(7 downto 0) := std_logic_vector(to_signed(CFG_SAT2_DOPPLER, 8));
    constant C_SAT3_DOPPLER_SLV : std_logic_vector(7 downto 0) := std_logic_vector(to_signed(CFG_SAT3_DOPPLER, 8));
    constant C_SAT1_PHASE_SLV   : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(CFG_SAT1_PHASE, 10));
    constant C_SAT2_PHASE_SLV   : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(CFG_SAT2_PHASE, 10));
    constant C_SAT3_PHASE_SLV   : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(CFG_SAT3_PHASE, 10));

    -- -- Componentes -------------------------------------------------------
    component clk_wiz_0
        port (clk_in1  : in  std_logic;
              clk_out1 : out std_logic;
              locked   : out std_logic);
    end component;



    component gps_ca_generator
        port (clk, clk_en, reset : in  std_logic;
              prn_num             : in  std_logic_vector(4 downto 0);
              ca_out, epoch_out   : out std_logic);
    end component;

    component doppler_mixer
        port (clk            : in  std_logic;
              reset_n        : in  std_logic;
              sample_en      : in  std_logic;
              clk_en         : in  std_logic;
              i1_sign        : in  std_logic;
              i0_mag         : in  std_logic;
              doppler_sel    : in  std_logic_vector(7 downto 0);
              doppler_offset : in  std_logic_vector(7 downto 0);
              rx_I           : out std_logic_vector(15 downto 0);
              rx_Q           : out std_logic_vector(15 downto 0));
    end component;

    component fft_controller
        generic (COHERENT_N_MS : integer := 1);
        port (clk, clk_en, reset_n : in  std_logic;
              rx_I         : in  std_logic_vector(15 downto 0);
              rx_Q         : in  std_logic_vector(15 downto 0);
              ca_bit_local : in  std_logic;
              ca_epoch     : in  std_logic;
                            coherent_n_ms_sel : in  std_logic_vector(1 downto 0);
              fft_out_ready: in  std_logic;
              fft_ready    : out std_logic;
              fft_data_out : out std_logic_vector(31 downto 0));
    end component;

    component peak_detector
          port (clk, reset_n, din_valid : in  std_logic;
              din_ready               : out std_logic;
              din_data                : in  std_logic_vector(31 downto 0);
              fba_clear               : in  std_logic;
              fba_last_block          : in  std_logic;
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
              sat1_gain    : in  std_logic_vector(2 downto 0);
              sat1_en      : in  std_logic;
              sat2_prn     : in  std_logic_vector(4 downto 0);
              sat2_doppler : in  std_logic_vector(7 downto 0);
              sat2_phase   : in  std_logic_vector(9 downto 0);
              sat2_gain    : in  std_logic_vector(2 downto 0);
              sat2_en      : in  std_logic;
              sat3_prn     : in  std_logic_vector(4 downto 0);
              sat3_doppler : in  std_logic_vector(7 downto 0);
              sat3_phase   : in  std_logic_vector(9 downto 0);
              sat3_gain    : in  std_logic_vector(2 downto 0);
              sat3_en      : in  std_logic;
              i1_out       : out std_logic;
              i0_out       : out std_logic;
              rx_out       : out std_logic;
              epoch_out    : out std_logic);
    end component;


    component uart_reporter
        generic (CLK_FREQ  : integer := 100_000_000;
                 BAUD_RATE : integer := 115_200;
                 NUM_PRNS  : integer := 32);
        port (clk          : in  std_logic;
              reset_n      : in  std_logic;
              wr_en        : in  std_logic;
              wr_addr      : in  std_logic_vector(4 downto 0);
              wr_acq       : in  std_logic;
              wr_doppler   : in  std_logic_vector(7 downto 0);
              wr_phase     : in  std_logic_vector(9 downto 0);
              wr_snr       : in  std_logic_vector(15 downto 0);
              wr_noise     : in  std_logic_vector(15 downto 0);
              wr_margin    : in  std_logic_vector(15 downto 0);
              wr_flags     : in  std_logic_vector(7 downto 0);
              diag_iq_valid: in  std_logic_vector(31 downto 0);
              diag_iq_err  : in  std_logic_vector(31 downto 0);
              agc_strong_count : in  std_logic_vector(31 downto 0);
              agc_total_count  : in  std_logic_vector(31 downto 0);
              sweep_time_cycles  : in  std_logic_vector(31 downto 0);
              candidate_count    : in  std_logic_vector(7 downto 0);
              sweep_start  : in  std_logic;
              report_start : in  std_logic;
              uart_tx_pin  : out std_logic;
              reporting    : out std_logic);
    end component;

begin

    -- Sincroniza entradas mecanicas al dominio clk_100MHz para evitar
    -- metastabilidad y rutas asincronas largas desde los pines SW.
    process(clk_100MHz)
    begin
        if rising_edge(clk_100MHz) then
            if sys_reset_n = '0' then
                sw_5_ff1 <= '0'; sw_5_sync <= '0';
            else
                sw_5_ff1 <= sw_5; sw_5_sync <= sw_5_ff1;
            end if;
        end if;
    end process;

    sw_test_mode   <= sw_5_sync;
    sys_reset   <= not locked;
    sys_reset_n <= locked;

    -- En sintetico se habilitan los satelites configurados en CFG_SATx_EN.
    sat1_en_s <= sw_test_mode when CFG_SAT1_EN else '0';
    sat2_en_s <= sw_test_mode when CFG_SAT2_EN else '0';
    sat3_en_s <= sw_test_mode when CFG_SAT3_EN else '0';

    -- En test sintetico y en real se usa siempre el par I1/I0 sign-magnitude
    -- alimentando la misma cadena de sincronizacion y mezclado.
    fe_i1_raw <= i1_synth when sw_test_mode = '1'                              else
                 i1_real;
    fe_i0_raw <= i0_synth when sw_test_mode = '1'                              else
                 i0_real;

    -- Captura bruta del par sign-magnitude en dominio sincronizado.
    process(clk_100MHz)
    begin
        if rising_edge(clk_100MHz) then
            if sys_reset_n = '0' then
                i1_fe_raw <= '0';
                i0_fe_raw <= '0';
            else
                i1_fe_raw <= fe_i1_raw;
                i0_fe_raw <= fe_i0_raw;
            end if;
        end if;
    end process;

    -- Doble FF anti-metaestabilidad para I1 e I0.
    process(clk_100MHz)
    begin
        if rising_edge(clk_100MHz) then
            if sys_reset_n = '0' then
                i1_ff1 <= '0'; i1_ff2 <= '0';
                i0_ff1 <= '0'; i0_ff2 <= '0';
            else
                i1_ff1 <= i1_fe_raw;
                i1_ff2 <= i1_ff1;
                i0_ff1 <= i0_fe_raw;
                i0_ff2 <= i0_ff1;
            end if;
        end if;
    end process;

    -- Sincronizacion del CLKOUT del MAX2769C (16.368 MHz, JA7:H1) al
    -- dominio clk_100MHz. Doble FF anti-metaestabilidad + deteccion de
    -- flanco de subida para generar sample_en_samp a ~16.368 MHz.
    -- El CLKOUT es el mismo oscilador que da cadencia a I1/I0, por lo que
    -- sample_en_samp queda sincrono con la llegada de nuevas muestras.
    process(clk_100MHz)
    begin
        if rising_edge(clk_100MHz) then
            if sys_reset_n = '0' then
                fe_clk_sync1   <= '0';
                fe_clk_sync2   <= '0';
                fe_clk_prev    <= '0';
                sample_en_samp <= '0';
            else
                fe_clk_sync1   <= fe_clkout;
                fe_clk_sync2   <= fe_clk_sync1;
                fe_clk_prev    <= fe_clk_sync2;
                sample_en_samp <= fe_clk_sync2 and not fe_clk_prev;
            end if;
        end if;
    end process;

    -- DDS 16.368 MHz para replay/sintetico: acc+=16368 cada ciclo;
    -- cuando acc >= 83632 (=100000-16368) se substrae 100000 y se pulsa en=1.
    -- Genera exactamente 16368 enables por cada 100000 ciclos de 100 MHz.
    process(clk_100MHz)
    begin
        if rising_edge(clk_100MHz) then
            int_sample_en <= '0';
            if sys_reset_n = '0' then
                int_sample_acc <= (others => '0');
            elsif sw_test_mode = '1' then
                if int_sample_acc >= to_unsigned(83632, 17) then
                    int_sample_acc <= int_sample_acc + 16368 - 100000;
                    int_sample_en  <= '1';
                else
                    int_sample_acc <= int_sample_acc + 16368;
                end if;
            else
                int_sample_acc <= (others => '0');
            end if;
        end if;
    end process;

    sample_en_drv <= int_sample_en when sw_test_mode = '1' else sample_en_samp;

    -- Decimador x16: captura 1 de cada 16 pulsos del NCO maestro
    -- clk_en_fe es el unico enable del pipeline de correlacion
    process(clk_100MHz)
    begin
        if rising_edge(clk_100MHz) then
            if sys_reset_n = '0' then
                dec_cnt   <= (others => '0');
                i1_dec    <= '0';
                i0_dec    <= '0';
                clk_en_fe <= '0';
            else
                clk_en_fe <= '0';
                if sample_en_drv = '1' then
                if dec_cnt = 15 then
                    dec_cnt <= (others => '0');
                else
                    dec_cnt <= dec_cnt + 1;
                    end if;
                if dec_cnt = 0 then
                        -- Camino unico real/sintetico para depuracion fiel: ambos
                        -- modos pasan por el mismo doble FF de entrada.
                        i1_dec <= i1_ff2;
                        i0_dec <= i0_ff2;
                        clk_en_fe <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process;


    fft_coherent_sel_s <= "00" when CFG_COHERENT_N_MS = 1 else
                          "01" when CFG_COHERENT_N_MS = 2 else
                          "10" when CFG_COHERENT_N_MS = 4 else
                          "11";
    doppler_offset_mux <= std_logic_vector(to_signed(CFG_REAL_DOPPLER_BIAS_BINS, 8)) when sw_test_mode = '0' else x"00";


    -- Diagnostico del front-end I1/I0 por barrido.
    -- diag_iq_valid_s cuenta cuantas muestras entraron al pipeline.
    -- diag_iq_err_s cuenta cambios de muestra I1/I0 entre muestras
    -- consecutivas (metrica de actividad de entrada real). En sintetico
    -- se mantiene en 0 para no mezclar con pruebas de RF real.
    process(clk_100MHz)
        variable v_valid : unsigned(31 downto 0);
        variable v_err   : unsigned(31 downto 0);
        variable v_agc_s : unsigned(31 downto 0);
        variable v_agc_t : unsigned(31 downto 0);
    begin
        if rising_edge(clk_100MHz) then
            if sys_reset_n = '0' then
                diag_iq_valid_s <= (others => '0');
                diag_iq_err_s   <= (others => '0');
                diag_i1_prev_s  <= '0';
                diag_i0_prev_s  <= '0';
                diag_prev_valid_s <= '0';
                agc_strong_count_s <= (others => '0');
                agc_total_count_s  <= (others => '0');
            elsif sweep_restart_s = '1' then
                diag_iq_valid_s <= (others => '0');
                diag_iq_err_s   <= (others => '0');
                diag_i1_prev_s  <= '0';
                diag_i0_prev_s  <= '0';
                diag_prev_valid_s <= '0';
                agc_strong_count_s <= (others => '0');
                agc_total_count_s  <= (others => '0');
            elsif sweep_running = '1' and clk_en_fe = '1' then
                v_valid := unsigned(diag_iq_valid_s);
                v_valid := v_valid + 1;

                v_err := unsigned(diag_iq_err_s);
                if sw_test_mode = '0' then
                    if diag_prev_valid_s = '1' and
                       ((i1_dec /= diag_i1_prev_s) or (i0_dec /= diag_i0_prev_s)) then
                        if v_err /= (31 downto 0 => '1') then
                            v_err := v_err + 1;
                        end if;
                    end if;
                    diag_i1_prev_s <= i1_dec;
                    diag_i0_prev_s <= i0_dec;
                    diag_prev_valid_s <= '1';
                end if;

                v_agc_t := unsigned(agc_total_count_s);
                v_agc_t := v_agc_t + 1;

                v_agc_s := unsigned(agc_strong_count_s);
                if i0_dec = '1' then
                    v_agc_s := v_agc_s + 1;
                end if;

                diag_iq_valid_s <= std_logic_vector(v_valid);
                diag_iq_err_s   <= std_logic_vector(v_err);
                agc_total_count_s  <= std_logic_vector(v_agc_t);
                agc_strong_count_s <= std_logic_vector(v_agc_s);
            end if;
        end if;
    end process;

    ca_local_reset <= sys_reset;

    -- En modo real, suma doppler_offset_mux al bin reportado para que el
    -- campo 'dop' UART refleje el bin Doppler absoluto incluyendo el sesgo
    -- aprendido del oscilador (osc_offset_learned).
    process(sw_test_mode, ram_wr_doppler, doppler_offset_mux)
    begin
        if sw_test_mode = '0' then
            ram_wr_doppler_rep <= std_logic_vector(
                signed(ram_wr_doppler) + signed(doppler_offset_mux));
        else
            ram_wr_doppler_rep <= ram_wr_doppler;
        end if;
    end process;

    -- Registro pipeline fft_ctrl -> peak_inst (1 ciclo de latencia).
    -- Rompe ruta critica entre la salida de la FFT y el calculo de magnitud.
    -- Aisla el routing cross-chip hacia la logica local del peak_detector.
    process(clk_100MHz)
    begin
        if rising_edge(clk_100MHz) then
            if sys_reset_n = '0' then
                correlation_data_r  <= (others => '0');
                correlation_valid_r <= '0';
            else
                correlation_data_r  <= correlation_data;
                correlation_valid_r <= correlation_valid;
            end if;
        end if;
    end process;

    -- Rompe ruta critica peak->acq con un registro intermedio.
    process(clk_100MHz)
    begin
        if rising_edge(clk_100MHz) then
            if sys_reset_n = '0' then
                peak_ready_acq  <= '0';
                pos_pico_acq    <= (others => '0');
                val_pico_acq    <= (others => '0');
                noise_floor_acq <= (others => '0');
            else
                peak_ready_acq  <= peak_ready_s;
                pos_pico_acq    <= pos_pico;
                val_pico_acq    <= val_pico;
                noise_floor_acq <= noise_floor_s;
            end if;
        end if;
    end process;



    -- LEDs
    process(clk_100MHz)
    begin
        if rising_edge(clk_100MHz) then
            if sys_reset_n = '0' then
                t1 <= 0; t2 <= 0;
            else
                if epoch_signal = '1' then t1 <= C_LED_HOLD_CYCLES;
                elsif t1 > 0 then t1 <= t1 - 1; end if;

                if acq_valid_s = '1' then
                    t2 <= C_LED_HOLD_CYCLES;
                elsif t2 > 0 then
                    t2 <= t2 - 1;
                end if;

                led(0) <= sw_test_mode;
                if t1 > 0 then led(1) <= '1'; else led(1) <= '0'; end if;
                if t2 > 0 then led(2) <= '1'; else led(2) <= '0'; end if;
            end if;
        end if;
    end process;

    -- Sondas en LEDs no usados. Son utiles para pinchar con analizador
    -- logico y validar que la entrada I1/I0 real no esta estatica.
    led(3)  <= fe_i1_raw;
    led(4)  <= fe_i0_raw;
    led(5)  <= i1_ff2;
    led(6)  <= i0_ff2;
    led(7)  <= i1_dec;
    led(8)  <= i0_dec;
    led(9)  <= clk_en_fe;
    led(10) <= sample_en_samp;
    led(11) <= epoch_signal;
    led(12) <= sweep_running;
    led(13) <= '0';
    led(14) <= '0';
    led(15) <= acq_done;

    -- ================================================================
    -- INSTANCIAS
    -- ================================================================

    reloj_inst : clk_wiz_0
        port map (clk_in1  => clk,
                  clk_out1 => clk_100MHz,
                  locked   => locked);



    ca_gen_local : gps_ca_generator
        port map (clk       => clk_100MHz,
                  clk_en    => clk_en_fe,
                  reset     => ca_local_reset,
                  prn_num   => prn_out,
                  ca_out    => ca_sig_local,
                  epoch_out => epoch_signal);

    -- El generador sintetico avanza su NCO de portadora a 100 MHz
    -- (clk_en_samp='1') para mantener IF=4.092 MHz con IF_INC=686524.
    multi_sat_inst : multi_sat_rx_gen
        port map (clk          => clk_100MHz,
                  clk_en       => clk_en_fe,
                  clk_en_samp  => '1',
                  reset        => sys_reset,
                  sat1_prn     => C_SAT1_PRN_IDX_SLV,
                  sat1_doppler => C_SAT1_DOPPLER_SLV,
                  sat1_phase   => C_SAT1_PHASE_SLV,
                  sat1_gain    => C_SAT1_GAIN_SLV,
                  sat1_en      => sat1_en_s,
                  sat2_prn     => C_SAT2_PRN_IDX_SLV,
                  sat2_doppler => C_SAT2_DOPPLER_SLV,
                  sat2_phase   => C_SAT2_PHASE_SLV,
                  sat2_gain    => C_SAT2_GAIN_SLV,
                  sat2_en      => sat2_en_s,
                  sat3_prn     => C_SAT3_PRN_IDX_SLV,
                  sat3_doppler => C_SAT3_DOPPLER_SLV,
                  sat3_phase   => C_SAT3_PHASE_SLV,
                  sat3_gain    => C_SAT3_GAIN_SLV,
                  sat3_en      => sat3_en_s,
                  i1_out       => i1_synth,
                  i0_out       => i0_synth,
                  rx_out       => open,
                  epoch_out    => open);

    mixer_inst : doppler_mixer
        port map (clk            => clk_100MHz,
                  reset_n        => sys_reset_n,
                  sample_en      => sample_en_drv,
                  clk_en         => clk_en_fe,
                  i1_sign        => i1_ff2,
                  i0_mag         => i0_ff2,
                  doppler_sel    => doppler_sel_auto,
                  doppler_offset => doppler_offset_mux,
                  rx_I           => rx_I_mixed,
                  rx_Q           => rx_Q_mixed);

    fft_ctrl_inst : fft_controller
        generic map (COHERENT_N_MS => CFG_COHERENT_N_MS)
        port map (clk          => clk_100MHz,
                  clk_en       => clk_en_fe,
                  reset_n      => sys_reset_n,
                  rx_I         => rx_I_mixed,
                  rx_Q         => rx_Q_mixed,
                  ca_bit_local => ca_sig_local,
                  ca_epoch     => epoch_signal,
                  coherent_n_ms_sel => fft_coherent_sel_s,
                  fft_out_ready=> correlation_ready,
                  fft_ready    => correlation_valid,
                  fft_data_out => correlation_data);

    peak_inst : peak_detector
        port map (clk            => clk_100MHz,
                  reset_n        => sys_reset_n,
                  din_valid      => correlation_valid_r,
                  din_ready      => correlation_ready,
                  din_data       => correlation_data_r,
                  fba_clear      => fba_clear_s,
                  fba_last_block => fba_last_block_s,
                  peak_pos       => pos_pico,
                  peak_val       => val_pico,
                  noise_floor    => noise_floor_s,
                  new_peak_ready => peak_ready_s);

    acq_ctrl_inst : entity work.acquisition_controller
        generic map (PEAK_THRESHOLD   => CFG_PEAK_THRESHOLD,
                     K_CFAR           => CFG_K_CFAR,
                     MIN_MARGIN       => CFG_MIN_MARGIN,
                     HYST_ACQ_SWEEPS  => CFG_HYST_ACQ_SWEEPS,
                     HYST_REL_SWEEPS  => CFG_HYST_REL_SWEEPS,
                     GLOBAL_TOP_N     => CFG_GLOBAL_TOP_N,
                     SCORE_CAND_MIN   => CFG_SCORE_CAND_MIN,
                     SCORE_PROMOTE_MIN=> CFG_SCORE_PROMOTE_MIN,
                     SCORE_HOLD_MIN   => CFG_SCORE_HOLD_MIN,
                     LOCK_SNR_MIN     => CFG_LOCK_SNR_MIN,
                     LOCK_SNR_HEADROOM=> CFG_LOCK_SNR_HEADROOM,
                     ADAPT_NOISE_SHIFT=> ADAPT_NOISE_SHIFT,
                     PHASE_BIAS       => CFG_PHASE_BIAS,
                     COHERENT_N_MS    => CFG_COHERENT_N_MS,
                     PEAK_TIMEOUT_EPOCHS => CFG_PEAK_TIMEOUT_EPOCHS,
                     NUM_PRNS         => CFG_NUM_PRNS,
                     N_INT            => CFG_N_INT,
                     BIN_RANGE        => CFG_BIN_RANGE,
                     INITIAL_BIN_RANGE => CFG_INITIAL_BIN_RANGE,
                     EARLY_TERM_K     => CFG_EARLY_TERM_K,
                     PRN_ENABLE_MASK  => CFG_PRN_ENABLE_MASK,
                     COARSE_STEP      => CFG_COARSE_STEP,
                 COARSE_N_INT     => CFG_COARSE_N_INT,
                 REFINE_WINDOW    => CFG_REFINE_WINDOW)
        port map (clk            => clk_100MHz,
                  reset_n        => sys_reset_n,
                  ca_epoch       => epoch_signal,
                  peak_ready     => peak_ready_acq,
                  peak_val       => val_pico_acq,
                  peak_pos       => pos_pico_acq,
                  noise_floor    => noise_floor_acq,
                  doppler_sel    => doppler_sel_auto,
                  prn_out        => prn_out,
                  ram_we         => ram_we,
                  ram_wr_addr    => ram_wr_addr,
                  ram_wr_acq     => ram_wr_acq,
                  ram_wr_doppler => ram_wr_doppler,
                  ram_wr_phase   => ram_wr_phase,
                  ram_wr_snr     => ram_wr_snr,
                  ram_wr_noise   => ram_wr_noise,
                  ram_wr_margin  => ram_wr_margin,
                  ram_wr_flags   => ram_wr_flags,
                  acq_done       => acq_done,
                  acq_valid      => acq_valid_s,
                  sat_count      => open,
                  sweep_bin      => open,
                  sweep_prn      => open,
                  sweep_running  => sweep_running,
                  sweep_restart  => sweep_restart_s,
                  sweep_time_cycles => sweep_time_cycles_s,
                  candidate_count    => candidate_count_s,
                  fba_clear          => fba_clear_s,
                  fba_last_block     => fba_last_block_s);

    uart_reporter_inst : uart_reporter
        generic map (CLK_FREQ  => CFG_CLK_FREQ,
                     BAUD_RATE => CFG_BAUD_RATE,
                     NUM_PRNS  => CFG_NUM_PRNS)
        port map (clk          => clk_100MHz,
                  reset_n      => sys_reset_n,
                  wr_en        => ram_we,
                  wr_addr      => ram_wr_addr,
                  wr_acq       => ram_wr_acq,
                  wr_doppler   => ram_wr_doppler_rep,
                  wr_phase     => ram_wr_phase,
                  wr_snr       => ram_wr_snr,
                  wr_noise     => ram_wr_noise,
                  wr_margin    => ram_wr_margin,
                  wr_flags     => ram_wr_flags,
                  diag_iq_valid=> diag_iq_valid_s,
                  diag_iq_err  => diag_iq_err_s,
                  agc_strong_count => agc_strong_count_s,
                  agc_total_count  => agc_total_count_s,
                  sweep_time_cycles => sweep_time_cycles_s,
                  candidate_count => candidate_count_s,
                  sweep_start  => sweep_restart_s,
                  report_start => acq_done,
                  uart_tx_pin  => uart_tx_pin);

end Structural;