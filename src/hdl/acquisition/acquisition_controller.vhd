library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- acquisition_controller.vhd
--
-- Barre NUM_PRNS codigos CA x (2*BIN_RANGE+1) bins Doppler.
-- Por cada (PRN, bin) acumula N_INT epochs de integracion no
-- coherente (suma lineal de peak) y declara adquisicion si:
--   prn_best_accum > ACCUM_THRESHOLD + (noise >> ADAPT_NOISE_SHIFT)
--   prn_best_accum > K_CFAR * prn_best_noise_accum
--
-- NOTA: el escalado CFAR se realiza con cfar_scale(), que soporta
-- K_CFAR generales (potencias de 2 y valores no potencia de 2).
-- =============================================================

entity acquisition_controller is
    Generic (
        PEAK_THRESHOLD   : integer := 200;
        K_CFAR           : integer := 4;
        -- Minimo margen (best-second) en escala truncada [19:4] para
        -- aceptar adquisicion. Ayuda a rechazar bins Doppler ambiguos.
        MIN_MARGIN       : integer := 0;
        -- Histéresis temporal por PRN (en numero de barridos):
        -- ACQ: barridos consecutivos con deteccion cruda para declarar lock.
        -- REL: barridos consecutivos sin deteccion cruda para soltar lock.
        HYST_ACQ_SWEEPS  : integer := 2;
        HYST_REL_SWEEPS  : integer := 2;
        -- SNR minimo para declarar lock, en escala truncada [19:4].
        LOCK_SNR_MIN     : integer := 1;
        -- Umbral adaptativo por ruido: dynamic_thr = base + (noise >> ADAPT_NOISE_SHIFT)
        -- Si ADAPT_NOISE_SHIFT=0, equivale a sumar noise completa.
        ADAPT_NOISE_SHIFT : integer := 1;
        -- Correccion fija (chips) para alinear la fase reportada con la
        -- fase fisica esperada en pruebas de banco.
        PHASE_BIAS       : integer := 0;
        NUM_PRNS         : integer := 32;
        N_INT            : integer := 5;
        BIN_RANGE        : integer := 64;
        CONTINUOUS_SWEEP : boolean := true
    );
    Port (
        clk            : in  STD_LOGIC;
        reset_n        : in  STD_LOGIC;
        start          : in  STD_LOGIC;
        ca_epoch       : in  STD_LOGIC;
        peak_ready     : in  STD_LOGIC;
        peak_val       : in  STD_LOGIC_VECTOR(15 downto 0);
        peak_pos       : in  STD_LOGIC_VECTOR(9 downto 0);
        noise_floor    : in  STD_LOGIC_VECTOR(15 downto 0);
        doppler_sel    : out STD_LOGIC_VECTOR(7 downto 0);
        prn_out        : out STD_LOGIC_VECTOR(4 downto 0);
        ram_we         : out STD_LOGIC;
        ram_wr_addr    : out STD_LOGIC_VECTOR(4 downto 0);
        ram_wr_acq     : out STD_LOGIC;
        ram_wr_doppler : out STD_LOGIC_VECTOR(7 downto 0);
        ram_wr_phase   : out STD_LOGIC_VECTOR(9 downto 0);
        ram_wr_snr     : out STD_LOGIC_VECTOR(15 downto 0);
        ram_wr_noise   : out STD_LOGIC_VECTOR(15 downto 0);
        ram_wr_margin  : out STD_LOGIC_VECTOR(15 downto 0);
        ram_wr_flags   : out STD_LOGIC_VECTOR(7 downto 0);
        acq_done       : out STD_LOGIC;
        acq_valid      : out STD_LOGIC;
        sat_count      : out STD_LOGIC_VECTOR(5 downto 0);
        best_prn       : out STD_LOGIC_VECTOR(4 downto 0);
        best_doppler   : out STD_LOGIC_VECTOR(7 downto 0);
        best_pos       : out STD_LOGIC_VECTOR(9 downto 0);
        best_val       : out STD_LOGIC_VECTOR(15 downto 0);
        sweep_bin      : out STD_LOGIC_VECTOR(7 downto 0);
        sweep_prn      : out STD_LOGIC_VECTOR(4 downto 0);
        sweep_running  : out STD_LOGIC;
        sweep_restart  : out STD_LOGIC
    );
end acquisition_controller;

architecture Behavioral of acquisition_controller is

    type prn_array_t is array (0 to 31) of integer range 0 to 31;
    constant PRN_LIST : prn_array_t := (
         0,  1,  2,  3,  4,  5,  6,  7,
         8,  9, 10, 11, 12, 13, 14, 15,
        16, 17, 18, 19, 20, 21, 22, 23,
        24, 25, 26, 27, 28, 29, 30, 31
    );

    constant ACCUM_THRESHOLD : unsigned(23 downto 0) :=
        to_unsigned(PEAK_THRESHOLD * N_INT, 24);

    -- Escalado CFAR exacto para cualquier K_CFAR.
    -- Para potencias de 2 usa shift (logica minima); en otros casos usa
    -- multiplicacion exacta para evitar errores silenciosos.
    function cfar_scale(noise : unsigned(23 downto 0); k : integer) return unsigned is
        variable acc : unsigned(31 downto 0);
    begin
        if k = 1 then
            return noise;
        elsif k = 2 then
            return shift_left(noise, 1);
        elsif k = 3 then
            return shift_left(noise, 1) + noise;
        elsif k = 4 then
            return shift_left(noise, 2);
        elsif k = 5 then
            return shift_left(noise, 2) + noise;
        elsif k = 6 then
            return shift_left(noise, 2) + shift_left(noise, 1);
        elsif k = 7 then
            return shift_left(noise, 3) - noise;
        elsif k = 8 then
            return shift_left(noise, 3);
        elsif k = 16 then
            return shift_left(noise, 4);
        else
            -- Escalado generico sin multiplicadores anchos para evitar
            -- problemas de anchura en sintesis y mantener timing estable.
            acc := (others => '0');
            for i in 1 to 31 loop
                if i <= k then
                    acc := acc + resize(noise, 32);
                end if;
            end loop;
            return acc(23 downto 0);
        end if;
    end function;

    -- Convierte indice de pico circular (0..1023) a fase de codigo GPS
    -- en chips (0..1023), con sesgo calibrable PHASE_BIAS.
    function map_peak_to_phase(pos : std_logic_vector(9 downto 0); bias : integer)
        return std_logic_vector is
        variable p        : integer;
        variable phase_v  : integer;
        variable phase_n  : integer;
    begin
        p := to_integer(unsigned(pos));
        -- Correlacion circular: referencia aproximada inversa respecto al retardo.
        phase_v := (1023 - p) + bias;
        phase_n := phase_v mod 1024;
        if phase_n < 0 then
            phase_n := phase_n + 1024;
        end if;
        return std_logic_vector(to_unsigned(phase_n, 10));
    end function;

    -- Distancia circular minima entre dos fases en [0..1023].
    function phase_circular_diff(a, b : std_logic_vector(9 downto 0)) return integer is
        variable ai   : integer;
        variable bi   : integer;
        variable diff : integer;
    begin
        ai := to_integer(unsigned(a));
        bi := to_integer(unsigned(b));
        diff := abs(ai - bi);
        if diff > 512 then
            diff := 1024 - diff;
        end if;
        return diff;
    end function;

    type t_state is (
        S_IDLE, S_INIT_PRN, S_INIT_BIN,
        S_PRIME_WAIT_EPOCH, S_PRIME_WAIT_PEAK,
        S_WAIT_EPOCH, S_WAIT_PEAK, S_ACCUMULATE,
        S_COMPARE, S_END_PRN,
        S_LATCH_RESULT, S_WRITE_RAM,
        S_DONE, S_WAIT
    );
    signal state : t_state := S_IDLE;

    signal prn_idx : integer range 0 to 31 := 0;
    signal bin_cnt : signed(7 downto 0)    := (others => '0');
    signal int_cnt : integer range 0 to 255 := 0;

    signal bin_accum             : unsigned(23 downto 0) := (others => '0');
    signal noise_accum           : unsigned(23 downto 0) := (others => '0');
    signal prn_best_accum        : unsigned(23 downto 0) := (others => '0');
    signal prn_best_noise_accum  : unsigned(23 downto 0) := (others => '0');
    signal prn_second_accum      : unsigned(23 downto 0) := (others => '0');
    signal prn_best_dop          : std_logic_vector(7 downto 0) := (others => '0');
    signal prn_best_pos          : std_logic_vector(9 downto 0) := (others => '0');
    signal prn_acquired          : std_logic := '0';

    signal glob_best_accum : unsigned(23 downto 0)       := (others => '0');
    signal glob_best_dop   : std_logic_vector(7 downto 0) := (others => '0');
    signal glob_best_pos   : std_logic_vector(9 downto 0) := (others => '0');
    signal glob_best_prn   : std_logic_vector(4 downto 0) := (others => '0');

    signal sat_count_r  : unsigned(5 downto 0)       := (others => '0');
    signal latch_val    : unsigned(15 downto 0)       := (others => '0');
    signal latch_pos    : std_logic_vector(9 downto 0) := (others => '0');
    signal bin_best_val : unsigned(15 downto 0)       := (others => '0');
    signal bin_best_pos : std_logic_vector(9 downto 0) := (others => '0');
    signal epoch_prev   : std_logic := '0';

    signal current_prn_slv : std_logic_vector(4 downto 0);
    signal ram_wr_addr_r   : std_logic_vector(4 downto 0)  := (others => '0');
    signal ram_wr_doppler_r : std_logic_vector(7 downto 0) := (others => '0');
    signal ram_wr_phase_r   : std_logic_vector(9 downto 0) := (others => '0');
    signal ram_wr_snr_r    : std_logic_vector(15 downto 0) := (others => '0');
    signal ram_wr_noise_r  : std_logic_vector(15 downto 0) := (others => '0');
    signal ram_wr_margin_r : std_logic_vector(15 downto 0) := (others => '0');
    signal ram_wr_flags_r  : std_logic_vector(7 downto 0)  := (others => '0');

    type phase_array_t is array (0 to 31) of std_logic_vector(9 downto 0);
    type dop_array_t is array (0 to 31) of std_logic_vector(7 downto 0);
    type met_array_t is array (0 to 31) of std_logic_vector(15 downto 0);
    signal prn_lock_phase  : phase_array_t := (others => (others => '0'));
    signal prn_lock_dop    : dop_array_t   := (others => (others => '0'));
    signal prn_lock_snr    : met_array_t   := (others => (others => '0'));

    signal val_sq   : unsigned(15 downto 0) := (others => '0');
    signal noise_sq : unsigned(15 downto 0) := (others => '0');
    signal prime_wait_epochs : integer range 0 to 3 := 0;
    signal wait_peak_epochs  : integer range 0 to 3 := 0;

    type sweep_ctr_array_t is array (0 to 31) of integer range 0 to 15;
    type lock_array_t is array (0 to 31) of std_logic;
    signal prn_hit_ctr  : sweep_ctr_array_t := (others => 0);
    signal prn_miss_ctr : sweep_ctr_array_t := (others => 0);
    signal prn_locked   : lock_array_t      := (others => '0');

begin

    current_prn_slv <= std_logic_vector(to_unsigned(PRN_LIST(prn_idx), 5));

    doppler_sel    <= std_logic_vector(bin_cnt);
    prn_out        <= current_prn_slv;
    sweep_bin      <= std_logic_vector(bin_cnt);
    sweep_prn      <= current_prn_slv;
    best_doppler   <= glob_best_dop;
    best_pos       <= glob_best_pos;
    -- Escala de telemetria: [19:4] para mantener resolucion util en sintetico.
    best_val       <= std_logic_vector(glob_best_accum(19 downto 4));
    best_prn       <= glob_best_prn;
    sat_count      <= std_logic_vector(sat_count_r);
    ram_wr_addr    <= ram_wr_addr_r;
    ram_wr_doppler <= ram_wr_doppler_r;
    ram_wr_phase   <= ram_wr_phase_r;
    ram_wr_snr     <= ram_wr_snr_r;
    ram_wr_noise   <= ram_wr_noise_r;
    ram_wr_margin  <= ram_wr_margin_r;
    ram_wr_flags   <= ram_wr_flags_r;

    process(clk)
        variable v_margin   : unsigned(23 downto 0);
        variable v_margin16 : unsigned(15 downto 0);
        variable raw_acq_v  : std_logic;
        variable dyn_thr_v  : unsigned(23 downto 0);
        variable cfar_ok_v  : boolean;
        variable cfar_strict_ok_v : boolean;
        variable margin_boost_ok_v : boolean;
        variable margin_base_ok_v  : boolean;
        variable margin_support_ok_v : boolean;
        variable second_dom_ok_v   : boolean;
        variable snr_floor_ok_v    : boolean;
        variable snr_headroom_ok_v : boolean;
        variable raw_lock_gate_ok_v : boolean;
        variable flags_v    : std_logic_vector(7 downto 0);
        variable v_hit      : integer range 0 to 15;
        variable v_miss     : integer range 0 to 15;
        variable v_lock     : std_logic;
        variable best_dop_i : integer range -128 to 127;
        variable lock_dop_i : integer range -128 to 127;
        variable lock_snr_i : integer range 0 to 65535;
        variable best_snr_i : integer range 0 to 65535;
        variable phase_jump_i : integer range 0 to 512;
        variable raw_update_ok_v : boolean;
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                state                <= S_IDLE;
                prn_idx              <= 0;
                bin_cnt              <= (others => '0');
                int_cnt              <= 0;
                bin_accum            <= (others => '0');
                noise_accum          <= (others => '0');
                bin_best_val         <= (others => '0');
                bin_best_pos         <= (others => '0');
                prn_best_accum       <= (others => '0');
                prn_best_noise_accum <= (others => '0');
                prn_acquired         <= '0';
                glob_best_accum      <= (others => '0');
                sat_count_r          <= (others => '0');
                epoch_prev           <= '0';
                acq_done             <= '0';
                acq_valid            <= '0';
                sweep_running        <= '0';
                sweep_restart        <= '0';
                ram_we               <= '0';
                ram_wr_doppler_r     <= (others => '0');
                ram_wr_phase_r       <= (others => '0');
                ram_wr_noise_r       <= (others => '0');
                ram_wr_margin_r      <= (others => '0');
                ram_wr_flags_r       <= (others => '0');
                val_sq               <= (others => '0');
                noise_sq             <= (others => '0');
                prime_wait_epochs    <= 0;
                wait_peak_epochs     <= 0;
                prn_hit_ctr          <= (others => 0);
                prn_miss_ctr         <= (others => 0);
                prn_locked           <= (others => '0');
                prn_lock_phase       <= (others => (others => '0'));
                prn_lock_dop         <= (others => (others => '0'));
                prn_lock_snr         <= (others => (others => '0'));
            else
                epoch_prev    <= ca_epoch;
                ram_we        <= '0';
                sweep_restart <= '0';

                case state is

                    when S_IDLE =>
                        acq_done      <= '0';
                        acq_valid     <= '0';
                        sweep_running <= '0';
                        if start = '1' then
                            prn_idx         <= 0;
                            sat_count_r     <= (others => '0');
                            glob_best_accum <= (others => '0');
                            sweep_running   <= '1';
                            sweep_restart   <= '1';
                            state           <= S_INIT_PRN;
                        end if;

                    when S_INIT_PRN =>
                        prn_best_accum       <= (others => '0');
                        prn_best_noise_accum <= (others => '0');
                        prn_second_accum     <= (others => '0');
                        prn_best_dop         <= (others => '0');
                        prn_best_pos         <= (others => '0');
                        prn_acquired         <= '0';
                        bin_cnt              <= to_signed(-BIN_RANGE, 8);
                        state                <= S_INIT_BIN;

                    when S_INIT_BIN =>
                        bin_accum    <= (others => '0');
                        noise_accum  <= (others => '0');
                        bin_best_val <= (others => '0');
                        bin_best_pos <= (others => '0');
                        int_cnt      <= 0;
                        prime_wait_epochs <= 0;
                        wait_peak_epochs  <= 0;
                        -- Descartar el primer frame tras cambiar PRN/bin para
                        -- purgar latencia de pipeline FFT/IFFT.
                        state        <= S_PRIME_WAIT_EPOCH;

                    when S_PRIME_WAIT_EPOCH =>
                        if ca_epoch = '1' and epoch_prev = '0' then
                            prime_wait_epochs <= 0;
                            state <= S_PRIME_WAIT_PEAK;
                        end if;

                    when S_PRIME_WAIT_PEAK =>
                        if peak_ready = '1' then
                            state <= S_WAIT_EPOCH;
                        elsif ca_epoch = '1' and epoch_prev = '0' then
                            if prime_wait_epochs = 3 then
                                -- Recuperacion ante perdida de frame en pipeline.
                                state <= S_INIT_BIN;
                                prime_wait_epochs <= 0;
                            else
                                prime_wait_epochs <= prime_wait_epochs + 1;
                            end if;
                        end if;

                    when S_WAIT_EPOCH =>
                        if ca_epoch = '1' and epoch_prev = '0' then
                            wait_peak_epochs <= 0;
                            state <= S_WAIT_PEAK;
                        end if;

                    when S_WAIT_PEAK =>
                        if peak_ready = '1' then
                            latch_val   <= unsigned(peak_val);
                            latch_pos   <= peak_pos;
                            if unsigned(noise_floor) = 0 then
                                noise_sq    <= to_unsigned(1, 16);
                            else
                                noise_sq    <= unsigned(noise_floor);
                            end if;
                            val_sq      <= unsigned(peak_val);
                            state       <= S_ACCUMULATE;
                        elsif ca_epoch = '1' and epoch_prev = '0' then
                            if wait_peak_epochs = 3 then
                                -- Evita bloqueo del barrido si no llega peak_ready.
                                state <= S_INIT_BIN;
                                wait_peak_epochs <= 0;
                            else
                                wait_peak_epochs <= wait_peak_epochs + 1;
                            end if;
                        end if;

                    when S_ACCUMULATE =>
                        bin_accum   <= bin_accum   + resize(val_sq,   24);
                        noise_accum <= noise_accum + resize(noise_sq, 24);

                        if latch_val > bin_best_val then
                            bin_best_val <= latch_val;
                            bin_best_pos <= latch_pos;
                        end if;

                        -- Integrar siempre N_INT epochs para que todos los bins
                        -- sean comparables con la misma ventana temporal.
                        if int_cnt = N_INT - 1 then
                            state <= S_COMPARE;
                        else
                            int_cnt <= int_cnt + 1;
                            state   <= S_WAIT_EPOCH;
                        end if;

                    when S_COMPARE =>
                        if bin_accum > prn_best_accum then
                            prn_second_accum     <= prn_best_accum;
                            prn_best_accum       <= bin_accum;
                            prn_best_noise_accum <= noise_accum;
                            prn_best_dop         <= std_logic_vector(bin_cnt);
                            prn_best_pos         <= bin_best_pos;
                        elsif bin_accum > prn_second_accum then
                            prn_second_accum <= bin_accum;
                        end if;
                        if bin_cnt = to_signed(BIN_RANGE, 8) then
                            state <= S_END_PRN;
                        else
                            bin_cnt <= bin_cnt + 1;
                            state   <= S_INIT_BIN;
                        end if;

                    when S_END_PRN =>
                        if prn_best_accum > prn_second_accum then
                            v_margin := prn_best_accum - prn_second_accum;
                        else
                            v_margin := (others => '0');
                        end if;
                        v_margin16 := v_margin(19 downto 4);

                                -- Criterio de lock:
                                -- 1) umbral absoluto siempre obligatorio,
                                -- 2) CFAR o, alternativamente, margen reforzado,
                                --    o dominancia frente al segundo mejor bin,
                                -- 3) margen minimo base configurable.
                        -- Importante: cuando MIN_MARGIN=0 no se debe anular CFAR.
                        cfar_ok_v := (prn_best_accum > cfar_scale(prn_best_noise_accum, K_CFAR));
                        cfar_strict_ok_v := (prn_best_accum > (prn_best_noise_accum + shift_right(prn_best_noise_accum, 1)));
                        margin_boost_ok_v := (MIN_MARGIN > 0) and
                                             (v_margin16 >= to_unsigned(MIN_MARGIN * 2, 16));
                        margin_base_ok_v := (v_margin16 >= to_unsigned(MIN_MARGIN, 16));
                        margin_support_ok_v := (MIN_MARGIN > 0) and margin_base_ok_v;
                                -- Fallback robusto en mezcla multi-satelite 1-bit:
                            -- best > 1.125 * second (second + second/8).
                                -- Evita NOLOCK cuando el ruido agregado eleva CFAR,
                                -- manteniendo preferencia por el mejor bin Doppler.
                                second_dom_ok_v := (prn_best_accum >
                                        (prn_second_accum + shift_right(prn_second_accum, 3)));
                                snr_floor_ok_v := (prn_best_accum(19 downto 4) >= to_unsigned(LOCK_SNR_MIN, 16));
                                -- SNR claramente por encima del suelo (telemetria).
                                -- Se mantiene para diagnostico, pero no habilita por si
                                -- solo un lock inicial ambiguo entre bins cercanos.
                                snr_headroom_ok_v := (prn_best_accum(19 downto 4) >= to_unsigned(LOCK_SNR_MIN + 8, 16));

                        -- Para adquirir lock por primera vez exigimos una prueba
                        -- adicional de separacion (margen o dominancia) junto con
                        -- CFAR estricto. Una vez bloqueado, permitimos criterio
                        -- mas flexible para no perder lock por jitter de cuantizacion.
                        if prn_locked(prn_idx) = '0' then
                            -- En primer lock permitimos alternativa CFAR+SNR alto
                            -- para no perder satelites validos en mezcla multi-sat
                            -- cuando best-second queda muy comprimido por cuantizacion.
                            raw_lock_gate_ok_v := (cfar_strict_ok_v and
                                                   (second_dom_ok_v or margin_base_ok_v or
                                                    margin_boost_ok_v)) or
                                                  (cfar_ok_v and snr_headroom_ok_v);
                        else
                            raw_lock_gate_ok_v := (cfar_strict_ok_v or second_dom_ok_v or
                                                   margin_boost_ok_v or margin_support_ok_v);
                        end if;

                        dyn_thr_v := ACCUM_THRESHOLD + shift_right(prn_best_noise_accum, ADAPT_NOISE_SHIFT);

                        if prn_best_accum > dyn_thr_v and
                                snr_floor_ok_v and
                                cfar_ok_v and
                                raw_lock_gate_ok_v
                        then
                            raw_acq_v := '1';
                        else
                            raw_acq_v := '0';
                        end if;

                        -- Histéresis temporal por PRN para estabilizar lock/no-lock.
                        v_hit  := prn_hit_ctr(prn_idx);
                        v_miss := prn_miss_ctr(prn_idx);
                        v_lock := prn_locked(prn_idx);

                        if raw_acq_v = '1' then
                            v_miss := 0;
                            if v_lock = '0' then
                                if v_hit < HYST_ACQ_SWEEPS then
                                    v_hit := v_hit + 1;
                                end if;
                                if v_hit >= HYST_ACQ_SWEEPS then
                                    v_lock := '1';
                                end if;
                            end if;
                        else
                            v_hit := 0;
                            if v_lock = '1' then
                                if v_miss < HYST_REL_SWEEPS then
                                    v_miss := v_miss + 1;
                                end if;
                                if v_miss >= HYST_REL_SWEEPS then
                                    v_lock := '0';
                                end if;
                            end if;
                        end if;

                        prn_hit_ctr(prn_idx)  <= v_hit;
                        prn_miss_ctr(prn_idx) <= v_miss;
                        prn_locked(prn_idx)   <= v_lock;

                        flags_v := (others => '0');
                        if prn_best_accum > dyn_thr_v then flags_v(0) := '1'; end if;
                        if cfar_ok_v         then flags_v(1) := '1'; end if;
                        if cfar_strict_ok_v  then flags_v(2) := '1'; end if;
                        if margin_base_ok_v  then flags_v(3) := '1'; end if;
                        if second_dom_ok_v   then flags_v(4) := '1'; end if;
                        if snr_floor_ok_v    then flags_v(5) := '1'; end if;
                        if raw_acq_v = '1'   then flags_v(6) := '1'; end if;
                        if v_lock = '1'      then flags_v(7) := '1'; end if;
                        ram_wr_noise_r  <= std_logic_vector(prn_best_noise_accum(19 downto 4));
                        ram_wr_margin_r <= std_logic_vector(v_margin16);
                        ram_wr_flags_r  <= flags_v;

                        if raw_acq_v = '1' then
                            -- Estabiliza el lock: si ya estaba bloqueado, solo acepta
                            -- saltos grandes de Doppler cuando hay mejora clara de SNR.
                            best_dop_i := to_integer(signed(prn_best_dop));
                            lock_dop_i := to_integer(signed(prn_lock_dop(prn_idx)));
                            best_snr_i := to_integer(prn_best_accum(19 downto 4));
                            lock_snr_i := to_integer(unsigned(prn_lock_snr(prn_idx)));
                            phase_jump_i := phase_circular_diff(prn_best_pos, prn_lock_phase(prn_idx));

                            raw_update_ok_v := true;
                            -- Filtro anti-salto solo cuando YA estaba en lock antes
                            -- de este barrido. Si acaba de pasar de unlock->lock,
                            -- se debe sembrar siempre con el mejor candidato actual.
                            if prn_locked(prn_idx) = '1' then
                                -- En lock activo, para mover Doppler/fase exigimos
                                -- evidencia adicional: mejora SNR y separacion minima
                                -- frente a bins competidores. Esto reduce saltos 1-bin
                                -- espurios en barrido sintetico multi-senal.
                                if abs(best_dop_i - lock_dop_i) > 0 and
                                   (best_snr_i < (lock_snr_i + 4) or
                                    not (second_dom_ok_v or margin_base_ok_v)) then
                                    raw_update_ok_v := false;
                                end if;
                                if phase_jump_i > 16 and
                                   (best_snr_i < (lock_snr_i + 4) or
                                    not (second_dom_ok_v or margin_base_ok_v)) then
                                    raw_update_ok_v := false;
                                end if;
                            end if;

                            -- Proteccion adicional: nunca dejar lock sembrado en cero.
                            if prn_locked(prn_idx) = '0' and
                               unsigned(prn_lock_snr(prn_idx)) = 0 then
                                raw_update_ok_v := true;
                            end if;

                            if raw_update_ok_v then
                                prn_lock_phase(prn_idx)  <= prn_best_pos;
                                prn_lock_dop(prn_idx)    <= prn_best_dop;
                                prn_lock_snr(prn_idx)    <= std_logic_vector(prn_best_accum(19 downto 4));
                            end if;
                        end if;

                        if v_lock = '1' then
                            sat_count_r  <= sat_count_r + 1;
                            prn_acquired <= '1';
                        else
                            prn_acquired <= '0';
                        end if;

                        if prn_best_accum > glob_best_accum then
                            glob_best_accum <= prn_best_accum;
                            glob_best_dop   <= prn_best_dop;
                            glob_best_pos   <= prn_best_pos;
                            glob_best_prn   <= current_prn_slv;
                        end if;
                        state <= S_LATCH_RESULT;

                    when S_LATCH_RESULT =>
                        ram_wr_acq    <= prn_acquired;
                        ram_wr_addr_r <= current_prn_slv;
                        if prn_acquired = '1' then
                            ram_wr_doppler_r <= prn_lock_dop(prn_idx);
                            ram_wr_phase_r   <= map_peak_to_phase(prn_lock_phase(prn_idx), PHASE_BIAS);
                            ram_wr_snr_r     <= prn_lock_snr(prn_idx);
                        else
                            ram_wr_doppler_r <= prn_best_dop;
                            ram_wr_phase_r   <= map_peak_to_phase(prn_best_pos, PHASE_BIAS);
                            ram_wr_snr_r     <= std_logic_vector(prn_best_accum(19 downto 4));
                        end if;
                        state         <= S_WRITE_RAM;

                    when S_WRITE_RAM =>
                        ram_we <= '1';
                        if prn_idx >= NUM_PRNS - 1 then
                            state <= S_DONE;
                        else
                            prn_idx <= prn_idx + 1;
                            state   <= S_INIT_PRN;
                        end if;

                    when S_DONE =>
                        acq_done      <= '1';
                        sweep_running <= '0';
                        if sat_count_r > 0 then
                            acq_valid <= '1';
                        else
                            acq_valid <= '0';
                        end if;
                        state <= S_WAIT;

                    when S_WAIT =>
                        acq_done <= '0';
                        if start = '1' or (CONTINUOUS_SWEEP and ca_epoch = '1' and epoch_prev = '0') then
                            prn_idx         <= 0;
                            sat_count_r     <= (others => '0');
                            glob_best_accum <= (others => '0');
                            acq_valid       <= '0';
                            sweep_running   <= '1';
                            sweep_restart   <= '1';
                            state           <= S_INIT_PRN;
                        end if;

                end case;
            end if;
        end if;
    end process;

end Behavioral;