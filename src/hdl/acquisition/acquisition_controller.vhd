library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.acquisition_utils_pkg.all;

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
        -- Histeresis temporal por PRN (en numero de barridos):
        -- ACQ: barridos consecutivos con deteccion cruda para declarar lock.
        -- REL: barridos consecutivos sin deteccion cruda para soltar lock.
        HYST_ACQ_SWEEPS  : integer := 1;
        HYST_REL_SWEEPS  : integer := 2;
        -- Promocion candidato->lock (agnostica de PRN): el PRN debe repetir
        -- deteccion con coherencia temporal en Doppler/fase antes de lock.
        CAND_DOP_MAX_DELTA : integer := 6;
        CAND_PHASE_MAX_DELTA : integer := 160;
        CAND_SNR_MARGIN : integer := 1;
        -- Arbitraje global por calidad: numero maximo de PRNs
        -- que pueden promocionar lock en un barrido.
        GLOBAL_TOP_N     : integer := 8;
        -- Minimos de score para candidatar/promocionar/mantener lock.
        SCORE_CAND_MIN   : integer := 28;
        SCORE_PROMOTE_MIN: integer := 44;
        SCORE_HOLD_MIN   : integer := 34;
        -- SNR minimo para declarar lock, en escala truncada [19:4].
        LOCK_SNR_MIN     : integer := 1;
        -- Headroom adicional sobre LOCK_SNR_MIN usado en el camino
        -- alternativo de primer lock (cfar_ok + snr alto).
        LOCK_SNR_HEADROOM: integer := 8;
        -- Umbral adaptativo por ruido: dynamic_thr = base + (noise >> ADAPT_NOISE_SHIFT)
        -- Si ADAPT_NOISE_SHIFT=0, equivale a sumar noise completa.
        ADAPT_NOISE_SHIFT : integer := 1;
        -- Correccion fija (chips) para alinear la fase reportada con la
        -- fase fisica esperada en pruebas de banco.
        PHASE_BIAS       : integer := 0;
        COHERENT_N_MS    : integer := 4;
        -- Maximo de epochs a esperar por peak_ready antes de rearmar bin.
        -- Debe cubrir la latencia efectiva del bloque FFT/IFFT.
        PEAK_TIMEOUT_EPOCHS : integer := 3;
        NUM_PRNS         : integer := 32;
        N_INT            : integer := 2;
        BIN_RANGE         : integer := 64;
        -- Rango del primer barrido tras reset. Posterior a este barrido
        -- se usa BIN_RANGE (centrado en osc_offset_learned ya aprendido).
        INITIAL_BIN_RANGE : integer := 160;
        -- Factor CFAR para terminacion temprana del coarse.
        -- Si bin_accum > EARLY_TERM_K x noise_accum, salta a refine sin
        -- evaluar los bins coarse restantes del PRN actual.
        -- 0 = deshabilitado. Valor recomendado: 3 x K_CFAR = 18.
        EARLY_TERM_K      : integer := 18;
        PRN_ENABLE_MASK  : std_logic_vector(31 downto 0) := (others => '1');
        COARSE_STEP      : integer := 2;
        COARSE_N_INT     : integer := 1;
        REFINE_WINDOW    : integer := 1
    );
    Port (
        clk            : in  STD_LOGIC;
        reset_n        : in  STD_LOGIC;
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
        sweep_bin      : out STD_LOGIC_VECTOR(7 downto 0);
        sweep_prn      : out STD_LOGIC_VECTOR(4 downto 0);
        sweep_running  : out STD_LOGIC;
        sweep_restart  : out STD_LOGIC;
        sweep_time_cycles : out STD_LOGIC_VECTOR(31 downto 0);
        candidate_count    : out STD_LOGIC_VECTOR(7 downto 0);
        -- FBA control: peak_detector BRAM management
        fba_clear          : out STD_LOGIC;  -- pulse to clear FBA BRAM (new bin)
        fba_last_block     : out STD_LOGIC   -- '1' on final epoch of bin
    );
end acquisition_controller;

architecture Behavioral of acquisition_controller is
    -- ----------------------------------------------------------------
    -- Convencion de nomenclatura de senales (referencia interna)
    -- ----------------------------------------------------------------
    -- Senales de estado del FSM:
    --   state          : estado actual del FSM principal
    --   prn_idx        : indice del PRN que se esta barriendo (0..NUM_PRNS-1)
    --   bin_cnt        : bin Doppler actual (signed, relativo al bias)
    --   int_cnt        : epoch de integracion actual (0..N_INT-1)
    --   coarse_mode    : '1' en etapa gruesa, '0' en etapa de refinamiento
    --
    -- Acumuladores por bin:
    --   bin_accum      : suma de peak_val en el bin actual [24 bits]
    --   noise_accum    : suma de noise_floor en el bin actual [24 bits]
    --   bin_best_val   : mejor peak_val visto en el bin actual [16 bits]
    --   bin_best_pos   : posicion del mejor peak en el bin actual [10 bits]
    --
    -- Acumuladores por PRN (mejor bin del barrido):
    --   prn_best_accum       : maximo bin_accum del PRN [24 bits]
    --   prn_best_noise_accum : noise_accum asociado al bin ganador [24 bits]
    --   prn_second_accum     : segundo mayor bin_accum [24 bits]
    --   prn_best_dop         : bin Doppler del bin ganador [8 bits signed]
    --   prn_best_pos         : posicion del pico del bin ganador [10 bits]
    --
    -- Histeresis temporal por PRN (arrays de 32 elementos):
    --   prn_hit_ctr    : contador de detecciones consecutivas [0..HYST_ACQ_SWEEPS]
    --   prn_miss_ctr   : contador de fallos consecutivos [0..HYST_REL_SWEEPS]
    --   prn_locked     : '1' si el PRN esta en estado lock
    --
    -- Seguimiento del lock actual:
    --   cur_lock_phase : fase del ultimo lock valido del PRN actual [chips]
    --   cur_lock_dop   : bin Doppler del ultimo lock valido [signed]
    --   cur_lock_snr   : SNR del ultimo lock valido [escala {19:4}]
    --   cur_locked     : '1' si el PRN esta locked al inicio del barrido actual
    --
    -- Salidas globales del barrido:
    --   glob_best_*    : mejor PRN del barrido completo (para telemetria)
    --   sat_count_r    : numero de PRNs con lock al final del barrido [6 bits]
    --   sweep_time_r   : ciclos de reloj del barrido en curso [32 bits]
    --   sweep_time_last_r : ciclos del ultimo barrido completo (reportado via UART)
    -- ----------------------------------------------------------------

    type t_state is (
        S_IDLE, S_INIT_PRN, S_INIT_BIN,
        S_DISCARD_EPOCH, S_WAIT_EPOCH,
        S_ACCUMULATE, S_WAIT_PEAK,
        S_COMPARE, S_END_PRN_EVAL, S_END_PRN_EVAL2, S_END_PRN_COMMIT,
        S_ARB_PREP, S_ARB_READ_ARRAYS, S_ARB_APPLY_PRN, S_ARB_COMMIT_PRN, S_ARB_WRITE_RAM,
        S_DONE, S_WAIT
    );
    signal state : t_state := S_IDLE;

    signal prn_idx : integer range 0 to 31 := 0;
    signal bin_cnt : signed(7 downto 0)    := (others => '0');
    signal int_cnt : integer range 0 to 255 := 0;
    signal coarse_mode : std_logic := '1';
    signal coarse_best_bin : signed(7 downto 0) := (others => '0');
    signal refine_bin_max : signed(7 downto 0) := (others => '0');
    -- Rango Doppler activo: INITIAL_BIN_RANGE en primer barrido, BIN_RANGE en el resto.
    signal eff_bin_range_r : integer range 0 to 255 := INITIAL_BIN_RANGE;

    -- FBA control registers
    signal fba_clear_r      : std_logic := '0';
    signal fba_last_block_r : std_logic := '0';

    signal bin_accum             : unsigned(23 downto 0) := (others => '0');
    signal noise_accum           : unsigned(23 downto 0) := (others => '0');
    signal prn_best_accum        : unsigned(23 downto 0) := (others => '0');
    signal prn_best_noise_accum  : unsigned(23 downto 0) := (others => '0');
    signal prn_second_accum      : unsigned(23 downto 0) := (others => '0');
    signal prn_best_dop          : std_logic_vector(7 downto 0) := (others => '0');
    signal prn_best_pos          : std_logic_vector(9 downto 0) := (others => '0');

    signal sat_count_r  : unsigned(5 downto 0)       := (others => '0');
    signal bin_best_pos : std_logic_vector(9 downto 0) := (others => '0');
    signal epoch_prev   : std_logic := '0';

    signal sweep_running_r : std_logic := '0';
    signal sweep_time_r    : unsigned(31 downto 0) := (others => '0');
    signal candidate_cnt_r : unsigned(7 downto 0)  := (others => '0');
    signal sweep_time_last_r    : unsigned(31 downto 0) := (others => '0');
    signal candidate_cnt_last_r : unsigned(7 downto 0)  := (others => '0');

    signal current_prn_slv : std_logic_vector(4 downto 0);
    signal ram_wr_addr_r   : std_logic_vector(4 downto 0)  := (others => '0');
    signal ram_wr_doppler_r : std_logic_vector(7 downto 0) := (others => '0');
    signal ram_wr_phase_r   : std_logic_vector(9 downto 0) := (others => '0');
    signal ram_wr_snr_r    : std_logic_vector(15 downto 0) := (others => '0');
    signal ram_wr_noise_r  : std_logic_vector(15 downto 0) := (others => '0');
    signal ram_wr_margin_r : std_logic_vector(15 downto 0) := (others => '0');
    signal ram_wr_flags_r  : std_logic_vector(7 downto 0)  := (others => '0');

    type phase_array_t is array (0 to NUM_PRNS - 1) of std_logic_vector(9 downto 0);
    type dop_array_t is array (0 to NUM_PRNS - 1) of std_logic_vector(7 downto 0);
    type met_array_t is array (0 to NUM_PRNS - 1) of std_logic_vector(15 downto 0);
    signal prn_lock_phase  : phase_array_t := (others => (others => '0'));
    signal prn_lock_dop    : dop_array_t   := (others => (others => '0'));
    signal prn_lock_snr    : met_array_t   := (others => (others => '0'));
    signal cur_lock_phase  : std_logic_vector(9 downto 0) := (others => '0');
    signal cur_lock_dop    : std_logic_vector(7 downto 0) := (others => '0');
    signal cur_lock_snr    : std_logic_vector(15 downto 0) := (others => '0');
    signal cur_locked      : std_logic := '0';

    signal wait_peak_epochs  : integer range 0 to 31 := 0;

    type sweep_ctr_array_t is array (0 to NUM_PRNS - 1) of integer range 0 to 15;
    type lock_array_t is array (0 to NUM_PRNS - 1) of std_logic;
    type flag_array_t is array (0 to NUM_PRNS - 1) of std_logic_vector(7 downto 0);
    type rank_array_t is array (0 to NUM_PRNS - 1) of std_logic_vector(7 downto 0);
    signal prn_hit_ctr  : sweep_ctr_array_t := (others => 0);
    signal prn_miss_ctr : sweep_ctr_array_t := (others => 0);
    signal prn_raw_ctr  : sweep_ctr_array_t := (others => 0);
    signal prn_locked   : lock_array_t      := (others => '0');
    signal prn_cand_valid : lock_array_t    := (others => '0');
    signal prn_cand_phase : phase_array_t   := (others => (others => '0'));
    signal prn_cand_dop   : dop_array_t     := (others => (others => '0'));

    -- Snapshot por PRN de las metricas del barrido actual.
    signal prn_eval_dop    : dop_array_t     := (others => (others => '0'));
    signal prn_eval_phase  : phase_array_t   := (others => (others => '0'));
    signal prn_eval_snr    : met_array_t     := (others => (others => '0'));
    signal prn_eval_score  : met_array_t     := (others => (others => '0'));
    signal prn_eval_margin : met_array_t     := (others => (others => '0'));
    signal prn_eval_flags  : flag_array_t    := (others => (others => '0'));
    signal prn_eval_raw    : lock_array_t    := (others => '0');
    signal prn_eval_cand   : lock_array_t    := (others => '0');

    attribute ram_style : string;
    attribute ram_style of prn_eval_dop    : signal is "block";
    attribute ram_style of prn_eval_phase  : signal is "block";
    attribute ram_style of prn_eval_snr    : signal is "block";
    attribute ram_style of prn_eval_score  : signal is "block";
    attribute ram_style of prn_eval_margin : signal is "block";
    attribute ram_style of prn_eval_flags  : signal is "block";
    signal prn_selected    : lock_array_t    := (others => '0');
    signal prn_rank        : rank_array_t    := (others => x"FF");
    signal prn_reason      : flag_array_t    := (others => (others => '0'));

    constant TOP_N_MAX : integer := 4;
    type top_score_array_t is array (0 to TOP_N_MAX - 1) of unsigned(15 downto 0);
    type top_prn_array_t   is array (0 to TOP_N_MAX - 1) of std_logic_vector(4 downto 0);
    signal top_scores_r : top_score_array_t := (others => (others => '0'));
    signal top_prns_r   : top_prn_array_t   := (others => (others => '0'));
    signal top_valid_r  : std_logic_vector(TOP_N_MAX - 1 downto 0) := (others => '0');

    -- Registros intermedios para el camino pipelined de evaluacion de lock.
    signal end_raw_acq_r        : std_logic := '0';
    signal end_update_lock_r    : std_logic := '0';
    signal end_second_dom_ok_r  : std_logic := '0';
    signal end_margin_boost_ok_r: std_logic := '0';
    signal end_margin_base_ok_r : std_logic := '0';
    signal end_snr_floor_ok_r   : std_logic := '0';
    signal end_dyn_thr_ok_r     : std_logic := '0';
    signal end_cfar_ok_r        : std_logic := '0';
    signal end_cfar_strict_ok_r : std_logic := '0';
    signal end_best_snr16_r     : std_logic_vector(15 downto 0) := (others => '0');
    signal end_margin16_r       : std_logic_vector(15 downto 0) := (others => '0');
    signal end_prn_best_dop_r   : std_logic_vector(7 downto 0) := (others => '0');
    signal end_prn_best_pos_r   : std_logic_vector(9 downto 0) := (others => '0');
    signal end_candidate_inc_r  : std_logic := '0';
    signal end_cand_event_r     : std_logic := '0';

    -- Pipeline registers for ARB stage 1 -> stage 2 split (timing fix).
    signal arb_v_hit_r       : integer range 0 to 15 := 0;
    signal arb_v_miss_r      : integer range 0 to 15 := 0;
    signal arb_v_lock_r      : std_logic := '0';
    signal arb_was_locked_r  : std_logic := '0';
    signal arb_selected_r    : std_logic := '0';
    signal arb_raw_event_r   : std_logic := '0';
    signal arb_cand_event_r  : std_logic := '0';
    signal arb_cand_coherent_r : std_logic := '0';
    signal arb_cand_snr_ok_r : std_logic := '0';
    signal arb_cand_score_ok_r : std_logic := '0';
    signal arb_hold_score_ok_r : std_logic := '0';
    signal arb_cand_valid_r  : std_logic := '0';
    signal arb_flags_r       : std_logic_vector(7 downto 0) := (others => '0');
    signal arb_eval_dop_r    : std_logic_vector(7 downto 0) := (others => '0');
    signal arb_eval_phase_r  : std_logic_vector(9 downto 0) := (others => '0');
    signal arb_eval_snr_r    : std_logic_vector(15 downto 0) := (others => '0');
    signal arb_eval_score_r  : std_logic_vector(15 downto 0) := (others => '0');
    signal arb_rank_r        : std_logic_vector(7 downto 0) := (others => '0');
    signal arb_lock_dop_r    : std_logic_vector(7 downto 0) := (others => '0');
    signal arb_lock_phase_r  : std_logic_vector(9 downto 0) := (others => '0');
    signal arb_lock_snr_r    : std_logic_vector(15 downto 0) := (others => '0');

    signal rd_hit_ctr       : integer range 0 to 15 := 0;
    signal rd_miss_ctr      : integer range 0 to 15 := 0;
    signal rd_locked        : std_logic := '0';
    signal rd_eval_flags    : std_logic_vector(7 downto 0) := (others => '0');
    signal rd_selected      : std_logic := '0';
    signal rd_eval_raw      : std_logic := '0';
    signal rd_eval_score    : std_logic_vector(15 downto 0) := (others => '0');
    signal rd_eval_cand     : std_logic := '0';
    signal rd_cand_valid    : std_logic := '0';
    signal rd_eval_dop      : std_logic_vector(7 downto 0) := (others => '0');
    signal rd_cand_dop      : std_logic_vector(7 downto 0) := (others => '0');
    signal rd_eval_phase    : std_logic_vector(9 downto 0) := (others => '0');
    signal rd_cand_phase    : std_logic_vector(9 downto 0) := (others => '0');
    signal rd_eval_snr      : std_logic_vector(15 downto 0) := (others => '0');
    signal rd_rank          : std_logic_vector(7 downto 0) := (others => '0');
    signal rd_lock_dop      : std_logic_vector(7 downto 0) := (others => '0');
    signal rd_lock_phase    : std_logic_vector(9 downto 0) := (others => '0');
    signal rd_lock_snr      : std_logic_vector(15 downto 0) := (others => '0');
    signal rd_raw_ctr       : integer range 0 to 15 := 0;

begin

    current_prn_slv <= std_logic_vector(to_unsigned(prn_idx, 5));

    doppler_sel    <= std_logic_vector(bin_cnt);
    prn_out        <= current_prn_slv;
    sweep_bin      <= std_logic_vector(bin_cnt);
    sweep_prn      <= current_prn_slv;
    sweep_running  <= sweep_running_r;
    sat_count      <= std_logic_vector(sat_count_r);
    ram_wr_addr    <= ram_wr_addr_r;
    ram_wr_doppler <= ram_wr_doppler_r;
    ram_wr_phase   <= ram_wr_phase_r;
    ram_wr_snr     <= ram_wr_snr_r;
    ram_wr_noise   <= ram_wr_noise_r;
    ram_wr_margin  <= ram_wr_margin_r;
    ram_wr_flags   <= ram_wr_flags_r;
    sweep_time_cycles <= std_logic_vector(sweep_time_last_r);
    candidate_count <= std_logic_vector(candidate_cnt_last_r);
    fba_clear        <= fba_clear_r;
    fba_last_block   <= fba_last_block_r;

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
        variable snr_gate_ok_v     : boolean;
        variable snr_headroom_ok_v : boolean;
        variable raw_lock_gate_ok_v : boolean;
        variable flags_v    : std_logic_vector(7 downto 0);
        variable v_hit      : integer range 0 to 15;
        variable v_miss     : integer range 0 to 15;
        variable v_lock     : std_logic;
        variable next_prn_i : integer range 0 to 32;
        variable found_prn_v : boolean;
        variable int_target_v : integer range 1 to 255;
        variable coarse_step_v : integer range 1 to 127;
        variable refine_window_v : integer range 0 to 64;
        variable n_ms_eff_v : integer range 1 to 15;
        variable peak_timeout_eff_v : integer range 1 to 31;
        variable coarse_best_i : integer range -128 to 127;
        variable refine_min_i : integer range -128 to 127;
        variable refine_max_i : integer range -128 to 127;
        variable doppler_delta_i : integer range 0 to 255;
        variable phase_delta_i : integer range 0 to 1023;
        variable lock_track_ok_v : boolean;
        variable snr_upgrade_ok_v : boolean;
        variable update_lock_v : boolean;
        variable best_snr16_v : unsigned(15 downto 0);
        variable cfar_limit_v : unsigned(23 downto 0);
        variable second_dom_limit_v : unsigned(23 downto 0);
        variable noise_half_v : unsigned(23 downto 0);
        variable prev_lock_snr_v : unsigned(15 downto 0);
        variable sign_flip_v : boolean;
        variable reacq_guard_ok_v : boolean;
        variable cand_dop_delta_i : integer range 0 to 255;
        variable cand_phase_delta_i : integer range 0 to 1023;
        variable cand_coherent_v : boolean;
        variable cand_snr_ok_v : boolean;
        variable cand_event_v : boolean;
        variable score_i : integer range 0 to 1023;
        variable selected_v : boolean;
        variable raw_event_v : boolean;
        variable was_locked_v : boolean;
        variable hold_score_ok_v : boolean;
        variable cand_score_ok_v : boolean;
        variable reason_v : std_logic_vector(7 downto 0);
        variable promote_window_v : boolean;
        variable raw_ctr_v : integer range 0 to 15;
        variable rank_prn_i : integer range 0 to 31;
        variable ins_rank_i : integer range 0 to TOP_N_MAX;
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                state                <= S_IDLE;
                prn_idx              <= 0;
                bin_cnt              <= (others => '0');
                int_cnt              <= 0;
                coarse_mode          <= '1';
                coarse_best_bin      <= (others => '0');
                refine_bin_max       <= (others => '0');
                bin_accum            <= (others => '0');
                noise_accum          <= (others => '0');
                bin_best_pos         <= (others => '0');
                prn_best_accum       <= (others => '0');
                prn_best_noise_accum <= (others => '0');
                sat_count_r          <= (others => '0');
                epoch_prev           <= '0';
                acq_done             <= '0';
                acq_valid            <= '0';
                sweep_running_r      <= '0';
                sweep_restart        <= '0';
                ram_we               <= '0';
                ram_wr_doppler_r     <= (others => '0');
                ram_wr_phase_r       <= (others => '0');
                ram_wr_noise_r       <= (others => '0');
                ram_wr_margin_r      <= (others => '0');
                ram_wr_flags_r       <= (others => '0');
                wait_peak_epochs     <= 0;
                prn_hit_ctr          <= (others => 0);
                prn_miss_ctr         <= (others => 0);
                prn_raw_ctr          <= (others => 0);
                prn_locked           <= (others => '0');
                prn_cand_valid       <= (others => '0');
                prn_cand_phase       <= (others => (others => '0'));
                prn_cand_dop         <= (others => (others => '0'));
                prn_eval_dop         <= (others => (others => '0'));
                prn_eval_phase       <= (others => (others => '0'));
                prn_eval_snr         <= (others => (others => '0'));
                prn_eval_score       <= (others => (others => '0'));
                prn_eval_margin      <= (others => (others => '0'));
                prn_eval_flags       <= (others => (others => '0'));
                prn_eval_raw         <= (others => '0');
                prn_eval_cand        <= (others => '0');
                prn_selected         <= (others => '0');
                prn_rank             <= (others => x"FF");
                prn_reason           <= (others => (others => '0'));
                top_scores_r         <= (others => (others => '0'));
                top_prns_r           <= (others => (others => '0'));
                top_valid_r          <= (others => '0');
                end_raw_acq_r        <= '0';
                end_update_lock_r    <= '0';
                end_second_dom_ok_r  <= '0';
                end_margin_boost_ok_r<= '0';
                end_margin_base_ok_r <= '0';
                end_snr_floor_ok_r   <= '0';
                end_dyn_thr_ok_r     <= '0';
                end_cfar_ok_r        <= '0';
                end_cfar_strict_ok_r <= '0';
                end_best_snr16_r     <= (others => '0');
                end_margin16_r       <= (others => '0');
                end_prn_best_dop_r   <= (others => '0');
                end_prn_best_pos_r   <= (others => '0');
                end_candidate_inc_r  <= '0';
                end_cand_event_r     <= '0';
                prn_lock_phase       <= (others => (others => '0'));
                prn_lock_dop         <= (others => (others => '0'));
                prn_lock_snr         <= (others => (others => '0'));
                cur_lock_phase       <= (others => '0');
                cur_lock_dop         <= (others => '0');
                cur_lock_snr         <= (others => '0');
                cur_locked           <= '0';
                sweep_time_r         <= (others => '0');
                candidate_cnt_r      <= (others => '0');
                sweep_time_last_r    <= (others => '0');
                candidate_cnt_last_r <= (others => '0');
                eff_bin_range_r      <= INITIAL_BIN_RANGE;
                fba_clear_r          <= '0';
                fba_last_block_r     <= '0';
                rd_hit_ctr           <= 0;
                rd_miss_ctr          <= 0;
                rd_locked            <= '0';
                rd_eval_flags        <= (others => '0');
                rd_selected          <= '0';
                rd_eval_raw          <= '0';
                rd_eval_score        <= (others => '0');
                rd_eval_cand         <= '0';
                rd_cand_valid        <= '0';
                rd_eval_dop          <= (others => '0');
                rd_cand_dop          <= (others => '0');
                rd_eval_phase        <= (others => '0');
                rd_cand_phase        <= (others => '0');
                rd_eval_snr          <= (others => '0');
                rd_rank              <= (others => '0');
                rd_lock_dop          <= (others => '0');
                rd_lock_phase        <= (others => '0');
                rd_lock_snr          <= (others => '0');
                rd_raw_ctr           <= 0;
            else
                -- Incrementar contador de ciclos de barrido mientras el barrido esta activo
                if sweep_running_r = '1' then
                    sweep_time_r <= sweep_time_r + 1;
                end if;

                epoch_prev    <= ca_epoch;
                ram_we        <= '0';
                sweep_restart <= '0';
                fba_clear_r   <= '0'; -- default: 1-cycle pulse

                -- Timeout dinamico para esperar peak_ready:
                -- con T7=8ms, peak_ready puede tardar 8+ epochs por bin.
                -- Si el timeout es fijo y corto, el FSM rearma el bin antes
                -- de que llegue el pico y el barrido aparenta "no avanzar".
                n_ms_eff_v := COHERENT_N_MS;
                peak_timeout_eff_v := PEAK_TIMEOUT_EPOCHS;
                -- Escala dinamico muy conservador para IFFT/coherent completion.
                -- IFFT puede tardar hasta n_ms * 3 epochs en piping con otras operaciones.
                -- Factor: n_ms_eff * 3 + 2 cubre latencias de pipeline + margen de seguridad.
                -- 1ms: max(3, 1*3+2) = 5 epochs
                -- 2ms: max(3, 2*3+2) = 8 epochs
                -- 4ms: max(3, 4*3+2) = 14 epochs
                -- 8ms: max(3, 8*3+2) = 26 epochs
                if n_ms_eff_v * 3 + 2 > peak_timeout_eff_v then
                    peak_timeout_eff_v := n_ms_eff_v * 3 + 2;
                end if;

                case state is

                    when S_IDLE =>
                        acq_done      <= '0';
                        acq_valid     <= '0';
                        sweep_running_r <= '0';
                        -- Comienza barrido automaticamente al recibir la primera epoca de C/A
                        if ca_epoch = '1' and epoch_prev = '0' then
                            eff_bin_range_r <= INITIAL_BIN_RANGE; -- primer barrido: rango ampliado
                            next_prn_i := next_enabled_prn(PRN_ENABLE_MASK, 0, NUM_PRNS);
                            found_prn_v := (next_prn_i < NUM_PRNS);
                            if found_prn_v then
                                prn_idx <= next_prn_i;
                            else
                                prn_idx <= 0;
                            end if;
                            sat_count_r     <= (others => '0');
                            top_scores_r    <= (others => (others => '0'));
                            top_prns_r      <= (others => (others => '0'));
                            top_valid_r     <= (others => '0');
                            prn_selected    <= (others => '0');
                            prn_rank        <= (others => x"FF");
                            sweep_running_r <= '1';
                            sweep_restart   <= '1';
                            sweep_time_r    <= (others => '0');
                            candidate_cnt_r <= (others => '0');
                            sweep_time_last_r    <= (others => '0');
                            candidate_cnt_last_r <= (others => '0');
                            if found_prn_v then
                                state <= S_INIT_PRN;
                            else
                                state <= S_DONE;
                            end if;
                        end if;

                    when S_INIT_PRN =>
                        prn_best_accum       <= (others => '0');
                        prn_best_noise_accum <= (others => '0');
                        prn_second_accum     <= (others => '0');
                        prn_best_dop         <= (others => '0');
                        prn_best_pos         <= (others => '0');
                        cur_lock_phase       <= prn_lock_phase(prn_idx);
                        cur_lock_dop         <= prn_lock_dop(prn_idx);
                        cur_lock_snr         <= prn_lock_snr(prn_idx);
                        cur_locked           <= prn_locked(prn_idx);
                        coarse_mode          <= '1';
                        bin_cnt              <= to_signed(-eff_bin_range_r, 8);
                        coarse_best_bin      <= to_signed(-eff_bin_range_r, 8);
                        refine_bin_max       <= to_signed(eff_bin_range_r, 8);
                        state                <= S_INIT_BIN;

                    when S_INIT_BIN =>
                        bin_accum    <= (others => '0');
                        noise_accum  <= (others => '0');
                        bin_best_pos <= (others => '0');
                        int_cnt      <= 0;
                        fba_last_block_r <= '0';
                        state        <= S_DISCARD_EPOCH;

                    when S_DISCARD_EPOCH =>
                        -- Purgamos el primer epoch que contiene datos del bin anterior
                        if ca_epoch = '1' and epoch_prev = '0' then
                            fba_clear_r <= '1';
                            state <= S_WAIT_EPOCH;
                        end if;

                    when S_WAIT_EPOCH =>
                        if ca_epoch = '1' and epoch_prev = '0' then
                                state <= S_ACCUMULATE;
                        end if;

                    when S_ACCUMULATE =>
                        if coarse_mode = '1' then
                            int_target_v := COARSE_N_INT;
                        else
                            int_target_v := N_INT;
                        end if;
                        if int_target_v < 1 then
                            int_target_v := 1;
                        elsif int_target_v > 255 then
                            int_target_v := 255;
                        end if;

                        if int_cnt + 1 = int_target_v then
                            fba_last_block_r <= '1';
                            wait_peak_epochs <= 0;
                            state <= S_WAIT_PEAK;
                        else
                            int_cnt <= int_cnt + 1;
                            state <= S_WAIT_EPOCH;
                        end if;

                    when S_WAIT_PEAK =>
                        -- Solo esperamos a peak_detector despues de haber procesado todas las epocas
                        if peak_ready = '1' then
                            bin_accum <= resize(unsigned(peak_val), 24);
                            noise_accum <= resize(unsigned(noise_floor), 24);
                            bin_best_pos <= peak_pos;
                            fba_last_block_r <= '0';
                            state <= S_COMPARE;
                        elsif ca_epoch = '1' and epoch_prev = '0' then
                            if wait_peak_epochs >= peak_timeout_eff_v then
                                bin_accum <= (others => '0');
                                noise_accum <= to_unsigned(1, 24);
                                fba_last_block_r <= '0';
                                state <= S_COMPARE;
                            else
                                wait_peak_epochs <= wait_peak_epochs + 1;
                            end if;
                        end if;

                    when S_COMPARE =>
                        coarse_step_v := COARSE_STEP;
                        if coarse_step_v < 1 then
                            coarse_step_v := 1;
                        elsif coarse_step_v > 127 then
                            coarse_step_v := 127;
                        end if;

                        refine_window_v := REFINE_WINDOW;
                        if refine_window_v < 0 then
                            refine_window_v := 0;
                        elsif refine_window_v > eff_bin_range_r then
                            refine_window_v := eff_bin_range_r;
                        end if;

                        if bin_accum > prn_best_accum then
                            prn_second_accum     <= prn_best_accum;
                            prn_best_accum       <= bin_accum;
                            prn_best_noise_accum <= noise_accum;
                            prn_best_dop         <= std_logic_vector(bin_cnt);
                            prn_best_pos         <= bin_best_pos;
                            if coarse_mode = '1' then
                                coarse_best_bin <= bin_cnt;
                            end if;
                        elsif bin_accum > prn_second_accum then
                            prn_second_accum <= bin_accum;
                        end if;
                        if coarse_mode = '1' then
                            if (to_integer(bin_cnt) + coarse_step_v > eff_bin_range_r) or
                                    (EARLY_TERM_K > 0 and bin_accum > cfar_scale(noise_accum, EARLY_TERM_K)) then
                                -- Transicion a refinamiento (fin de coarse o terminacion temprana).
                                if bin_accum >= prn_best_accum then
                                    coarse_best_i := to_integer(bin_cnt);
                                else
                                    coarse_best_i := to_integer(coarse_best_bin);
                                end if;
                                refine_min_i := coarse_best_i - refine_window_v;
                                refine_max_i := coarse_best_i + refine_window_v;
                                if refine_min_i < -eff_bin_range_r then
                                    refine_min_i := -eff_bin_range_r;
                                end if;
                                if refine_max_i > eff_bin_range_r then
                                    refine_max_i := eff_bin_range_r;
                                end if;
                                refine_bin_max   <= to_signed(refine_max_i, 8);
                                coarse_mode      <= '0';
                                bin_cnt          <= to_signed(refine_min_i, 8);
                                -- Reinicio del segundo mejor al entrar en refine
                                -- para no mezclar acumulados coarse y refine.
                                prn_second_accum <= (others => '0');
                                prn_best_accum   <= (others => '0');
                                prn_best_noise_accum <= (others => '0');
                                bin_accum        <= (others => '0');
                                noise_accum      <= (others => '0');
                                bin_best_pos     <= (others => '0');
                                int_cnt          <= 0;
                                fba_last_block_r <= '0';
                                state            <= S_DISCARD_EPOCH;
                            else
                                bin_cnt <= bin_cnt + to_signed(coarse_step_v, 8);
                                state   <= S_INIT_BIN;
                            end if;
                        else
                            if bin_cnt = refine_bin_max then
                                state <= S_END_PRN_EVAL;
                            else
                                bin_cnt <= bin_cnt + 1;
                                state   <= S_INIT_BIN;
                            end if;
                        end if;

                    when S_END_PRN_EVAL =>
                        best_snr16_v := prn_best_accum(19 downto 4);
                        prev_lock_snr_v := unsigned(prn_lock_snr(prn_idx));
                        if prn_best_accum > prn_second_accum then
                            v_margin := prn_best_accum - prn_second_accum;
                        else
                            v_margin := (others => '0');
                        end if;
                        v_margin16 := v_margin(19 downto 4);

                        cfar_limit_v := cfar_scale(prn_best_noise_accum, K_CFAR);
                        noise_half_v := shift_right(prn_best_noise_accum, 1);
                        -- Dominancia mas estricta del mejor pico frente al segundo
                        -- para recortar adquisiciones ambiguas (1.5x).
                        second_dom_limit_v := prn_second_accum + shift_right(prn_second_accum, 1);
                        cfar_ok_v := (prn_best_accum > cfar_limit_v);
                        cfar_strict_ok_v := (prn_best_accum > (prn_best_noise_accum + noise_half_v));
                        margin_boost_ok_v := (MIN_MARGIN > 0) and
                                             (v_margin16 >= to_unsigned(MIN_MARGIN * 2, 16));
                        margin_base_ok_v := (v_margin16 >= to_unsigned(MIN_MARGIN, 16));
                        margin_support_ok_v := (MIN_MARGIN > 0) and margin_base_ok_v;
                        second_dom_ok_v := (prn_best_accum > second_dom_limit_v);
                        snr_floor_ok_v  := (best_snr16_v >= to_unsigned(LOCK_SNR_MIN, 16));
                        snr_headroom_ok_v := (best_snr16_v >=
                                               to_unsigned(LOCK_SNR_MIN + LOCK_SNR_HEADROOM, 16));

                        if cur_locked = '0' then
                            -- FBA: simplified lock gate. With full-bin accumulation,
                            -- CFAR alone is a reliable detector (no second_dom/margin needed).
                            raw_lock_gate_ok_v := cfar_ok_v;
                        else
                            -- FBA: simplified maintenance. CFAR or close Doppler tracking.
                            doppler_delta_i := abs_signed_bin_diff(prn_best_dop, cur_lock_dop);
                            if doppler_delta_i <= 8 then
                                raw_lock_gate_ok_v := cfar_ok_v;
                            else
                                raw_lock_gate_ok_v := false;
                            end if;
                        end if;

                        -- Evita flips de signo al reacquirir un PRN ya conocido
                        -- salvo que el nuevo candidato sea claramente mejor.
                        reacq_guard_ok_v := true;
                        sign_flip_v := false;
                        if cur_locked = '0' and prev_lock_snr_v >= to_unsigned(LOCK_SNR_MIN, 16) then
                            sign_flip_v := (prn_best_dop(7) /= prn_lock_dop(prn_idx)(7)) and
                                           (prn_best_dop /= std_logic_vector(to_signed(0, 8))) and
                                           (prn_lock_dop(prn_idx) /= std_logic_vector(to_signed(0, 8)));
                            if sign_flip_v then
                                reacq_guard_ok_v := second_dom_ok_v and
                                                    (best_snr16_v >= (prev_lock_snr_v + to_unsigned(1, 16)));
                            end if;
                        end if;

                        -- FBA: SNR gate is just the floor check.
                        snr_gate_ok_v := snr_floor_ok_v;

                        dyn_thr_v := to_unsigned(PEAK_THRESHOLD * N_INT, 24) +
                                     shift_right(prn_best_noise_accum, ADAPT_NOISE_SHIFT);

                        -- Evitar doble compuerta CFAR estricta en serie.
                        -- raw_lock_gate_ok_v ya encapsula la logica estructural de lock;
                        -- exigir cfar_strict_ok_v otra vez aqui invalidaria
                        -- candidatos validos.
                        if prn_best_accum > dyn_thr_v and
                                snr_gate_ok_v and
                                raw_lock_gate_ok_v and
                                reacq_guard_ok_v
                        then
                            raw_acq_v := '1';
                        else
                            raw_acq_v := '0';
                        end if;

                        doppler_delta_i := 0;
                        phase_delta_i   := 0;
                        lock_track_ok_v := false;
                        snr_upgrade_ok_v := false;
                        update_lock_v   := false;

                        if raw_acq_v = '1' then
                            if cur_locked = '0' then
                                update_lock_v := true;
                            else
                                doppler_delta_i := abs_signed_bin_diff(prn_best_dop, cur_lock_dop);
                                phase_delta_i := phase_circular_distance(prn_best_pos, cur_lock_phase);
                                lock_track_ok_v := (doppler_delta_i <= 1) and (phase_delta_i <= 64);
                                snr_upgrade_ok_v := (best_snr16_v >=
                                                     (unsigned(cur_lock_snr) + to_unsigned(4, 16)));
                                update_lock_v := lock_track_ok_v or
                                                 (snr_upgrade_ok_v and
                                                  (second_dom_ok_v or margin_boost_ok_v or margin_base_ok_v));
                            end if;
                        end if;

                        end_raw_acq_r <= raw_acq_v;
                        if update_lock_v then
                            end_update_lock_r <= '1';
                        else
                            end_update_lock_r <= '0';
                        end if;
                        if second_dom_ok_v then
                            end_second_dom_ok_r <= '1';
                        else
                            end_second_dom_ok_r <= '0';
                        end if;
                        if margin_boost_ok_v then
                            end_margin_boost_ok_r <= '1';
                        else
                            end_margin_boost_ok_r <= '0';
                        end if;
                        if margin_base_ok_v then
                            end_margin_base_ok_r <= '1';
                        else
                            end_margin_base_ok_r <= '0';
                        end if;
                        if snr_floor_ok_v then
                            end_snr_floor_ok_r <= '1';
                        else
                            end_snr_floor_ok_r <= '0';
                        end if;
                        if prn_best_accum > dyn_thr_v then
                            end_dyn_thr_ok_r <= '1';
                        else
                            end_dyn_thr_ok_r <= '0';
                        end if;
                        if cfar_ok_v then
                            end_cfar_ok_r <= '1';
                        else
                            end_cfar_ok_r <= '0';
                        end if;
                        if cfar_strict_ok_v then
                            end_cfar_strict_ok_r <= '1';
                        else
                            end_cfar_strict_ok_r <= '0';
                        end if;

                        end_best_snr16_r      <= std_logic_vector(best_snr16_v);
                        end_margin16_r        <= std_logic_vector(v_margin16);
                        end_prn_best_dop_r    <= prn_best_dop;
                        end_prn_best_pos_r    <= prn_best_pos;
                        state <= S_END_PRN_EVAL2;

                    when S_END_PRN_EVAL2 =>
                        -- Cycle 2: compute cand_event and candidate_inc from
                        -- registered end_*_r values to shorten the critical path.
                        cand_event_v := (prn_best_accum > (prn_best_noise_accum + shift_right(prn_best_noise_accum, 1))) and
                                        (end_snr_floor_ok_r = '1') and
                                        (end_margin_base_ok_r = '1' or end_second_dom_ok_r = '1');
                        if cand_event_v then
                            end_cand_event_r <= '1';
                        else
                            end_cand_event_r <= '0';
                        end if;
                        if end_cfar_ok_r = '1' and end_raw_acq_r = '0' then
                            end_candidate_inc_r <= '1';
                        else
                            end_candidate_inc_r <= '0';
                        end if;
                        state <= S_END_PRN_COMMIT;

                    when S_END_PRN_COMMIT =>
                        if end_candidate_inc_r = '1' then
                            candidate_cnt_r <= candidate_cnt_r + 1;
                        end if;

                        -- Score continuo por PRN calculado sobre evidencia registrada
                -- para acortar camino critico desde accum/noise.
                        --
                -- Clamp en 1023 para evitar saturacion prematura y mantener
                -- discriminacion entre senales muy fuertes (ej. inyectadas sinteticas).
                        score_i := to_integer(unsigned(end_best_snr16_r));
                        if score_i > 1023 then
                            score_i := 1023;
                        end if;
                        if end_cfar_strict_ok_r = '1' then
                            score_i := score_i + 16;
                        end if;
                        if end_second_dom_ok_r = '1' then
                            score_i := score_i + 72;
                        end if;
                        if end_margin_boost_ok_r = '1' then
                            score_i := score_i + 48;
                        end if;
                        if end_margin_base_ok_r = '1' then
                            score_i := score_i + 24;
                        end if;
                        if end_cfar_ok_r = '1' then
                            score_i := score_i + 96;
                        end if;
                        if score_i > 1023 then
                            score_i := 1023;
                        end if;

                        flags_v := (others => '0');
                        if end_dyn_thr_ok_r = '1'      then flags_v(0) := '1'; end if;
                        if end_cfar_ok_r = '1'        then flags_v(1) := '1'; end if;
                        if end_cfar_strict_ok_r = '1' then flags_v(2) := '1'; end if;
                        if end_margin_base_ok_r = '1' then flags_v(3) := '1'; end if;
                        if end_second_dom_ok_r = '1'  then flags_v(4) := '1'; end if;
                        if end_snr_floor_ok_r = '1'   then flags_v(5) := '1'; end if;
                        if end_raw_acq_r = '1'        then flags_v(6) := '1'; end if;
                        if end_cand_event_r = '1'     then flags_v(7) := '1'; end if;

                -- Snapshot de metricas por PRN para arbitraje global posterior.
                        prn_eval_dop(prn_idx)    <= end_prn_best_dop_r;
                        prn_eval_phase(prn_idx)  <= end_prn_best_pos_r;
                        prn_eval_snr(prn_idx)    <= end_best_snr16_r;
                        prn_eval_score(prn_idx)  <= std_logic_vector(to_unsigned(score_i, 16));
                        prn_eval_margin(prn_idx) <= end_margin16_r;
                        prn_eval_flags(prn_idx)  <= flags_v;
                        prn_eval_raw(prn_idx)    <= end_raw_acq_r;
                        prn_eval_cand(prn_idx)   <= end_cand_event_r;

                -- Insertar candidato en TOP-N global del barrido.
                        if end_cand_event_r = '1' and
                           to_unsigned(score_i, 16) >= to_unsigned(SCORE_CAND_MIN, 16) then
                            ins_rank_i := TOP_N_MAX;
                            for shift_i in 0 to TOP_N_MAX - 1 loop
                                if shift_i < GLOBAL_TOP_N then
                                    if top_valid_r(shift_i) = '0' then
                                        ins_rank_i := shift_i;
                                        exit;
                                    elsif to_unsigned(score_i, 16) > top_scores_r(shift_i) then
                                        ins_rank_i := shift_i;
                                        exit;
                                    end if;
                                end if;
                            end loop;

                            if ins_rank_i < GLOBAL_TOP_N then
                                for shift_i in TOP_N_MAX - 1 downto 1 loop
                                    if shift_i < GLOBAL_TOP_N and shift_i > ins_rank_i then
                                        top_scores_r(shift_i) <= top_scores_r(shift_i - 1);
                                        top_prns_r(shift_i)   <= top_prns_r(shift_i - 1);
                                        top_valid_r(shift_i)  <= top_valid_r(shift_i - 1);
                                    end if;
                                end loop;
                                top_scores_r(ins_rank_i) <= to_unsigned(score_i, 16);
                                top_prns_r(ins_rank_i)   <= std_logic_vector(to_unsigned(prn_idx, 5));
                                top_valid_r(ins_rank_i)  <= '1';
                            end if;
                        end if;

                        next_prn_i := next_enabled_prn(PRN_ENABLE_MASK, prn_idx + 1, NUM_PRNS);
                        found_prn_v := (next_prn_i < NUM_PRNS);
                        if found_prn_v then
                            prn_idx <= next_prn_i;
                            state   <= S_INIT_PRN;
                        else
                            state   <= S_ARB_PREP;
                        end if;

                    when S_ARB_PREP =>
                        sat_count_r   <= (others => '0');
                        prn_selected  <= (others => '0');
                        prn_rank      <= (others => x"FF");

                        for shift_i in 0 to TOP_N_MAX - 1 loop
                            if shift_i < GLOBAL_TOP_N and top_valid_r(shift_i) = '1' then
                                rank_prn_i := to_integer(unsigned(top_prns_r(shift_i)));
                                prn_selected(rank_prn_i) <= '1';
                                prn_rank(rank_prn_i) <= std_logic_vector(to_unsigned(shift_i, 8));
                            end if;
                        end loop;

                        next_prn_i := next_enabled_prn(PRN_ENABLE_MASK, 0, NUM_PRNS);
                        found_prn_v := (next_prn_i < NUM_PRNS);
                        if found_prn_v then
                            prn_idx <= next_prn_i;
                            state   <= S_ARB_READ_ARRAYS;
                        else
                            state   <= S_DONE;
                        end if;

                    when S_ARB_READ_ARRAYS =>
                        rd_hit_ctr       <= prn_hit_ctr(prn_idx);
                        rd_miss_ctr      <= prn_miss_ctr(prn_idx);
                        rd_locked        <= prn_locked(prn_idx);
                        rd_eval_flags    <= prn_eval_flags(prn_idx);
                        rd_selected      <= prn_selected(prn_idx);
                        rd_eval_raw      <= prn_eval_raw(prn_idx);
                        rd_eval_score    <= prn_eval_score(prn_idx);
                        rd_eval_cand     <= prn_eval_cand(prn_idx);
                        rd_cand_valid    <= prn_cand_valid(prn_idx);
                        rd_eval_dop      <= prn_eval_dop(prn_idx);
                        rd_cand_dop      <= prn_cand_dop(prn_idx);
                        rd_eval_phase    <= prn_eval_phase(prn_idx);
                        rd_cand_phase    <= prn_cand_phase(prn_idx);
                        rd_eval_snr      <= prn_eval_snr(prn_idx);
                        rd_rank          <= prn_rank(prn_idx);
                        rd_lock_dop      <= prn_lock_dop(prn_idx);
                        rd_lock_phase    <= prn_lock_phase(prn_idx);
                        rd_lock_snr      <= prn_lock_snr(prn_idx);
                        rd_raw_ctr       <= prn_raw_ctr(prn_idx);
                        state            <= S_ARB_APPLY_PRN;

                    -- ============================================================
                    -- ARB pipeline: stage 1 (S_ARB_READ_ARRAYS) reads arrays,
                    -- stage 2 (S_ARB_APPLY_PRN) computes comparisons and distances,
                    -- stage 3 (S_ARB_COMMIT_PRN) runs the lock FSM and writes
                    -- outputs. This 3-stage split completely cuts the critical
                    -- path from array MUXing into the arithmetic carry chains.
                    -- ============================================================
                    when S_ARB_APPLY_PRN =>
                        -- Stage 2: compute comparisons and update intermediate registers.
                        arb_v_hit_r   <= rd_hit_ctr;
                        arb_v_miss_r  <= rd_miss_ctr;
                        arb_v_lock_r  <= rd_locked;
                        if rd_locked = '1' then
                            arb_was_locked_r <= '1';
                        else
                            arb_was_locked_r <= '0';
                        end if;

                        arb_flags_r <= prn_eval_flags(prn_idx);

                        -- Evitar starvation de arbitraje cuando TOP-N no incluye
                        -- PRNs con evidencia cruda valida.
                        if prn_selected(prn_idx) = '1' or
                           (prn_eval_raw(prn_idx) = '1' and
                            unsigned(prn_eval_score(prn_idx)) >= to_unsigned(SCORE_CAND_MIN, 16)) then
                            arb_selected_r <= '1';
                        else
                            arb_selected_r <= '0';
                        end if;

                        raw_event_v := (rd_eval_raw = '1');
                        if raw_event_v then arb_raw_event_r <= '1';
                        else arb_raw_event_r <= '0'; end if;

                        if rd_eval_cand = '1' then arb_cand_event_r <= '1';
                        else arb_cand_event_r <= '0'; end if;

                        if unsigned(rd_eval_score) >= to_unsigned(SCORE_PROMOTE_MIN, 16) then
                            arb_cand_score_ok_r <= '1';
                        else arb_cand_score_ok_r <= '0'; end if;
                        if unsigned(rd_eval_score) >= to_unsigned(SCORE_HOLD_MIN, 16) then
                            arb_hold_score_ok_r <= '1';
                        else arb_hold_score_ok_r <= '0'; end if;

                        arb_cand_valid_r <= rd_cand_valid;
                        cand_coherent_v := false;
                        if rd_cand_valid = '1' then
                            cand_dop_delta_i := abs_signed_bin_diff(rd_eval_dop,
                                                                    rd_cand_dop);
                            cand_phase_delta_i := phase_circular_distance(rd_eval_phase,
                                                                          rd_cand_phase);
                            cand_coherent_v := (cand_dop_delta_i <= CAND_DOP_MAX_DELTA) and
                                               ((cand_phase_delta_i <= CAND_PHASE_MAX_DELTA) or
                                                raw_event_v);
                        end if;
                        if cand_coherent_v then arb_cand_coherent_r <= '1';
                        else arb_cand_coherent_r <= '0'; end if;

                        -- Apply same tuning as S_END_PRN_EVAL so cand_snr_ok is consistent
                        -- with the raw acquisition threshold.
                        if unsigned(rd_eval_snr) >= to_unsigned(LOCK_SNR_MIN + CAND_SNR_MARGIN, 16) then
                            arb_cand_snr_ok_r <= '1';
                        else arb_cand_snr_ok_r <= '0'; end if;

                        arb_eval_dop_r   <= rd_eval_dop;
                        arb_eval_phase_r <= rd_eval_phase;
                        arb_eval_snr_r   <= rd_eval_snr;
                        arb_eval_score_r <= rd_eval_score;
                        arb_rank_r       <= rd_rank;
                        arb_lock_dop_r   <= rd_lock_dop;
                        arb_lock_phase_r <= rd_lock_phase;
                        arb_lock_snr_r   <= rd_lock_snr;

                        state <= S_ARB_COMMIT_PRN;

                    when S_ARB_COMMIT_PRN =>
                        -- Stage 3: lock FSM update and output generation from registered values.
                        v_hit  := arb_v_hit_r;
                        v_miss := arb_v_miss_r;
                        raw_ctr_v := rd_raw_ctr;
                        v_lock := arb_v_lock_r;
                        was_locked_v   := (arb_was_locked_r = '1');
                        selected_v     := (arb_selected_r = '1');
                        raw_event_v    := (arb_raw_event_r = '1');
                        cand_event_v   := (arb_cand_event_r = '1');
                        cand_score_ok_v := (arb_cand_score_ok_r = '1');
                        hold_score_ok_v := (arb_hold_score_ok_r = '1');
                        cand_coherent_v := (arb_cand_coherent_r = '1');
                        cand_snr_ok_v  := (arb_cand_snr_ok_r = '1');
                        flags_v        := arb_flags_r;

                        -- Memoria temporal de evidencia cruda por PRN.
                        -- Evita perder sensibilidad cuando TOP-N oscila entre barridos.
                        if raw_event_v then
                            if raw_ctr_v < 15 then
                                raw_ctr_v := raw_ctr_v + 1;
                            end if;
                        elsif raw_ctr_v > 0 then
                            raw_ctr_v := raw_ctr_v - 1;
                        end if;
                        prn_raw_ctr(prn_idx) <= raw_ctr_v;

                        promote_window_v := selected_v or (raw_ctr_v >= 2);

                        if v_lock = '0' then
                            v_miss := 0;
                            -- Histeresis estructural de promocion.
                            -- La evidencia temporal requiere raw_event consecutivo
                            -- y coherencia Doppler/fase.
                            if promote_window_v and raw_event_v then
                                if arb_cand_valid_r = '1' then
                                    if cand_coherent_v and cand_snr_ok_v then
                                        if v_hit < HYST_ACQ_SWEEPS then
                                            v_hit := v_hit + 1;
                                        end if;
                                    else
                                        -- No destruir historial por un barrido ambiguo:
                                        -- mantener ventana de histeresis sin reset completo.
                                        if v_hit > 0 then
                                            v_hit := v_hit - 1;
                                        else
                                            v_hit := 0;
                                        end if;
                                    end if;
                                else
                                    v_hit := 1;
                                end if;

                                prn_cand_valid(prn_idx) <= '1';
                                prn_cand_dop(prn_idx)   <= arb_eval_dop_r;
                                prn_cand_phase(prn_idx) <= arb_eval_phase_r;

                                if v_hit >= HYST_ACQ_SWEEPS and cand_score_ok_v then
                                    v_lock := '1';
                                    v_hit  := 0;
                                    prn_cand_valid(prn_idx) <= '0';
                                end if;
                            else
                                if v_hit > 0 then
                                    v_hit := v_hit - 1;
                                else
                                    v_hit := 0;
                                end if;
                                if not raw_event_v then
                                    prn_cand_valid(prn_idx) <= '0';
                                end if;
                            end if;
                        else
                            prn_cand_valid(prn_idx) <= '0';
                            if raw_event_v and selected_v then
                                v_miss := 0;
                            else
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
                        cur_locked            <= v_lock;

                        if v_lock = '1' and raw_event_v and selected_v then
                            prn_lock_phase(prn_idx) <= arb_eval_phase_r;
                            prn_lock_dop(prn_idx)   <= arb_eval_dop_r;
                            prn_lock_snr(prn_idx)   <= arb_eval_snr_r;
                            cur_lock_phase          <= arb_eval_phase_r;
                            cur_lock_dop            <= arb_eval_dop_r;
                            cur_lock_snr            <= arb_eval_snr_r;
                        end if;

                        reason_v := (others => '0');
                        if selected_v      then reason_v(0) := '1'; end if;
                        if cand_event_v    then reason_v(1) := '1'; end if;
                        if raw_event_v     then reason_v(2) := '1'; end if;
                        if cand_coherent_v then reason_v(3) := '1'; end if;
                        if cand_score_ok_v then reason_v(4) := '1'; end if;
                        if hold_score_ok_v then reason_v(5) := '1'; end if;
                        if was_locked_v    then reason_v(6) := '1'; end if;
                        if v_lock = '1'    then reason_v(7) := '1'; end if;
                        prn_reason(prn_idx) <= reason_v;

                        if selected_v then
                            flags_v(6) := '1';
                        else
                            flags_v(6) := '0';
                        end if;
                        if v_lock = '1' then
                            flags_v(7) := '1';
                        else
                            flags_v(7) := '0';
                        end if;

                        ram_wr_addr_r <= std_logic_vector(to_unsigned(prn_idx, 5));
                        ram_wr_acq    <= v_lock;
                        if v_lock = '1' then
                            if raw_event_v then
                                ram_wr_doppler_r <= arb_eval_dop_r;
                                ram_wr_phase_r   <= map_peak_to_phase(arb_eval_phase_r, PHASE_BIAS);
                                ram_wr_snr_r     <= arb_eval_snr_r;
                            else
                                ram_wr_doppler_r <= arb_lock_dop_r;
                                ram_wr_phase_r   <= map_peak_to_phase(arb_lock_phase_r, PHASE_BIAS);
                                ram_wr_snr_r     <= arb_lock_snr_r;
                            end if;
                        else
                            ram_wr_doppler_r <= arb_eval_dop_r;
                            ram_wr_phase_r   <= map_peak_to_phase(arb_eval_phase_r, PHASE_BIAS);
                            ram_wr_snr_r     <= arb_eval_snr_r;
                        end if;
                        ram_wr_noise_r  <= arb_eval_score_r;
                        ram_wr_margin_r <= reason_v & arb_rank_r;
                        ram_wr_flags_r  <= flags_v;

                        if v_lock = '1' then
                            sat_count_r <= sat_count_r + 1;
                        end if;

                        state <= S_ARB_WRITE_RAM;

                    when S_ARB_WRITE_RAM =>
                        ram_we <= '1';
                        next_prn_i := next_enabled_prn(PRN_ENABLE_MASK, prn_idx + 1, NUM_PRNS);
                        found_prn_v := (next_prn_i < NUM_PRNS);

                        if found_prn_v then
                            prn_idx <= next_prn_i;
                            state   <= S_ARB_READ_ARRAYS;
                        else
                            state   <= S_DONE;
                        end if;

                    when S_DONE =>
                        acq_done      <= '1';
                        sweep_running_r <= '0';
                        sweep_time_last_r    <= sweep_time_r;
                        candidate_cnt_last_r <= candidate_cnt_r;
                        if sat_count_r > 0 then
                            acq_valid <= '1';
                        else
                            acq_valid <= '0';
                        end if;
                        state <= S_WAIT;

                    when S_WAIT =>
                        acq_done <= '0';
                        if ca_epoch = '1' and epoch_prev = '0' then
                            eff_bin_range_r <= BIN_RANGE; -- barridos 2+: rango normal
                            next_prn_i := next_enabled_prn(PRN_ENABLE_MASK, 0, NUM_PRNS);
                            found_prn_v := (next_prn_i < NUM_PRNS);
                            if found_prn_v then
                                prn_idx <= next_prn_i;
                            else
                                prn_idx <= 0;
                            end if;
                            sat_count_r     <= (others => '0');
                            acq_valid       <= '0';
                            top_scores_r    <= (others => (others => '0'));
                            top_prns_r      <= (others => (others => '0'));
                            top_valid_r     <= (others => '0');
                            prn_selected    <= (others => '0');
                            prn_rank        <= (others => x"FF");
                            sweep_running_r <= '1';
                            sweep_restart   <= '1';
                            sweep_time_r    <= (others => '0');
                            candidate_cnt_r <= (others => '0');
                            if found_prn_v then
                                state <= S_INIT_PRN;
                            else
                                state <= S_DONE;
                            end if;
                        end if;

                    when others => state <= S_IDLE;

                end case;
            end if;
        end if;
    end process;

end Behavioral;