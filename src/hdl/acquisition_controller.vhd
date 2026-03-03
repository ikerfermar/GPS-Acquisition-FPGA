library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- acquisition_controller.vhd
--
-- Barre NUM_PRNS codigos CA x (2*BIN_RANGE+1) bins Doppler.
-- Por cada (PRN, bin) acumula N_INT epochs de integracion no
-- coherente (suma de peak^2) y declara adquisicion si:
--   prn_best_accum > ACCUM_THRESHOLD  Y
--   prn_best_accum > K_CFAR * prn_best_noise_accum
--
-- NOTA: K_CFAR_SHIFT = log2(K_CFAR). Con K_CFAR=4, shift=2.
-- Si se cambia K_CFAR a un valor no potencia de 2, actualizar
-- K_CFAR_SHIFT o reemplazar shift_left por multiplicacion.
-- =============================================================

entity acquisition_controller is
    Generic (
        PEAK_THRESHOLD   : integer := 200;
        K_CFAR           : integer := 4;
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

    constant ACCUM_THRESHOLD : unsigned(39 downto 0) :=
        to_unsigned(PEAK_THRESHOLD * PEAK_THRESHOLD * N_INT, 40);

    constant EARLY_EXIT_THR : unsigned(39 downto 0) :=
        to_unsigned(PEAK_THRESHOLD * PEAK_THRESHOLD * N_INT * 4, 40);

    -- FIX: K_CFAR_SHIFT derivado automaticamente de K_CFAR (log2).
    --      Valores soportados: K_CFAR=2->shift=1, 4->2, 8->3, 16->4.
    --      Si K_CFAR no es potencia de 2, usar multiplicacion directa en lugar
    --      de shift_left para evitar errores silenciosos al cambiar el generic.
    function log2_cfar(k : integer) return integer is
    begin
        if    k <= 2  then return 1;
        elsif k <= 4  then return 2;
        elsif k <= 8  then return 3;
        else               return 4;
        end if;
    end function;
    constant K_CFAR_SHIFT : integer := log2_cfar(K_CFAR);

    type t_state is (
        S_IDLE, S_INIT_PRN, S_INIT_BIN,
        S_WAIT_EPOCH, S_WAIT_PEAK, S_ACCUMULATE,
        S_COMPARE, S_END_PRN,
        S_CONFIRM_EPOCH, S_CONFIRM_WAIT_EPOCH,
        S_CONFIRM_PEAK, S_LATCH_RESULT, S_WRITE_RAM,
        S_DONE, S_WAIT
    );
    signal state : t_state := S_IDLE;

    signal prn_idx : integer range 0 to 31 := 0;
    signal bin_cnt : signed(7 downto 0)    := (others => '0');
    signal int_cnt : integer range 0 to 255 := 0;

    signal bin_accum             : unsigned(39 downto 0) := (others => '0');
    signal noise_accum           : unsigned(39 downto 0) := (others => '0');
    signal prn_best_accum        : unsigned(39 downto 0) := (others => '0');
    signal prn_best_noise_accum  : unsigned(39 downto 0) := (others => '0');
    signal prn_best_dop          : std_logic_vector(7 downto 0) := (others => '0');
    signal prn_best_pos          : std_logic_vector(9 downto 0) := (others => '0');
    signal prn_acquired          : std_logic := '0';

    signal glob_best_accum : unsigned(39 downto 0)       := (others => '0');
    signal glob_best_dop   : std_logic_vector(7 downto 0) := (others => '0');
    signal glob_best_pos   : std_logic_vector(9 downto 0) := (others => '0');
    signal glob_best_prn   : std_logic_vector(4 downto 0) := (others => '0');

    signal sat_count_r  : unsigned(5 downto 0)       := (others => '0');
    signal latch_val    : unsigned(15 downto 0)       := (others => '0');
    signal latch_pos    : std_logic_vector(9 downto 0) := (others => '0');
    signal latch_noise  : unsigned(15 downto 0)       := (others => '0');
    signal bin_best_val : unsigned(15 downto 0)       := (others => '0');
    signal bin_best_pos : std_logic_vector(9 downto 0) := (others => '0');
    signal confirm_pos  : std_logic_vector(9 downto 0) := (others => '0');
    signal epoch_prev   : std_logic := '0';

    signal current_prn_slv : std_logic_vector(4 downto 0);
    signal ram_wr_addr_r   : std_logic_vector(4 downto 0)  := (others => '0');
    signal ram_wr_snr_r    : std_logic_vector(15 downto 0) := (others => '0');

    signal val_sq   : unsigned(31 downto 0) := (others => '0');
    signal noise_sq : unsigned(31 downto 0) := (others => '0');

begin

    current_prn_slv <= std_logic_vector(to_unsigned(PRN_LIST(prn_idx), 5));

    doppler_sel    <= std_logic_vector(bin_cnt);
    prn_out        <= current_prn_slv;
    sweep_bin      <= std_logic_vector(bin_cnt);
    sweep_prn      <= current_prn_slv;
    best_doppler   <= glob_best_dop;
    best_pos       <= glob_best_pos;
    best_val       <= std_logic_vector(glob_best_accum(39 downto 24));
    best_prn       <= glob_best_prn;
    sat_count      <= std_logic_vector(sat_count_r);
    ram_wr_addr    <= ram_wr_addr_r;
    ram_wr_doppler <= prn_best_dop;
    ram_wr_phase   <= prn_best_pos;
    ram_wr_snr     <= ram_wr_snr_r;

    process(clk)
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
                val_sq               <= (others => '0');
                noise_sq             <= (others => '0');
                confirm_pos          <= (others => '0');
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
                        state        <= S_WAIT_EPOCH;

                    when S_WAIT_EPOCH =>
                        if ca_epoch = '1' and epoch_prev = '0' then
                            state <= S_WAIT_PEAK;
                        end if;

                    when S_WAIT_PEAK =>
                        if peak_ready = '1' then
                            latch_val   <= unsigned(peak_val);
                            latch_pos   <= peak_pos;
                            latch_noise <= unsigned(noise_floor);
                            val_sq      <= unsigned(peak_val)    * unsigned(peak_val);
                            noise_sq    <= unsigned(noise_floor) * unsigned(noise_floor);
                            state       <= S_ACCUMULATE;
                        end if;

                    when S_ACCUMULATE =>
                        bin_accum   <= bin_accum   + resize(val_sq,   40);
                        noise_accum <= noise_accum + resize(noise_sq, 40);

                        if latch_val > bin_best_val then
                            bin_best_val <= latch_val;
                            bin_best_pos <= latch_pos;
                        end if;

                        if bin_accum + resize(val_sq, 40) > EARLY_EXIT_THR then
                            state <= S_COMPARE;
                        elsif int_cnt = N_INT - 1 then
                            state <= S_COMPARE;
                        else
                            int_cnt <= int_cnt + 1;
                            state   <= S_WAIT_EPOCH;
                        end if;

                    when S_COMPARE =>
                        if bin_accum > prn_best_accum then
                            prn_best_accum       <= bin_accum;
                            prn_best_noise_accum <= noise_accum;
                            prn_best_dop         <= std_logic_vector(bin_cnt);
                            prn_best_pos         <= bin_best_pos;
                        end if;
                        if bin_cnt = to_signed(BIN_RANGE, 8) then
                            state <= S_END_PRN;
                        else
                            bin_cnt <= bin_cnt + 1;
                            state   <= S_INIT_BIN;
                        end if;

                    when S_END_PRN =>
                        -- CFAR: peak > K_CFAR * noise (shift_left = multiply por potencia de 2)
                        if prn_best_accum > ACCUM_THRESHOLD and
                           prn_best_accum > shift_left(prn_best_noise_accum, K_CFAR_SHIFT)
                        then
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
                        state <= S_CONFIRM_EPOCH;

                    when S_CONFIRM_EPOCH =>
                        bin_cnt <= signed(prn_best_dop);
                        state   <= S_CONFIRM_WAIT_EPOCH;

                    when S_CONFIRM_WAIT_EPOCH =>
                        if ca_epoch = '1' and epoch_prev = '0' then
                            state <= S_CONFIRM_PEAK;
                        end if;

                    when S_CONFIRM_PEAK =>
                        if peak_ready = '1' then
                            confirm_pos <= peak_pos;
                            state       <= S_LATCH_RESULT;
                        end if;

                    when S_LATCH_RESULT =>
                        ram_wr_acq    <= prn_acquired;
                        ram_wr_addr_r <= current_prn_slv;
                        prn_best_pos  <= confirm_pos;
                        ram_wr_snr_r  <= std_logic_vector(prn_best_accum(23 downto 8));
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
                        if start = '1' or CONTINUOUS_SWEEP then
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