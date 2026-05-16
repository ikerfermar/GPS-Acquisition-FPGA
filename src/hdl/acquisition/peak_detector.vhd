library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- peak_detector.vhd  (rev: BRAM-infer fix v2)
--
-- Fix Synth 8-4767 / 8-2914: inferencia BRAM Simple Dual-Port.
--
-- Patron Xilinx UG901 "Simple Dual-Port RAM, Single Clock":
--   - Un unico proceso sincrono hace TODAS las escrituras.
--   - La direccion y dato de escritura se multiplexan en logica
--     combinacional ANTES del proceso (clear vs acumulacion).
--   - Un proceso de lectura separado con latencia 1 ciclo.
--   Vivado infiere esto como RAMB36 sin ambiguedad.
--
-- El problema original (Synth 8-4767) era que fba_ram se escribia
-- desde multiples ramas de un mismo proceso clocked. Vivado no
-- puede mapear eso a BRAM y caia a 20480 FFs.
-- =============================================================

entity peak_detector is
    Port (
        clk           : in  STD_LOGIC;
        reset_n       : in  STD_LOGIC;
        din_valid     : in  STD_LOGIC;
        din_ready     : out STD_LOGIC;
        din_data      : in  STD_LOGIC_VECTOR(31 downto 0);
        fba_clear     : in  STD_LOGIC;
        fba_last_block: in  STD_LOGIC;
        peak_pos      : out STD_LOGIC_VECTOR(9 downto 0);
        peak_val      : out STD_LOGIC_VECTOR(15 downto 0);
        noise_floor   : out STD_LOGIC_VECTOR(15 downto 0);
        new_peak_ready: out STD_LOGIC
    );
end peak_detector;

architecture Behavioral of peak_detector is
    constant FRAME_LAST : unsigned(9 downto 0) := to_unsigned(1023, 10);
    constant ABS_SAT_16 : unsigned(15 downto 0) := to_unsigned(32767, 16);
    signal re, im : signed(15 downto 0);

    -- ----------------------------------------------------------------
    -- FBA RAM: patron Simple Dual-Port UG901
    --   - wr_en / wr_addr / wr_data : muxeados en combinacional
    --   - rd_addr                   : leido con latencia 1 ciclo
    -- Un solo proceso de escritura -> Vivado infiere RAMB36.
    -- ----------------------------------------------------------------
    type fba_ram_t is array (0 to 1023) of unsigned(19 downto 0);
    signal fba_ram    : fba_ram_t := (others => (others => '0'));
    attribute ram_style : string;
    attribute ram_style of fba_ram : signal is "block";

    -- Write port (combinacional mux -> un proceso sincrono)
    signal wr_en   : std_logic := '0';
    signal wr_addr : unsigned(9 downto 0) := (others => '0');
    signal wr_data : unsigned(19 downto 0) := (others => '0');

    -- Read port
    signal rd_addr : unsigned(9 downto 0) := (others => '0');
    signal rd_data : unsigned(19 downto 0) := (others => '0');

    -- Clear controller
    signal clear_active : std_logic := '0';
    signal clear_cnt    : unsigned(9 downto 0) := (others => '0');

    -- Accumulation pipeline
    signal sample_cnt     : unsigned(9 downto 0)  := (others => '0');
    signal acc_pipe_valid : std_logic := '0';
    signal acc_pipe_mag   : unsigned(15 downto 0) := (others => '0');
    signal acc_pipe_idx   : unsigned(9 downto 0) := (others => '0');
    signal acc_pipe_last  : std_logic := '0';
    signal acc_pipe_final : std_logic := '0';

    -- Acumulacion write-back signals (calculados en FSM, aplicados via mux)
    signal acc_we   : std_logic := '0';
    signal acc_addr : unsigned(9 downto 0) := (others => '0');
    signal acc_data : unsigned(19 downto 0) := (others => '0');
    -- Read address driven by FSM
    signal fsm_rd_addr : unsigned(9 downto 0) := (others => '0');

    -- FSM
    type state_t is (S_IDLE, S_ACCUMULATE, S_INTER_FLUSH, S_INTER_DONE,
                     S_SCAN_WAIT, S_SCAN, S_DONE);
    signal state : state_t := S_IDLE;

    -- Scan
    signal scan_cnt       : unsigned(10 downto 0) := (others => '0');
    signal scan_max       : unsigned(19 downto 0) := (others => '0');
    signal scan_max_pos   : unsigned(9 downto 0) := (others => '0');
    signal scan_sum       : unsigned(29 downto 0) := (others => '0');
    signal scan_pipe_valid: std_logic := '0';
    signal scan_pipe_cnt  : unsigned(9 downto 0) := (others => '0');

    -- Output registers
    signal max_pos_reg     : std_logic_vector(9 downto 0)  := (others => '0');
    signal max_val_reg     : std_logic_vector(15 downto 0) := (others => '0');
    signal noise_floor_reg : std_logic_vector(15 downto 0) := (others => '0');

begin
    din_ready <= '1';
    re <= signed(din_data(15 downto 0));
    im <= signed(din_data(31 downto 16));

    -- ----------------------------------------------------------------
    -- Mux combinacional de escritura:
    --   clear_active=1 -> escribe ceros desde clear_cnt
    --   clear_active=0 -> escribe desde la FSM de acumulacion
    -- Un unico proceso sincrono aplica la escritura -> RAMB36.
    -- ----------------------------------------------------------------
    wr_en   <= '1' when (clear_active = '1') else acc_we;
    wr_addr <= clear_cnt when (clear_active = '1') else acc_addr;
    wr_data <= (others => '0') when (clear_active = '1') else acc_data;

    -- Mux de direccion de lectura: scan usa rd_addr directamente,
    -- acumulacion usa fsm_rd_addr
    rd_addr <= fsm_rd_addr;

    -- ----------------------------------------------------------------
    -- Proceso unico de escritura BRAM (patron UG901 Simple Dual-Port)
    -- ----------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            if wr_en = '1' then
                fba_ram(to_integer(wr_addr)) <= wr_data;
            end if;
        end if;
    end process;

    -- ----------------------------------------------------------------
    -- Proceso de lectura BRAM (latencia 1 ciclo)
    -- ----------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            rd_data <= fba_ram(to_integer(rd_addr));
        end if;
    end process;

    -- ----------------------------------------------------------------
    -- Controlador de clear
    -- ----------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                clear_active <= '0';
                clear_cnt    <= (others => '0');
            else
                if fba_clear = '1' and clear_active = '0' then
                    clear_active <= '1';
                    clear_cnt    <= (others => '0');
                end if;
                if clear_active = '1' then
                    if clear_cnt = FRAME_LAST then
                        clear_active <= '0';
                    else
                        clear_cnt <= clear_cnt + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- ----------------------------------------------------------------
    -- FSM principal: acumulacion + scan
    -- ----------------------------------------------------------------
    process(clk)
        variable v_re_abs  : unsigned(15 downto 0);
        variable v_im_abs  : unsigned(15 downto 0);
        variable v_mag     : unsigned(15 downto 0);
        variable v_new_acc : unsigned(19 downto 0);
        variable noise_sum : unsigned(29 downto 0);
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                state           <= S_IDLE;
                sample_cnt      <= (others => '0');
                acc_pipe_valid  <= '0';
                scan_pipe_valid <= '0';
                new_peak_ready  <= '0';
                max_val_reg     <= (others => '0');
                max_pos_reg     <= (others => '0');
                noise_floor_reg <= (others => '0');
                scan_max        <= (others => '0');
                scan_max_pos    <= (others => '0');
                scan_sum        <= (others => '0');
                acc_we          <= '0';
                acc_addr        <= (others => '0');
                acc_data        <= (others => '0');
                fsm_rd_addr     <= (others => '0');
            else
                new_peak_ready  <= '0';
                acc_pipe_valid  <= '0';
                scan_pipe_valid <= '0';
                acc_we          <= '0';

                case state is
                when S_IDLE =>
                    sample_cnt <= (others => '0');
                    if din_valid = '1' and clear_active = '0' then
                        -- Lanzar lectura BRAM para addr 0 (resultado en sig ciclo)
                        fsm_rd_addr <= (others => '0');
                        -- Calcular magnitud primera muestra
                        if re = "1000000000000000" then v_re_abs := ABS_SAT_16;
                        else v_re_abs := unsigned(abs(re)); end if;
                        if im = "1000000000000000" then v_im_abs := ABS_SAT_16;
                        else v_im_abs := unsigned(abs(im)); end if;
                        v_mag := v_re_abs + v_im_abs;
                        acc_pipe_valid <= '1';
                        acc_pipe_mag   <= v_mag;
                        acc_pipe_idx   <= (others => '0');
                        acc_pipe_last  <= '0';
                        acc_pipe_final <= fba_last_block;
                        sample_cnt     <= to_unsigned(1, 10);
                        state          <= S_ACCUMULATE;
                    end if;

                when S_ACCUMULATE =>
                    -- Writeback: rd_data tiene el valor antiguo para acc_pipe_idx
                    if acc_pipe_valid = '1' then
                        v_new_acc := rd_data + resize(acc_pipe_mag, 20);
                        acc_we   <= '1';
                        acc_addr <= acc_pipe_idx;
                        acc_data <= v_new_acc;
                    end if;

                    if din_valid = '1' then
                        if re = "1000000000000000" then v_re_abs := ABS_SAT_16;
                        else v_re_abs := unsigned(abs(re)); end if;
                        if im = "1000000000000000" then v_im_abs := ABS_SAT_16;
                        else v_im_abs := unsigned(abs(im)); end if;
                        v_mag := v_re_abs + v_im_abs;

                        fsm_rd_addr    <= sample_cnt;
                        acc_pipe_valid <= '1';
                        acc_pipe_mag   <= v_mag;
                        acc_pipe_idx   <= sample_cnt;
                        acc_pipe_final <= fba_last_block;

                        if sample_cnt = FRAME_LAST then
                            acc_pipe_last <= '1';
                            sample_cnt    <= (others => '0');
                            if acc_pipe_final = '1' then
                                state <= S_SCAN_WAIT;
                            else
                                state <= S_INTER_FLUSH;
                            end if;
                        else
                            acc_pipe_last <= '0';
                            sample_cnt    <= sample_cnt + 1;
                        end if;
                    else
                        acc_pipe_valid <= '0';
                    end if;

                when S_INTER_FLUSH =>
                    if acc_pipe_valid = '1' then
                        v_new_acc := rd_data + resize(acc_pipe_mag, 20);
                        acc_we   <= '1';
                        acc_addr <= acc_pipe_idx;
                        acc_data <= v_new_acc;
                        acc_pipe_valid <= '0';
                    else
                        state <= S_INTER_DONE;
                    end if;

                when S_INTER_DONE =>
                    max_val_reg     <= (others => '0');
                    max_pos_reg     <= (others => '0');
                    noise_floor_reg <= (others => '0');
                    new_peak_ready  <= '1';
                    state <= S_IDLE;

                when S_SCAN_WAIT =>
                    if acc_pipe_valid = '1' then
                        v_new_acc := rd_data + resize(acc_pipe_mag, 20);
                        acc_we   <= '1';
                        acc_addr <= acc_pipe_idx;
                        acc_data <= v_new_acc;
                        acc_pipe_valid <= '0';
                    else
                        scan_cnt        <= (others => '0');
                        scan_max        <= (others => '0');
                        scan_max_pos    <= (others => '0');
                        scan_sum        <= (others => '0');
                        fsm_rd_addr     <= (others => '0');
                        scan_pipe_valid <= '0';
                        state <= S_SCAN;
                    end if;

                when S_SCAN =>
                    if scan_pipe_valid = '1' then
                        if rd_data > scan_max then
                            scan_max     <= rd_data;
                            scan_max_pos <= scan_pipe_cnt;
                        end if;
                        scan_sum <= scan_sum + resize(rd_data, 30);
                    end if;

                    if scan_cnt <= FRAME_LAST then
                        fsm_rd_addr     <= scan_cnt(9 downto 0);
                        scan_pipe_valid <= '1';
                        scan_pipe_cnt   <= scan_cnt(9 downto 0);
                        scan_cnt <= scan_cnt + 1;
                    else
                        if scan_pipe_valid = '1' then
                            scan_pipe_valid <= '0';
                        else
                            state <= S_DONE;
                        end if;
                    end if;

                when S_DONE =>
                    if scan_max > to_unsigned(65535, 20) then
                        max_val_reg <= x"FFFF";
                    else
                        max_val_reg <= std_logic_vector(scan_max(15 downto 0));
                    end if;
                    max_pos_reg <= std_logic_vector(scan_max_pos);
                    
                    -- Protección del cálculo de ruido (restar pico y faldas adyacentes)
                    -- Aproximamos la energía del pico y sus faldas como 2 * scan_max
                    if scan_sum > shift_left(resize(scan_max, 30), 1) then
                        noise_sum := scan_sum - shift_left(resize(scan_max, 30), 1);
                    else
                        noise_sum := (others => '0');
                    end if;
                    
                    if noise_sum(29 downto 26) /= "0000" then
                        noise_floor_reg <= x"FFFF";
                    else
                        noise_floor_reg <= std_logic_vector(noise_sum(25 downto 10));
                    end if;
                    new_peak_ready <= '1';
                    state <= S_IDLE;

                when others =>
                    state <= S_IDLE;
                end case;
            end if;
        end if;
    end process;

    peak_pos    <= max_pos_reg;
    peak_val    <= max_val_reg;
    noise_floor <= noise_floor_reg;

end Behavioral;
