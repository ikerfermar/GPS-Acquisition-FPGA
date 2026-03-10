library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- uart_reporter.vhd  (v12 - salida HEX, sin division, mejor timing)
--
-- Formato de salida (todo hexadecimal, sin divisiones):
--
-- Header: "\r\n=== GPS ACQUISITION RESULTS ===\r\n"
--
-- SAT adquirido (43 bytes):
--   "SAT XX: ACQ dop=SXX ph=XXX snr=XXXX\r\n"
--   XX  = PRN (hex 2 digitos, 1-based -> prn_addr+1)
--   S   = '+' o '-'
--   XX  = valor absoluto del bin Doppler (hex 2 digitos)
--   XXX = fase (hex 3 digitos, 0-3FF)
--   XXXX= SNR  (hex 4 digitos)
--
-- SAT no adquirido (16 bytes):
--   "SAT XX: NOLOCK\r\n"
--
-- Footer total:
--   "TOTAL: XX sats\r\n"
--   "================================\r\n"
--
-- Ventaja: nibble->hex no requiere ninguna division/mod.
-- Elimina los 14 estados PREP_* de v11 -> FSM mucho mas pequenia.
-- =============================================================

entity uart_reporter is
    Generic (
        CLK_FREQ  : integer := 100_000_000;
        BAUD_RATE : integer := 115_200;
        NUM_PRNS  : integer := 32  -- FIX: coherente con CFG_NUM_PRNS del paquete
    );
    Port (
        clk          : in  STD_LOGIC;
        reset_n      : in  STD_LOGIC;
        force_reset  : in  STD_LOGIC;   -- '1' cuando SW14=0 (NOT sw_auto_mode)
        wr_en        : in  STD_LOGIC;
        wr_addr      : in  STD_LOGIC_VECTOR(4 downto 0);
        wr_acq       : in  STD_LOGIC;
        wr_doppler   : in  STD_LOGIC_VECTOR(7 downto 0);
        wr_phase     : in  STD_LOGIC_VECTOR(9 downto 0);
        wr_snr       : in  STD_LOGIC_VECTOR(15 downto 0);
        wr_noise     : in  STD_LOGIC_VECTOR(15 downto 0);
        wr_margin    : in  STD_LOGIC_VECTOR(15 downto 0);
        wr_flags     : in  STD_LOGIC_VECTOR(7 downto 0);
        sweep_start  : in  STD_LOGIC;
        report_start : in  STD_LOGIC;
        uart_tx_pin  : out STD_LOGIC;
        reporting    : out STD_LOGIC
    );
end uart_reporter;

architecture Behavioral of uart_reporter is

    component uart_tx
        Generic (CLK_FREQ  : integer := 100_000_000;
                 BAUD_RATE : integer := 115_200);
        Port (clk     : in  std_logic;
              reset_n : in  std_logic;
              tx_data : in  std_logic_vector(7 downto 0);
              tx_start: in  std_logic;
              tx_busy : out std_logic;
              tx_pin  : out std_logic);
    end component;

    type t_state is (
        S_IDLE,
        S_SEND_HDR,
        S_WAIT_WR,
        S_FORMAT_SAT,
        S_SEND_LINE,
        S_FORMAT_TOTAL,
        S_SEND_TOTAL,
        S_SEND_FTR,
        S_DONE
    );
    signal state : t_state := S_IDLE;

    signal tx_data_r  : std_logic_vector(7 downto 0) := (others => '0');
    signal tx_start_r : std_logic := '0';
    signal tx_busy    : std_logic;

    signal pending_prn : std_logic_vector(4 downto 0) := (others => '0');
    signal pending_acq : std_logic := '0';
    signal pending_dop : std_logic_vector(7 downto 0) := (others => '0');
    signal pending_ph  : std_logic_vector(9 downto 0) := (others => '0');
    signal pending_snr : std_logic_vector(15 downto 0) := (others => '0');
    signal pending_nse : std_logic_vector(15 downto 0) := (others => '0');
    signal pending_mrg : std_logic_vector(15 downto 0) := (others => '0');
    signal pending_flg : std_logic_vector(7 downto 0) := (others => '0');

    constant FIFO_DEPTH : integer := 64;
    subtype fifo_idx_t is integer range 0 to FIFO_DEPTH - 1;
    type fifo_prn_t is array (fifo_idx_t) of std_logic_vector(4 downto 0);
    type fifo_acq_t is array (fifo_idx_t) of std_logic;
    type fifo_dop_t is array (fifo_idx_t) of std_logic_vector(7 downto 0);
    type fifo_ph_t  is array (fifo_idx_t) of std_logic_vector(9 downto 0);
    type fifo_m16_t is array (fifo_idx_t) of std_logic_vector(15 downto 0);
    type fifo_f8_t  is array (fifo_idx_t) of std_logic_vector(7 downto 0);

    attribute ram_style : string;

    signal fifo_prn : fifo_prn_t := (others => (others => '0'));
    signal fifo_acq : fifo_acq_t := (others => '0');
    signal fifo_dop : fifo_dop_t := (others => (others => '0'));
    signal fifo_ph  : fifo_ph_t  := (others => (others => '0'));
    signal fifo_snr : fifo_m16_t := (others => (others => '0'));
    signal fifo_nse : fifo_m16_t := (others => (others => '0'));
    signal fifo_mrg : fifo_m16_t := (others => (others => '0'));
    signal fifo_flg : fifo_f8_t  := (others => (others => '0'));
    attribute ram_style of fifo_prn : signal is "distributed";
    attribute ram_style of fifo_acq : signal is "distributed";
    attribute ram_style of fifo_dop : signal is "distributed";
    attribute ram_style of fifo_ph  : signal is "distributed";
    attribute ram_style of fifo_snr : signal is "distributed";
    attribute ram_style of fifo_nse : signal is "distributed";
    attribute ram_style of fifo_mrg : signal is "distributed";
    attribute ram_style of fifo_flg : signal is "distributed";
    signal fifo_wr_ptr : fifo_idx_t := 0;
    signal fifo_rd_ptr : fifo_idx_t := 0;
    signal fifo_count  : integer range 0 to FIFO_DEPTH := 0;

    signal report_pend : std_logic := '0';
    signal sweep_pend  : std_logic := '0';

    signal sat_cnt  : unsigned(5 downto 0) := (others => '0');
    signal hdr_idx  : integer range 0 to 34 := 0;
    signal ftr_idx  : integer range 0 to 33 := 0;

    signal line_idx : integer range 0 to 71 := 0;
    signal line_len : integer range 0 to 71 := 0;

    signal tot_idx  : integer range 0 to 31 := 0;
    signal tot_len  : integer range 0 to 31 := 0;

    -- Line buffer: max 72 bytes.
    type buf72_t is array (0 to 71) of std_logic_vector(7 downto 0);
    signal lbuf : buf72_t := (others => x"20");
    -- Total buffer
    type buf32_t is array (0 to 31) of std_logic_vector(7 downto 0);
    signal tbuf : buf32_t := (others => x"20");

    constant HDR_LEN : integer := 35;
    type rom35_t is array (0 to 34) of std_logic_vector(7 downto 0);
    -- "\r\n=== GPS ACQUISITION RESULTS ===\r\n"
    constant HEADER : rom35_t := (
        x"0D", x"0A",
        x"3D", x"3D", x"3D", x"20",
        x"47", x"50", x"53", x"20",
        x"41", x"43", x"51", x"55",
        x"49", x"53", x"49", x"54",
        x"49", x"4F", x"4E", x"20",
        x"52", x"45", x"53", x"55",
        x"4C", x"54", x"53", x"20",
        x"3D", x"3D", x"3D",
        x"0D", x"0A"
    );

    constant FTR_LEN : integer := 34;
    type rom34_t is array (0 to 33) of std_logic_vector(7 downto 0);
    constant FOOTER : rom34_t := (
        x"3D", x"3D", x"3D", x"3D", x"3D", x"3D", x"3D", x"3D",
        x"3D", x"3D", x"3D", x"3D", x"3D", x"3D", x"3D", x"3D",
        x"3D", x"3D", x"3D", x"3D", x"3D", x"3D", x"3D", x"3D",
        x"3D", x"3D", x"3D", x"3D", x"3D", x"3D", x"3D", x"3D",
        x"0D", x"0A"
    );

    -- nibble (0-15) -> ASCII hex character '0'..'9','A'..'F'
    function n2h(nib : std_logic_vector(3 downto 0)) return std_logic_vector is
        variable v : unsigned(3 downto 0);
    begin
        v := unsigned(nib);
        if v < 10 then
            return std_logic_vector(to_unsigned(48 + to_integer(v), 8));  -- '0'..'9'
        else
            return std_logic_vector(to_unsigned(55 + to_integer(v), 8));  -- 'A'..'F'
        end if;
    end function;

begin

    assert NUM_PRNS <= 32
        report "uart_reporter: NUM_PRNS > 32 no esta soportado"
        severity note;

    u_uart : uart_tx
        generic map (CLK_FREQ => CLK_FREQ, BAUD_RATE => BAUD_RATE)
        port map (clk => clk, reset_n => reset_n,
                  tx_data => tx_data_r, tx_start => tx_start_r,
                  tx_busy => tx_busy,  tx_pin   => uart_tx_pin);

    process(clk)
        variable vi      : integer range 0 to 71;
        variable b       : buf72_t;
        variable tb      : buf32_t;
        variable ti      : integer range 0 to 31;
        variable prn1    : unsigned(5 downto 0);   -- PRN+1 (1-based, 6 bits)
        variable dop_s   : std_logic;              -- sign bit of doppler
        variable dop_abs : std_logic_vector(7 downto 0); -- abs(doppler)
    begin
        if rising_edge(clk) then
            tx_start_r <= '0';

            if reset_n = '0' then
                state       <= S_IDLE;
                reporting   <= '0';
                pending_snr <= (others => '0');
                pending_nse <= (others => '0');
                pending_mrg <= (others => '0');
                pending_flg <= (others => '0');
                fifo_wr_ptr <= 0;
                fifo_rd_ptr <= 0;
                fifo_count  <= 0;
                report_pend <= '0';
                sweep_pend  <= '0';
                sat_cnt     <= (others => '0');
                hdr_idx     <= 0;
                ftr_idx     <= 0;
                line_idx    <= 0;
                line_len    <= 0;
                tot_idx     <= 0;
                tot_len     <= 0;

            else
                -- -------------------------------------------------------
                -- Latch sweep_start: captura el pulso de 1 ciclo en
                -- cualquier estado del reporter.
                -- -------------------------------------------------------
                if sweep_start = '1' then
                    sweep_pend <= '1';
                end if;

                -- -------------------------------------------------------
                -- FIFO de resultados por PRN. Evita perdida de lineas si
                -- llegan varios wr_en mientras UART sigue transmitiendo.
                -- -------------------------------------------------------
                if wr_en = '1' then
                    if fifo_count < FIFO_DEPTH then
                        fifo_prn(fifo_wr_ptr) <= wr_addr;
                        fifo_acq(fifo_wr_ptr) <= wr_acq;
                        fifo_dop(fifo_wr_ptr) <= wr_doppler;
                        fifo_ph (fifo_wr_ptr) <= wr_phase;
                        fifo_snr(fifo_wr_ptr) <= wr_snr;
                        fifo_nse(fifo_wr_ptr) <= wr_noise;
                        fifo_mrg(fifo_wr_ptr) <= wr_margin;
                        fifo_flg(fifo_wr_ptr) <= wr_flags;
                        if fifo_wr_ptr = FIFO_DEPTH - 1 then
                            fifo_wr_ptr <= 0;
                        else
                            fifo_wr_ptr <= fifo_wr_ptr + 1;
                        end if;
                        fifo_count <= fifo_count + 1;
                    end if;
                end if;

                if report_start = '1' then
                    report_pend <= '1';
                end if;

                -- -------------------------------------------------------
                -- FSM
                -- -------------------------------------------------------
                case state is

                    when S_IDLE =>
                        reporting   <= '0';
                        report_pend <= '0';
                        if force_reset = '1' then
                            sweep_pend <= '0';
                            sat_cnt    <= (others => '0');
                            fifo_count <= 0;
                            fifo_wr_ptr <= 0;
                            fifo_rd_ptr <= 0;
                        elsif sweep_pend = '1' or fifo_count > 0 then
                            sweep_pend <= '0';
                            sat_cnt    <= (others => '0');
                            hdr_idx    <= 0;
                            reporting  <= '1';
                            state      <= S_SEND_HDR;
                        end if;

                    when S_SEND_HDR =>
                        if tx_busy = '0' and tx_start_r = '0' then
                            tx_data_r  <= HEADER(hdr_idx);
                            tx_start_r <= '1';
                            if hdr_idx = HDR_LEN - 1 then
                                state <= S_WAIT_WR;
                            else
                                hdr_idx <= hdr_idx + 1;
                            end if;
                        end if;

                    when S_WAIT_WR =>
                        if fifo_count > 0 then
                            pending_prn <= fifo_prn(fifo_rd_ptr);
                            pending_acq <= fifo_acq(fifo_rd_ptr);
                            pending_dop <= fifo_dop(fifo_rd_ptr);
                            pending_ph  <= fifo_ph (fifo_rd_ptr);
                            pending_snr <= fifo_snr(fifo_rd_ptr);
                            pending_nse <= fifo_nse(fifo_rd_ptr);
                            pending_mrg <= fifo_mrg(fifo_rd_ptr);
                            pending_flg <= fifo_flg(fifo_rd_ptr);
                            if fifo_acq(fifo_rd_ptr) = '1' then
                                sat_cnt <= sat_cnt + 1;
                            end if;
                            if fifo_rd_ptr = FIFO_DEPTH - 1 then
                                fifo_rd_ptr <= 0;
                            else
                                fifo_rd_ptr <= fifo_rd_ptr + 1;
                            end if;
                            fifo_count <= fifo_count - 1;
                            state      <= S_FORMAT_SAT;
                        elsif report_pend = '1' then
                            report_pend <= '0';
                            state       <= S_FORMAT_TOTAL;
                        elsif force_reset = '1' then
                            state <= S_FORMAT_TOTAL;
                        end if;

                    when S_FORMAT_SAT =>
                        -- Build line in combinational variable, then latch to lbuf.
                        -- ACQ  line: SAT XX: ACQ dop=SXX ph=XXX snr=XXXX n=XXXX m=XXXX f=XX\r\n
                        -- NOLOCK   : SAT XX: NOLOCK n=XXXX m=XXXX f=XX\r\n
                        b := (others => x"20");
                        prn1    := unsigned('0' & pending_prn) + 1;
                        dop_s   := pending_dop(7);
                        if dop_s = '1' then
                            dop_abs := std_logic_vector(unsigned(not pending_dop) + 1);
                        else
                            dop_abs := pending_dop;
                        end if;
                        vi := 0;
                        b(vi) := x"53"; vi := vi + 1; -- S
                        b(vi) := x"41"; vi := vi + 1; -- A
                        b(vi) := x"54"; vi := vi + 1; -- T
                        b(vi) := x"20"; vi := vi + 1;
                        b(vi) := n2h("00" & std_logic_vector(prn1(5 downto 4))); vi := vi + 1;
                        b(vi) := n2h(std_logic_vector(prn1(3 downto 0))); vi := vi + 1;
                        b(vi) := x"3A"; vi := vi + 1;
                        b(vi) := x"20"; vi := vi + 1;
                        if pending_acq = '1' then
                            b(vi) := x"41"; vi := vi + 1; -- A
                            b(vi) := x"43"; vi := vi + 1; -- C
                            b(vi) := x"51"; vi := vi + 1; -- Q
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"64"; vi := vi + 1; -- d
                            b(vi) := x"6F"; vi := vi + 1; -- o
                            b(vi) := x"70"; vi := vi + 1; -- p
                            b(vi) := x"3D"; vi := vi + 1;
                            if dop_s = '1' then
                                b(vi) := x"2D"; vi := vi + 1;  -- -
                            else
                                b(vi) := x"2B"; vi := vi + 1;  -- +
                            end if;
                            b(vi) := n2h(dop_abs(7 downto 4)); vi := vi + 1;
                            b(vi) := n2h(dop_abs(3 downto 0)); vi := vi + 1;
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"70"; vi := vi + 1; -- p
                            b(vi) := x"68"; vi := vi + 1; -- h
                            b(vi) := x"3D"; vi := vi + 1;
                            -- phase: 10 bits -> 3 nibbles: "00"&ph[9:8], ph[7:4], ph[3:0]
                            b(vi) := n2h("00" & pending_ph(9 downto 8)); vi := vi + 1;
                            b(vi) := n2h(pending_ph(7 downto 4)); vi := vi + 1;
                            b(vi) := n2h(pending_ph(3 downto 0)); vi := vi + 1;
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"73"; vi := vi + 1; -- s
                            b(vi) := x"6E"; vi := vi + 1; -- n
                            b(vi) := x"72"; vi := vi + 1; -- r
                            b(vi) := x"3D"; vi := vi + 1;
                            b(vi) := n2h(pending_snr(15 downto 12)); vi := vi + 1;
                            b(vi) := n2h(pending_snr(11 downto 8));  vi := vi + 1;
                            b(vi) := n2h(pending_snr(7  downto 4));  vi := vi + 1;
                            b(vi) := n2h(pending_snr(3  downto 0));  vi := vi + 1;
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"6E"; vi := vi + 1; -- n
                            b(vi) := x"3D"; vi := vi + 1;
                            b(vi) := n2h(pending_nse(15 downto 12)); vi := vi + 1;
                            b(vi) := n2h(pending_nse(11 downto 8));  vi := vi + 1;
                            b(vi) := n2h(pending_nse(7  downto 4));  vi := vi + 1;
                            b(vi) := n2h(pending_nse(3  downto 0));  vi := vi + 1;
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"6D"; vi := vi + 1; -- m
                            b(vi) := x"3D"; vi := vi + 1;
                            b(vi) := n2h(pending_mrg(15 downto 12)); vi := vi + 1;
                            b(vi) := n2h(pending_mrg(11 downto 8));  vi := vi + 1;
                            b(vi) := n2h(pending_mrg(7  downto 4));  vi := vi + 1;
                            b(vi) := n2h(pending_mrg(3  downto 0));  vi := vi + 1;
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"66"; vi := vi + 1; -- f
                            b(vi) := x"3D"; vi := vi + 1;
                            b(vi) := n2h(pending_flg(7 downto 4));   vi := vi + 1;
                            b(vi) := n2h(pending_flg(3 downto 0));   vi := vi + 1;
                            b(vi) := x"0D"; vi := vi + 1;
                            b(vi) := x"0A"; vi := vi + 1;
                        else
                            -- NOLOCK
                            b(vi) := x"4E"; vi := vi + 1; -- N
                            b(vi) := x"4F"; vi := vi + 1; -- O
                            b(vi) := x"4C"; vi := vi + 1; -- L
                            b(vi) := x"4F"; vi := vi + 1; -- O
                            b(vi) := x"43"; vi := vi + 1; -- C
                            b(vi) := x"4B"; vi := vi + 1; -- K
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"6E"; vi := vi + 1; -- n
                            b(vi) := x"3D"; vi := vi + 1;
                            b(vi) := n2h(pending_nse(15 downto 12)); vi := vi + 1;
                            b(vi) := n2h(pending_nse(11 downto 8));  vi := vi + 1;
                            b(vi) := n2h(pending_nse(7  downto 4));  vi := vi + 1;
                            b(vi) := n2h(pending_nse(3  downto 0));  vi := vi + 1;
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"6D"; vi := vi + 1; -- m
                            b(vi) := x"3D"; vi := vi + 1;
                            b(vi) := n2h(pending_mrg(15 downto 12)); vi := vi + 1;
                            b(vi) := n2h(pending_mrg(11 downto 8));  vi := vi + 1;
                            b(vi) := n2h(pending_mrg(7  downto 4));  vi := vi + 1;
                            b(vi) := n2h(pending_mrg(3  downto 0));  vi := vi + 1;
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"66"; vi := vi + 1; -- f
                            b(vi) := x"3D"; vi := vi + 1;
                            b(vi) := n2h(pending_flg(7 downto 4));   vi := vi + 1;
                            b(vi) := n2h(pending_flg(3 downto 0));   vi := vi + 1;
                            b(vi) := x"0D"; vi := vi + 1;
                            b(vi) := x"0A"; vi := vi + 1;
                        end if;
                        lbuf     <= b;
                        line_len <= vi;
                        line_idx <= 0;
                        state    <= S_SEND_LINE;

                    when S_SEND_LINE =>
                        if tx_busy = '0' and tx_start_r = '0' then
                            tx_data_r  <= lbuf(line_idx);
                            tx_start_r <= '1';
                            if line_idx = line_len - 1 then
                                state <= S_WAIT_WR;
                            else
                                line_idx <= line_idx + 1;
                            end if;
                        end if;

                    when S_FORMAT_TOTAL =>
                        -- "TOTAL: XX sats\r\n" (hex sat count)
                        tb := (others => x"20");
                        ti := 0;
                        tb(ti) := x"54"; ti := ti + 1; -- T
                        tb(ti) := x"4F"; ti := ti + 1; -- O
                        tb(ti) := x"54"; ti := ti + 1; -- T
                        tb(ti) := x"41"; ti := ti + 1; -- A
                        tb(ti) := x"4C"; ti := ti + 1; -- L
                        tb(ti) := x"3A"; ti := ti + 1; -- :
                        tb(ti) := x"20"; ti := ti + 1;
                        tb(ti) := n2h("00" & std_logic_vector(sat_cnt(5 downto 4))); ti := ti + 1;
                        tb(ti) := n2h(std_logic_vector(sat_cnt(3 downto 0))); ti := ti + 1;
                        tb(ti) := x"20"; ti := ti + 1;
                        tb(ti) := x"73"; ti := ti + 1; -- s
                        tb(ti) := x"61"; ti := ti + 1; -- a
                        tb(ti) := x"74"; ti := ti + 1; -- t
                        tb(ti) := x"73"; ti := ti + 1; -- s
                        tb(ti) := x"0D"; ti := ti + 1;
                        tb(ti) := x"0A"; ti := ti + 1;
                        tbuf    <= tb;
                        tot_len <= ti;
                        tot_idx <= 0;
                        state <= S_SEND_TOTAL;

                    when S_SEND_TOTAL =>
                        if tx_busy = '0' and tx_start_r = '0' then
                            tx_data_r  <= tbuf(tot_idx);
                            tx_start_r <= '1';
                            if tot_idx = tot_len - 1 then
                                ftr_idx <= 0;
                                state   <= S_SEND_FTR;
                            else
                                tot_idx <= tot_idx + 1;
                            end if;
                        end if;

                    when S_SEND_FTR =>
                        if tx_busy = '0' and tx_start_r = '0' then
                            tx_data_r  <= FOOTER(ftr_idx);
                            tx_start_r <= '1';
                            if ftr_idx = FTR_LEN - 1 then
                                state <= S_DONE;
                            else
                                ftr_idx <= ftr_idx + 1;
                            end if;
                        end if;

                    when S_DONE =>
                        reporting   <= '0';
                        report_pend <= '0';
                        if force_reset = '1' then
                            sweep_pend <= '0';
                            sat_cnt    <= (others => '0');
                            fifo_count <= 0;
                            fifo_wr_ptr <= 0;
                            fifo_rd_ptr <= 0;
                            state      <= S_IDLE;
                        elsif sweep_pend = '1' or fifo_count > 0 then
                            sweep_pend <= '0';
                            sat_cnt    <= (others => '0');
                            hdr_idx    <= 0;
                            reporting  <= '1';
                            state      <= S_SEND_HDR;
                        end if;

                end case;
            end if;
        end if;
    end process;

end Behavioral;
