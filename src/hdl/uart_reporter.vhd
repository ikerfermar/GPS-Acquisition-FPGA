library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- uart_reporter.vhd  (v11 - force_reset no interrumpe, cierra limpio)
--
-- PROBLEMA CORREGIDO respecto a v10
-- ────────────────────────────────────────────────────────────
-- v10 usaba un bloque de alta prioridad "elsif force_reset='1'"
-- que saltaba a S_IDLE INMEDIATAMENTE desde cualquier estado.
-- Si SW14 bajaba mientras el reporter estaba en S_WAIT_WR,
-- el TOTAL y el FOOTER nunca se enviaban: el adquisidor se
-- reseteaba, dejaba de mandar wr_en/acq_done, y el reporter
-- se quedaba en S_WAIT_WR hasta que force_reset lo sacaba
-- a S_IDLE sin pasar por S_FORMAT_TOTAL ni S_SEND_FTR.
--
-- FIX v11: force_reset se gestiona DENTRO de los estados:
--
--   S_IDLE    → si force_reset='1': limpiar pendientes, no arrancar.
--               Si force_reset='0' y sweep_pend='1': arrancar normal.
--
--   S_WAIT_WR → si force_reset='1' y no hay pending_wr/report_pend:
--               ir a S_FORMAT_TOTAL → enviar TOTAL (parcial) + FOOTER.
--               El receptor verá un mensaje completo aunque el barrido
--               fue interrumpido a mitad.
--
--   S_DONE    → si force_reset='1': limpiar y volver a S_IDLE.
--               No relanzar barrido hasta que SW14 suba.
--
-- Los estados S_SEND_HDR, S_FORMAT_SAT, S_SEND_LINE, S_FORMAT_TOTAL,
-- S_SEND_TOTAL, S_SEND_FTR completan su trabajo sin interrupciones.
-- Esto garantiza que el UART siempre manda mensajes completos.
--
-- Flujo con SW14:
--   SW14↓ → reporter termina línea actual → S_WAIT_WR
--          → detecta force_reset → S_FORMAT_TOTAL → TOTAL → FOOTER
--          → S_DONE → S_IDLE (limpio, silencioso)
--   SW14↑ → adquisidor emite sweep_restart → sweep_pend='1'
--          → S_IDLE detecta sweep_pend → HEADER → barrido limpio
-- =============================================================

entity uart_reporter is
    Generic (
        CLK_FREQ  : integer := 100_000_000;
        BAUD_RATE : integer := 115_200;
        NUM_PRNS  : integer := 8
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

    signal pending_wr  : std_logic := '0';
    signal pending_prn : std_logic_vector(4 downto 0) := (others => '0');
    signal pending_acq : std_logic := '0';
    signal pending_dop : std_logic_vector(7 downto 0) := (others => '0');
    signal pending_ph  : std_logic_vector(9 downto 0) := (others => '0');
    signal pending_snr : std_logic_vector(15 downto 0) := (others => '0');
    signal report_pend : std_logic := '0';
    signal sweep_pend  : std_logic := '0';

    signal sat_cnt  : unsigned(5 downto 0) := (others => '0');
    signal hdr_idx  : integer range 0 to 34 := 0;
    signal ftr_idx  : integer range 0 to 33 := 0;

    signal line_idx : integer range 0 to 71 := 0;
    signal line_len : integer range 0 to 71 := 0;

    signal tot_idx  : integer range 0 to 31 := 0;
    signal tot_len  : integer range 0 to 31 := 0;

    type buf72_t is array (0 to 71) of std_logic_vector(7 downto 0);
    signal lbuf : buf72_t := (others => x"20");
    signal tbuf : buf72_t := (others => x"20");

    constant HDR_LEN : integer := 35;
    type rom35_t is array (0 to 34) of std_logic_vector(7 downto 0);
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

    function d2a(n : integer) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(n + 48, 8));
    end function;

begin

    u_uart : uart_tx
        generic map (CLK_FREQ => CLK_FREQ, BAUD_RATE => BAUD_RATE)
        port map (clk => clk, reset_n => reset_n,
                  tx_data => tx_data_r, tx_start => tx_start_r,
                  tx_busy => tx_busy,  tx_pin   => uart_tx_pin);

    process(clk)
        variable vi    : integer range 0 to 71;
        variable dop_s : signed(7 downto 0);
        variable dop_a : integer range 0 to 40960;
        variable ph_v  : integer range 0 to 1023;
        variable snr_v : integer range 0 to 65535;
        variable prn_d : integer range 1 to 32;
        variable cnt_v : integer range 0 to 63;
        variable b     : buf72_t;
    begin
        if rising_edge(clk) then
            tx_start_r <= '0';

            -- -------------------------------------------------------
            -- RESET HARDWARE (power-on / MMCM unlock)
            -- Prioridad máxima.
            -- -------------------------------------------------------
            if reset_n = '0' then
                state       <= S_IDLE;
                reporting   <= '0';
                pending_wr  <= '0';
                pending_snr <= (others => '0');
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
                -- Latch wr_en: guarda el resultado de cada PRN.
                -- sat_cnt se resetea en S_SEND_HDR, no aquí.
                -- -------------------------------------------------------
                if wr_en = '1' then
                    pending_wr  <= '1';
                    pending_prn <= wr_addr;
                    pending_acq <= wr_acq;
                    pending_dop <= wr_doppler;
                    pending_ph  <= wr_phase;
                    pending_snr <= wr_snr;
                    if wr_acq = '1' then
                        sat_cnt <= sat_cnt + 1;
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
                            -- SW14=0: limpiar pendientes, no arrancar barrido.
                            sweep_pend <= '0';
                            sat_cnt    <= (others => '0');
                            pending_wr <= '0';
                        elsif sweep_pend = '1' or pending_wr = '1' then
                            sweep_pend <= '0';
                            sat_cnt    <= (others => '0');  -- reset limpio
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
                        if pending_wr = '1' then
                            pending_wr <= '0';
                            state      <= S_FORMAT_SAT;
                        elsif report_pend = '1' then
                            report_pend <= '0';
                            state       <= S_FORMAT_TOTAL;
                        elsif force_reset = '1' then
                            -- SW14 bajado: el adquisidor se reseteó y no
                            -- mandará más wr_en ni acq_done. Enviar TOTAL
                            -- con el parcial acumulado y cerrar el barrido.
                            state <= S_FORMAT_TOTAL;
                        end if;

                    when S_FORMAT_SAT =>
                        vi    := 0;
                        b     := lbuf;
                        prn_d := to_integer(unsigned(pending_prn)) + 1;

                        b(vi) := x"53"; vi := vi + 1;  -- S
                        b(vi) := x"41"; vi := vi + 1;  -- A
                        b(vi) := x"54"; vi := vi + 1;  -- T
                        b(vi) := x"20"; vi := vi + 1;
                        b(vi) := d2a(prn_d / 10);   vi := vi + 1;
                        b(vi) := d2a(prn_d mod 10); vi := vi + 1;
                        b(vi) := x"3A"; vi := vi + 1;  -- :
                        b(vi) := x"20"; vi := vi + 1;

                        if pending_acq = '1' then
                            b(vi) := x"41"; vi := vi + 1;  -- A
                            b(vi) := x"43"; vi := vi + 1;  -- C
                            b(vi) := x"51"; vi := vi + 1;  -- Q
                            b(vi) := x"55"; vi := vi + 1;  -- U
                            b(vi) := x"49"; vi := vi + 1;  -- I
                            b(vi) := x"52"; vi := vi + 1;  -- R
                            b(vi) := x"45"; vi := vi + 1;  -- E
                            b(vi) := x"44"; vi := vi + 1;  -- D
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"64"; vi := vi + 1;  -- d
                            b(vi) := x"6F"; vi := vi + 1;  -- o
                            b(vi) := x"70"; vi := vi + 1;  -- p
                            b(vi) := x"70"; vi := vi + 1;  -- p
                            b(vi) := x"6C"; vi := vi + 1;  -- l
                            b(vi) := x"65"; vi := vi + 1;  -- e
                            b(vi) := x"72"; vi := vi + 1;  -- r
                            b(vi) := x"3D"; vi := vi + 1;  -- =
                            dop_s := -signed(pending_dop);
                            if dop_s < 0 then
                                b(vi) := x"2D"; vi := vi + 1;  -- -
                                dop_a := to_integer(-dop_s) * 320;
                            else
                                b(vi) := x"2B"; vi := vi + 1;  -- +
                                dop_a := to_integer(dop_s) * 320;
                            end if;
                            b(vi) := d2a(dop_a / 10000);          vi := vi + 1;
                            b(vi) := d2a((dop_a / 1000) mod 10);  vi := vi + 1;
                            b(vi) := d2a((dop_a / 100)  mod 10);  vi := vi + 1;
                            b(vi) := d2a((dop_a / 10)   mod 10);  vi := vi + 1;
                            b(vi) := d2a(dop_a mod 10);           vi := vi + 1;
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"48"; vi := vi + 1;  -- H
                            b(vi) := x"7A"; vi := vi + 1;  -- z
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"70"; vi := vi + 1;  -- p
                            b(vi) := x"68"; vi := vi + 1;  -- h
                            b(vi) := x"61"; vi := vi + 1;  -- a
                            b(vi) := x"73"; vi := vi + 1;  -- s
                            b(vi) := x"65"; vi := vi + 1;  -- e
                            b(vi) := x"3D"; vi := vi + 1;  -- =
                            ph_v := to_integer(unsigned(pending_ph));
                            b(vi) := d2a(ph_v / 1000);          vi := vi + 1;
                            b(vi) := d2a((ph_v / 100) mod 10);  vi := vi + 1;
                            b(vi) := d2a((ph_v / 10)  mod 10);  vi := vi + 1;
                            b(vi) := d2a(ph_v mod 10);          vi := vi + 1;
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"63"; vi := vi + 1;  -- c
                            b(vi) := x"68"; vi := vi + 1;  -- h
                            b(vi) := x"69"; vi := vi + 1;  -- i
                            b(vi) := x"70"; vi := vi + 1;  -- p
                            b(vi) := x"73"; vi := vi + 1;  -- s
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"73"; vi := vi + 1;  -- s
                            b(vi) := x"6E"; vi := vi + 1;  -- n
                            b(vi) := x"72"; vi := vi + 1;  -- r
                            b(vi) := x"3D"; vi := vi + 1;  -- =
                            snr_v := to_integer(unsigned(pending_snr));
                            b(vi) := d2a(snr_v / 10000);          vi := vi + 1;
                            b(vi) := d2a((snr_v / 1000) mod 10);  vi := vi + 1;
                            b(vi) := d2a((snr_v / 100)  mod 10);  vi := vi + 1;
                            b(vi) := d2a((snr_v / 10)   mod 10);  vi := vi + 1;
                            b(vi) := d2a(snr_v mod 10);           vi := vi + 1;
                            -- vi = 66
                        else
                            b(vi) := x"4E"; vi := vi + 1;  -- N
                            b(vi) := x"4F"; vi := vi + 1;  -- O
                            b(vi) := x"20"; vi := vi + 1;
                            b(vi) := x"4C"; vi := vi + 1;  -- L
                            b(vi) := x"4F"; vi := vi + 1;  -- O
                            b(vi) := x"43"; vi := vi + 1;  -- C
                            b(vi) := x"4B"; vi := vi + 1;  -- K
                            -- vi = 17
                        end if;

                        b(vi) := x"0D"; vi := vi + 1;
                        b(vi) := x"0A"; vi := vi + 1;

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
                        vi    := 0;
                        b     := tbuf;
                        cnt_v := to_integer(sat_cnt);
                        b(vi) := x"54"; vi := vi + 1;  -- T
                        b(vi) := x"4F"; vi := vi + 1;  -- O
                        b(vi) := x"54"; vi := vi + 1;  -- T
                        b(vi) := x"41"; vi := vi + 1;  -- A
                        b(vi) := x"4C"; vi := vi + 1;  -- L
                        b(vi) := x"3A"; vi := vi + 1;  -- :
                        b(vi) := x"20"; vi := vi + 1;
                        b(vi) := d2a(cnt_v / 10);   vi := vi + 1;
                        b(vi) := d2a(cnt_v mod 10); vi := vi + 1;
                        b(vi) := x"20"; vi := vi + 1;
                        b(vi) := x"73"; vi := vi + 1;  -- s
                        b(vi) := x"61"; vi := vi + 1;  -- a
                        b(vi) := x"74"; vi := vi + 1;  -- t
                        b(vi) := x"65"; vi := vi + 1;  -- e
                        b(vi) := x"6C"; vi := vi + 1;  -- l
                        b(vi) := x"6C"; vi := vi + 1;  -- l
                        b(vi) := x"69"; vi := vi + 1;  -- i
                        b(vi) := x"74"; vi := vi + 1;  -- t
                        b(vi) := x"65"; vi := vi + 1;  -- e
                        b(vi) := x"73"; vi := vi + 1;  -- s
                        b(vi) := x"0D"; vi := vi + 1;
                        b(vi) := x"0A"; vi := vi + 1;
                        tbuf    <= b;
                        tot_len <= vi;
                        tot_idx <= 0;
                        state   <= S_SEND_TOTAL;

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
                        pending_wr  <= '0';
                        if force_reset = '1' then
                            -- SW14=0: quedarse en S_IDLE sin relanzar barrido.
                            sweep_pend <= '0';
                            sat_cnt    <= (others => '0');
                            state      <= S_IDLE;
                        elsif sweep_pend = '1' or pending_wr = '1' then
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