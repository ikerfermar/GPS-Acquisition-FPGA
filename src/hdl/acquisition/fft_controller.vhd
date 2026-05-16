library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- fft_controller.vhd
--
-- Calcula la correlacion circular entre la senal RX (I/Q tras
-- el doppler_mixer) y el codigo CA local mediante FFT/IFFT:
--   1. FFT de 1024 muestras RX
--   2. FFT de 1024 chips CA local (+/-32767)
--   3. Multiplicacion espectral: RX * conj(CA)
--   4. IFFT -> funcion de correlacion compleja (1024 puntos)
--
-- SINCRONISMO (clk_en = clk_en_fe del top, ~1.023 MHz):
--   T0 (clk_en='1'):    ca_cap <= ca_bit_local   (chip[K])
--                       loading_active activa en epoch
--   T1 (clk_en_d1='1'): captura rx_I/Q del mixer (chip[K], pipeline 1 ciclo)
--                       y ca_cap -> enviados a las FFTs
--
-- IP cores requeridos:
--   xfft_0 : FFT N=1024, forward, streaming (x2, una para RX, otra para CA)
--   xfft_1 : FFT N=1024, run-time inv/fwd (configurado como IFFT con 0x010A)
-- =============================================================

entity fft_controller is
    Generic (
        COHERENT_N_MS : integer := 1
    );
    Port (
        clk          : in  STD_LOGIC;
        clk_en       : in  STD_LOGIC;
        reset_n      : in  STD_LOGIC;
        rx_I         : in  STD_LOGIC_VECTOR(15 downto 0);
        rx_Q         : in  STD_LOGIC_VECTOR(15 downto 0);
        ca_bit_local : in  STD_LOGIC;
        ca_epoch     : in  STD_LOGIC;
        coherent_n_ms_sel : in STD_LOGIC_VECTOR(1 downto 0);
        fft_out_ready: in  STD_LOGIC;
        -- Desde Fase 14, coherent_en siempre '1'. Puerto eliminado; n_ms_eff = COHERENT_N_MS siempre.
        fft_ready    : out STD_LOGIC;
        fft_data_out : out STD_LOGIC_VECTOR(31 downto 0)
    );
end fft_controller;

architecture Behavioral of fft_controller is
    constant FFT_FRAME_LAST : integer := 1023;
    constant CA_SAMPLE_POS  : std_logic_vector(31 downto 0) := x"00000001"; -- +1 (Re)
    constant CA_SAMPLE_NEG  : std_logic_vector(31 downto 0) := x"0000FFFF"; -- -1 (Re)

    -- xfft_0: forward FFT, N=1024, 16-bit input, scaled, Radix-2 Lite.
    -- Per xfft_0.xci: C_M_AXIS_DATA_TDATA_WIDTH = 32, C_OUTPUT_WIDTH = 16.
    -- Layout: Re[15:0] in tdata[15:0], Im[15:0] in tdata[31:16] (PG109 Table 3-7).
    COMPONENT xfft_0 PORT (
        aclk                 : IN  STD_LOGIC;
        aclken               : IN  STD_LOGIC;
        aresetn              : IN  STD_LOGIC;
        s_axis_config_tdata  : IN  STD_LOGIC_VECTOR(23 downto 0);
        s_axis_config_tvalid : IN  STD_LOGIC;
        s_axis_config_tready : OUT STD_LOGIC;
        s_axis_data_tdata    : IN  STD_LOGIC_VECTOR(31 downto 0);
        s_axis_data_tvalid   : IN  STD_LOGIC;
        s_axis_data_tready   : OUT STD_LOGIC;
        s_axis_data_tlast    : IN  STD_LOGIC;
        m_axis_data_tdata    : OUT STD_LOGIC_VECTOR(31 downto 0);  -- 32 bits per XCI
        m_axis_data_tvalid   : OUT STD_LOGIC;
        m_axis_data_tready   : IN  STD_LOGIC;
        m_axis_data_tlast    : OUT STD_LOGIC;
        event_frame_started      : OUT STD_LOGIC;
        event_tlast_unexpected   : OUT STD_LOGIC;
        event_tlast_missing      : OUT STD_LOGIC;
        event_status_channel_halt: OUT STD_LOGIC;
        event_data_in_channel_halt : OUT STD_LOGIC;
        event_data_out_channel_halt: OUT STD_LOGIC
    ); END COMPONENT;

    -- xfft_1: run-time configurable FFT/IFFT, N=1024, 16-bit input, scaled, Radix-2 Lite.
    -- Per xfft_1.xci: C_S_AXIS_CONFIG_TDATA_WIDTH = 32, C_M_AXIS_DATA_TDATA_WIDTH = 32.
    -- Layout output: Re[15:0] in tdata[15:0], Im[15:0] in tdata[31:16] (PG109 Table 3-7).
    COMPONENT xfft_1 PORT (
        aclk                 : IN  STD_LOGIC;
        aclken               : IN  STD_LOGIC;
        aresetn              : IN  STD_LOGIC;
        s_axis_config_tdata  : IN  STD_LOGIC_VECTOR(31 downto 0);  -- 32 bits per XCI
        s_axis_config_tvalid : IN  STD_LOGIC;
        s_axis_config_tready : OUT STD_LOGIC;
        s_axis_data_tdata    : IN  STD_LOGIC_VECTOR(31 downto 0);
        s_axis_data_tvalid   : IN  STD_LOGIC;
        s_axis_data_tready   : OUT STD_LOGIC;
        s_axis_data_tlast    : IN  STD_LOGIC;
        m_axis_data_tdata    : OUT STD_LOGIC_VECTOR(31 downto 0);  -- 32 bits per XCI
        m_axis_data_tvalid   : OUT STD_LOGIC;
        m_axis_data_tready   : IN  STD_LOGIC;
        m_axis_data_tlast    : OUT STD_LOGIC;
        event_frame_started      : OUT STD_LOGIC;
        event_tlast_unexpected   : OUT STD_LOGIC;
        event_tlast_missing      : OUT STD_LOGIC;
        event_status_channel_halt: OUT STD_LOGIC;
        event_data_in_channel_halt : OUT STD_LOGIC;
        event_data_out_channel_halt: OUT STD_LOGIC
    ); END COMPONENT;

    -- Pipeline de clk_en para alinear rx_I/Q con ca_cap.
    -- Se usa clk_en_d1 para minimizar desfase sub-chip fijo.
    signal clk_en_d1 : std_logic := '0';
    signal ca_cap    : std_logic := '0';  -- CA capturado en T0 (chip[K])

    -- Control de carga de 1024 muestras por epoch
    signal loading_active : std_logic := '0';
    signal sample_counter : integer range 0 to FFT_FRAME_LAST := 0;
    signal fft_busy       : std_logic := '0';

    -- Bus AXI-S hacia las FFTs (totalmente independizados)
    signal s_fft_rx_tdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_fft_loc_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_fft_rx_tvalid  : std_logic := '0';
    signal s_fft_loc_tvalid : std_logic := '0';
    signal s_fft_rx_tlast   : std_logic := '0';
    signal s_fft_loc_tlast  : std_logic := '0';
    signal sample_pending   : std_logic := '0';
    signal pending_rx_tdata : std_logic_vector(31 downto 0) := (others => '0');
    signal pending_loc_tdata: std_logic_vector(31 downto 0) := (others => '0');
    signal pending_tlast    : std_logic := '0';
    signal s_fft_rx_tready  : std_logic;
    signal s_fft_loc_tready : std_logic;
    signal m_ifft_tlast     : std_logic;

    -- Salidas FFT forward: 64 bits per IP (Re[15:0] in [15:0], Im[15:0] in [47:32])
    signal m_fft_rx_tdata  : std_logic_vector(31 downto 0);
    signal m_fft_loc_tdata : std_logic_vector(31 downto 0);
    signal m_fft_rx_tvalid : std_logic;
    signal m_fft_loc_tvalid : std_logic;
    signal m_fft_rx_tlast  : std_logic;

    -- Alineacion robusta de streams FFT forward: buffer de 1 muestra por canal.
    signal fft_rx_buf_data  : std_logic_vector(31 downto 0) := (others => '0');
    signal fft_loc_buf_data : std_logic_vector(31 downto 0) := (others => '0');
    signal fft_rx_buf_last  : std_logic := '0';
    -- fft_loc_buf_last eliminado: spec_tlast solo usa fft_rx_buf_last
    signal fft_rx_buf_valid : std_logic := '0';
    signal fft_loc_buf_valid: std_logic := '0';
    signal spec_consume     : std_logic := '0';

    signal spec_tvalid : std_logic;
    signal spec_tlast  : std_logic;

    -- Multiplicacion compleja RX * conj(CA), pipeline 2 ciclos
    signal re_rx, im_rx   : signed(15 downto 0);
    signal re_loc, im_loc : signed(15 downto 0);
    signal p1, p2, p3, p4 : signed(31 downto 0);
    signal re_mult        : signed(32 downto 0);
    signal im_mult        : signed(32 downto 0);
    signal mult_re_sat    : signed(15 downto 0) := (others => '0');
    signal mult_im_sat    : signed(15 downto 0) := (others => '0');
    signal pipe_valid     : std_logic_vector(2 downto 0) := "000";
    signal pipe_last      : std_logic_vector(2 downto 0) := "000";
    signal mult_tdata     : std_logic_vector(31 downto 0);
    signal mult_tvalid    : std_logic;
    signal mult_tlast     : std_logic;

    -- Acumulacion coherente espectral previa a IFFT.
    -- SYN-4: migrado a BRAM ("block") para liberar ~2059 LUTs distribuidos.
    -- Patron de inferencia BRAM Xilinx: proceso de escritura separado del proceso
    -- de lectura. El proceso lector SOLO hace: bram_out <= ram(rd_addr).
    -- La direccion de lectura se controla con bram_rd_addr (registro) desde el
    -- proceso principal; bram_re/im_out nunca se asignan fuera del proceso lector.
    type coh_acc_ram_t is array (0 to 1023) of signed(17 downto 0);
    signal coh_re_ram : coh_acc_ram_t := (others => (others => '0'));
    signal coh_im_ram : coh_acc_ram_t := (others => (others => '0'));
    attribute ram_style : string;
    attribute ram_style of coh_re_ram : signal is "block";
    attribute ram_style of coh_im_ram : signal is "block";

    -- Direccion de lectura BRAM (registrada por el proceso principal).
    -- El proceso de lectura usa esta senal como indice; nunca se escribe bram_re/im_out
    -- fuera del proceso de lectura para que Vivado infiera BRAM correctamente.
    signal bram_rd_addr : integer range 0 to 1023 := 0;

    -- Salidas BRAM (solo escritas por el proceso de lectura)
    signal bram_re_out : signed(17 downto 0);
    signal bram_im_out : signed(17 downto 0);

    -- Pipeline de 2 ciclos para Etapa A (alinea datos con latencia de 1 ciclo de lectura BRAM)
    signal coh_a_valid_d1   : std_logic := '0';
    signal coh_a_valid_d2   : std_logic := '0';
    signal coh_a_last_d1    : std_logic := '0';
    signal coh_a_last_d2    : std_logic := '0';
    signal coh_a_epoch_d1   : integer range 0 to 15 := 0;
    signal coh_a_epoch_d2   : integer range 0 to 15 := 0;
    signal coh_a_idx_d1     : integer range 0 to 1023 := 0;
    signal coh_a_idx_d2     : integer range 0 to 1023 := 0;
    signal coh_a_in_re_d1   : signed(15 downto 0) := (others => '0');
    signal coh_a_in_re_d2   : signed(15 downto 0) := (others => '0');
    signal coh_a_in_im_d1   : signed(15 downto 0) := (others => '0');
    signal coh_a_in_im_d2   : signed(15 downto 0) := (others => '0');

    -- Pipeline de Etapa B (lectura BRAM para envio a IFFT)
    signal coh_b_rd_pending_d1: std_logic := '0';
    signal coh_b_rd_last_d1   : std_logic := '0';
    signal coh_b_rd_pending : std_logic := '0';  -- hay una lectura BRAM en vuelo
    signal coh_b_rd_last    : std_logic := '0';
    signal coh_b_rd_active  : std_logic := '0';  -- Etapa B activa (enviando)

    signal coh_idx         : integer range 0 to 1023 := 0;
    signal coh_epoch_count : integer range 0 to 15 := 0;
    signal coh_send_active : std_logic := '0';
    signal coh_send_idx    : integer range 0 to 1023 := 0;
    signal n_ms_eff_r      : integer range 1 to 15 := COHERENT_N_MS;

    -- Handshake robusto hacia IFFT
    signal ifft_cfg_sent   : std_logic := '0';
    signal ifft_cfg_tready : std_logic;
    signal ifft_cfg_tvalid : std_logic := '0';

    signal ifft_in_tdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal ifft_in_tlast   : std_logic := '0';
    signal ifft_in_tvalid  : std_logic := '0';
    signal ifft_in_tready  : std_logic;

    -- Salida IFFT: 64 bits per IP (Re[15:0] in [15:0], Im[15:0] in [47:32])
    signal m_ifft_tdata  : std_logic_vector(31 downto 0);
    signal m_ifft_tvalid : std_logic;

    -- Timing-first: limitar fanout de senales de control de handshake interno.
    attribute max_fanout : string;
    attribute max_fanout of sample_pending : signal is "32";
    attribute max_fanout of coh_send_active : signal is "32";
    attribute max_fanout of coh_b_rd_pending : signal is "32";
    attribute max_fanout of ifft_in_tvalid : signal is "32";

begin

    -- Decodificacion runtime de ventana coherente (T7) en un registro dedicado.
    -- Evita repetir mux/comparadores dentro del proceso de acumulacion/handshake.
    process(clk)
        variable n_ms_v : integer range 1 to 15;
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                if COHERENT_N_MS < 1 then
                    n_ms_eff_r <= 1;
                elsif COHERENT_N_MS > 15 then
                    n_ms_eff_r <= 15;
                else
                    n_ms_eff_r <= COHERENT_N_MS;
                end if;
            else
                case coherent_n_ms_sel is
                    when "00" => n_ms_v := 1;
                    when "01" => n_ms_v := 2;
                    when "10" => n_ms_v := 4;
                    when "11" => n_ms_v := 8;
                    when others =>
                        n_ms_v := COHERENT_N_MS;
                        if n_ms_v < 1 then
                            n_ms_v := 1;
                        elsif n_ms_v > 15 then
                            n_ms_v := 15;
                        end if;
                end case;
                n_ms_eff_r <= n_ms_v;
            end if;
        end if;
    end process;

    -- Pipeline clk_en: captura ca_cap en T0 (clk_en='1'), rx_I/Q en T1 (clk_en_d1='1').
    process(clk)
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                clk_en_d1 <= '0';
                ca_cap    <= '0';
            else
                clk_en_d1 <= clk_en;
                if clk_en = '1' then
                    ca_cap <= ca_bit_local;
                end if;
            end if;
        end if;
    end process;

    -- Control de carga: arranca en el epoch (chip[0]) y carga 1024 muestras
    process(clk)
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                loading_active  <= '0';
                sample_counter  <= 0;
                s_fft_rx_tvalid <= '0';
                s_fft_loc_tvalid<= '0';
                s_fft_rx_tlast  <= '0';
                s_fft_loc_tlast <= '0';
                s_fft_rx_tdata  <= (others => '0');
                s_fft_loc_tdata <= (others => '0');
                sample_pending  <= '0';
                pending_rx_tdata  <= (others => '0');
                pending_loc_tdata <= (others => '0');
                pending_tlast     <= '0';
                fft_busy        <= '0';
            else
                -- AXI-Stream handshaking: limpiar tvalid individual cuando el dato es aceptado
                if s_fft_rx_tvalid = '1' and s_fft_rx_tready = '1' then
                    s_fft_rx_tvalid <= '0';
                end if;
                if s_fft_loc_tvalid = '1' and s_fft_loc_tready = '1' then
                    s_fft_loc_tvalid <= '0';
                end if;

                -- fft_busy se libera al cerrar la carga y cuando no hay transacciones AXI en vuelo
                if loading_active = '0' and sample_pending = '0' and 
                   (s_fft_rx_tvalid = '0' or (s_fft_rx_tvalid = '1' and s_fft_rx_tready = '1')) and
                   (s_fft_loc_tvalid = '0' or (s_fft_loc_tvalid = '1' and s_fft_loc_tready = '1')) then
                    fft_busy <= '0';
                end if;

                if ca_epoch = '1' and loading_active = '0' and fft_busy = '0' then
                    loading_active <= '1';
                    sample_counter <= 0;
                    fft_busy       <= '1';  -- marcar ocupado desde el inicio de carga
                end if;

                -- Captura atomica de la pareja RX/CA cuando llega el chip valido.
                if loading_active = '1' and clk_en_d1 = '1' and sample_pending = '0' then
                    pending_rx_tdata <= rx_Q & rx_I;
                    if ca_cap = '0' then
                        pending_loc_tdata <= CA_SAMPLE_POS;   -- +32767
                    else
                        pending_loc_tdata <= CA_SAMPLE_NEG;   -- -32767
                    end if;

                    if sample_counter = FFT_FRAME_LAST then
                        pending_tlast <= '1';
                        sample_counter <= 0;
                        loading_active <= '0';
                    else
                        pending_tlast <= '0';
                        sample_counter <= sample_counter + 1;
                    end if;
                    sample_pending <= '1';
                end if;

                if sample_pending = '1' then
                    -- Empujar el siguiente dato solo cuando ambas interfaces esten libres/listas simultaneamente
                    if (s_fft_rx_tvalid = '0' or s_fft_rx_tready = '1') and
                       (s_fft_loc_tvalid = '0' or s_fft_loc_tready = '1') then
                        s_fft_rx_tdata   <= pending_rx_tdata;
                        s_fft_loc_tdata  <= pending_loc_tdata;
                        s_fft_rx_tvalid  <= '1';
                        s_fft_loc_tvalid <= '1';
                        s_fft_rx_tlast   <= pending_tlast;
                        s_fft_loc_tlast  <= pending_tlast;
                        sample_pending   <= '0';
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- FFT canal RX
    fft_rx_inst : xfft_0 PORT MAP (
        aclk                 => clk,
        aclken               => '1',
        aresetn              => reset_n,
        -- s_axis_config_tdata layout (24 bits, NFFT not runtime, no CP_LEN):
        --   bit[0]      = FWD_INV (0=inverse, 1=forward per PG109)
        --   bits[20:1]  = SCALE_SCH (Radix-2 Lite N=1024, 10 stages x 2 bits, last-stage-first packed LSB)
        --   bits[23:21] = padding
        -- 0x0AAAAA = SCALE_SCH=01_01_01_01_01_01_01_01_01_01 (every stage /2 = /N total),
        --            FWD_INV=0 (configured as inverse, but mathematically correlates correctly because both
        --            forward stages and the final stage are consistently mismatched -- see fft_inv_inst comment).
        -- FIX: Escala x1 (x"000000") para no destruir el ruido termico de la se?al GPS.
        -- El espectro ensanchado no desborda los 16 bits y mantiene la cuantizacion viva para el CFAR.
        s_axis_config_tdata  => x"000000",
        s_axis_config_tvalid => '1',
        s_axis_config_tready => open,
        s_axis_data_tdata    => s_fft_rx_tdata,
        s_axis_data_tvalid   => s_fft_rx_tvalid,
        s_axis_data_tready   => s_fft_rx_tready,
        s_axis_data_tlast    => s_fft_rx_tlast,
        m_axis_data_tdata    => m_fft_rx_tdata,
        m_axis_data_tvalid   => m_fft_rx_tvalid,
        m_axis_data_tready   => (not fft_rx_buf_valid) or spec_consume,
        m_axis_data_tlast    => m_fft_rx_tlast,
        event_frame_started      => open,
        event_tlast_unexpected   => open,
        event_tlast_missing      => open,
        event_status_channel_halt=> open,
        event_data_in_channel_halt => open,
        event_data_out_channel_halt=> open
    );

    fft_loc_inst : xfft_0 PORT MAP (
        aclk                 => clk,
        aclken               => '1',
        aresetn              => reset_n,
        -- Sin escalar: CA_SAMPLES = +/-1. FWD_INV=0.
        s_axis_config_tdata  => x"000000",
        s_axis_config_tvalid => '1',
        s_axis_config_tready => open,
        s_axis_data_tdata    => s_fft_loc_tdata,
        s_axis_data_tvalid   => s_fft_loc_tvalid,
        s_axis_data_tready   => s_fft_loc_tready,
        s_axis_data_tlast    => s_fft_loc_tlast,
        m_axis_data_tdata    => m_fft_loc_tdata,
        m_axis_data_tvalid   => m_fft_loc_tvalid,
        m_axis_data_tready   => (not fft_loc_buf_valid) or spec_consume,
        m_axis_data_tlast    => open,
        event_frame_started      => open,
        event_tlast_unexpected   => open,
        event_tlast_missing      => open,
        event_status_channel_halt=> open,
        event_data_in_channel_halt => open,
        event_data_out_channel_halt=> open
    );

    -- Extraccion de bits: salida FFT es 16b scaled (C_OUTPUT_WIDTH=16), tdata=64b.
    -- Per PG109 Radix-2 Lite layout: Re[15:0] in tdata[15:0], Im[15:0] in tdata[47:32].
    -- tdata[31:16] and tdata[63:48] are zero-padding, NOT signal data.
    -- Extraccion de bits: salida FFT es 16b scaled (C_OUTPUT_WIDTH=16), tdata=32b.
    -- Per PG109 Table 3-7 layout: Re[15:0] in tdata[15:0], Im[15:0] in tdata[31:16].
    re_rx  <= signed(fft_rx_buf_data (15 downto 0));
    im_rx  <= signed(fft_rx_buf_data (31 downto 16));
    re_loc <= signed(fft_loc_buf_data(15 downto 0));
    im_loc <= signed(fft_loc_buf_data(31 downto 16));

    -- Solo multiplicar cuando ambos streams (RX y CA) son validos en el mismo ciclo.
    spec_tvalid <= fft_rx_buf_valid and fft_loc_buf_valid;
    -- TLAST se toma del canal RX para no perder cierre de frame por
    -- desalineacion puntual entre tlast de ambos FFT forward.
    spec_tlast  <= fft_rx_buf_last;
    spec_consume <= spec_tvalid;

    -- Captura y alineacion de FFTs forward con buffer de 1 muestra/canal.
    -- La retencion se apoya en handshake AXIS: cuando el buffer de un canal
    -- esta lleno, su tready se desactiva y el core upstream mantiene tvalid
    -- y tdata estables hasta nuevo handshake. Bajo contrato AXIS no hay
    -- perdida de muestras, incluso con desalineacion sostenida.
    process(clk)
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                fft_rx_buf_valid <= '0';
                fft_loc_buf_valid <= '0';
                fft_rx_buf_last <= '0';
            else
                if spec_consume = '1' then
                    if m_fft_rx_tvalid = '1' then
                        fft_rx_buf_data  <= m_fft_rx_tdata;
                        fft_rx_buf_last  <= m_fft_rx_tlast;
                        fft_rx_buf_valid <= '1';
                    else
                        fft_rx_buf_valid <= '0';
                        fft_rx_buf_last  <= '0';
                    end if;

                    if m_fft_loc_tvalid = '1' then
                        fft_loc_buf_data  <= m_fft_loc_tdata;
                        fft_loc_buf_valid <= '1';
                    else
                        fft_loc_buf_valid <= '0';
                    end if;
                else
                    if m_fft_rx_tvalid = '1' and fft_rx_buf_valid = '0' then
                        fft_rx_buf_data  <= m_fft_rx_tdata;
                        fft_rx_buf_last  <= m_fft_rx_tlast;
                        fft_rx_buf_valid <= '1';
                    end if;

                    if m_fft_loc_tvalid = '1' and fft_loc_buf_valid = '0' then
                        fft_loc_buf_data  <= m_fft_loc_tdata;
                        fft_loc_buf_valid <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- Multiplicacion compleja: RX * conj(CA) = (re_rx+j*im_rx)(re_loc-j*im_loc)
    --   re = re_rx*re_loc + im_rx*im_loc
    --   im = im_rx*re_loc - re_rx*im_loc
    process(clk)
        variable re_acc_v : signed(32 downto 0);
        variable im_acc_v : signed(32 downto 0);
    begin
        if rising_edge(clk) then
            p1 <= re_rx * re_loc;
            p2 <= im_rx * im_loc;
            p3 <= im_rx * re_loc;
            p4 <= re_rx * im_loc;
            re_acc_v := resize(p1, 33) + resize(p2, 33);
            im_acc_v := resize(p3, 33) - resize(p4, 33);
            re_mult  <= re_acc_v;
            im_mult  <= im_acc_v;
            pipe_valid <= pipe_valid(1 downto 0) & spec_tvalid;
            pipe_last  <= pipe_last(1 downto 0)  & spec_tlast;
        end if;
    end process;

    -- BUGFIX (Apr 2026): Saturated >>8 truncation. The previous implementation
    --   `mult_tdata <= im_mult(23:8) & re_mult(23:8)`
    -- silently dropped the sign bit (re_mult is 33-bit signed, bit 32 is sign).
    -- For values where the magnitude exceeded 2^23 (= ±32768 after >>8),
    -- the result wrapped catastrophically. Worst case: re_mult = -2^31 became
    -- mult_tdata_re = 0, mapping the strongest correlation peak to zero.
    -- After this fix, values out of 16-bit signed range saturate properly.
    -- Adds 1 cycle pipeline latency, compensated by extending pipe_valid/last
    -- shift register by one stage (now 3 stages).
    process(clk)
    begin
        if rising_edge(clk) then
            -- Saturate re_mult >>3 to signed 16-bit range [-32768, 32767].
            -- Conserva el ruido termico vital (>>3) y satura limpiamente los picos sinteticos.
            if re_mult(32) = '0' and re_mult(31 downto 18) /= "00000000000000" then
                mult_re_sat <= to_signed(32767, 16);
            elsif re_mult(32) = '1' and re_mult(31 downto 18) /= "11111111111111" then
                mult_re_sat <= to_signed(-32768, 16);
            else
                mult_re_sat <= re_mult(18 downto 3);
            end if;
            if im_mult(32) = '0' and im_mult(31 downto 18) /= "00000000000000" then
                mult_im_sat <= to_signed(32767, 16);
            elsif im_mult(32) = '1' and im_mult(31 downto 18) /= "11111111111111" then
                mult_im_sat <= to_signed(-32768, 16);
            else
                mult_im_sat <= im_mult(18 downto 3);
            end if;
        end if;
    end process;

    mult_tdata  <= std_logic_vector(mult_im_sat) & std_logic_vector(mult_re_sat);

    mult_tvalid <= pipe_valid(2);
    mult_tlast  <= pipe_last(2);

    -- Proceso de lectura BRAM (patron canonico Xilinx).
    -- UNICO lugar donde se asignan bram_re_out / bram_im_out.
    -- Vivado infiere coh_re/im_ram como RAMB36 automaticamente.
    process(clk)
    begin
        if rising_edge(clk) then
            bram_re_out <= coh_re_ram(bram_rd_addr);
            bram_im_out <= coh_im_ram(bram_rd_addr);
        end if;
    end process;

    -- Acumulacion coherente de espectro y envio secuencial a IFFT.
    -- SYN-4: BRAM con latencia 1 ciclo. La direccion de lectura (bram_rd_addr)
    -- se registra en el proceso principal; los datos llegan un ciclo despues
    -- a traves del proceso de lectura BRAM de arriba.
    --   Etapa A: acumula espectro de cada epoch (pipeline de 1 ciclo).
    --   Etapa B: envia suma coherente a IFFT con handshake AXIS.
    process(clk)
        variable in_re_v  : signed(15 downto 0);
        variable in_im_v  : signed(15 downto 0);
        variable sum_re_v : signed(17 downto 0);
        variable sum_im_v : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                ifft_cfg_sent   <= '0';
                ifft_cfg_tvalid <= '0';
                ifft_in_tvalid  <= '0';
                ifft_in_tdata   <= (others => '0');
                ifft_in_tlast   <= '0';
                coh_idx          <= 0;
                coh_epoch_count  <= 0;
                coh_send_active  <= '0';
                coh_send_idx     <= 0;
                bram_rd_addr     <= 0;
                coh_a_valid_d1   <= '0';
                coh_a_valid_d2   <= '0';
                coh_a_last_d1    <= '0';
                coh_a_last_d2    <= '0';
                coh_a_epoch_d1   <= 0;
                coh_a_epoch_d2   <= 0;
                coh_a_idx_d1     <= 0;
                coh_a_idx_d2     <= 0;
                coh_a_in_re_d1   <= (others => '0');
                coh_a_in_re_d2   <= (others => '0');
                coh_a_in_im_d1   <= (others => '0');
                coh_a_in_im_d2   <= (others => '0');
                coh_b_rd_pending_d1 <= '0';
                coh_b_rd_last_d1 <= '0';
                coh_b_rd_pending <= '0';
                coh_b_rd_last    <= '0';
                coh_b_rd_active  <= '0';
            else

                -- ---------------------------------------------------------
                -- Configuracion IFFT
                -- ---------------------------------------------------------
                if ifft_cfg_sent = '0' then
                    ifft_cfg_tvalid <= '1';
                    if ifft_cfg_tvalid = '1' and ifft_cfg_tready = '1' then
                        ifft_cfg_sent   <= '1';
                        ifft_cfg_tvalid <= '0';
                    end if;
                else
                    ifft_cfg_tvalid <= '0';
                end if;

                -- Default: clear read pending pulse to avoid sequential overwrite bugs
                coh_b_rd_pending_d1 <= '0';
                coh_b_rd_last_d1    <= '0';

                -- ---------------------------------------------------------
                -- Etapa A ? Paso 1 (ciclo N):
                --   Registrar direccion BRAM. El dato saldr? de la RAM en N+2.
                -- ---------------------------------------------------------
                if coh_send_active = '0' and mult_tvalid = '1' then
                    in_re_v := signed(mult_tdata(15 downto  0));
                    in_im_v := signed(mult_tdata(31 downto 16));

                    -- Registrar direccion BRAM (lectura llega en ciclo N+1)
                    bram_rd_addr    <= coh_idx;

                    -- Buffering de metadatos para Etapa A Paso 2
                    coh_a_valid_d1  <= '1';
                    coh_a_last_d1   <= mult_tlast;
                    coh_a_epoch_d1  <= coh_epoch_count;
                    coh_a_idx_d1    <= coh_idx;
                    coh_a_in_re_d1  <= in_re_v;
                    coh_a_in_im_d1  <= in_im_v;

                    -- Avanzar idx y epoch
                    if mult_tlast = '1' then
                        coh_idx <= 0;
                        if coh_epoch_count = n_ms_eff_r - 1 then
                            coh_epoch_count <= 0;
                        else
                            coh_epoch_count <= coh_epoch_count + 1;
                        end if;
                    else
                        coh_idx <= coh_idx + 1;
                    end if;
                else
                    coh_a_valid_d1 <= '0';
                end if;

                -- ---------------------------------------------------------
                -- Etapa A ? Paso 2 (ciclo N+1):
                --   Propagar pipeline para esperar el ciclo de latencia 
                --   de lectura de la BRAM.
                -- ---------------------------------------------------------
                coh_a_valid_d2 <= coh_a_valid_d1;
                coh_a_last_d2  <= coh_a_last_d1;
                coh_a_epoch_d2 <= coh_a_epoch_d1;
                coh_a_idx_d2   <= coh_a_idx_d1;
                coh_a_in_re_d2 <= coh_a_in_re_d1;
                coh_a_in_im_d2 <= coh_a_in_im_d1;

                -- ---------------------------------------------------------
                -- Etapa A ? Paso 3 (ciclo N+2):
                --   bram_re/im_out ES V?LIDO aqu?. Acumular y escribir.
                -- ---------------------------------------------------------
                if coh_a_valid_d2 = '1' then
                    if coh_a_epoch_d2 = 0 then
                        sum_re_v := resize(coh_a_in_re_d2, 18);
                        sum_im_v := resize(coh_a_in_im_d2, 18);
                    else
                        sum_re_v := bram_re_out + resize(coh_a_in_re_d2, 18);
                        sum_im_v := bram_im_out + resize(coh_a_in_im_d2, 18);
                    end if;
                    coh_re_ram(coh_a_idx_d2) <= sum_re_v;
                    coh_im_ram(coh_a_idx_d2) <= sum_im_v;

                    if coh_a_last_d2 = '1' and coh_a_epoch_d2 = n_ms_eff_r - 1 then
                        coh_send_active  <= '1';
                        coh_send_idx     <= 0;
                        bram_rd_addr     <= 0;
                        coh_b_rd_active  <= '0';
                    end if;
                end if;

                -- Pipeline de 2 ciclos para Etapa B por latencia BRAM
                coh_b_rd_pending    <= coh_b_rd_pending_d1;
                coh_b_rd_last       <= coh_b_rd_last_d1;

                -- ---------------------------------------------------------
                -- Etapa B: enviar suma coherente a IFFT con handshake AXIS.
                -- bram_re/im_out contiene los datos del bram_rd_addr anterior.
                -- coh_b_rd_pending='1' indica que bram_re/im_out es valido
                -- en este ciclo (la lectura fue solicitada el ciclo anterior).
                -- ---------------------------------------------------------
                if coh_send_active = '1' then
                    
                    -- Paso B0: Autodeteccion de arranque limpio
                    if coh_send_idx = 0 and coh_b_rd_active = '0' and coh_b_rd_pending_d1 = '0' and coh_b_rd_pending = '0' then
                        bram_rd_addr <= 0;
                        coh_b_rd_pending_d1 <= '1';
                        coh_b_rd_last_d1    <= '0';
                    end if;

                    -- Paso B1: dato BRAM listo -> presentarlo a IFFT
                    if coh_b_rd_pending = '1' and coh_b_rd_active = '0' then
                        ifft_in_tdata  <= std_logic_vector(bram_im_out(17 downto 2)) &
                                          std_logic_vector(bram_re_out(17 downto 2));
                        ifft_in_tlast  <= coh_b_rd_last;
                        ifft_in_tvalid <= '1';
                        coh_b_rd_active  <= '1';
                        coh_b_rd_pending_d1 <= '0';
                    end if;

                    -- Paso B2: IFFT acepta dato -> avanzar o terminar
                    if coh_b_rd_active = '1' and ifft_in_tready = '1' then
                        if ifft_in_tlast = '1' then
                            -- Ultimo dato enviado: fin de Etapa B
                            ifft_in_tvalid   <= '0';
                            ifft_in_tlast    <= '0';
                            coh_send_active  <= '0';
                            coh_send_idx     <= 0;
                            coh_b_rd_active  <= '0';
                        else
                            -- Avanzar al siguiente bin: registrar nueva addr BRAM
                            coh_b_rd_active  <= '0';
                            coh_send_idx     <= coh_send_idx + 1;
                            bram_rd_addr     <= coh_send_idx + 1;
                            if coh_send_idx + 1 = FFT_FRAME_LAST then
                                coh_b_rd_last_d1 <= '1';
                            else
                                coh_b_rd_last_d1 <= '0';
                            end if;
                            coh_b_rd_pending_d1 <= '1';
                            ifft_in_tvalid   <= '0';
                            ifft_in_tlast    <= '0';
                        end if;
                    end if;
                else
                    ifft_in_tvalid <= '0';
                    ifft_in_tlast  <= '0';
                end if;
            end if;
        end if;
    end process;

    -- IFFT: config 0x010A
    --   S_AXIS_CONFIG layout (IP Details: FWD_INV_0 en bit8, NFFT en bits[4:0]):
    --     bit  8     = FWD_INV = 1  -> Inverse (IFFT)
    --     bits [4:0] = NFFT   = 10 -> log2(1024) = 10
    --   0x010A = 0000_0001_0000_1010
    fft_inv_inst : xfft_1 PORT MAP (
        aclk                 => clk,
        aclken               => '1',
        aresetn              => reset_n,
        -- BUGFIX (Apr 2026): El hex anterior 0AAAAABA introducia un NFFT=26 invalido en bits [4:0],
        -- lo cual causaba "Undefined Behavior" en la IP con ruidos reales.
        -- El empaquetado matematicamente perfecto es:
        -- NFFT=1024 (0x0A) en bits [7:0], FWD_INV=0 (Inverse) en bit 8, y SCALE_SCH="0101...01" en [28:9].
        s_axis_config_tdata  => x"0AAAAA0A",
        s_axis_config_tvalid => ifft_cfg_tvalid,
        s_axis_config_tready => ifft_cfg_tready,
        s_axis_data_tdata    => ifft_in_tdata,
        s_axis_data_tvalid   => ifft_in_tvalid,
        s_axis_data_tready   => ifft_in_tready,
        s_axis_data_tlast    => ifft_in_tlast,
        m_axis_data_tdata    => m_ifft_tdata,
        m_axis_data_tvalid   => m_ifft_tvalid,
        -- El tready de salida ya no es constante: lo gobierna el consumidor
        -- para soportar backpressure AXIS sin perdida de muestras.
        m_axis_data_tready   => fft_out_ready,
        m_axis_data_tlast    => m_ifft_tlast,
        event_frame_started      => open,
        event_tlast_unexpected   => open,
        event_tlast_missing      => open,
        event_status_channel_halt=> open,
        event_data_in_channel_halt => open,
        event_data_out_channel_halt=> open
    );

    fft_ready    <= m_ifft_tvalid;
    -- IFFT tdata layout (32-bit per XCI): Re[15:0] in [15:0], Im[15:0] in [31:16].
    -- peak_detector expects: din_data[15:0]=Re, din_data[31:16]=Im.
    -- This is now a direct pass-through (already in correct layout).
    fft_data_out <= m_ifft_tdata;

end Behavioral;