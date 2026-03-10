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
    Port (
        clk          : in  STD_LOGIC;
        clk_en       : in  STD_LOGIC;
        reset_n      : in  STD_LOGIC;
        rx_I         : in  STD_LOGIC_VECTOR(15 downto 0);
        rx_Q         : in  STD_LOGIC_VECTOR(15 downto 0);
        ca_bit_local : in  STD_LOGIC;
        ca_epoch     : in  STD_LOGIC;
        fft_ready    : out STD_LOGIC;
        fft_data_out : out STD_LOGIC_VECTOR(31 downto 0)
    );
end fft_controller;

architecture Behavioral of fft_controller is

    COMPONENT xfft_0 PORT (
        aclk                 : IN  STD_LOGIC;
        aclken               : IN  STD_LOGIC;
        aresetn              : IN  STD_LOGIC;
        s_axis_config_tdata  : IN  STD_LOGIC_VECTOR(7 downto 0);
        s_axis_config_tvalid : IN  STD_LOGIC;
        s_axis_config_tready : OUT STD_LOGIC;
        s_axis_data_tdata    : IN  STD_LOGIC_VECTOR(31 downto 0);
        s_axis_data_tvalid   : IN  STD_LOGIC;
        s_axis_data_tready   : OUT STD_LOGIC;
        s_axis_data_tlast    : IN  STD_LOGIC;
        m_axis_data_tdata    : OUT STD_LOGIC_VECTOR(63 downto 0);
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

    COMPONENT xfft_1 PORT (
        aclk                 : IN  STD_LOGIC;
        aclken               : IN  STD_LOGIC;
        aresetn              : IN  STD_LOGIC;
        s_axis_config_tdata  : IN  STD_LOGIC_VECTOR(15 downto 0);
        s_axis_config_tvalid : IN  STD_LOGIC;
        s_axis_config_tready : OUT STD_LOGIC;
        s_axis_data_tdata    : IN  STD_LOGIC_VECTOR(31 downto 0);
        s_axis_data_tvalid   : IN  STD_LOGIC;
        s_axis_data_tready   : OUT STD_LOGIC;
        s_axis_data_tlast    : IN  STD_LOGIC;
        m_axis_data_tdata    : OUT STD_LOGIC_VECTOR(63 downto 0);
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
    signal sample_counter : integer range 0 to 1023 := 0;
    signal fft_busy       : std_logic := '0';

    -- Bus AXI-S hacia las FFTs (RX y CA comparten tvalid/tlast)
    signal s_fft_rx_tdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_fft_loc_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_fft_tvalid     : std_logic := '0';
    signal s_fft_tlast      : std_logic := '0';
    signal sample_pending   : std_logic := '0';
    signal pending_rx_tdata : std_logic_vector(31 downto 0) := (others => '0');
    signal pending_loc_tdata: std_logic_vector(31 downto 0) := (others => '0');
    signal pending_tlast    : std_logic := '0';
    signal s_fft_rx_tready  : std_logic;  -- FIX: tready individual para cada FFT
    signal s_fft_loc_tready : std_logic;
    signal s_fft_tready     : std_logic;  -- AND de ambos tready
    signal m_ifft_tlast     : std_logic;  -- FIX: fft_busy no se borra hasta que IFFT termina

    -- Salidas FFT forward
    signal m_fft_rx_tdata  : std_logic_vector(63 downto 0);
    signal m_fft_loc_tdata : std_logic_vector(63 downto 0);
    signal m_fft_rx_tvalid : std_logic;
    signal m_fft_loc_tvalid : std_logic;
    signal m_fft_rx_tlast  : std_logic;
    signal m_fft_loc_tlast  : std_logic;

    signal spec_tvalid : std_logic;
    signal spec_tlast  : std_logic;

    -- Multiplicacion compleja RX * conj(CA), pipeline 2 ciclos
    signal re_rx, im_rx   : signed(15 downto 0);
    signal re_loc, im_loc : signed(15 downto 0);
    signal p1, p2, p3, p4 : signed(31 downto 0);
    signal re_mult        : signed(26 downto 0);
    signal im_mult        : signed(26 downto 0);
    signal pipe_valid     : std_logic_vector(1 downto 0) := "00";
    signal pipe_last      : std_logic_vector(1 downto 0) := "00";
    signal mult_tdata     : std_logic_vector(31 downto 0);
    signal mult_tvalid    : std_logic;
    signal mult_tlast     : std_logic;

    -- Handshake robusto hacia IFFT
    signal ifft_cfg_sent   : std_logic := '0';
    signal ifft_cfg_tready : std_logic;
    signal ifft_cfg_tvalid : std_logic := '0';

    signal ifft_in_tdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal ifft_in_tlast   : std_logic := '0';
    signal ifft_in_tvalid  : std_logic := '0';
    signal ifft_in_tready  : std_logic;

    -- Salida IFFT
    signal m_ifft_tdata  : std_logic_vector(63 downto 0);
    signal m_ifft_tvalid : std_logic;

begin

    -- Pipeline clk_en: captura ca_cap en T0, rx_I/Q en T2
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
                s_fft_tvalid    <= '0';
                s_fft_tlast     <= '0';
                s_fft_rx_tdata  <= (others => '0');
                s_fft_loc_tdata <= (others => '0');
                sample_pending  <= '0';
                pending_rx_tdata  <= (others => '0');
                pending_loc_tdata <= (others => '0');
                pending_tlast     <= '0';
                fft_busy        <= '0';
            else
                s_fft_tvalid <= '0';
                s_fft_tlast  <= '0';

                -- fft_busy se pone a '1' al iniciar carga para cerrar la ventana
                -- de carrera entre epochs.
                -- Debe liberarse solo cuando termina la IFFT completa para evitar
                -- solape de frames entre epochs.
                if m_ifft_tlast = '1' then
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
                        pending_loc_tdata <= x"00007FFF";   -- +32767
                    else
                        pending_loc_tdata <= x"00008001";   -- -32767
                    end if;

                    if sample_counter = 1023 then
                        pending_tlast <= '1';
                        sample_counter <= 0;
                        loading_active <= '0';
                    else
                        pending_tlast <= '0';
                        sample_counter <= sample_counter + 1;
                    end if;
                    sample_pending <= '1';
                end if;

                if sample_pending = '1' and s_fft_tready = '1' then
                    s_fft_rx_tdata  <= pending_rx_tdata;
                    s_fft_loc_tdata <= pending_loc_tdata;
                    s_fft_tvalid    <= '1';
                    s_fft_tlast     <= pending_tlast;
                    sample_pending  <= '0';
                end if;
            end if;
        end if;
    end process;

    -- FIX: tready combinado: la carga solo avanza si AMBAS FFTs estan listas
    s_fft_tready <= s_fft_rx_tready and s_fft_loc_tready;

    -- FFT canal RX
    fft_rx_inst : xfft_0 PORT MAP (
        aclk                 => clk,
        aclken               => '1',
        aresetn              => reset_n,
        s_axis_config_tdata  => x"01",
        s_axis_config_tvalid => '1',
        s_axis_config_tready => open,
        s_axis_data_tdata    => s_fft_rx_tdata,
        s_axis_data_tvalid   => s_fft_tvalid,
        s_axis_data_tready   => s_fft_rx_tready,
        s_axis_data_tlast    => s_fft_tlast,
        m_axis_data_tdata    => m_fft_rx_tdata,
        m_axis_data_tvalid   => m_fft_rx_tvalid,
        m_axis_data_tready   => '1',
        m_axis_data_tlast    => m_fft_rx_tlast,
        event_frame_started      => open,
        event_tlast_unexpected   => open,
        event_tlast_missing      => open,
        event_status_channel_halt=> open,
        event_data_in_channel_halt => open,
        event_data_out_channel_halt=> open
    );

    -- FFT canal CA local (comparte tvalid/tlast con RX - FIX: tready conectado)
    fft_loc_inst : xfft_0 PORT MAP (
        aclk                 => clk,
        aclken               => '1',
        aresetn              => reset_n,
        s_axis_config_tdata  => x"01",
        s_axis_config_tvalid => '1',
        s_axis_config_tready => open,
        s_axis_data_tdata    => s_fft_loc_tdata,
        s_axis_data_tvalid   => s_fft_tvalid,
        s_axis_data_tready   => s_fft_loc_tready,
        s_axis_data_tlast    => s_fft_tlast,
        m_axis_data_tdata    => m_fft_loc_tdata,
        m_axis_data_tvalid   => m_fft_loc_tvalid,
        m_axis_data_tready   => '1',
        m_axis_data_tlast    => m_fft_loc_tlast,
        event_frame_started      => open,
        event_tlast_unexpected   => open,
        event_tlast_missing      => open,
        event_status_channel_halt=> open,
        event_data_in_channel_halt => open,
        event_data_out_channel_halt=> open
    );

    -- Extraccion de bits utiles: salida FFT es 27b (unscaled).
    -- Real:  tdata[26:0]  -> MSB en bit 26. Tomamos [26:11] (16 MSBs, incluye signo).
    -- Imag:  tdata[58:32] -> MSB en bit 58. Tomamos [58:43] (16 MSBs, incluye signo).
    -- Bug previo: [25:10]/[57:42] descartaba el bit de signo -> valores negativos grandes
    -- se interpretaban como positivos, corrompiendo la multiplicacion espectral.
    re_rx  <= signed(m_fft_rx_tdata (26 downto 11));
    im_rx  <= signed(m_fft_rx_tdata (58 downto 43));
    re_loc <= signed(m_fft_loc_tdata(26 downto 11));
    im_loc <= signed(m_fft_loc_tdata(58 downto 43));

    -- Solo multiplicar cuando ambos streams (RX y CA) son validos en el mismo ciclo.
    spec_tvalid <= m_fft_rx_tvalid and m_fft_loc_tvalid;
    -- TLAST se toma del canal RX para no perder cierre de frame por
    -- desalineacion puntual entre tlast de ambos FFT forward.
    spec_tlast  <= m_fft_rx_tlast;

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
            re_mult  <= re_acc_v(26 downto 0);
            im_mult  <= im_acc_v(26 downto 0);
            pipe_valid <= pipe_valid(0 downto 0) & spec_tvalid;
            pipe_last  <= pipe_last(0 downto 0)  & spec_tlast;
        end if;
    end process;

    -- Seleccion de 16 bits con signo desde acumuladores de 33 bits.
    -- Evita truncado temprano que corrompia bins fuertes.
    mult_tdata  <= std_logic_vector(im_mult(26 downto 11)) &
                   std_logic_vector(re_mult(26 downto 11));
    mult_tvalid <= pipe_valid(1);
    mult_tlast  <= pipe_last(1);

    -- Handshake AXIS robusto para IFFT:
    -- 1) Envia config una sola vez y espera tready
    -- 2) Retiene cada muestra hasta handshake (tvalid&tready)
    process(clk)
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                ifft_cfg_sent  <= '0';
                ifft_cfg_tvalid <= '0';
                ifft_in_tvalid <= '0';
                ifft_in_tdata  <= (others => '0');
                ifft_in_tlast  <= '0';
            else
                if ifft_cfg_sent = '0' then
                    ifft_cfg_tvalid <= '1';
                    if ifft_cfg_tvalid = '1' and ifft_cfg_tready = '1' then
                        ifft_cfg_sent   <= '1';
                        ifft_cfg_tvalid <= '0';
                    end if;
                else
                    ifft_cfg_tvalid <= '0';
                end if;

                -- Permite throughput 1 muestra/ciclo si la IFFT esta lista.
                -- Bug previo: se liberaba TVALID un ciclo y la muestra de ese
                -- mismo ciclo se perdia, descartando aproximadamente la mitad
                -- de los bins espectrales.
                if ifft_in_tvalid = '1' then
                    if ifft_in_tready = '1' then
                        if mult_tvalid = '1' then
                            ifft_in_tdata  <= mult_tdata;
                            ifft_in_tlast  <= mult_tlast;
                            ifft_in_tvalid <= '1';
                        else
                            ifft_in_tvalid <= '0';
                        end if;
                    end if;
                elsif mult_tvalid = '1' then
                    ifft_in_tdata  <= mult_tdata;
                    ifft_in_tlast  <= mult_tlast;
                    ifft_in_tvalid <= '1';
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
        s_axis_config_tdata  => x"010A",
        s_axis_config_tvalid => ifft_cfg_tvalid,
        s_axis_config_tready => ifft_cfg_tready,
        s_axis_data_tdata    => ifft_in_tdata,
        s_axis_data_tvalid   => ifft_in_tvalid,
        s_axis_data_tready   => ifft_in_tready,
        s_axis_data_tlast    => ifft_in_tlast,
        m_axis_data_tdata    => m_ifft_tdata,
        m_axis_data_tvalid   => m_ifft_tvalid,
        m_axis_data_tready   => '1',
        m_axis_data_tlast    => m_ifft_tlast,
        event_frame_started      => open,
        event_tlast_unexpected   => open,
        event_tlast_missing      => open,
        event_status_channel_halt=> open,
        event_data_in_channel_halt => open,
        event_data_out_channel_halt=> open
    );

    fft_ready    <= m_ifft_tvalid;
    -- Igual que FFT forward: real en [26:11], imag en [58:43] del tdata de 64b.
    fft_data_out <= m_ifft_tdata(58 downto 43) & m_ifft_tdata(26 downto 11);

end Behavioral;