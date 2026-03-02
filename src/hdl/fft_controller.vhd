library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- fft_controller.vhd
--
-- Calcula la correlacion circular entre la señal RX (I/Q tras
-- el doppler_mixer) y el codigo CA local mediante FFT/IFFT:
--   1. FFT de 1024 muestras RX
--   2. FFT de 1024 chips CA local (±32767)
--   3. Multiplicacion espectral: RX * conj(CA)
--   4. IFFT → funcion de correlacion compleja (1024 puntos)
--
-- SINCRONISMO (clk_en = clk_en_fe del top, ~1.023 MHz):
--   T0 (clk_en='1'):    ca_cap <= ca_bit_local   (chip[K])
--                       loading_active activa en epoch
--   T2 (clk_en_d2='1'): captura rx_I/Q del mixer (chip[K], pipeline 2 ciclos)
--                        y ca_cap → enviados a las FFTs
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
        s_axis_data_tdata    : IN  STD_LOGIC_VECTOR(31 downto 0);
        s_axis_data_tvalid   : IN  STD_LOGIC;
        s_axis_data_tready   : OUT STD_LOGIC;
        s_axis_data_tlast    : IN  STD_LOGIC;
        m_axis_data_tdata    : OUT STD_LOGIC_VECTOR(63 downto 0);
        m_axis_data_tvalid   : OUT STD_LOGIC;
        m_axis_data_tready   : IN  STD_LOGIC;
        m_axis_data_tlast    : OUT STD_LOGIC
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
        m_axis_data_tlast    : OUT STD_LOGIC
    ); END COMPONENT;

    -- Pipeline de clk_en para alinear rx_I/Q con ca_cap
    -- El mixer tiene 2 ciclos de latencia: capturamos en clk_en_d2
    signal clk_en_d1 : std_logic := '0';
    signal clk_en_d2 : std_logic := '0';
    signal ca_cap    : std_logic := '0';  -- CA capturado en T0 (chip[K])

    -- Control de carga de 1024 muestras por epoch
    signal loading_active : std_logic := '0';
    signal sample_counter : integer range 0 to 1023 := 0;
    signal fft_busy       : std_logic := '0';

    -- Bus AXI-S hacia las FFTs (RX y CA comparten tvalid/tlast)
    signal s_fft_rx_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_fft_loc_tdata : std_logic_vector(31 downto 0) := (others => '0');
    signal s_fft_tvalid    : std_logic := '0';
    signal s_fft_tlast     : std_logic := '0';
    signal s_fft_tready    : std_logic;

    -- Salidas FFT forward
    signal m_fft_rx_tdata  : std_logic_vector(63 downto 0);
    signal m_fft_loc_tdata : std_logic_vector(63 downto 0);
    signal m_fft_rx_tvalid : std_logic;
    signal m_fft_rx_tlast  : std_logic;

    -- Multiplicacion compleja RX * conj(CA), pipeline 2 ciclos
    signal re_rx, im_rx   : signed(15 downto 0);
    signal re_loc, im_loc : signed(15 downto 0);
    signal p1, p2, p3, p4 : signed(31 downto 0);
    signal re_mult        : signed(32 downto 0);
    signal im_mult        : signed(32 downto 0);
    signal pipe_valid     : std_logic_vector(1 downto 0) := "00";
    signal pipe_last      : std_logic_vector(1 downto 0) := "00";
    signal mult_tdata     : std_logic_vector(31 downto 0);
    signal mult_tvalid    : std_logic;
    signal mult_tlast     : std_logic;

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
                clk_en_d2 <= '0';
                ca_cap    <= '0';
            else
                clk_en_d1 <= clk_en;
                clk_en_d2 <= clk_en_d1;
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
                fft_busy        <= '0';
            else
                if m_fft_rx_tlast = '1' then
                    fft_busy <= '0';
                elsif m_fft_rx_tvalid = '1' then
                    fft_busy <= '1';
                end if;

                if ca_epoch = '1' and loading_active = '0' and fft_busy = '0' then
                    loading_active <= '1';
                    sample_counter <= 0;
                end if;

                if loading_active = '1' and clk_en_d2 = '1' and s_fft_tready = '1' then
                    s_fft_rx_tdata <= rx_Q & rx_I;

                    if ca_cap = '0' then
                        s_fft_loc_tdata <= x"00007FFF";   -- +32767
                    else
                        s_fft_loc_tdata <= x"00008001";   -- -32767
                    end if;

                    if sample_counter = 1023 then
                        s_fft_tvalid   <= '1';
                        s_fft_tlast    <= '1';
                        sample_counter <= 0;
                        loading_active <= '0';
                    else
                        s_fft_tvalid   <= '1';
                        s_fft_tlast    <= '0';
                        sample_counter <= sample_counter + 1;
                    end if;
                else
                    s_fft_tvalid <= '0';
                    s_fft_tlast  <= '0';
                end if;
            end if;
        end if;
    end process;

    -- FFT canal RX
    fft_rx_inst : xfft_0 PORT MAP (
        aclk                 => clk,
        aclken               => '1',
        aresetn              => reset_n,
        s_axis_config_tdata  => x"01",
        s_axis_config_tvalid => '1',
        s_axis_data_tdata    => s_fft_rx_tdata,
        s_axis_data_tvalid   => s_fft_tvalid,
        s_axis_data_tready   => s_fft_tready,
        s_axis_data_tlast    => s_fft_tlast,
        m_axis_data_tdata    => m_fft_rx_tdata,
        m_axis_data_tvalid   => m_fft_rx_tvalid,
        m_axis_data_tready   => '1',
        m_axis_data_tlast    => m_fft_rx_tlast
    );

    -- FFT canal CA local (comparte tvalid/tlast con RX → siempre sincronizados)
    fft_loc_inst : xfft_0 PORT MAP (
        aclk                 => clk,
        aclken               => '1',
        aresetn              => reset_n,
        s_axis_config_tdata  => x"01",
        s_axis_config_tvalid => '1',
        s_axis_data_tdata    => s_fft_loc_tdata,
        s_axis_data_tvalid   => s_fft_tvalid,
        s_axis_data_tready   => open,
        s_axis_data_tlast    => s_fft_tlast,
        m_axis_data_tdata    => m_fft_loc_tdata,
        m_axis_data_tvalid   => open,
        m_axis_data_tready   => '1',
        m_axis_data_tlast    => open
    );

    -- Extraccion de bits utiles: salida FFT es 27b, tomamos bits [25:10]
    re_rx  <= signed(m_fft_rx_tdata (25 downto 10));
    im_rx  <= signed(m_fft_rx_tdata (57 downto 42));
    re_loc <= signed(m_fft_loc_tdata(25 downto 10));
    im_loc <= signed(m_fft_loc_tdata(57 downto 42));

    -- Multiplicacion compleja: RX * conj(CA) = (re_rx+j*im_rx)(re_loc-j*im_loc)
    --   re = re_rx*re_loc + im_rx*im_loc
    --   im = im_rx*re_loc - re_rx*im_loc
    process(clk)
    begin
        if rising_edge(clk) then
            p1 <= re_rx * re_loc;
            p2 <= im_rx * im_loc;
            p3 <= im_rx * re_loc;
            p4 <= re_rx * im_loc;
            re_mult    <= resize(p1, 33) + resize(p2, 33);
            im_mult    <= resize(p3, 33) - resize(p4, 33);
            pipe_valid <= pipe_valid(0 downto 0) & m_fft_rx_tvalid;
            pipe_last  <= pipe_last(0 downto 0)  & m_fft_rx_tlast;
        end if;
    end process;

    mult_tdata  <= std_logic_vector(im_mult(25 downto 10)) &
                   std_logic_vector(re_mult(25 downto 10));
    mult_tvalid <= pipe_valid(1);
    mult_tlast  <= pipe_last(1);

    -- IFFT: config 0x010A
    --   S_AXIS_CONFIG layout (IP Details: FWD_INV_0 en bit8, NFFT en bits[4:0]):
    --     bit  8     = FWD_INV = 1  → Inverse (IFFT)
    --     bits [4:0] = NFFT   = 10 → log2(1024) = 10
    --   0x010A = 0000_0001_0000_1010
    fft_inv_inst : xfft_1 PORT MAP (
        aclk                 => clk,
        aclken               => '1',
        aresetn              => reset_n,
        s_axis_config_tdata  => x"010A",
        s_axis_config_tvalid => '1',
        s_axis_config_tready => open,
        s_axis_data_tdata    => mult_tdata,
        s_axis_data_tvalid   => mult_tvalid,
        s_axis_data_tready   => open,
        s_axis_data_tlast    => mult_tlast,
        m_axis_data_tdata    => m_ifft_tdata,
        m_axis_data_tvalid   => m_ifft_tvalid,
        m_axis_data_tready   => '1',
        m_axis_data_tlast    => open
    );

    fft_ready    <= m_ifft_tvalid;
    fft_data_out <= m_ifft_tdata(57 downto 42) & m_ifft_tdata(25 downto 10);

end Behavioral;