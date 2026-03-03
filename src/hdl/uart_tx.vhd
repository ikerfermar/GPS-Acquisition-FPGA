library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- uart_tx.vhd
--
-- Transmisor UART simple, 8N1.
-- Parametrizable via generic CLK_FREQ y BAUD_RATE.
-- Default: 100 MHz / 115200 baud -> divisor = 868.
--
-- Interface:
--   tx_data  : byte a transmitir
--   tx_start : pulso de 1 ciclo para iniciar transmision
--   tx_busy  : '1' mientras transmite (no acepta nuevos datos)
--   tx_pin   : linea serie hacia el pin USB-UART de la Basys-3
--
-- La Basys-3 tiene un puente FTDI FT2232HQ conectado al pin
-- JA[0] (o directamente al pad USB). Revisar el XDC:
--   set_property PACKAGE_PIN A18 [get_ports uart_tx_pin]  -- FTDI RXD
-- =============================================================

entity uart_tx is
    Generic (
        CLK_FREQ  : integer := 100_000_000;
        BAUD_RATE : integer := 115_200
    );
    Port (
        clk      : in  STD_LOGIC;
        reset_n  : in  STD_LOGIC;
        tx_data  : in  STD_LOGIC_VECTOR(7 downto 0);
        tx_start : in  STD_LOGIC;
        tx_busy  : out STD_LOGIC;
        tx_pin   : out STD_LOGIC
    );
end uart_tx;

architecture Behavioral of uart_tx is

    constant CLKS_PER_BIT : integer := CLK_FREQ / BAUD_RATE;  -- 868 @ 100MHz/115200

    type t_state is (S_IDLE, S_START, S_DATA, S_STOP);
    signal state    : t_state := S_IDLE;

    signal clk_cnt  : integer range 0 to CLKS_PER_BIT - 1 := 0;
    signal bit_idx  : integer range 0 to 7 := 0;
    signal tx_reg   : std_logic_vector(7 downto 0) := (others => '0');
    signal tx_out   : std_logic := '1';

begin

    tx_pin  <= tx_out;

    process(clk)
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                state   <= S_IDLE;
                tx_out  <= '1';
                tx_busy <= '0';
                clk_cnt <= 0;
                bit_idx <= 0;
            else
                case state is

                    when S_IDLE =>
                        tx_out  <= '1';
                        tx_busy <= '0';
                        clk_cnt <= 0;
                        bit_idx <= 0;
                        if tx_start = '1' then
                            tx_reg  <= tx_data;
                            tx_busy <= '1';
                            state   <= S_START;
                        end if;

                    when S_START =>
                        tx_out <= '0';   -- bit de start
                        if clk_cnt = CLKS_PER_BIT - 1 then
                            clk_cnt <= 0;
                            state   <= S_DATA;
                        else
                            clk_cnt <= clk_cnt + 1;
                        end if;

                    when S_DATA =>
                        tx_out <= tx_reg(bit_idx);
                        if clk_cnt = CLKS_PER_BIT - 1 then
                            clk_cnt <= 0;
                            if bit_idx = 7 then
                                bit_idx <= 0;
                                state   <= S_STOP;
                            else
                                bit_idx <= bit_idx + 1;
                            end if;
                        else
                            clk_cnt <= clk_cnt + 1;
                        end if;

                    when S_STOP =>
                        tx_out <= '1';   -- bit de stop
                        if clk_cnt = CLKS_PER_BIT - 1 then
                            clk_cnt <= 0;
                            tx_busy <= '0';
                            state   <= S_IDLE;
                        else
                            clk_cnt <= clk_cnt + 1;
                        end if;

                end case;
            end if;
        end if;
    end process;

end Behavioral;