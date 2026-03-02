library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- doppler_mixer.vhd
--
-- Demodulador IF + compensacion Doppler para señal 1-bit del
-- MAX2769C (Device State 2): IF = 4.092 MHz, Fs = 16.368 Msps,
-- decimada x16 → clk_en a ~1.023 MHz.
--
-- PIPELINE (2 ciclos de latencia respecto a clk_en):
--   T0 (clk_en='1'): captura i1_in → bpsk_reg (+32767 / -32767)
--                    captura cos/sin del NCO   → cos_cap / sin_cap
--   T1              : producto registrado      → prod_I / prod_Q
--   rx_I/rx_Q       : estable desde T1 hasta proximo clk_en
--
-- NCO:
--   Avanza cada ciclo de 100 MHz con:
--     nco_inc = IF_INC + (doppler_sel + doppler_offset) * 54
--   IF_INC = round(4.092e6/100e6 * 2^24) = 686421
--   Factor 54: 1 bin Doppler (320 Hz) en acumulador 24b a 100 MHz
--              320/100e6 * 2^24 = 53.7 → 54
-- =============================================================

entity doppler_mixer is
    Port (
        clk            : in  STD_LOGIC;
        reset_n        : in  STD_LOGIC;
        clk_en         : in  STD_LOGIC;
        i1_in          : in  STD_LOGIC;
        doppler_sel    : in  STD_LOGIC_VECTOR(7 downto 0);
        doppler_offset : in  STD_LOGIC_VECTOR(7 downto 0);
        rx_I           : out STD_LOGIC_VECTOR(15 downto 0);
        rx_Q           : out STD_LOGIC_VECTOR(15 downto 0)
    );
end doppler_mixer;

architecture Behavioral of doppler_mixer is

    type lut_t is array (0 to 255) of signed(15 downto 0);

    constant cos_lut : lut_t := (
        x"7FFF", x"7FF5", x"7FD8", x"7FA6", x"7F61", x"7F09", x"7E9C", x"7E1D",
        x"7D89", x"7CE3", x"7C29", x"7B5C", x"7A7C", x"7989", x"7884", x"776B",
        x"7641", x"7504", x"73B5", x"7254", x"70E2", x"6F5E", x"6DC9", x"6C23",
        x"6A6D", x"68A6", x"66CF", x"64E8", x"62F1", x"60EB", x"5ED7", x"5CB3",
        x"5A82", x"5842", x"55F5", x"539B", x"5133", x"4EBF", x"4C3F", x"49B4",
        x"471C", x"447A", x"41CE", x"3F17", x"3C56", x"398C", x"36BA", x"33DF",
        x"30FB", x"2E11", x"2B1F", x"2826", x"2528", x"2223", x"1F1A", x"1C0B",
        x"18F9", x"15E2", x"12C8", x"0FAB", x"0C8C", x"096A", x"0648", x"0324",
        x"0000", x"FCDC", x"F9B8", x"F696", x"F374", x"F055", x"ED38", x"EA1E",
        x"E707", x"E3F5", x"E0E6", x"DDDD", x"DAD8", x"D7DA", x"D4E1", x"D1EF",
        x"CF05", x"CC21", x"C946", x"C674", x"C3AA", x"C0E9", x"BE32", x"BB86",
        x"B8E4", x"B64C", x"B3C1", x"B141", x"AECD", x"AC65", x"AA0B", x"A7BE",
        x"A57E", x"A34D", x"A129", x"9F15", x"9D0F", x"9B18", x"9931", x"975A",
        x"9593", x"93DD", x"9237", x"90A2", x"8F1E", x"8DAC", x"8C4B", x"8AFC",
        x"89BF", x"8895", x"877C", x"8677", x"8584", x"84A4", x"83D7", x"831D",
        x"8277", x"81E3", x"8164", x"80F7", x"809F", x"805A", x"8028", x"800B",
        x"8001", x"800B", x"8028", x"805A", x"809F", x"80F7", x"8164", x"81E3",
        x"8277", x"831D", x"83D7", x"84A4", x"8584", x"8677", x"877C", x"8895",
        x"89BF", x"8AFC", x"8C4B", x"8DAC", x"8F1E", x"90A2", x"9237", x"93DD",
        x"9593", x"975A", x"9931", x"9B18", x"9D0F", x"9F15", x"A129", x"A34D",
        x"A57E", x"A7BE", x"AA0B", x"AC65", x"AECD", x"B141", x"B3C1", x"B64C",
        x"B8E4", x"BB86", x"BE32", x"C0E9", x"C3AA", x"C674", x"C946", x"CC21",
        x"CF05", x"D1EF", x"D4E1", x"D7DA", x"DAD8", x"DDDD", x"E0E6", x"E3F5",
        x"E707", x"EA1E", x"ED38", x"F055", x"F374", x"F696", x"F9B8", x"FCDC",
        x"0000", x"0324", x"0648", x"096A", x"0C8C", x"0FAB", x"12C8", x"15E2",
        x"18F9", x"1C0B", x"1F1A", x"2223", x"2528", x"2826", x"2B1F", x"2E11",
        x"30FB", x"33DF", x"36BA", x"398C", x"3C56", x"3F17", x"41CE", x"447A",
        x"471C", x"49B4", x"4C3F", x"4EBF", x"5133", x"539B", x"55F5", x"5842",
        x"5A82", x"5CB3", x"5ED7", x"60EB", x"62F1", x"64E8", x"66CF", x"68A6",
        x"6A6D", x"6C23", x"6DC9", x"6F5E", x"70E2", x"7254", x"73B5", x"7504",
        x"7641", x"776B", x"7884", x"7989", x"7A7C", x"7B5C", x"7C29", x"7CE3",
        x"7D89", x"7E1D", x"7E9C", x"7F09", x"7F61", x"7FA6", x"7FD8", x"7FF5"
    );

    constant sin_lut : lut_t := (
        x"0000", x"0324", x"0648", x"096A", x"0C8C", x"0FAB", x"12C8", x"15E2",
        x"18F9", x"1C0B", x"1F1A", x"2223", x"2528", x"2826", x"2B1F", x"2E11",
        x"30FB", x"33DF", x"36BA", x"398C", x"3C56", x"3F17", x"41CE", x"447A",
        x"471C", x"49B4", x"4C3F", x"4EBF", x"5133", x"539B", x"55F5", x"5842",
        x"5A82", x"5CB3", x"5ED7", x"60EB", x"62F1", x"64E8", x"66CF", x"68A6",
        x"6A6D", x"6C23", x"6DC9", x"6F5E", x"70E2", x"7254", x"73B5", x"7504",
        x"7641", x"776B", x"7884", x"7989", x"7A7C", x"7B5C", x"7C29", x"7CE3",
        x"7D89", x"7E1D", x"7E9C", x"7F09", x"7F61", x"7FA6", x"7FD8", x"7FF5",
        x"7FFF", x"7FF5", x"7FD8", x"7FA6", x"7F61", x"7F09", x"7E9C", x"7E1D",
        x"7D89", x"7CE3", x"7C29", x"7B5C", x"7A7C", x"7989", x"7884", x"776B",
        x"7641", x"7504", x"73B5", x"7254", x"70E2", x"6F5E", x"6DC9", x"6C23",
        x"6A6D", x"68A6", x"66CF", x"64E8", x"62F1", x"60EB", x"5ED7", x"5CB3",
        x"5A82", x"5842", x"55F5", x"539B", x"5133", x"4EBF", x"4C3F", x"49B4",
        x"471C", x"447A", x"41CE", x"3F17", x"3C56", x"398C", x"36BA", x"33DF",
        x"30FB", x"2E11", x"2B1F", x"2826", x"2528", x"2223", x"1F1A", x"1C0B",
        x"18F9", x"15E2", x"12C8", x"0FAB", x"0C8C", x"096A", x"0648", x"0324",
        x"0000", x"FCDC", x"F9B8", x"F696", x"F374", x"F055", x"ED38", x"EA1E",
        x"E707", x"E3F5", x"E0E6", x"DDDD", x"DAD8", x"D7DA", x"D4E1", x"D1EF",
        x"CF05", x"CC21", x"C946", x"C674", x"C3AA", x"C0E9", x"BE32", x"BB86",
        x"B8E4", x"B64C", x"B3C1", x"B141", x"AECD", x"AC65", x"AA0B", x"A7BE",
        x"A57E", x"A34D", x"A129", x"9F15", x"9D0F", x"9B18", x"9931", x"975A",
        x"9593", x"93DD", x"9237", x"90A2", x"8F1E", x"8DAC", x"8C4B", x"8AFC",
        x"89BF", x"8895", x"877C", x"8677", x"8584", x"84A4", x"83D7", x"831D",
        x"8277", x"81E3", x"8164", x"80F7", x"809F", x"805A", x"8028", x"800B",
        x"8001", x"800B", x"8028", x"805A", x"809F", x"80F7", x"8164", x"81E3",
        x"8277", x"831D", x"83D7", x"84A4", x"8584", x"8677", x"877C", x"8895",
        x"89BF", x"8AFC", x"8C4B", x"8DAC", x"8F1E", x"90A2", x"9237", x"93DD",
        x"9593", x"975A", x"9931", x"9B18", x"9D0F", x"9F15", x"A129", x"A34D",
        x"A57E", x"A7BE", x"AA0B", x"AC65", x"AECD", x"B141", x"B3C1", x"B64C",
        x"B8E4", x"BB86", x"BE32", x"C0E9", x"C3AA", x"C674", x"C946", x"CC21",
        x"CF05", x"D1EF", x"D4E1", x"D7DA", x"DAD8", x"DDDD", x"E0E6", x"E3F5",
        x"E707", x"EA1E", x"ED38", x"F055", x"F374", x"F696", x"F9B8", x"FCDC"
    );

    constant IF_INC : signed(24 downto 0) := to_signed(686421, 25);

    signal phase_accum : unsigned(23 downto 0) := (others => '0');
    signal nco_inc     : signed(24 downto 0)   := (others => '0');

    signal bpsk_reg : signed(15 downto 0) := (others => '0');
    signal cos_cap  : signed(15 downto 0) := to_signed(32767, 16);
    signal sin_cap  : signed(15 downto 0) := (others => '0');

    signal prod_I : signed(31 downto 0) := (others => '0');
    signal prod_Q : signed(31 downto 0) := (others => '0');

begin

    -- ── Etapa 0: calculo nco_inc ──────────────────────────────────────────
    process(clk)
        variable dsel    : signed(8 downto 0);
        variable doffset : signed(8 downto 0);
        variable dsum    : signed(8 downto 0);
        variable dop_inc : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                nco_inc <= IF_INC;
            else
                dsel    := resize(signed(doppler_sel),    9);
                doffset := resize(signed(doppler_offset), 9);
                dsum    := dsel + doffset;
                dop_inc := resize(dsum * to_signed(54, 7), 16);
                nco_inc <= IF_INC + resize(dop_inc, 25);
            end if;
        end if;
    end process;

    -- ── Etapa 1: NCO (100 MHz) + captura señal y portadora en clk_en ─────
    process(clk)
        variable new_phase : unsigned(24 downto 0);
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                phase_accum <= (others => '0');
                bpsk_reg    <= (others => '0');
                cos_cap     <= to_signed(32767, 16);
                sin_cap     <= (others => '0');
            else
                new_phase   := ('0' & phase_accum) + unsigned(resize(nco_inc, 25));
                phase_accum <= new_phase(23 downto 0);
                if clk_en = '1' then
                    if i1_in = '1' then
                        bpsk_reg <= to_signed( 32767, 16);
                    else
                        bpsk_reg <= to_signed(-32767, 16);
                    end if;
                    cos_cap <= cos_lut(to_integer(new_phase(23 downto 16)));
                    sin_cap <= sin_lut(to_integer(new_phase(23 downto 16)));
                end if;
            end if;
        end if;
    end process;

    -- ── Etapa 2: multiplicacion registrada ───────────────────────────────
    process(clk)
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                prod_I <= (others => '0');
                prod_Q <= (others => '0');
            else
                prod_I <=  bpsk_reg * cos_cap;
                prod_Q <= -(bpsk_reg * sin_cap);
            end if;
        end if;
    end process;

    rx_I <= std_logic_vector(prod_I(30 downto 15));
    rx_Q <= std_logic_vector(prod_Q(30 downto 15));

end Behavioral;