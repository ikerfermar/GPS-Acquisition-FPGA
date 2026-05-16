library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.gps_config_pkg.all;

-- =============================================================
-- doppler_mixer.vhd
--
-- Demodulador IF + compensacion Doppler para senal I-only
-- sign-magnitude (2 bits) del MAX2769C (Device State 2):
-- IF = 4.092 MHz, Fs = 16.368 Msps,
-- decimada x16 -> clk_en a ~1.023 MHz.
--
-- PIPELINE (2 ciclos de latencia respecto a clk_en):
--   T0 (clk_en='1'): captura i1_sign/i0_mag -> bpsk_reg (+/-1, +/-2)
--                    captura cos/sin del NCO   -> cos_cap / sin_cap
--   T1              : producto registrado      -> prod_I / prod_Q
--   rx_I/rx_Q       : estable desde T1 hasta proximo clk_en
--
-- NCO:
--   Avanza cada ciclo de 100 MHz con:
--     nco_inc = IF_INC + (doppler_sel + doppler_offset) * 54
--   IF_INC = round(4.092e6/100e6 * 2^24) = 686524
--   Factor 54: 1 bin Doppler (320 Hz) en acumulador 24b a 100 MHz
--              320/100e6 * 2^24 = 53.7 -> 54
--
-- OPT-LUT: cos_lut/sin_lut declaradas como signal con ram_style="block"
--   para que Vivado las infiera como BRAM (2x RAMB18) en lugar de
--   expandirlas como arboles de multiplexores (~2700 LUTs distribuidos).
-- =============================================================

entity doppler_mixer is
    Generic (
        -- A/B Fase 9A: true mantiene la ruta pipeline (baseline actual),
        -- false usa ruta directa (sin registro intermedio dsum_r).
        -- Rollback inmediato: volver a true desde gps_config_pkg/top.
        USE_INC_PIPELINE : boolean := true
    );
    Port (
        clk            : in  STD_LOGIC;
        reset_n        : in  STD_LOGIC;
        sample_en      : in  STD_LOGIC;   -- sample-rate enable (~16.368 MHz)
        clk_en         : in  STD_LOGIC;   -- chip-rate enable (~1.023 MHz, 1-of-16)
        i1_sign        : in  STD_LOGIC;
        i0_mag         : in  STD_LOGIC;
        doppler_sel    : in  STD_LOGIC_VECTOR(7 downto 0);
        doppler_offset : in  STD_LOGIC_VECTOR(7 downto 0);
        rx_I           : out STD_LOGIC_VECTOR(15 downto 0);
        rx_Q           : out STD_LOGIC_VECTOR(15 downto 0)
    );
end doppler_mixer;

architecture Behavioral of doppler_mixer is

    type lut_t is array (0 to 255) of signed(15 downto 0);

    -- OPT-LUT: Declaradas como signal (no constant) para que Vivado pueda
    -- inferirlas como BRAM con ram_style="block". Como constant, el
    -- sintetizador las expande en arboles de MUX (~2700 LUTs por las dos).
    -- El acceso sincronizado en el proceso de Etapa 1 cumple el patron
    -- de inferencia BRAM de Xilinx (lectura registrada con enable).
    signal cos_lut : lut_t := (
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

    signal sin_lut : lut_t := (
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

    -- Atributo BRAM: fuerza inferencia como bloque RAM (2x RAMB18) en lugar
    -- de distribuida. Ahorra ~2700 LUTs (arbol MUX 256:1 x 16 bits x 2 tablas).
    attribute ram_style : string;
    attribute ram_style of cos_lut : signal is "block";
    attribute ram_style of sin_lut : signal is "block";

    -- IF_INC CORREGIDO: round(4.092e6 / 100e6 * 2^24) = round(686523.68) = 686524
    -- El error anterior (686421) creaba una rotacion de 612 Hz en el dominio chip-rate
    -- (muestreo no entero 100MHz/1.023MHz = 97.75 -> residuo = (686421*97.75) mod 2^24
    --  = -10037 LSB, mapeado a 612 Hz), causando 87% de perdida coherente a 4ms.
    -- El valor correcto da residuo = +31 LSB = 1.9 Hz -> perdida despreciable (0.02%).
    -- NOTA: multi_sat_rx_gen.vhd debe usar el mismo valor para mantener la cancelacion
    -- de portadora en modo sintetico (ambos NCOs al mismo IF evitan desviacion cruzada).
    constant IF_INC : signed(24 downto 0) := to_signed(CFG_IF_NCO_INC, 25);
    constant AMP_STRONG_POS : signed(15 downto 0) := to_signed(3, 16);
    constant AMP_WEAK_POS   : signed(15 downto 0) := to_signed(1, 16);
    constant AMP_STRONG_NEG : signed(15 downto 0) := to_signed(-3, 16);
    constant AMP_WEAK_NEG   : signed(15 downto 0) := to_signed(-1, 16);

    signal phase_accum : unsigned(23 downto 0) := (others => '0');
    signal nco_inc     : signed(24 downto 0)   := (others => '0');
    signal dsum_r      : signed(8 downto 0)    := (others => '0');  -- etapa 0a pipeline

    -- Indice de fase registrado para lectura sincrona de BRAM
    signal lut_addr_r  : integer range 0 to 255 := 0;
    signal lut_rd_en   : std_logic := '0';

    signal bpsk_reg : signed(15 downto 0) := (others => '0');
    signal cos_cap  : signed(15 downto 0) := AMP_STRONG_POS;
    signal sin_cap  : signed(15 downto 0) := (others => '0');

    signal prod_I : signed(31 downto 0) := (others => '0');
    signal prod_Q : signed(31 downto 0) := (others => '0');

    -- Integrate-and-dump: accumulate sample-rate products across 16 samples/chip.
    -- Product is 32-bit (16x16 sign-magnitude x cos/sin). We extract [30:15] = 16-bit
    -- per sample, then sum 16 of those -> 20-bit max. Output [17:2] with saturation.
    signal iad_acc_I : signed(19 downto 0) := (others => '0');
    signal iad_acc_Q : signed(19 downto 0) := (others => '0');
    signal iad_out_I : signed(15 downto 0) := (others => '0');
    signal iad_out_Q : signed(15 downto 0) := (others => '0');

begin

    -- -- Etapa 0: calculo nco_inc con A/B --------------------------------
    -- Modo A (USE_INC_PIPELINE=true):
    --   Etapa 0a: suma doppler_sel + doppler_offset -> dsum_r (1 reg)
    --   Etapa 0b: nco_inc = IF_INC + dsum_r * 54   -> nco_inc (1 reg)
    -- Modo B (USE_INC_PIPELINE=false):
    --   Ruta directa: nco_inc = IF_INC + (doppler_sel+doppler_offset)*54
    --                  en el mismo ciclo (sin registro intermedio).
    -- x54 = x32 + x16 + x4 + x2 (sin DSP, solo shifts+sumas LUT)
    process(clk)
        variable dsel    : signed(8 downto 0);
        variable doffset : signed(8 downto 0);
        variable ds25    : signed(24 downto 0);
        variable dop_inc : signed(24 downto 0);
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                dsum_r  <= (others => '0');
                nco_inc <= IF_INC;
            else
                dsel   := resize(signed(doppler_sel),    9);
                doffset := resize(signed(doppler_offset), 9);
                if USE_INC_PIPELINE then
                    -- Modo A: baseline con registro intermedio (timing-friendly).
                    dsum_r <= dsel + doffset;
                    ds25   := resize(dsum_r, 25);
                else
                    -- Modo B: ruta directa para comparar latencia/area.
                    dsum_r <= dsel + doffset;
                    ds25   := resize(dsel + doffset, 25);
                end if;
                dop_inc := shift_left(ds25, 5) +   -- *32
                           shift_left(ds25, 4) +   -- *16
                           shift_left(ds25, 2) +   -- *4
                           shift_left(ds25, 1);    -- *2
                nco_inc <= IF_INC + dop_inc;
            end if;
        end if;
    end process;

    -- -- Etapa 1: NCO (100 MHz) + captura senal y portadora en sample_en --
    -- OPT-LUT: La lectura de cos_lut/sin_lut se hace de forma sincrona
    -- (patron BRAM Xilinx): registramos el indice un ciclo antes (lut_addr_r)
    -- y leemos en el flanco siguiente. Esto introduce 1 ciclo extra de latencia
    -- en cos_cap/sin_cap respecto al original, compensado con bpsk_reg_d1.
    -- I&D: capture BPSK and cos/sin at EVERY sample (16.368 MHz),
    -- not just at chip rate. This feeds the sample-rate multiplier.
    process(clk)
        variable new_phase : unsigned(24 downto 0);
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                phase_accum <= (others => '0');
                bpsk_reg    <= (others => '0');
                lut_addr_r  <= 0;
                lut_rd_en   <= '0';
                cos_cap     <= AMP_STRONG_POS;
                sin_cap     <= (others => '0');
            else
                new_phase   := ('0' & phase_accum) + unsigned(resize(nco_inc, 25));
                phase_accum <= new_phase(23 downto 0);
                lut_rd_en   <= sample_en;
                -- Registrar la direccion y datos BPSK cuando llega sample_en.
                -- cos_cap/sin_cap se actualizan el ciclo siguiente (latencia BRAM).
                if sample_en = '1' then
                    lut_addr_r <= to_integer(new_phase(23 downto 16));
                    -- Sign-magnitude MAX2769C:
                    -- i1_sign=0 positivo, i1_sign=1 negativo
                    -- i0_mag =0 debil (1), i0_mag =1 fuerte (3)
                    if i1_sign = '0' then
                        if i0_mag = '1' then
                            bpsk_reg <= AMP_STRONG_POS;   -- +3
                        else
                            bpsk_reg <= AMP_WEAK_POS;     -- +1 (15/3 = 5)
                        end if;
                    else
                        if i0_mag = '1' then
                            bpsk_reg <= AMP_STRONG_NEG;   -- -3
                        else
                            bpsk_reg <= AMP_WEAK_NEG;     -- -1 (15/3 = 5)
                        end if;
                    end if;
                end if;
                -- Lectura sincrona BRAM (1 ciclo de latencia desde lut_addr_r).
                -- lut_rd_en guarda el enable del ciclo anterior.
                if lut_rd_en = '1' then
                    cos_cap <= cos_lut(lut_addr_r);
                    sin_cap <= sin_lut(lut_addr_r);
                end if;
            end if;
        end if;
    end process;

    -- -- Etapa 2: multiplicacion registrada (sample rate) ------------------
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

    -- -- Etapa 3: Integrate-and-dump accumulator ---------------------------
    -- BUGFIX (Apr 2026): el acumulador IAD se ejecuta SOLO cuando sample_en='1'.
    --   Antes corria cada ciclo del reloj de 100 MHz (el mismo prod_I se sumaba
    --   ~6 veces por sample_en debido a la diferencia 100MHz / 16.368MHz = 6.11),
    --   produciendo overflow del acumulador de 20 bits en ~22% de los chips y
    --   saturacion al rango [17:2] en ~75% de los chips bajo senal aleatoria.
    --   Esto destruye completamente la SNR de correlacion: la salida iad_out queda
    --   pegada a +/-32767 la mayor parte del tiempo y los detalles del CA se pierden.
    --
    -- Pipeline: sample_en (T0) -> bpsk_reg (T+1) / cos_cap (T+2) -> prod_I (T+3)
    -- prod_I queda estable hasta el siguiente sample_en. Acumular prod_I[30:15]
    -- exactamente una vez por sample_en (gateado por sample_en) da 16 muestras
    -- por chip (salvo en clk_en que coincide con un sample_en y dispara el dump).
    --
    -- En el chip boundary (clk_en='1' AND sample_en='1' simultaneamente):
    --   - sum_I tiene la suma de chip K-1 (16 sample_en previos contribuyeron).
    --   - El sample_en actual es el primero de chip K -> resetear iad_acc con sample_I.
    process(clk)
        variable sample_I : signed(15 downto 0);
        variable sample_Q : signed(15 downto 0);
        variable sum_I    : signed(19 downto 0);
        variable sum_Q    : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                iad_acc_I <= (others => '0');
                iad_acc_Q <= (others => '0');
                iad_out_I <= (others => '0');
                iad_out_Q <= (others => '0');
            else
                sample_I := prod_I(30 downto 15);
                sample_Q := prod_Q(30 downto 15);

                if clk_en = '1' then
                    -- Chip boundary: latch accumulated sum with [17:2] saturation
                    sum_I := iad_acc_I;
                    sum_Q := iad_acc_Q;
                    -- Saturate [17:2]: if bits [19:16] != sign extension, clamp
                    if sum_I > to_signed(131071, 20) then      -- > 2^17-1
                        iad_out_I <= to_signed(32767, 16);
                    elsif sum_I < to_signed(-131072, 20) then  -- < -2^17
                        iad_out_I <= to_signed(-32768, 16);
                    else
                        iad_out_I <= sum_I(17 downto 2);
                    end if;
                    if sum_Q > to_signed(131071, 20) then
                        iad_out_Q <= to_signed(32767, 16);
                    elsif sum_Q < to_signed(-131072, 20) then
                        iad_out_Q <= to_signed(-32768, 16);
                    else
                        iad_out_Q <= sum_Q(17 downto 2);
                    end if;
                    -- Reset accumulator. clk_en coincide siempre con un sample_en
                    -- (es el primer sample del nuevo chip), asi que arrancamos con
                    -- la muestra actual.
                    iad_acc_I <= resize(sample_I, 20);
                    iad_acc_Q <= resize(sample_Q, 20);
                elsif sample_en = '1' then
                    -- Sample-rate accumulation: solo cuando llega una nueva muestra.
                    iad_acc_I <= iad_acc_I + resize(sample_I, 20);
                    iad_acc_Q <= iad_acc_Q + resize(sample_Q, 20);
                -- else: hold (sin sample_en y sin clk_en, no hacer nada)
                end if;
            end if;
        end if;
    end process;

    rx_I <= std_logic_vector(iad_out_I);
    rx_Q <= std_logic_vector(iad_out_Q);

end Behavioral;