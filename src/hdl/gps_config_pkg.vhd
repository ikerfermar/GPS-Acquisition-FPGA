library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- gps_config_pkg.vhd
--
-- Parametros centrales del sistema.
-- Tiempo de barrido aproximado:
--   T ~= N_PRN * (ceil((2*BIN_RANGE+1)/COARSE_STEP)*COARSE_N_INT
--                + (2*REFINE_WINDOW+1)*N_INT) * COHERENT_N_MS
-- =============================================================

package gps_config_pkg is

    -- ----------------------------------------------------------------
    -- PARAMETROS DE ADQUISICION
    -- ----------------------------------------------------------------

    -- Numero de PRNs que busca el barrido automatico (1..32)
    -- Tiempo proporcional: duplicar NUM_PRNS duplica el tiempo de barrido.
    --   4  -> validacion rapida (incluye SAT1..SAT3 del generador sintetico)
    --  32  -> busca todos los satelites GPS habilitados
    constant CFG_NUM_PRNS : integer := 32;               -- search all 32 GPS PRNs (matches Python default)

    -- Mascara de PRNs habilitados en barrido (bit 0=PRN1, bit 31=PRN32).
    -- x"FFFFFFFF" habilita los 32 PRNs.
    constant CFG_PRN_ENABLE_MASK : std_logic_vector(31 downto 0) := x"FFFFFFFF"; -- all 32 PRNs enabled

    -- Epochs de integracion no coherente por bin Doppler
    -- Ganancia SNR = 10 x log10(N_INT) dB
    --   1  ->  0 dB, mas rapido
    --   3  -> +4.8 dB (validacion rapida sintetica)
    --   5  -> +7 dB  (sintetico robusto)
    --  10  -> +10 dB (recomendado para GPS real minimo)
    --  20  -> +13 dB (buen balance velocidad/SNR para GPS real con cielo parcial)
    constant CFG_N_INT : integer := 2;                   -- [epocas/bin] Maximiza velocidad LEO (~2s sweep)

    -- Integracion coherente previa (en ms) antes de IFFT/no-coherente.
    constant CFG_COHERENT_N_MS : integer := 4;           -- [ms]

    -- Timeout de epochs esperando peak_ready en acquisition_controller.
    constant CFG_PEAK_TIMEOUT_EPOCHS : integer := 16;    -- [epocas]

    -- Integracion usada en etapa gruesa del barrido Doppler.
    -- Debe ser <= CFG_N_INT para acelerar sin perder refinamiento final.
    constant CFG_COARSE_N_INT : integer := 1;            -- [epocas/bin] Barrido ultra-rapido

    -- Rango de barrido Doppler en bins (barrido: -BIN_RANGE..+BIN_RANGE RELATIVO al bias).
    -- Frecuencia absoluta cubierta = [bias - BIN_RANGE, bias + BIN_RANGE] x 320 Hz.
    constant CFG_BIN_RANGE : integer := 124;             -- [bins] Cobertura ampliada para dinamica LEO (+-39.7 kHz)

    -- Rango de barrido para el PRIMER barrido tras reset (cold start).
    -- Debe ser igual a BIN_RANGE a menos que se implemente logica de tracking
    -- para reducir el rango Doppler en barridos posteriores.
    constant CFG_INITIAL_BIN_RANGE : integer := 124;     -- [bins]

    -- Paso en bins para etapa gruesa (1=desactivado, 2=mitad de bins, etc.).
    -- Con 4ms coherentes (ancho de banda = +/-125Hz), un paso > 1 causa atenuacion Sinc.
    constant CFG_COARSE_STEP : integer := 1;             -- [bins]

    -- Ventana de refinamiento alrededor del mejor bin grueso.
    -- Se evalua en paso 1 dentro de [best-REFINE_WINDOW, best+REFINE_WINDOW].
    constant CFG_REFINE_WINDOW : integer := 2;           -- [bins]

    -- Umbral minimo del pico de correlacion por epoch.
    -- Umbral absoluto: ACCUM_THRESHOLD = PEAK_THRESHOLD * N_INT.
    -- Con N_INT=2: ACCUM_THRESHOLD = 2*2 = 4. Threshold dinamico depende de noise/2.
    -- Sirve como primer filtro para descartar bins que solo contienen ruido puro.
    constant CFG_PEAK_THRESHOLD : integer := 2;          -- [energia/epoca]

    -- Factor CFAR: umbral principal de deteccion (el pico debe superar en factor K al ruido).
    -- Para el front-end MAX2769C en State 2 se recomienda K=4.
    constant CFG_K_CFAR : integer := 4;                  -- [factor x noise]

    -- Factor CFAR para terminacion temprana del barrido coarse por PRN.
    -- Si bin_accum > EARLY_TERM_K x noise_accum, aborta el coarse y salta a refine.
    constant CFG_EARLY_TERM_K : integer := 18;           -- [factor x noise]

    -- Margen minimo (best-second) para aceptar adquisicion, en la misma
    -- escala que UART 'm' (valor de 16 bits truncado [19:4]).
    -- Margen mayor para evitar locks ambiguos en ruido real debido a
    -- multipath y reflexiones que generan picos secundarios proximos.
    constant CFG_MIN_MARGIN : integer := 2;              -- [escala {19:4}]

    -- SNR minimo para declarar lock en escala truncada [19:4] (UART snr).
    constant CFG_LOCK_SNR_MIN : integer := 5;            -- [escala {19:4}]

    -- Headroom adicional para el criterio de primer lock basado en SNR.
    -- Valor menor reduce falsos NOLOCK en RF real debil sin relajar CFAR base.
    constant CFG_LOCK_SNR_HEADROOM : integer := 6;       -- [escala {19:4}]

    -- Histeresis temporal (en numero de barridos) para estabilizar lock/no-lock.
    -- ACQ=2: requiere 2 barridos consecutivos con deteccion para declarar lock.
    -- REL=3: requiere 3 fallos consecutivos para soltar lock.
    constant CFG_HYST_ACQ_SWEEPS : integer := 1;         -- [barridos] Zero-delay acq para alta dinamica
    constant CFG_HYST_REL_SWEEPS : integer := 2;         -- [barridos] Margen minimo para soltar

    -- Arbitraje global por sweep.
    -- TOP_N limita promocion simultanea por calidad para evitar flood.
    constant CFG_GLOBAL_TOP_N       : integer := 6;      -- [PRNs/sweep]
    constant CFG_SCORE_CAND_MIN     : integer := 24;     -- score minimo para entrar a ranking global
    constant CFG_SCORE_PROMOTE_MIN  : integer := 40;     -- score minimo para candidate->confirmed
    constant CFG_SCORE_HOLD_MIN     : integer := 72;     -- score minimo para sostener lock sin release

    -- Sesgo de fase (chips) para alinear la fase UART con la fase fisica.
    -- Ajuste fino recomendado tras validar en hardware: 0..1022.
    constant CFG_PHASE_BIAS : integer := 0;              -- [chips]

    -- ----------------------------------------------------------------
    -- UART
    -- ----------------------------------------------------------------
    constant CFG_CLK_FREQ  : integer := 100_000_000;  -- Hz (no cambiar salvo cambio de MMCM)
    constant CFG_BAUD_RATE : integer := 115_200;       -- 9600 | 57600 | 115200 | 230400

    -- ----------------------------------------------------------------
    -- SATELITES DE PRUEBA SINTETICOS (sw[5]=1)
    -- ----------------------------------------------------------------
    -- Doppler en bins signed (x 320 Hz/bin). Rango: -128..+127
    -- Phase en chips. Rango: 0..1022
    -- Para desactivar un satelite: sat_N_en = false

    constant CFG_SAT1_PRN    : integer := 1;     -- PRN 1..32
    constant CFG_SAT1_DOPPLER: integer := 8;     -- bins (+8 -> +2560 Hz)
    constant CFG_SAT1_PHASE  : integer := 100;   -- chips
    constant CFG_SAT1_GAIN   : integer := 5;
    constant CFG_SAT1_EN     : boolean := true;

    constant CFG_SAT2_PRN    : integer := 2;
    constant CFG_SAT2_DOPPLER: integer := -12;   -- bins (-12 -> -3840 Hz)
    constant CFG_SAT2_PHASE  : integer := 300;   -- chips
    constant CFG_SAT2_GAIN   : integer := 5;
    constant CFG_SAT2_EN     : boolean := true;

    constant CFG_SAT3_PRN    : integer := 3;
    constant CFG_SAT3_DOPPLER: integer := 16;    -- bins (+16 -> +5120 Hz)
    constant CFG_SAT3_PHASE  : integer := 600;   -- chips
    constant CFG_SAT3_GAIN   : integer := 5;
    constant CFG_SAT3_EN     : boolean := true;

    constant ADAPT_NOISE_SHIFT : integer := 1;           -- [bits]  dynamic_thr += noise >> ADAPT_NOISE_SHIFT

    -- Sesgo Doppler inicial [bins]. Permite compensar de base el offset
    -- del oscilador de cristal (ej. cristal +13ppm = +64 bins).
    constant CFG_REAL_DOPPLER_BIAS_BINS : integer := -23;  -- [bins] Compensacion TCXO (-4.7 ppm)


    -- ----------------------------------------------------------------
    -- PARAMETROS DE HARDWARE (evitar literales dispersos en el RTL)
    -- ----------------------------------------------------------------

    -- Incremento NCO para IF = 4.092 MHz a 100 MHz: round(4.092e6/100e6 * 2^24)
    constant CFG_IF_NCO_INC : integer := 686524;

    -- Ciclos de retardo del LED de lock (~50 ms a 100 MHz)
    constant CFG_LED_HOLD_CYCLES : integer := 5_000_000;

    -- ----------------------------------------------------------------
    -- ASCII / nibble-to-hex helper
    -- Shared by uart_reporter.
    -- ----------------------------------------------------------------
    constant CFG_ASCII_0          : integer := 48;   -- character '0'
    constant CFG_ASCII_A_MINUS_10 : integer := 55;   -- character 'A' - 10

    function n2h(nib : std_logic_vector(3 downto 0)) return std_logic_vector;

end package gps_config_pkg;

package body gps_config_pkg is

    function n2h(nib : std_logic_vector(3 downto 0)) return std_logic_vector is
        variable v : unsigned(3 downto 0);
    begin
        v := unsigned(nib);
        if v < 10 then
            return std_logic_vector(to_unsigned(CFG_ASCII_0 + to_integer(v), 8));
        else
            return std_logic_vector(to_unsigned(CFG_ASCII_A_MINUS_10 + to_integer(v), 8));
        end if;
    end function n2h;

end package body gps_config_pkg;