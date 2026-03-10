library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- gps_config_pkg.vhd
--
-- Parametros centrales del sistema. Editar aqui para propagar
-- cambios a top_gps_system, acquisition_controller y uart_reporter.
-- Anadir al proyecto como fuente adicional (Add Sources).
--
-- Tiempo de barrido: T = NUM_PRNS * (2*BIN_RANGE+1) * N_INT * 1ms
--   Perfil actual: 4 * 41 * 24 * 1ms = 3.936 s
-- =============================================================

package gps_config_pkg is

    -- ----------------------------------------------------------------
    -- PARAMETROS DE ADQUISICION
    -- ----------------------------------------------------------------

    -- Numero de PRNs que busca el barrido automatico (1..32)
    -- Tiempo proporcional: duplicar NUM_PRNS duplica el tiempo de barrido.
    --   4  -> validacion rapida (incluye SAT1..SAT3 del generador sintetico)
    --  32  -> busca todos los satelites GPS
    constant CFG_NUM_PRNS : integer := 4;

    -- Epochs de integracion no coherente por bin Doppler
    -- Ganancia SNR = 10 x log10(N_INT) dB
    --   1  ->  0 dB, mas rapido
    --   3  -> +4.8 dB (validacion rapida sintetica)
    --   5  -> +7 dB  (sintetico robusto)
    --  10  -> +10 dB (recomendado para GPS real)
    --  20  -> +13 dB
    -- Integracion mas larga para mejorar deteccion de satelites no dominantes
    -- en mezcla multi-senal con cuantizacion limitada.
    constant CFG_N_INT : integer := 24;

    -- Rango de barrido Doppler en bins (barrido: -BIN_RANGE..+BIN_RANGE)
    -- Frecuencia cubierta = BIN_RANGE x 320 Hz
    --  20  -> +/-6400 Hz  (GPS real, satelites con velocidad orbital normal)
    --  24  -> +/-7680 Hz  (simulacion rapida, cubre SAT1..SAT3 por defecto)
    --  64  -> +/-20480 Hz (sintetico amplio)
    -- 128  -> +/-40960 Hz (rango maximo del sistema)
    -- Rango ajustado al escenario sintetico actual (dopplers +/-8, -12, +16).
    -- Menos bins reduce falsos maximos lejanos y acelera el barrido.
    constant CFG_BIN_RANGE : integer := 20;

    -- Umbral minimo del pico de correlacion por epoch.
    -- Valor normal para evitar falsas adquisiciones.
    -- Umbral absoluto relajado para mezcla multi-satelite de cuantizacion baja.
    constant CFG_PEAK_THRESHOLD : integer := 8;

    -- Factor CFAR: el pico debe ser K veces mayor que el ruido medio.
    constant CFG_K_CFAR : integer := 2;

    -- Margen minimo (best-second) para aceptar adquisicion, en la misma
    -- escala que UART 'm' (valor de 16 bits truncado [19:4]).
    -- 0x0100 filtra locks ambiguos donde d/a se intercambian entre barridos.
    -- Margen minimo moderado: filtra intercambios inestables de bin pero
    -- evita bloquear adquisiciones validas de potencia media.
    constant CFG_MIN_MARGIN : integer := 16#0002#;

    -- SNR minimo para declarar lock en escala truncada [19:4] (UART snr).
    -- Ayuda a eliminar locks espurios de baja energia (p.ej. SAT4 en test).
    constant CFG_LOCK_SNR_MIN : integer := 3;

    -- Histeresis temporal (en numero de barridos) para estabilizar lock/no-lock.
    -- En sintetico multi-sat conviene lock rapido (1 barrido bueno) para
    -- evitar alternancia prolongada NOLOCK/ACQ cuando hay bins cercanos.
    constant CFG_HYST_ACQ_SWEEPS : integer := 1;
    constant CFG_HYST_REL_SWEEPS : integer := 2;

    -- Sesgo de fase (chips) para alinear la fase UART con la fase fisica.
    -- Ajuste fino recomendado tras validar en hardware: 0..1022.
    constant CFG_PHASE_BIAS : integer := 0;

    -- Offset Doppler por switches en modo auto.
    -- FALSE recomendado para validacion sintetica robusta: evita que sw[13:6]
    -- desplace el barrido sin querer.
    constant CFG_AUTO_DOPPLER_OFFSET_ENABLE : boolean := false;

    -- ----------------------------------------------------------------
    -- BARRIDO CONTINUO
    -- ----------------------------------------------------------------
    -- TRUE  -> al terminar un barrido se relanza automaticamente.
    --         Con sw[14]=1 el sistema barre sin parar (como GPSTest).
    --         No hace falta bajar y subir sw[14].
    -- FALSE -> comportamiento original: espera un nuevo flanco en sw[14].
    constant CFG_CONTINUOUS : boolean := true;

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
    constant CFG_SAT1_GAIN   : integer := 4;     -- refuerzo SAT1 para balancear near-far
    constant CFG_SAT1_EN     : boolean := true;

    constant CFG_SAT2_PRN    : integer := 2;
    constant CFG_SAT2_DOPPLER: integer := -12;   -- bins (-12 -> -3840 Hz)
    constant CFG_SAT2_PHASE  : integer := 300;
    constant CFG_SAT2_GAIN   : integer := 3;     -- refuerzo SAT2 en mezcla multi-sat
    constant CFG_SAT2_EN     : boolean := true;

    constant CFG_SAT3_PRN    : integer := 3;
    constant CFG_SAT3_DOPPLER: integer := 16;    -- bins (+16 -> +5120 Hz)
    constant CFG_SAT3_PHASE  : integer := 600;
    constant CFG_SAT3_GAIN   : integer := 5;     -- refuerzo SAT3 para mejorar robustez de lock en barrido sintetico
    constant CFG_SAT3_EN     : boolean := true;

    -- Fase A: umbral adaptativo por ruido (dynamic_thr = base + noise>>shift)
    constant ADAPT_NOISE_SHIFT : integer := 1;

end package gps_config_pkg;