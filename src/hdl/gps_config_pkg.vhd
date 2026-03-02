library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- gps_config_pkg.vhd
--
-- Parametros centrales del sistema. Editar aqui para propagar
-- cambios a top_gps_system, acquisition_controller y uart_reporter.
-- Añadir al proyecto como fuente adicional (Add Sources).
--
-- Tiempo de barrido: T = NUM_PRNS * (2*BIN_RANGE+1) * N_INT * 1ms
--   Ejemplo actual: 32 * 129 * 5 * 1ms = 20.6 s
-- =============================================================

package gps_config_pkg is

    -- ----------------------------------------------------------------
    -- PARÁMETROS DE ADQUISICIÓN
    -- ----------------------------------------------------------------

    -- Número de PRNs que busca el barrido automático (1..32)
    -- Tiempo proporcional: duplicar NUM_PRNS duplica el tiempo de barrido.
    --   8  → busca PRN 1..8  (test rápido)
    --  32  → busca todos los satélites GPS
    constant CFG_NUM_PRNS : integer := 32;

    -- Epochs de integración no coherente por bin Doppler
    -- Ganancia SNR = 10 × log10(N_INT) dB
    --   1  →  0 dB, más rápido
    --   5  → +7 dB  (recomendado para test sintético)
    --  10  → +10 dB (recomendado para GPS real)
    --  20  → +13 dB
    constant CFG_N_INT : integer := 5;

    -- Rango de barrido Doppler en bins (barrido: -BIN_RANGE..+BIN_RANGE)
    -- Frecuencia cubierta = BIN_RANGE × 320 Hz
    --  20  → ±6400 Hz  (GPS real, satélites con velocidad orbital normal)
    --  64  → ±20480 Hz (test sintético, cubre satélites con Doppler alto)
    -- 128  → ±40960 Hz (rango máximo del sistema)
    constant CFG_BIN_RANGE : integer := 64;

    -- Umbral mínimo del pico de correlación por epoch
    -- Subir si hay falsas detecciones. Bajar si no detecta señales débiles.
    constant CFG_PEAK_THRESHOLD : integer := 200;

    -- Factor CFAR: el pico debe ser K veces mayor que el ruido medio
    -- Aumentar para mayor selectividad. Reducir para señales débiles.
    constant CFG_K_CFAR : integer := 4;

    -- ----------------------------------------------------------------
    -- BARRIDO CONTINUO
    -- ----------------------------------------------------------------
    -- TRUE  → al terminar un barrido se relanza automáticamente.
    --         Con sw[14]=1 el sistema barre sin parar (como GPSTest).
    --         No hace falta bajar y subir sw[14].
    -- FALSE → comportamiento original: espera un nuevo flanco en sw[14].
    constant CFG_CONTINUOUS : boolean := true;

    -- ----------------------------------------------------------------
    -- UART
    -- ----------------------------------------------------------------
    constant CFG_CLK_FREQ  : integer := 100_000_000;  -- Hz (no cambiar salvo cambio de MMCM)
    constant CFG_BAUD_RATE : integer := 115_200;       -- 9600 | 57600 | 115200 | 230400

    -- ----------------------------------------------------------------
    -- SATÉLITES DE PRUEBA SINTÉTICOS (sw[5]=1)
    -- ----------------------------------------------------------------
    -- Doppler en bins signed (× 320 Hz/bin). Rango: -128..+127
    -- Phase en chips. Rango: 0..1022
    -- Para desactivar un satélite: sat_N_en = false

    constant CFG_SAT1_PRN    : integer := 1;     -- PRN 1..32
    constant CFG_SAT1_DOPPLER: integer := 20;    -- bins (+20 → +6400 Hz)
    constant CFG_SAT1_PHASE  : integer := 100;   -- chips
    constant CFG_SAT1_EN     : boolean := true;

    constant CFG_SAT2_PRN    : integer := 2;
    constant CFG_SAT2_DOPPLER: integer := -30;   -- bins (-30 → -9600 Hz)
    constant CFG_SAT2_PHASE  : integer := 300;
    constant CFG_SAT2_EN     : boolean := true;

    constant CFG_SAT3_PRN    : integer := 3;
    constant CFG_SAT3_DOPPLER: integer := 60;    -- bins (+60 → +19200 Hz)
    constant CFG_SAT3_PHASE  : integer := 600;
    constant CFG_SAT3_EN     : boolean := true;


end package gps_config_pkg;