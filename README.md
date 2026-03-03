# GPS Acquisition FPGA — Basys-3 (Artix-7)

Receptor GPS L1 C/A de adquisición implementado en VHDL-93 sobre la placa **Digilent Basys-3** (Artix-7 XC7A35T). El sistema captura la señal de 1 bit del front-end **MAX2769C**, correlaciona con los 32 códigos C/A, detecta el pico de correlación en el espacio tiempo-frecuencia y reporta los resultados por UART. Incluye un generador sintético de señal para validación sin hardware RF.

---

## Índice

1. [Hardware requerido](#hardware-requerido)
2. [Arquitectura del sistema](#arquitectura-del-sistema)
3. [Estructura del proyecto](#estructura-del-proyecto)
4. [Fuentes VHDL](#fuentes-vhdl)
5. [IP Cores (Vivado)](#ip-cores-vivado)
6. [Parámetros de configuración](#parámetros-de-configuración)
7. [Interfaz de usuario (switches y LEDs)](#interfaz-de-usuario-switches-y-leds)
8. [Display de 7 segmentos](#display-de-7-segmentos)
9. [Protocolo UART](#protocolo-uart)
10. [Monitor Python](#monitor-python)
11. [Señal sintética (modo test)](#señal-sintética-modo-test)
12. [Instrucciones de síntesis y programación](#instrucciones-de-síntesis-y-programación)
13. [Parámetros técnicos de referencia](#parámetros-técnicos-de-referencia)

---

## Hardware requerido

| Componente | Descripción |
|---|---|
| **Digilent Basys-3** | FPGA Artix-7 XC7A35T-1CPG236C, 100 MHz on-board |
| **MAX2769C** | Front-end GNSS universal (Maxim/Analog Devices) |
| **Cable USB-A a Micro-B** | Programación FPGA + puente UART FTDI |
| **Cable Pmod** | Conexión I1/I0 del MAX2769C al conector JA |

### Conexión MAX2769C → Basys-3

El MAX2769C debe configurarse en **Preconfigured State 2** (ver [`docs/GPS_specs.md`](docs/GPS_specs.md)):

| Señal MAX2769C | Pin Pmod JA | Pin FPGA |
|---|---|---|
| I1 (salida activa, 1 bit) | JA pin 1 | J2 |
| I0 (salida pasiva, diferencial) | JA pin 4 | G2 |
| GND | JA pin 5 | GND |
| VCC 3.3V | JA pin 6 | VCC |

> Los pines se pueden cambiar modificando `PACKAGE_PIN` en [`src/constraints/Basys_3.xdc`](src/constraints/Basys_3.xdc).

**Parámetros del MAX2769C en State 2:**

| Parámetro | Valor |
|---|---|
| Frecuencia de muestreo (Fs) | 16.368 Msps |
| Frecuencia intermedia (IF) | 4.092 MHz |
| Bits por muestra (I only) | 1 bit CMOS |
| Ancho de banda IF | 2.5 MHz |
| Nivel lógico de salida | CMOS 3.3 V |

---

## Arquitectura del sistema

```
              ┌───────────────────────────────────────────────────────────┐
              │                   top_gps_system                          │  
              │                                                           │
  i1_real ───►│ doble-FF ──► i1_dec ──► doppler_mixer ──► rx_I / rx_Q     │
  (MAX2769C)  │  (metaestab.)  │  clk_en_fe              (NCO 100 MHz)    │
              │                │                               │          │
  sw[5]=1 ───►│ multi_sat_rx_gen  ◄──── gps_config_pkg         │          │
  (sintético) │  (hasta 3 sats) │                              │          │
              │                 │                         fft_controller  │
              │    clk_wiz_0 ───►100 MHz                       │          │
              │    (MMCM)       │                         (FFT-RX)        │
              │                 │                         (FFT-CA)        │
              │     NCO maestro ►16.368 MHz               (IFFT corr.)    │
              │     dec x16     ►clk_en_fe (~1.023 MHz)        │          │
              │                 │                              │          │
              │    gps_ca_generator ◄── prn_local_sel    peak_detector    │
              │    (códigos G1/G2)                             │          │
              │                                    acquisition_controller │
              │                                               │           │
              │                                         uart_reporter     │
              │                                               │           │
              └───────────────────────────────────────────────┼───────────┘
                                                              │
                                                     uart_tx_pin (A18)
                                                              │
                                                        PC / gps_monitor.py
```

### Pipeline de correlación

1. **NCO maestro** — acumulador de 16 bits a 100 MHz, genera `sample_en_samp` a ~16.368 MHz (`INC = 10727`).
2. **Decimador ×16** — cada 16 pulsos del NCO maestro produce un `clk_en_fe` a ~1.023 MHz y captura la muestra `i1_dec`.
3. **`doppler_mixer`** — NCO de 24 bits a 100 MHz. LUTs de seno/coseno de 256 entradas × 16 bits. Mezcla BPSK con IF + Doppler → `rx_I`, `rx_Q` (16 bits).
4. **`fft_controller`** — por cada epoch (1023 chips):
   - Carga 1024 muestras `rx_I/Q` en `xfft_0` (FFT-RX).
   - Carga 1024 chips CA locales en una segunda instancia `xfft_0` (FFT-CA).
   - Multiplica espectralmente: `RX × conj(CA)`.
   - Calcula IFFT mediante `xfft_1` → función de correlación circular (1024 puntos).
5. **`peak_detector`** — magnitud `|re| + |im|`, encuentra el máximo y calcula `noise_floor = (suma − pico) / 1024`.
6. **`acquisition_controller`** — barrido PRN × bin Doppler. Integración no coherente de `N_INT` epochs. Decisión CFAR: `accum > THRESHOLD` y `accum > K_CFAR × noise_accum`.
7. **`uart_reporter`** — transmite resultados por UART 8N1 al completar cada barrido.

---

## Estructura del proyecto

```
GPS_Acquisition_FPGA/
├── docs/
│   └── GPS_specs.md              # Especificaciones MAX2769C (State 2)
├── scripts/
│   └── gps_monitor.py            # Monitor Python en tiempo real
├── src/
│   ├── constraints/
│   │   └── Basys_3.xdc           # Asignación de pines y timing
│   ├── hdl/
│   │   ├── gps_config_pkg.vhd    # Paquete central de parámetros
│   │   ├── top_gps_system.vhd    # Top-level (entidad raíz)
│   │   ├── clk_wiz_0             # (IP) MMCM 100 MHz
│   │   ├── doppler_mixer.vhd     # NCO + mezclador IF/Doppler
│   │   ├── fft_controller.vhd    # Control FFT/IFFT + mult. compleja
│   │   ├── gps_ca_generator.vhd  # Generador código C/A (G1/G2)
│   │   ├── multi_sat_rx_gen.vhd  # Generador señal sintética (≤3 sats)
│   │   ├── peak_detector.vhd     # Detector de pico + noise floor
│   │   ├── acquisition_controller.vhd  # FSM de barrido + CFAR
│   │   ├── seven_seg_controller.vhd    # Driver display 7 segmentos
│   │   ├── uart_reporter.vhd     # Formateador y transmisor UART
│   │   └── uart_tx.vhd           # Transmisor UART 8N1
│   └── ip/
│       ├── clk_wiz_0.xci         # MMCM Clocking Wizard
│       ├── xfft_0.xci            # FFT forward N=1024 (×2 instancias)
│       └── xfft_1.xci            # FFT run-time inv/fwd (IFFT)
├── vivado/
│   └── GPS_Acquisition_FPGA.xpr  # Proyecto Vivado
├── vhdl_ls.toml                  # Configuración VHDL Language Server
└── README.md
```

---

## Fuentes VHDL

### `gps_config_pkg.vhd` — Paquete de configuración central

**Único fichero a editar** para cambiar el comportamiento del sistema. Todos los genéricos del top y subcomponentes se alimentan desde aquí.

| Constante | Valor por defecto | Descripción |
|---|---|---|
| `CFG_NUM_PRNS` | `32` | PRNs a barrer (1..32) |
| `CFG_N_INT` | `5` | Epochs de integración no coherente (+7 dB SNR) |
| `CFG_BIN_RANGE` | `64` | Rango Doppler: −64..+64 bins (±20.48 kHz) |
| `CFG_PEAK_THRESHOLD` | `200` | Umbral mínimo de pico por epoch |
| `CFG_K_CFAR` | `4` | Factor CFAR (debe ser potencia de 2) |
| `CFG_CONTINUOUS` | `true` | Barrido continuo automático |
| `CFG_CLK_FREQ` | `100_000_000` | Frecuencia de reloj en Hz |
| `CFG_BAUD_RATE` | `115_200` | Baudios UART |

**Tiempo de barrido completo:**

$$T_{barrido} = N_{PRN} \times (2 \times BIN\_RANGE + 1) \times N_{INT} \times 1\text{ ms}$$

Con los valores por defecto: $32 \times 129 \times 5 \times 1\text{ ms} \approx \mathbf{20.6\text{ s}}$

---

### `top_gps_system.vhd` — Top-level

Entidad raíz. Instancia todos los módulos y gestiona:
- Selección señal real/sintética (`sw[5]`).
- Sincronización de doble flip-flop para `i1_real`.
- NCO maestro 16.368 MHz + decimador ×16 → `clk_en_fe`.
- Lógica de disparo de barrido por flanco de `sw[14]`.
- Multiplexado del display según `sw[14]` y `sw[15]`.

---

### `doppler_mixer.vhd` — Mezclador IF + Doppler

NCO de 24 bits corriendo a 100 MHz con tablas LUT de seno/coseno (256 entradas, 16 bits). Pipeline de 2 ciclos de latencia desde `clk_en`.

$$f_{NCO} = IF_{INC} + (D_{sel} + D_{offset}) \times 54$$

donde `IF_INC = 686421` (→ 4.092 MHz a 100 MHz) y el factor `54` mapea 1 bin = 320 Hz.

---

### `fft_controller.vhd` — Controlador FFT/IFFT

Gestiona el flujo AXI-Stream hacia las tres instancias FFT:

| IP | Función | Configuración |
|---|---|---|
| `xfft_0` (inst. RX) | FFT forward señal RX | `s_axis_config = 0x01` |
| `xfft_0` (inst. CA) | FFT forward código CA | `s_axis_config = 0x01` |
| `xfft_1` (inst. IFFT) | IFFT del producto espectral | `s_axis_config = 0x010A` |

La multiplicación compleja `RX × conj(CA)` usa pipeline de 2 ciclos:

$$re_{out} = re_{RX} \cdot re_{CA} + im_{RX} \cdot im_{CA}$$
$$im_{out} = im_{RX} \cdot re_{CA} - re_{RX} \cdot im_{CA}$$

Los bits útiles de la salida FFT (27 bits de precisión interna) se extraen como `[25:10]` de cada palabra de 64 bits.

---

### `gps_ca_generator.vhd` — Generador código C/A

Implementa los registros de desplazamiento G1 (10 bits, realimentación bits 3 y 10) y G2 (10 bits, realimentación bits 2,3,6,8,9,10) según IS-GPS-200. Los taps G2 para los 32 PRNs están codificados en tablas `get_tap_a` / `get_tap_b`. `epoch_out = '1'` se emite en el chip 0 de cada nuevo epoch.

---

### `acquisition_controller.vhd` — FSM de barrido

Estados principales:

```
S_IDLE → S_INIT_PRN → S_INIT_BIN → S_WAIT_EPOCH → S_WAIT_PEAK
      → S_ACCUMULATE → S_COMPARE → [S_END_PRN → confirmación]
      → S_WRITE_RAM → S_DONE → S_WAIT → (siguiente PRN o finaliza)
```

Criterio de adquisición (CFAR doble):

$$accum > PEAK\_THR^2 \times N_{INT} \quad \text{y} \quad accum > K_{CFAR} \times noise\_accum$$

Salida a la RAM: PRN, bin Doppler, fase del pico (chips), SNR proxy.

---

### `peak_detector.vhd` — Detector de pico

Magnitud aproximada: $|re| + |im|$. Maneja saturación de `signed(-32768)` para evitar overflow en la suma.

`noise_floor = (sum\_total - peak\_val) \gg 10` con mínimo forzado a 1 para garantizar que `K × noise ≠ 0` en el CFAR.

---

### `uart_reporter.vhd` — Reporter UART

FSM de 9 estados que formatea y transmite un informe completo al finalizar cada barrido. Gestiona interrupciones limpias de `sw[14]` (force_reset sin cortar mensajes a mitad). Formato de salida — ver sección [Protocolo UART](#protocolo-uart).

---

### `multi_sat_rx_gen.vhd` — Generador sintético

Genera hasta 3 satélites BPSK simultáneos con PRN, Doppler y fase de código configurables desde `gps_config_pkg`. La señal compuesta se cuantiza a 1 bit por mayoría de votos. Los parámetros de los tres satélites sintéticos por defecto son:

| Satélite | PRN | Doppler | Fase |
|---|---|---|---|
| SAT1 | 1 | +20 bins (+6400 Hz) | 100 chips |
| SAT2 | 2 | −30 bins (−9600 Hz) | 300 chips |
| SAT3 | 3 | +60 bins (+19200 Hz) | 600 chips |

---

## IP Cores (Vivado)

| Archivo XCI | IP | Parámetros clave |
|---|---|---|
| `clk_wiz_0.xci` | Clocking Wizard (MMCM) | `clk_in1=100 MHz`, `clk_out1=100 MHz`, `locked` activo alto |
| `xfft_0.xci` | Xilinx FFT v9.x | `N=1024`, forward, Radix-2 Lite, 16b entrada, 26b salida (→ 64b bus) |
| `xfft_1.xci` | Xilinx FFT v9.x | `N=1024`, run-time configurable (FWD/INV), Pipelined, 16b entrada |

> Para regenerar las IPs: en Vivado abrir el proyecto, ir a **IP Catalog → Re-customize IP** y hacer **Generate Output Products**.

---

## Parámetros de configuración

Todos en [`src/hdl/gps_config_pkg.vhd`](src/hdl/gps_config_pkg.vhd). No es necesario modificar ningún otro fichero para cambiar el comportamiento del sistema.

**Ejemplo para señal GPS real:**
```vhdl
constant CFG_NUM_PRNS    : integer := 32;
constant CFG_N_INT       : integer := 10;  -- +10 dB ganancia
constant CFG_BIN_RANGE   : integer := 20;  -- ±6.4 kHz (satélites en órbita normal)
constant CFG_K_CFAR      : integer := 4;
constant CFG_CONTINUOUS  : boolean := true;
```

**Ejemplo para validación rápida (señal sintética):**
```vhdl
constant CFG_NUM_PRNS    : integer := 8;   -- solo PRN 1..8
constant CFG_N_INT       : integer := 5;
constant CFG_BIN_RANGE   : integer := 64;  -- cubre Doppler del generador sintético
```

---

## Interfaz de usuario (switches y LEDs)

### Switches

| Switch | Función |
|---|---|
| `sw[4:0]` | PRN manual (modo manual: PRN = sw[4:0] + 1, rango 1..32) |
| `sw[5]` | `0` = señal real (MAX2769C) · `1` = señal sintética |
| `sw[13:6]` | Offset Doppler en modo manual (complemento a 2, pasos de 320 Hz) |
| `sw[14]` | `0` = modo manual · `1` = activa barrido automático (flanco subida = inicio) |
| `sw[15]` | Modo display: `0` = bin Doppler actual · `1` = PRN actual |

### LEDs

| LED | Función |
|---|---|
| `led[0]` | Señal sintética activa (`sw[5] = 1`) |
| `led[1]` | Parpadeo de epoch (~1 Hz, indica que el código C/A está corriendo) |
| `led[2]` | **Modo auto:** satélite adquirido · **Modo manual:** pico fuerte (>1000) |

---

## Display de 7 segmentos

Refresco multiplexado a ~381 Hz. Muestra un número de 2 dígitos decimales:

| `sw[14]` | `sw[15]` | Display |
|---|---|---|
| `0` (manual) | `0` | PRN manual seleccionado (`sw[4:0] + 1`) |
| `0` (manual) | `1` | Fase del pico de correlación (bits [5:0]) |
| `1` (auto) | `0` | Bin Doppler del barrido actual (bits [7:2]) |
| `1` (auto) | `1` | PRN del barrido actual |

El display se actualiza cada 250 ms para facilitar la lectura.

---

## Protocolo UART

**Configuración:** 115200 baud, 8N1, sin control de flujo.  
**Pin FPGA:** A18 (conector USB-UART FTDI integrado en la Basys-3).

Al completar cada barrido el sistema emite:

```
\r\n
=== GPS ACQUISITION RESULTS ===\r\n
SAT 01: ACQUIRED  doppler=+06400 Hz  phase=0100 chips  snr=04882\r\n
SAT 02: ACQUIRED  doppler=-09600 Hz  phase=0300 chips  snr=03541\r\n
SAT 03: ACQUIRED  doppler=+19200 Hz  phase=0600 chips  snr=02917\r\n
SAT 04: NO LOCK\r\n
...
SAT 32: NO LOCK\r\n
TOTAL: 03 satellites\r\n
================================\r\n
```

Con `CFG_CONTINUOUS = true` el informe se emite automáticamente al finalizar cada barrido sin intervención del usuario.

---

## Monitor Python

[`scripts/gps_monitor.py`](scripts/gps_monitor.py) — visualización en tiempo real de los resultados UART.

### Instalación

```bash
pip install pyserial matplotlib numpy
```

### Uso

```bash
# Detección automática de puerto
python scripts/gps_monitor.py

# Puerto explícito (Windows)
python scripts/gps_monitor.py --port COM3

# Puerto explícito (Linux/Mac)
python scripts/gps_monitor.py --port /dev/ttyUSB0

# Número de PRNs (por defecto 32)
python scripts/gps_monitor.py --port COM3 --prns 32

# Modo demo (sin hardware, datos simulados)
python scripts/gps_monitor.py --demo
```

### Ventanas

- **Ventana 1 (Live):** barras por PRN actualizadas en tiempo real. Altura = SNR proxy. Etiquetas: Doppler (kHz) y fase (chips). Verde = adquirido, naranja = escaneando, gris = no lock.
- **Ventana 2 (Debug):** historial de los últimos 30 barridos, varianza Doppler/fase por PRN, tasa de detección, duración del barrido.

> **Nota:** ejecutar desde terminal nativo, no desde Spyder ni Jupyter (requiere backend de Matplotlib interactivo).

---

## Señal sintética (modo test)

Con `sw[5] = 1` la señal real del MAX2769C se reemplaza por `multi_sat_rx_gen`. El módulo genera hasta 3 señales GPS BPSK con los parámetros de `gps_config_pkg`:

1. Subir `sw[5]` → LED0 encendido.
2. Subir `sw[14]` → inicia barrido automático.
3. Esperar ~20 s (con los parámetros por defecto).
4. Verificar por UART que SAT 01, SAT 02 y SAT 03 aparecen como `ACQUIRED` con los valores de Doppler y fase configurados.

**Cálculo de Doppler esperado en la UART:**

$$f_{Doppler} = D_{bins} \times 320 \text{ Hz}$$

SAT1: +20 bins → **+6400 Hz** | SAT2: −30 bins → **−9600 Hz** | SAT3: +60 bins → **+19200 Hz**

---

## Instrucciones de síntesis y programación

### 1. Abrir el proyecto en Vivado

```
Vivado → Open Project → vivado/GPS_Acquisition_FPGA.xpr
```

### 2. Regenerar IP cores (si es necesario)

```
IP Sources → clic derecho en cada IP → Generate Output Products → Generate
```

### 3. Síntesis e implementación

```
Flow Navigator → Run Synthesis → Run Implementation → Generate Bitstream
```

Tiempo aproximado de síntesis: ~5 min en un PC moderno.

### 4. Programar la FPGA

```
Open Hardware Manager → Open Target → Auto Connect
→ Program Device → seleccionar .bit → Program
```

### 5. Verificar funcionamiento

1. Abrir un terminal serie (PuTTY, Minicom o `gps_monitor.py`) a 115200 baud.
2. Subir `sw[14]` → barrido automático en marcha (LED1 parpadea con el epoch).
3. Tras ~20 s aparece el primer informe UART.

### Recursos FPGA utilizados (estimados, XC7A35T)

| Recurso | Uso estimado |
|---|---|
| LUTs | ~8 500 / 20 800 (41 %) |
| FFs | ~4 200 / 41 600 (10 %) |
| DSPs | ~12 / 90 (13 %) |
| BRAMs | ~6 / 50 (12 %) |
| Frecuencia máx. (Fmax) | >100 MHz (timing cerrado) |

---

## Parámetros técnicos de referencia

| Magnitud | Valor | Cálculo |
|---|---|---|
| Frecuencia de reloj | 100 MHz | MMCM desde cristal 100 MHz |
| NCO maestro INC | 10727 | `round(16.368e6 / 100e6 × 2^16)` |
| Frecuencia de muestreo efectiva | ~16.368 Msps | NCO maestro |
| Frecuencia de correlación (chip rate) | ~1.023 Mcps | ÷16 del NCO maestro |
| IF_INC (doppler_mixer) | 686421 | `round(4.092e6 / 100e6 × 2^24)` |
| Factor Doppler | 54 LSB/bin | `round(320 / 100e6 × 2^24)` |
| Resolución Doppler | 320 Hz/bin | Ancho de bin FFT de 1024 puntos a 1.023 MHz |
| Longitud FFT | 1024 puntos | = 1 epoch C/A (1023 chips + 1 pad) |
| Latencia pipeline mixer | 2 ciclos de clk_en | Captura en `clk_en_d2` |
| SNR proxy (N_INT=5, señal sintética) | ~4882 | `peak_val² × N_INT >> 8` |

---

## Licencia

Proyecto académico (TFM). Uso libre para investigación y educación.
