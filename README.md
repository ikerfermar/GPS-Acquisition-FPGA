# GPS Acquisition FPGA — Basys-3 (Artix-7)

Receptor GPS L1 C/A de adquisición implementado en VHDL-93 sobre la placa **Digilent Basys-3** (Artix-7 XC7A35T). El sistema captura la señal I-only `sign-magnitude` de 2 bits (`I1/I0`) del front-end **MAX2769C**, correlaciona con los 32 códigos C/A, detecta el pico de correlación en el espacio tiempo-frecuencia y reporta los resultados por UART. Incluye un generador sintético de señal para validación sin hardware RF.

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
| I1 (bit de signo, MSB) | JA pin 3 | J2 |
| I0 (bit de magnitud, LSB) | JA pin 4 | G2 |
| GND | JA pin 5 | GND |
| VCC 3.3V | JA pin 6 | VCC |

> Los pines se pueden cambiar modificando `PACKAGE_PIN` en [`src/constraints/Basys_3.xdc`](src/constraints/Basys_3.xdc).

**Parámetros del MAX2769C en State 2:**

| Parámetro | Valor |
|---|---|
| Frecuencia de muestreo (Fs) | 16.368 Msps |
| Frecuencia intermedia (IF) | 4.092 MHz |
| Bits por muestra (I only) | 2 bits sign-magnitude CMOS |
| Ancho de banda IF | 2.5 MHz |
| Nivel lógico de salida | CMOS 3.3 V |

---

## Arquitectura del sistema

```
              ┌───────────────────────────────────────────────────────────┐
              │                   top_gps_system                          │
              │                                                           │
 i1/i0_real ─►│ doble-FF ──► i1/i0_dec ─► doppler_mixer ─► rx_I / rx_Q    │
 (MAX2769C)   │  (metaestab.)   │ clk_en_fe              (NCO 100 MHz)    │
              │                 │                              │          │
  sw[5]=1 ───►│ multi_sat_rx_gen  ◄──── gps_config_pkg         │          │
  (sintético) │  (hasta 3 sats) │                              │          │
              │                 │                         fft_controller  │
              │    clk_wiz_0 ───►100 MHz                       │          │
              │    (MMCM)       │                           (FFT-RX)      │
              │                 │                           (FFT-CA)      │
              │     NCO maestro ►16.368 MHz               (IFFT corr.)    │
              │     dec x16     ►clk_en_fe (~1.023 MHz)        │          │
              │                 │                              │          │
              │    gps_ca_generator ◄── prn_local_sel    peak_detector    │
              │    (códigos G1/G2)                             │          │
              │                                    acquisition_controller │
              │                                                │          │
              │                                         uart_reporter     │
              │                                                │          │
              └────────────────────────────────────────────────┼──────────┘
                                                               │
                                                         uart_tx_pin (A18)
                                                               │
                                                        PC / gps_monitor.py
```

### Pipeline de correlación

1. **NCO maestro** — acumulador de 16 bits a 100 MHz, genera `sample_en_samp` a ~16.368 MHz (`INC = 10727`).
2. **Decimador ×16** — cada 16 pulsos del NCO maestro produce un `clk_en_fe` a ~1.023 MHz y captura la muestra `i1_dec/i0_dec`.
3. **`doppler_mixer`** — NCO de 24 bits a 100 MHz. LUTs de seno/coseno de 256 entradas × 16 bits. Convierte `I1/I0` sign-magnitude en amplitud bipolar de 4 niveles ({+2,+1,-1,-2}) y mezcla con IF + Doppler → `rx_I`, `rx_Q` (16 bits).
4. **`fft_controller`** — por cada epoch (1023 chips):
   - Carga 1024 muestras `rx_I/Q` en `xfft_0` (FFT-RX).
   - Carga 1024 chips CA locales en una segunda instancia `xfft_0` (FFT-CA).
   - Multiplica espectralmente: `RX × conj(CA)`.
   - Calcula IFFT mediante `xfft_1` → función de correlación circular (1024 puntos).
5. **`peak_detector`** — magnitud `|re| + |im|`, encuentra el máximo y calcula `noise_floor = (suma − pico) / 1024`.
6. **`acquisition_controller`** — barrido PRN × bin Doppler. Integración no coherente de `N_INT` epochs. Decisión con umbral absoluto y criterio robusto: CFAR (`accum > K_CFAR × noise_accum`) o dominancia frente al segundo mejor bin Doppler.
7. **`uart_reporter`** — transmite resultados por UART 8N1 al completar cada barrido.

---

## Estructura del proyecto

```
GPS_Acquisition_FPGA/
├── artifacts/
│   ├── .gitkeep                   # Mantiene la carpeta en git (el .bit está gitignoreado)
│   └── GPS_Acquisition_FPGA.bit  # Bitstream de salida — aparece tras ejecutar el script de build
├── docs/
│   └── GPS_specs.md              # Especificaciones MAX2769C (State 2)
├── scripts/
│   ├── gps_monitor.py            # Monitor Python en tiempo real
│   ├── program_fpga.tcl          # Script TCL: programa la FPGA por JTAG sin GUI
│   ├── run_program_fpga.ps1      # Lanzador PowerShell para programar la FPGA
│   ├── run_vivado_build.ps1      # Lanzador PowerShell del build automatizado
│   └── vivado_rebuild_and_bitstream.tcl  # Script TCL: crea proyecto + IPs + sintetiza + bitstream
├── src/
│   ├── constraints/
│   │   └── Basys_3.xdc           # Asignación de pines y timing
│   ├── hdl/
│   │   ├── gps_config_pkg.vhd    # Paquete central de parámetros
│   │   ├── top_gps_system.vhd    # Top-level (entidad raíz)
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
│   └── GPS_Acquisition_FPGA_rebuild/  # Proyecto generado por el script (no versionar)
├── vhdl_ls.toml                  # Configuración VHDL Language Server
└── README.md
```

---

## Fuentes VHDL

### `gps_config_pkg.vhd` — Paquete de configuración central

**Único fichero a editar** para cambiar el comportamiento del sistema. Todos los genéricos del top y subcomponentes se alimentan desde aquí.

| Constante | Valor por defecto | Descripción |
|---|---|---|
| `CFG_NUM_PRNS` | `4` | PRNs a barrer (1..4) para validación rápida |
| `CFG_N_INT` | `24` | Epochs de integración no coherente (más robustez para captar SAT2/SAT3 en barrido multi-sat de cuantización baja) |
| `CFG_BIN_RANGE` | `20` | Rango Doppler: −20..+20 bins (±6.4 kHz, cubre con margen SAT1/SAT2/SAT3) |
| `CFG_PEAK_THRESHOLD` | `8` | Umbral mínimo de pico por epoch (modo sintético más sensible) |
| `CFG_K_CFAR` | `2` | Factor CFAR base (best > K_CFAR*noise) |
| `CFG_MIN_MARGIN` | `1` | Margen mínimo best-second en escala UART (`m`) para reducir saltos de bin ambiguos en barrido |
| `CFG_LOCK_SNR_MIN` | `2` | SNR mínimo truncado para declarar lock (elimina locks espurios de baja energía) |
| `CFG_HYST_ACQ_SWEEPS` | `1` | Barridos buenos consecutivos para declarar lock (modo sintetico: lock rapido) |
| `CFG_PHASE_BIAS` | `0` | Ajuste de fase (chips) para alinear `ph` UART con fase física |
| `CFG_CONTINUOUS` | `true` | Barrido continuo automático |
| `CFG_CLK_FREQ` | `100_000_000` | Frecuencia de reloj en Hz |
| `CFG_BAUD_RATE` | `115_200` | Baudios UART |

**Tiempo de barrido completo:**

$$T_{barrido} = N_{PRN} \times (2 \times BIN\_RANGE + 1) \times N_{INT} \times 1\text{ ms}$$

Con los valores por defecto: $4 \times 41 \times 24 \times 1\text{ ms} \approx \mathbf{3.94\text{ s}}$

---

### `top_gps_system.vhd` — Top-level

Entidad raíz. Instancia todos los módulos y gestiona:
- Selección señal real/sintética (`sw[5]`).
- Sincronización de doble flip-flop para `i1_real/i0_real`.
- NCO maestro 16.368 MHz + decimador ×16 → `clk_en_fe`.
- Lógica de disparo de barrido por flanco de `sw[14]`.
- Multiplexado del display según `sw[14]` y `sw[15]`.

---

### `doppler_mixer.vhd` — Mezclador IF + Doppler

NCO de 24 bits corriendo a 100 MHz con tablas LUT de seno/coseno (256 entradas, 16 bits). Pipeline de 1 ciclo de latencia desde `clk_en`.

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

Los bits útiles de la salida FFT (27 bits de precisión interna) se extraen como `[26:11]` para conservar el bit de signo.

---

### `gps_ca_generator.vhd` — Generador código C/A

Implementa los registros de desplazamiento G1 (10 bits, realimentación bits 3 y 10) y G2 (10 bits, realimentación bits 2,3,6,8,9,10) según IS-GPS-200. Los taps G2 para los 32 PRNs están codificados en tablas `get_tap_a` / `get_tap_b`. `epoch_out = '1'` se emite en el chip 0 de cada nuevo epoch.

---

### `acquisition_controller.vhd` — FSM de barrido

Estados principales:

```
S_IDLE -> S_INIT_PRN -> S_INIT_BIN
   -> S_PRIME_WAIT_EPOCH -> S_PRIME_WAIT_PEAK
   -> S_WAIT_EPOCH -> S_WAIT_PEAK
   -> S_ACCUMULATE -> S_COMPARE -> S_END_PRN
       -> S_LATCH_RESULT -> S_WRITE_RAM -> S_DONE -> S_WAIT
       -> (siguiente PRN o finaliza)
```

Criterio de adquisición:

$$accum > PEAK\_THR \times N_{INT}$$

y además se exige CFAR base.
En primera adquisicion de cada PRN (sin lock previo) se exige CFAR estricto y separacion adicional (margen o dominancia frente al segundo mejor bin), o bien SNR claramente por encima del suelo de lock.
Con lock ya activo se permite criterio mas flexible para no perder lock por jitter de cuantizacion.

Salida a la RAM: PRN, bin Doppler, fase del pico (chips), SNR proxy y telemetria de depuracion (`noise`, `margin`, `flags`).

---

### `peak_detector.vhd` — Detector de pico

Magnitud aproximada: $|re| + |im|$. Maneja saturación de `signed(-32768)` para evitar overflow en la suma.

`noise_floor = (sum\_total - peak\_val) \gg 10`. El mínimo forzado a 1 se aplica en `acquisition_controller`.

---

### `uart_reporter.vhd` — Reporter UART

FSM de 9 estados que formatea y transmite un informe completo al finalizar cada barrido. Gestiona interrupciones limpias de `sw[14]` (force_reset sin cortar mensajes a mitad). Formato de salida — ver sección [Protocolo UART](#protocolo-uart).

---

### `multi_sat_rx_gen.vhd` — Generador sintético

Genera hasta 3 satélites BPSK simultáneos con PRN, Doppler y fase de código configurables desde `gps_config_pkg`. La señal compuesta se cuantiza en formato `sign-magnitude` (`I1/I0`) usando suma ponderada por satélite (ganancias configurables). Los parámetros de los tres satélites sintéticos por defecto son:

| Satélite | PRN | Doppler | Fase |
|---|---|---|---|
| SAT1 | 1 | +8 bins (+2560 Hz) | 100 chips |
| SAT2 | 2 | −12 bins (−3840 Hz) | 300 chips |
| SAT3 | 3 | +16 bins (+5120 Hz) | 600 chips |

Ganancias sintéticas por defecto (composición ponderada multi-sat):

| Satélite | Ganancia relativa |
|---|---:|
| SAT1 | 4 |
| SAT2 | 3 |
| SAT3 | 3 |

---

## IP Cores (Vivado)

| Archivo XCI | IP | Parámetros clave |
|---|---|---|
| `clk_wiz_0.xci` | Clocking Wizard (MMCM) | `clk_in1=100 MHz`, `clk_out1=100 MHz`, `locked` activo alto |
| `xfft_0.xci` | Xilinx FFT v9.x | `N=1024`, forward, Radix-2 Lite, 16b entrada, 26b salida (→ 64b bus) |
| `xfft_1.xci` | Xilinx FFT v9.x | `N=1024`, run-time configurable (FWD/INV), Pipelined, 16b entrada |

> Los IP cores se importan y sintetizan automáticamente por el script TCL (`import_ip` + `generate_target all` + síntesis OOC). No hace falta regenerarlos manualmente salvo que se modifique la configuración de un IP en el GUI de Vivado, en cuyo caso hay que volver a exportar el `.xci` a `src/ip/`.

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
constant CFG_NUM_PRNS    : integer := 4;   -- cubre SAT1..SAT3 y acelera el barrido
constant CFG_N_INT       : integer := 3;
constant CFG_BIN_RANGE   : integer := 24;  -- ±7.68 kHz, suficiente para Doppler sintético por defecto
```

---

## Interfaz de usuario (switches y LEDs)

### Switches

| Switch | Función |
|---|---|
| `sw[4:0]` | PRN manual (modo manual: PRN = sw[4:0] + 1, rango 1..32). En sintético selecciona el PRN local de correlación, con señal multi-satélite activa. |
| `sw[5]` | `0` = señal real (MAX2769C) · `1` = señal sintética |
| `sw[13:6]` | Doppler en modo manual: bin directo (complemento a 2, 1 bin = 320 Hz); en modo auto: offset fino añadido sobre el bin automático |
| `sw[14]` | `0` = modo manual · `1` = activa barrido automático (flanco subida = inicio) |
| `sw[15]` | Modo display — en auto: `0` = bin Doppler · `1` = PRN del barrido; en manual: `0` = PRN manual · `1` = fase del pico (ver tabla Display) |

### LEDs

| LED | Función |
|---|---|
| `led[0]` | Señal sintética activa (`sw[5] = 1`) |
| `led[1]` | Sistema corriendo: ON continuo mientras el código C/A avanza (epoch cada 1 ms) |
| `led[2]` | **Modo auto:** satélite adquirido · **Modo manual:** pico por encima de ruido (best > noise) con persistencia (contador >=6) |

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

Al completar cada barrido el sistema emite (formato HEX en el firmware actual):

```
\r\n
=== GPS ACQUISITION RESULTS ===\r\n
SAT 01: ACQ dop=+08 ph=064 snr=1D40 n=02B0 m=0A80 f=FF\r\n
SAT 02: ACQ dop=-0C ph=12C snr=0FB3 n=01D4 m=0460 f=FF\r\n
SAT 03: ACQ dop=+10 ph=258 snr=0CD0 n=0190 m=0318 f=FF\r\n
SAT 04: NOLOCK n=0170 m=0008 f=3F\r\n
TOTAL: 03 sats\r\n
================================\r\n
```

Significado de `f` (bitfield hex, `f[7:0]`):
- `bit0`: supera umbral absoluto `PEAK_THRESHOLD*N_INT`.
- `bit1`: pasa CFAR base (`best > K_CFAR*noise`).
- `bit2`: pasa CFAR estricto (`best > 1.5*noise`).
- `bit3`: pasa margen base (`margin >= MIN_MARGIN`, se usa como refuerzo cuando `MIN_MARGIN>0`).
- `bit4`: pasa dominancia sobre segundo mejor bin (`best > 1.125*second`).
- `bit5`: pasa suelo de SNR (`snr >= LOCK_SNR_MIN`).
- `bit6`: deteccion cruda (`raw_acq`) en ese barrido.
- `bit7`: lock final tras histeresis temporal (`prn_locked`).

Nota de lectura en barrido: un estado `f=7F` con `NOLOCK` indica que la detección cruda ya pasó (`bit6=1`) pero aún falta completar la histéresis para elevar `bit7`. Es normal en el primer barrido válido tras arrancar `sw[14]=1`.

En lock activo, el firmware aplica anti-saltos de Doppler/fase: un cambio grande solo se acepta si la mejora de `snr` es clara. Esto reduce intercambios espurios entre PRNs en barrido multi-senal.

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

- En `sw[14]=1` (auto): se habilitan los 3 satélites configurados (escenario multi-señal similar al front-end real).
- En `sw[14]=0` (manual): la señal sintética sigue siendo multi-satélite (3 satélites por defecto), y `sw[4:0]` selecciona el PRN local a correlacionar.
- El NCO de portadora del generador sintético avanza a 100 MHz (`clk_en_samp='1'`) para mantener IF=4.092 MHz coherente con `doppler_mixer`.
- En ambos caminos (real y sintético) se captura el par `I1/I0` en formato sign-magnitude y se sincroniza con doble FF antes del decimador.

1. Subir `sw[5]` → LED0 encendido.
2. Subir `sw[14]` → inicia barrido automático.
3. Esperar ~4 s (con los parámetros por defecto actuales, ~3.94 s por barrido).
4. Verificar por UART que SAT 01, SAT 02 y SAT 03 aparecen como `ACQ` con los valores de Doppler y fase configurados.
   SAT 04 debe quedar `NOLOCK` con la configuración sintética por defecto (no existe SAT4 inyectado).

Nota sobre fase reportada: el campo `ph` es el índice de máximo de una correlación circular
de 1024 puntos. Para una fase inyectada `p` puede observarse aproximadamente
`ph ~= (1023 - p + d_pipeline) mod 1024`, donde `d_pipeline` es un desfase digital fijo.

Para alinear la fase UART con la fase física, ajustar `CFG_PHASE_BIAS` (chips).

**Cálculo de Doppler esperado en la UART:**

$$f_{Doppler} = D_{bins} \times 320 \text{ Hz}$$

SAT1: +8 bins → **+2560 Hz** | SAT2: −12 bins → **−3840 Hz** | SAT3: +16 bins → **+5120 Hz**

### Cuántos satélites puede adquirir a la vez

- En la señal sintetica por defecto actual: **hasta 3 simultaneos** (SAT1, SAT2, SAT3), porque son los tres inyectados.
- Con front-end real: la arquitectura soporta multiples satelites, pero este bitstream solo barre los PRNs definidos por `CFG_NUM_PRNS` (actualmente 4). Si quieres ver mas satelites sin tocar RTL, deja `CFG_NUM_PRNS` en 32 en un bitstream "operativo" y usa los switches para modo manual/auto; asi evitas recompilar por ajustes menores de prueba.

### Modo manual con señal multi-satélite

Con `sw[5]=1` (sintética) y `sw[14]=0` (manual), la señal contiene varios satélites y `sw[4:0]` selecciona el PRN local de correlación:

| Satélite sintético | PRN | `sw[4:0]` | Doppler bins | `sw[13:6]` (C2) |
|---|---:|---|---:|---|
| SAT1 | 1 | `00000` | +8  | `00001000` |
| SAT2 | 2 | `00001` | -12 | `11110100` |
| SAT3 | 3 | `00010` | +16 | `00010000` |

Con `sw[15]=1` el display muestra la fase de pico y sirve como verificación adicional de que la correlación está activa.

---

## Instrucciones de síntesis y programación

### 1. Build automatizado (recomendado)

Requiere Vivado 2025.1 instalado. Definir la variable de entorno (una sola vez por sesión de terminal):

```powershell
$env:XILINX_VIVADO = "C:\Xilinx\2025.1\Vivado"
```

Ejecutar el script:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_vivado_build.ps1
```

El script (`scripts/vivado_rebuild_and_bitstream.tcl`) realiza automáticamente:
1. Crea un proyecto nuevo en `vivado/GPS_Acquisition_FPGA_rebuild/`
2. Importa los IP cores (`clk_wiz_0`, `xfft_0`, `xfft_1`) con `import_ip` para desbloquearlos
3. Genera los targets de IP y deja que la síntesis top-level resuelva las dependencias (sin OOC manual)
4. Ejecuta síntesis, implementación y generación de bitstream del diseño completo
5. Copia el `.bit` final a **`artifacts/GPS_Acquisition_FPGA.bit`**

Tiempo aproximado: ~10–15 min en un PC moderno.

### 2. Programar la FPGA

Con la Basys-3 conectada por USB, sin abrir Vivado:

```powershell
$env:XILINX_VIVADO = "C:\Xilinx\2025.1\Vivado"
powershell -ExecutionPolicy Bypass -File scripts/run_program_fpga.ps1
```

El script detecta automáticamente el target JTAG, carga `artifacts/GPS_Acquisition_FPGA.bit` y cierra.

Si hay más de un cable/target JTAG o más de un dispositivo, se selecciona de forma determinista (orden estable) y se muestran avisos. Para forzar una selección concreta:

```powershell
$env:PROGRAM_HW_TARGET_FILTER = "*Digilent*"
$env:PROGRAM_HW_DEVICE_FILTER = "xc7a35t*"
powershell -ExecutionPolicy Bypass -File scripts/run_program_fpga.ps1
```

### 3. Verificar funcionamiento

1. Abrir un terminal serie (PuTTY, Minicom o `gps_monitor.py`) a 115200 baud.
2. Subir `sw[14]` → barrido automático en marcha (LED1 ON continuo mientras avanza el C/A).
3. Tras ~4 s aparece el primer informe UART (con parámetros por defecto actuales).

### Recursos FPGA utilizados (estimados, XC7A35T)

| Recurso | Uso estimado |
|---|---|
| LUTs | ~8 500 / 20 800 (41 %) |
| FFs | ~4 200 / 41 600 (10 %) |
| DSPs | ~12 / 90 (13 %) |
| BRAMs | ~6 / 50 (12 %) |
| Timing | Revisar el `vivado.log`/reportes de la compilación actual (puede variar por versión y semillas) |

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
| Latencia pipeline mixer | 1 ciclo de clk_en | Captura y emisión con retardo de 1 habilitación |
| SNR proxy (N_INT=24, señal sintética) | proporcional a `N_INT` | `sum(peak_val) >> 4` |

---

## Limitaciones del Front-End (MAX2769C State 2)

Estas limitaciones condicionan lo que puede conseguir el barrido en FPGA:

1. **Cuantización a 2 bits sign-magnitude (4 niveles):** mejor que 1 bit (~1.25 dB de mejora de SNR sobre 1 bit, ~0.55 dB de pérdida frente a cuantización infinita), pero sigue siendo limitado frente a soluciones de 3+ bits. La ausencia del nivel cero es una ventaja: evita ambigüedad de signo en muestras nulas.
2. **Modo I-only:** no se dispone de canal Q analógico nativo del front-end en esta configuración (IQEN=0 en CONF2, Table 24, p.23 del datasheet).
3. **Dependencia de IF fija (4.092 MHz):** variaciones térmicas del cristal (±10 ppm según gráfica p.9 del datasheet) o errores de modelado Doppler afectan directamente la correlación.
4. **Efecto near-far en mezcla multi-satélite:** con solo 4 niveles de amplitud, el AGC interno puede saturarse ante satélites muy fuertes, enmascarando señales débiles. El parámetro GAINREF (CONF2 bits 26:15, Table 24) controla la densidad de bits de magnitud objetivo (valor óptimo 170 = 33% para ADC de 2 bits, según p.14 del datasheet).
5. **Sin estimador analógico de amplitud instantánea en FPGA:** las decisiones dependen de acumulación temporal de picos de correlación; no es posible medir potencia de señal en una sola muestra.
6. **Sensibilidad a glitches digitales en I1/I0:** transiciones espúreas en las líneas de datos degradan la correlación. En formato sign-magnitude, I1 e I0 **no son señales complementarias**; ambas pueden ser 0 o 1 simultáneamente, por lo que la mitigación se basa exclusivamente en doble flip-flop de sincronización independiente por línea.

### Estado de Mitigaciones (implementado vs pendiente)

| Limitación | Mitigación en diseño | Estado |
|---|---|---|
| Cuantización 2 bits | Integración no coherente y criterio estadístico de adquisición | Implementado |
| I-only (sin Q analógica) | Mezcla digital I/Q en FPGA con NCO y LUT seno/coseno | Implementado |
| IF fija 4.092 MHz | Barrido Doppler en adquisición | Implementado (adquisición). Seguimiento FLL/PLL: pendiente |
| Near-far multi-sat | Ganancias por satélite en generador sintético | Implementado para test. AGC/SIC real: pendiente |
| Glitches I1/I0 | Doble FF de sincronización independiente por línea (I1 e I0) | Implementado |

Respuesta a la duda I/Q: en State 2 solo se digitaliza I, y eso **sí** está contemplado en este diseño. La FPGA genera la componente en cuadratura internamente con NCO digital para mezclar y estimar Doppler; no hay cancelación analógica de imagen en el front-end.

### ¿Implementar Todo Ya?

No es la mejor estrategia implementarlo todo a la vez. Es posible, pero aumenta mucho el riesgo de romper el estado funcional actual y de volver inestable el timing.

Estrategia recomendada por fases:

1. **Fase A (bajo riesgo, alta ganancia):** umbral adaptativo por ruido, contador de validez I1/I0 por ventana, telemetría UART adicional (errores I1/I0, ruido medio, margen best/second).
2. **Fase B (riesgo medio):** integración coherente 4-10 ms opcional en modo sintético y FLL simple de frecuencia residual (solo adquisición/seguimiento grueso).
3. **Fase C (riesgo alto):** PLL de fase, AGC por SPI al MAX2769C y cancelación sucesiva de interferencia (SIC).
4. **Fase D (muy alto coste):** Hilbert digital completo para Q sintética y cadena de tracking multicanal completa.

Regla práctica: no introducir más de una mitigación estructural por iteración de bitstream. Con este flujo se identifica rápido qué mejora de verdad y se evita "mezclar" regresiones.

Estado recomendado actual: una vez estabilizado barrido sintético (3 sats recurrentes con Doppler/fase estables), sí es buen momento de pasar a mitigaciones **Fase A** (telemetría y umbral adaptativo) antes de FLL/PLL.

---

## Licencia

Proyecto académico (TFM). Uso libre para investigación y educación.
