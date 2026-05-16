# GPS Acquisition FPGA - Basys-3 (Artix-7)

Receptor de adquisicion GPS L1 C/A implementado en VHDL-93 sobre Digilent Basys-3 (XC7A35T).

La entrada principal es I-only de 2 bits sign-magnitude desde MAX2769C (State 2). El sistema ejecuta barrido PRN x Doppler, correlacion por FFT/IFFT y reporte por UART. Incluye tambien generador sintetico interno para validacion sin RF externo.

## Documentacion

- [Arquitectura del sistema](docs/arquitectura_sistema.md)
- [Especificaciones del front-end MAX2769C](docs/GPS_specs.md)
- [Historial de cambios](CHANGELOG.md)

## Estado Actual

- Pipeline RTL funcional de extremo a extremo (CDC, decimacion, mezcla IF+Doppler, FFT/IFFT, deteccion y UART).
- Perfil actual orientado a alta dinamica (LEO): barrido rapido y adquisicion sin retardo.
- Flujo de monitorizacion mejorado: captura de log UART en vivo y reproduccion offline de trazas.

## Requisitos

- Vivado 2025.x
- Python 3.8+
- Basys-3
- Front-end MAX2769C (opcional si se usa modo sintetico o replay)

## Quick Start

### 1) Build

```powershell
cd C:\path\to\GPS_Acquisition_FPGA
powershell -ExecutionPolicy Bypass -File scripts/run_vivado_build.ps1
```

Opcionalmente puedes fijar la ruta de Vivado:

```powershell
$env:XILINX_VIVADO = "C:\AMD\Vivado\2025.1"
powershell -ExecutionPolicy Bypass -File scripts/run_vivado_build.ps1
```

### 2) Programar FPGA

```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_program_fpga.ps1
```

### 3) Monitor UART en vivo

```powershell
pip install pyserial matplotlib numpy
python scripts/gps_monitor.py --port COM3 --log logs\sesion_uart.log
```

### 4) Replay de logs (sin hardware)

```powershell
python scripts/gps_monitor.py --replay logs\PruebaLEO --speed 1.0
```

Tambien puedes revisar escenarios terrestres:

```powershell
python scripts/gps_monitor.py --replay logs\PruebaTierraAjustada --speed 1.0
python scripts/gps_monitor.py --replay logs\PruebaTierraSinAjustar --speed 1.0
```

### 5) Analisis offline de capturas .capbin

```powershell
python scripts/analyze_capbin.py captures\mi_captura.capbin
```

Si no pasas ruta, el script intenta autodetectar un `.capbin` en `captures/`.

## Perfiles de Mision: Tierra vs LEO

El proyecto se esta validando con dos perfiles de adquisicion distintos segun el entorno de operacion.

| Metrica | Perfil Tierra (alta sensibilidad) | Perfil LEO (alta dinamica) |
|---|---:|---:|
| CFG_N_INT | 20 | 2 |
| CFG_COARSE_N_INT | 10 | 1 |
| CFG_HYST_ACQ_SWEEPS | 2 | 1 |
| CFG_HYST_REL_SWEEPS | 3 | 2 |
| Integracion por bin | 80 ms | 8 ms |
| Barrido 32 PRNs (orden de magnitud) | ~22 s | ~2-5 s |
| Ganancia no coherente | +13.0 dB | +3.0 dB |
| Objetivo | maxima sensibilidad | maxima agilidad Doppler |

Notas:

- En Tierra, el modo sensible prioriza robustez frente a ruido/multipath.
- En LEO, el Doppler deriva rapido y obliga a refresco agresivo para no perder el pico.

## Parametros Actuales de la FPGA

Archivo: src/hdl/gps_config_pkg.vhd.

```vhdl
constant CFG_N_INT                  : integer := 2;
constant CFG_COARSE_N_INT           : integer := 1;
constant CFG_HYST_ACQ_SWEEPS        : integer := 1;
constant CFG_HYST_REL_SWEEPS        : integer := 2;
constant CFG_REAL_DOPPLER_BIAS_BINS : integer := -23;
```

Estos valores corresponden al perfil de alta dinamica y compensan el sesgo de oscilador medido en banco.

## Interpretacion de la fase en modo Tierra

En los logs terrestres, la fase de pico `ph` puede desplazarse de forma casi lineal entre barridos aunque el PRN siga bloqueado. Ese comportamiento es esperado y se asocia al code Doppler.

De forma aproximada:

$$
\Delta \text{chips por barrido} \approx f_{code,dopp} \cdot T_{sweep}
$$

donde $f_{code,dopp}$ es la deriva del codigo equivalente en chips/s y $T_{sweep}$ es el tiempo de barrido real. Si $T_{sweep}$ aumenta (perfil Tierra), el avance de fase por barrido tambien aumenta.

## Formato UART

Configuracion: 115200 baud, 8N1, sin flow control.

Campos principales por satelite:

- `dop`: bin Doppler firmado.
- `ph`: fase de pico (0..1023).
- `snr`: energia integrada del pico.
- `n`: ruido integrado.
- `m`: margen sobre el segundo pico.
- `f`: flags de decision.

Resumen por barrido:

- `DIAG`: salud de la captura (`iq`, `iq_er`, `agc`).
- `TOTAL`: numero de PRNs en lock.
- `SWEEP`: duracion y candidatos descartados.

## Interfaz de Usuario

### Switches

| Senal | Funcion |
|---|---|
| sw_5 (sw[5]) | 0: entrada real MAX2769C, 1: generador sintetico |

### LEDs (Basys-3)

| LED | Senal |
|---|---|
| led(0) | sw_5 |
| led(1) | pulso de actividad (epoch) |
| led(2) | estado de adquisicion |
| led(3) | fe_i1_raw |
| led(4) | fe_i0_raw |
| led(5) | i1_ff2 |
| led(6) | i0_ff2 |
| led(7) | i1_dec |
| led(8) | i0_dec |
| led(9) | clk_en_fe |
| led(10) | sample_en_samp |
| led(11) | epoch_signal |
| led(12) | sweep_running |
| led(15) | acq_done |

## Estructura del Proyecto

```text
GPS_Acquisition_FPGA/
|-- artifacts/
|-- build_logs/
|   `-- .gitkeep
|-- captures/
|   `-- .gitkeep
|-- docs/
|   |-- arquitectura_sistema.md
|   |-- GPS_specs.md
|   `-- hardware datasheets/
|-- logs/
|-- scripts/
|-- src/
|   |-- constraints/
|   |-- hdl/
|   `-- ip/
|-- vivado/
|-- CHANGELOG.md
`-- README.md
```
