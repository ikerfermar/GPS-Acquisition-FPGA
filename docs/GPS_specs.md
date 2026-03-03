# Especificaciones Técnicas: Adquisición GPS (MAX2769C)

Este documento detalla la configuración de hardware del receptor universal GNSS **MAX2769C** utilizado en este proyecto, operando bajo el **Estado Preconfigurado 2**.

## 1. Configuración del Dispositivo (Preconfigured State 2)
El sistema se ha configurado siguiendo los parámetros del estado 2, optimizado para una salida de datos CMOS y frecuencias específicas.

### Parámetros Eléctricos Principales
| Parámetro | Valor |
| :--- | :--- |
| **Reference Frequency** | 16.368 MHz |
| **Main Division Ratio** | 16 |
| **Reference Division Ratio** | 1536 |
| **I and Q or I Only** | I Only |
| **Number of IQ Bits** | 1 bit |
| **IQ Logic Level** | **CMOS** |
| **IF Center Frequency** | 4.092 MHz |
| **IF Filter BW** | 2.5 MHz |
| **IF Filter Order** | 5th |

### Configuración de Pines de Control (3-Wire Interface)
* **SCLK:** 1
* **DATA:** 0
* **CS:** 0

## 2. Definición de Registros (Default Settings - PGM=1)
Para el **Estado 2**, los registros internos del MAX2769C deben cargarse con los siguientes valores hexadecimales:

* **CONF1:** `0x0A291A7` (Configuración de Gain, Filter y FCEN)
* **CONF2:** `0x055121C`
* **CONF3:** `0xEAFE1DC`
* **PLLCONF:** `0x9EC0008`
* **DIV:** `0x0C00080`
* **FDIV:** `0x8000070`
* **STRM:** `0x8000000`
* **CLK:** `0x10061B2`

## 3. Conexión física al Pmod JA de la Basys-3

| Señal MAX2769C | Pin Pmod JA | Pin FPGA (XDC) | Descripción |
| :--- | :--- | :--- | :--- |
| I1 (salida activa) | Pin 3 | **J2** | Señal de 1 bit procesada por la FPGA |
| I0 (salida pasiva) | Pin 4 | **G2** | Referencia diferencial (solo conexión eléctrica) |
| GND | Pin 5 | GND | Masa común |
| VCC 3.3 V | Pin 6 | VCC | Alimentación del MAX2769C |

## 4. Detalles de Implementación en VHDL
El diseño en la FPGA Basys-3 (Artix-7) implementa:
1. **Frecuencia de Muestreo:** La señal I1 llega a 16.368 Msps. Un NCO maestro de 16 bits a 100 MHz genera un enable a ~16.368 MHz, y un decimador ×16 produce el chip-rate de ~1.023 Mcps (`clk_en_fe`).
2. **Niveles Lógicos:** La salida del MAX2769C está configurada como **CMOS 3.3 V**, compatible con los pines I/O LVCMOS33 de la Basys-3. Se aplican `false_path` en el XDC y doble flip-flop de sincronización en la FPGA.
3. **Formato de Datos:** La salida es **1 bit, solo canal I** (`i1_real : in STD_LOGIC`). El canal I0 se conecta eléctricamente pero no se procesa. La mezcla I/Q se realiza internamente en la FPGA mediante el `doppler_mixer` (NCO + LUT seno/coseno de 16 bits).

---
*Nota: Información extraída del manual oficial de Maxim Integrated, páginas 10, 20 y 22.*