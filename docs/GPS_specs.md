# Especificaciones Técnicas: Adquisición GPS (MAX2769C)

Este documento detalla la configuración de hardware del receptor universal GNSS **MAX2769C** utilizado en este proyecto, operando bajo el **Estado Preconfigurado 2**.

## 1. Configuración del Dispositivo (Preconfigured State 2)
El sistema se ha configurado siguiendo los parámetros del estado 2 (resaltado en la documentación técnica), optimizado para una salida de datos CMOS y frecuencias específicas.

### Parámetros Eléctricos Principales
| Parámetro | Valor |
| :--- | :--- |
| **Reference Frequency** | 16.368 MHz |
| **Main Division Ratio** | 16 |
| **Reference Division Ratio** | 1536 |
| **I and Q or I Only** | I and Q |
| **Number of IQ Bits** | 2 bits |
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

* **CONF1:** `0xA291A7` (Configuración de Gain, Filter y FCEN)
* **CONF2:** `0x055121C`
* **CONF3:** `0xEAFE1DC`
* **PLLCONF:** `0x9EC0008`
* **DIV:** `0x0C00080`
* **FDIV:** `0x8000070`
* **STRM:** `0x8000000`
* **CLK:** `0x10061B2`

## 3. Detalles de Implementación en VHDL
El diseño en la FPGA Basys 3 debe considerar:
1. **Frecuencia de Muestreo:** Basada en la salida del reloj de referencia de 16.368 MHz.
2. **Niveles Lógicos:** La salida está configurada como **CMOS**, compatible con los niveles de voltaje de los pines I/O de la Basys 3 (Artix-7).
3. **Formato de Datos:** Salida de 2 bits para I y Q, lo que requiere un procesamiento de señal digital (DSP) que maneje esta profundidad de bit.

---
*Nota: Información extraída del manual oficial de Maxim Integrated, páginas 10, 20 y 22.*