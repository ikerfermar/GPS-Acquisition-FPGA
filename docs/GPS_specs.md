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
| **Number of IQ Bits** | 2 bits |
| **IQ Logic Level** | CMOS |
| **IF Center Frequency** | 4.092 MHz |
| **IF Filter BW** | 2.5 MHz |
| **IF Filter Order** | 5th |

### Configuración de Pines de Control (3-Wire Interface)
* **SCLK:** 0
* **DATA:** 1
* **CS:** 0

## 2. Definición de Registros (Default Settings - PGM=1)
Para el **Estado 2**, los registros internos del MAX2769C deben cargarse con los siguientes valores hexadecimales (Table 22, p.22 del datasheet):

* **CONF1:** `A2919A7`
* **CONF2:** `055121C`
* **CONF3:** `EAFE1DC`
* **PLLCONF:** `9EC0008`
* **DIV:** `0C00080`
* **FDIV:** `8000070`
* **STRM:** `8000000`
* **CLK:** `10061B2`
* **TEST1:** `1E0F401`
* **TEST2:** `28C0402`

## 3. Conexión física al Pmod JA de la Basys-3

| Señal MAX2769C | Pin chip | Pin Pmod JA | Pin FPGA (XDC) | Descripción |
| :--- | :--- | :--- | :--- | :--- |
| I1 (bit de signo, MSB) | 21 | Pin 3 | **J2** | Signo de la muestra I |
| I0 (bit de magnitud, LSB) | 20 | Pin 4 | **G2** | Magnitud de la muestra I |
| GND | EP | Pin 5 | GND | Masa común |
| VCC 3.3 V | VCC_RF | Pin 6 | VCC | Alimentación del MAX2769C |

> **Codificación sign-magnitude de 2 bits** (Table 13 FORMAT=01 y Table 16, p.16 del datasheet; confirmado en Note 2, p.4):
> I1 e I0 no forman un entero binario estándar sino una muestra en formato **sign-magnitude**. I1 codifica el signo (0 = positivo, 1 = negativo) e I0 codifica la magnitud (0 = débil, 1 = fuerte). El nivel cero no existe por diseño — la señal GPS siempre contiene ruido térmico, por lo que el ADC siempre produce ±1 o ±2. Ambas líneas deben muestrearse en el mismo flanco de reloj.
>
> | I1I0 | I1 (signo) | I0 (magnitud) | Valor entero |
> | :---: | :---: | :---: | :---: |
> | 01 | 0 | 1 | +2 (señal fuerte positiva) |
> | 00 | 0 | 0 | +1 (señal débil positiva) |
> | 10 | 1 | 0 | −1 (señal débil negativa) |
> | 11 | 1 | 1 | −2 (señal fuerte negativa) |

## 4. Detalles de Implementación en VHDL
El diseño en la FPGA Basys-3 (Artix-7) implementa:

1. **Frecuencia de Muestreo:** Las señales I0/I1 llegan a 16.368 Msps. Un NCO maestro de 16 bits a 100 MHz genera un enable a ~16.368 MHz, y un decimador ×16 produce el chip-rate de ~1.023 Mcps (`clk_en_fe`).
2. **Niveles Lógicos:** La salida del MAX2769C está configurada como **CMOS 3.3 V**, compatible con los pines I/O LVCMOS33 de la Basys-3. Se aplican `false_path` en el XDC y doble flip-flop de sincronización independiente en la FPGA para cada línea (I1 e I0 por separado).
3. **Formato de Datos:** La salida es **2 bits sign-magnitude, solo canal I** (`i1_real, i0_real : in STD_LOGIC`), donde I1 es el bit de signo e I0 el bit de magnitud. La FPGA convierte a entero con signo según: si I1=0 → valor = +(1+I0), si I1=1 → valor = −(1+I0), produciendo los cuatro niveles {+2, +1, −1, −2}. La mezcla I/Q se realiza internamente en la FPGA mediante el `doppler_mixer` (NCO + LUT seno/coseno de 16 bits). La fase reportada por UART aplica un mapeo de correlación circular y admite calibración con un sesgo fijo configurable (`CFG_PHASE_BIAS`).

## 5. Limitaciones del Front-End en este proyecto

1. **Cuantización a 2 bits sign-magnitude (4 niveles):** mejor que 1 bit (~1.25 dB de mejora de SNR sobre 1 bit, ~0.55 dB de pérdida frente a cuantización infinita), pero sigue siendo limitado frente a soluciones de 3+ bits. La ausencia del nivel cero es una ventaja: evita ambigüedad de signo en muestras nulas.
2. **Modo I-only:** no se dispone de canal Q analógico nativo del front-end en esta configuración (IQEN=0 en CONF2, Table 24, p.23 del datasheet).
3. **Dependencia de IF fija (4.092 MHz):** variaciones térmicas del cristal (±10 ppm según gráfica p.9 del datasheet) o errores de modelado Doppler afectan directamente la correlación.
4. **Efecto near-far en mezcla multi-satélite:** con solo 4 niveles de amplitud, el AGC interno puede saturarse ante satélites muy fuertes, enmascarando señales débiles. El parámetro GAINREF (CONF2 bits 26:15, Table 24) controla la densidad de bits de magnitud objetivo (valor óptimo 170 = 33% para ADC de 2 bits, según p.14 del datasheet).
5. **Sin estimador analógico de amplitud instantánea en FPGA:** las decisiones dependen de acumulación temporal de picos de correlación; no es posible medir potencia de señal en una sola muestra.
6. **Sensibilidad a glitches digitales en I1/I0:** transiciones espúreas en las líneas de datos degradan la correlación. En formato sign-magnitude, I1 e I0 **no son señales complementarias** — ambas pueden ser 0 o 1 simultáneamente — por lo que la mitigación se basa exclusivamente en doble flip-flop de sincronización independiente por línea, tal como impone el cruce de dominio de reloj entre el MAX2769C y la FPGA.

## 6. Mitigaciones implementadas en FPGA

| Limitación | Mitigación en diseño | Estado |
| :--- | :--- | :--- |
| Cuantización 2 bits | Integración no coherente y criterio estadístico de adquisición | Implementado |
| I-only (sin Q analógica) | Mezcla digital I/Q en FPGA con NCO y LUT seno/coseno | Implementado |
| IF fija 4.092 MHz | Barrido Doppler en adquisición | Implementado (adquisición). Seguimiento FLL/PLL: pendiente |
| Near-far multi-sat | Ganancias por satélite en generador sintético | Implementado para test. AGC/SIC real: pendiente |
| Glitches I1/I0 | Doble FF de sincronización independiente por línea (I1 e I0) | Implementado |

### Capacidad simultánea de satélites (contexto práctico)

- **Señal sintética de este proyecto:** hasta 3 simultáneos (SAT1/SAT2/SAT3), porque el generador inyecta 3 fuentes.
- **Front-end real (MAX2769C State 2):** la cadena de adquisición es multisitio por barrido PRN×Doppler; el número visible por barrido depende de `CFG_NUM_PRNS`, `CFG_BIN_RANGE` y `CFG_N_INT` (compromiso cobertura/tiempo).

Para reducir recompilaciones durante pruebas, conviene mantener un bitstream de operación con `CFG_NUM_PRNS=32` y ajustar la validación rápida con switches (manual/auto, doppler manual) en lugar de cambiar constantes en cada iteración.

## 7. Plan de Implementación Recomendado (sin romper timing)

Implementar todas las mitigaciones de golpe no es recomendable en esta plataforma si se quiere mantener trazabilidad de resultados.

Fases sugeridas:

1. **Bajo riesgo:** umbral adaptativo por ruido, contadores de errores de muestreo I1/I0 y exportación UART de métricas.
2. **Riesgo medio:** integración coherente extendida (4-10 ms) y FLL simple de frecuencia residual.
3. **Riesgo alto:** PLL de seguimiento fino y AGC vía SPI al MAX2769C (registro CONF2, bits GAINREF 26:15 y AGCMODE 12:11, Table 24, p.23).
4. **Muy alto coste:** cancelación sucesiva de interferencia (SIC) y/o Q sintética por transformada de Hilbert digital.

Este orden reduce regresiones y facilita validar cada mejora frente a logs UART/Vivado.

En la práctica: cuando el barrido sintético alcance 3 satélites de forma recurrente y estable (sin intercambios frecuentes de Doppler/fase), es razonable iniciar la **Fase A** sin esperar a conectar el front-end real.

---
*Nota: Información extraída del datasheet oficial MAX2769C de Maxim Integrated (Rev. 1, 10/2016), páginas 4, 10, 14, 16, 20, 22, 23.*