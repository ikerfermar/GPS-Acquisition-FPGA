# MAX2769C — Especificaciones del Front-End GPS

**MAX2769C** (Maxim/Analog Devices, Rev. 1, Oct 2016) operando en **Preconfigured State 2**. En este estado el dispositivo lee su configuración de la interfaz hardware 3-wire (PGM=1); la FPGA no interviene en la configuración del chip.

> Datasheet y Reference Manual disponibles en `docs/hardware datasheets/`.

---

## 1. Modos de operación

El MAX2769C admite configuración por registro (SPI, 3 hilos) o por estados pre-grabados en pines físicos. En este proyecto se usa **State 2 (hardware)**, que fija todos los parámetros sin intervención de la FPGA:

| Pin | Estado en State 2 |
|---|---|
| SCLK | 0 |
| DATA | 1 |
| CS | 0 |

---

## 2. Parámetros operativos — State 2

*(Tabla 22, p. 22 del datasheet)*

| Parámetro | Valor |
|---|---|
| Frecuencia de referencia | 16.368 MHz |
| Main Division Ratio | 16 |
| Reference Division Ratio | 1536 |
| Frecuencia intermedia (IF) | 4.092 MHz |
| Ancho de banda filtro IF | 2.5 MHz (5.º orden) |
| Frecuencia de muestreo ADC (Fs) | 16.368 Msps |
| Canal de salida | I Only — IQEN = 0 |
| Bits por muestra | 2 bits sign-magnitude |
| Nivel lógico de salida | CMOS 3.3 V |

> La IF de 4.092 MHz es exactamente 4 x chip-rate (1.023 MHz). Tras el decimador x16 de la FPGA, la portadora se pliega cerca de banda base. En este proyecto se mantiene mezcla digital IF+Doppler en `doppler_mixer` para robustez y control fino del barrido.

---

## 3. Registros internos — State 2

Valores hexadecimales de los 10 registros para State 2 *(Tabla 22, p. 22)*:

| Registro | Valor hex | Campos relevantes |
|---|---|---|
| CONF1 | `A2919A7` | — |
| CONF2 | `055028C` | BITS=01 (2 b sign-mag); GAINREF=0x2A=170; AGCMODE=00 |
| CONF3 | `FAFE1DC` | ANTEN=1 (bias DC activo para antena activa) |
| PLLCONF | `9EC0008` | — |
| DIV | `0C00080` | — |
| FDIV | `8000070` | — |
| STRM | `8000000` | — |
| CLK | `10061B2` | — |
| TEST1 | `1E0F401` | — |
| TEST2 | `28C0402` | — |

### Campos clave de CONF2 *(Tabla 24, p. 23)*

| Campo | Bits | Valor | Significado |
|---|---|---|---|
| GAINREF | 26:15 | 0x2A = 170 | Densidad objetivo de bits fuertes ~33 % (óptimo para ADC 2-bit, p. 14) |
| AGCMODE | — | 00 | AGC automático activo |
| BITS | 8:6 | 01 | 2 bits sign-magnitude |
| DRVCFG | 5:4 | 00 | Salida CMOS |
| IQEN | — | 0 | Solo canal I |

### Campo ANTEN en CONF3 *(Tabla 27)*

CONF3 = `0xFAFE1DC`:

```
Bits: 27:24  23:20  19:16  15:12  11:8   7:4   3:0
Hex:   F      A      F      E      1      D     C
Bin:  1111   1010   1111   1110   0001  1101  1100
             ^
        bit 24 = 1  →  ANTEN = 1
```

| ANTEN | Comportamiento |
|---|---|
| 1 | Inyecta bias DC en RFIN. Requerido con antenas activas (LNA integrado). |
| 0 | Sin bias DC. Solo para antenas pasivas. |

---

## 4. Codificación de la muestra

*(Tabla 13, FORMAT=01; Tabla 16, p. 16; Nota 2, p. 4)*

I1 e I0 forman una muestra en formato **sign-magnitude**, no un entero binario estándar:

- **I1** codifica el signo (0 = positivo, 1 = negativo).
- **I0** codifica la magnitud (0 = débil, 1 = fuerte).
- El nivel cero **no existe** por diseño.

| I1 | I0 | Signo | Magnitud | Valor relativo | Valor FPGA |
|:---:|:---:|:---:|:---:|:---:|:---:|
| 0 | 1 | + | fuerte | +3 | +32767 |
| 0 | 0 | + | débil | +1 | +10922 |
| 1 | 0 | − | débil | −1 | −10922 |
| 1 | 1 | − | fuerte | −3 | −32767 |

El ratio 3:1 entre magnitudes se mantiene en la cadena digital (`doppler_mixer`). Ambas líneas I1 e I0 deben muestrearse en el mismo flanco de reloj; no son señales complementarias.

---

## 5. Conexión física — Pmod JA de la Basys-3

| Señal MAX2769C | Pin chip | Pin Pmod JA | Pin FPGA | Descripción |
|---|:---:|:---:|:---:|---|
| I1 (signo, MSB) | 21 | Pin 3 | **J2** | Bit de signo |
| I0 (magnitud, LSB) | 20 | Pin 4 | **G2** | Bit de magnitud |
| CLKOUT (16.368 MHz) | 18 | Pin 7 | **H1** | Reloj ADC — cable coaxial |
| GND (blindaje coaxial) | — | Pin 8 | K2 (no declarado) | Blindaje del coaxial |
| GND | EP | Pin 5 | GND | Masa común |
| VCC 3.3 V | VCC_RF | Pin 6 | VCC | Alimentación 3.3 V |

CLKOUT se conecta con cable coaxial (centro = señal, blindaje a JA8). El blindaje no se declara como puerto FPGA en el XDC. La señal entra con dos flip-flops anti-metaestabilidad al dominio de 100 MHz de la FPGA.

---

## 6. Alimentación de antena activa

Una antena GPS activa (parche con LNA integrado) necesita tensión de polarización DC inyectada por el cable RF. El MAX2769C la controla con el bit **ANTEN** de CONF3.

En State 2: **ANTEN = 1** → bias DC activo en RFIN.

### Antena del banco de pruebas

| Parámetro | Valor |
|---|---|
| Sistemas soportados | GPS L1 / GLONASS / Galileo |
| Rango de frecuencia | 1.575 – 1.610 GHz |
| Ganancia LNA | 27 – 28 dB |
| Protección | IP67 |
| Cable | RG-174, 3 m |
| Conector | SMA Macho |
| Tensión de polarización medida | **3.44 V** (confirma ANTEN=1 activo) |

### Verificacion con multimetro

1. Desconectar la antena del SMA del MAX2769C.
2. Medir tensión DC en el pin central del SMA respecto a masa.
3. Con ANTEN=1: debería leerse **+3.0 – 3.6 V** (sin carga).
4. Reconectar la antena: la tensión baja por la caída en la resistencia serie (I_LNA ≈ 8.5 mA, R_serie = 10 Ω → V ≈ 3.21 V).
5. Con ANTEN=0: lectura ~0 V.

---

## 7. Características y limitaciones

| Aspecto | Detalle |
|---|---|
| Cuantizacion 2 bits | ~1.25 dB de mejora sobre 1 bit; ~0.55 dB de perdida frente a cuantizacion continua. La ausencia del nivel cero evita ambiguedad de signo. |
| Sin canal Q nativo | IQEN=0 en State 2. La cadena digital genera `rx_Q` mediante el NCO de `doppler_mixer`. |
| IF fija 4.092 MHz | Variaciones termicas del cristal (±10 ppm, p. 9) afectan directamente la correlacion. Medido en hardware: cristal ~+13 ppm -> offset +20.48 kHz = bin +64. |
| Ancho de banda IF 2.5 MHz | Señales fuera de este rango quedan atenuadas antes del ADC. |
| AGC de 4 niveles | Con satélites de potencia muy diferente, el AGC puede saturarse (near-far). GAINREF=170 ajusta la densidad objetivo al 33% de bits fuertes. |
| Salida CMOS 3.3 V | Compatible con pines LVCMOS33; no requiere traductores de nivel. |
| Nota de campo | La adquisición GPS requiere cielo despejado sin obstáculos en ~45° sobre el horizonte. Con edificios en línea de visión, la C/N₀ cae por debajo de los ~33 dBHz que el receptor necesita para lock estable. |

---

## 8. Calibracion de offset Doppler en banco

Ademas de las especificaciones del datasheet, en las ultimas pruebas se midio un sesgo sistematico de frecuencia asociado al oscilador de referencia (TCXO/cristal), visible como desplazamiento global de Doppler en todos los PRNs.

### Resultado de calibracion usado en la FPGA

- Offset estimado: -4.7 ppm (orden de magnitud de laboratorio)
- Equivalencia en bins de 320 Hz: aproximadamente -23 bins
- Parametro aplicado: CFG_REAL_DOPPLER_BIAS_BINS = -23

Interpretacion:

- Sin compensacion, la ventana de busqueda pierde tiempo barriendo zona de frecuencia poco probable.
- Con compensacion, la ventana se recentra sobre la region donde realmente aparecen los picos.

En los logs se observa que la version ajustada mantiene adquisiciones mas estables y consistentes en escenarios terrestres:

- logs/PruebaTierraSinAjustar
- logs/PruebaTierraAjustada

## 9. Implicaciones para sensibilidad y dinamica

Este front-end impone limites fisicos que condicionan el trade-off de configuracion:

- Cuantizacion 2-bit sign-magnitude: penalizacion conocida frente a cuantizacion ideal.
- Canal I-only en hardware (sin canal Q analogico): menor informacion instantanea.
- Integracion no coherente: introduce perdida por no linealidad (squaring loss).

Por ello, en modo Tierra se favorece integracion larga e histeresis conservadora. En modo LEO, al haber mejor C/N0 y alta derivada Doppler, se prioriza refresco rapido del barrido.

*Información extraída del datasheet oficial MAX2769C (Maxim Integrated, Rev. 1, Oct 2016), pp. 4, 9, 14, 16, 22–23, y complementada con caracterización experimental del banco de pruebas.*
