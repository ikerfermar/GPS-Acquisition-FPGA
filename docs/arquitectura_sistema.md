# Arquitectura del Sistema

Resumen arquitectonico del receptor de adquisicion GPS L1 C/A implementado en FPGA, con foco en los compromisos Tierra vs LEO observados en laboratorio y simulacion Skydel.

## Objetivo funcional

- Barrer PRN x Doppler en modo serie sobre 32 satelites.
- Correlacionar por FFT/IFFT con integracion coherente y no coherente.
- Detectar picos robustos con metricas de ruido, dominancia y CFAR.
- Publicar resultados y diagnostico por UART para monitor en vivo/replay.

## Cadena de procesamiento

1. Entrada del front-end (I1/I0) con sincronizacion CDC.
2. Decimacion hasta chip-rate efectiva.
3. Mezcla IF + Doppler (NCO digital) para obtener rx_I/rx_Q.
4. Generacion local de codigo C/A por PRN.
5. Correlacion espectral: FFT RX + FFT CA + producto complejo + IFFT.
6. Deteccion de maximo, segundo maximo y ruido integrado.
7. FSM de adquisicion: barrido grueso/fino, hysteresis y arbitraje.
8. Reporte UART por satelite y resumen por sweep.

## Bloques principales

- src/hdl/top_gps_system.vhd: integracion y seleccion de fuente real/sintetica.
- src/hdl/gps_config_pkg.vhd: parametros globales de adquisicion y entorno.
- src/hdl/acquisition/acquisition_controller.vhd: control de barrido y decisiones de lock.
- src/hdl/acquisition/fft_controller.vhd: pipeline FFT/IFFT y acumulaciones.
- src/hdl/acquisition/peak_detector.vhd: estimacion de pico/ruido/segundo pico.
- src/hdl/acquisition/doppler_mixer.vhd: compensacion IF/Doppler y conversion de muestra.
- src/hdl/acquisition/uart_reporter.vhd: empaquetado ASCII del estado de adquisicion.

## Perfiles de operacion

### Perfil Tierra (alta sensibilidad)

- Integracion larga por bin para elevar ganancia de procesamiento.
- Histeresis de adquisicion conservadora para filtrar falsos positivos.
- Barrido total mas lento, util en escenarios de baja C/N0 y multipath.

Parametros tipicos:

- CFG_N_INT = 20
- CFG_COARSE_N_INT = 10
- CFG_HYST_ACQ_SWEEPS = 2
- CFG_HYST_REL_SWEEPS = 3

### Perfil LEO (alta dinamica)

- Integracion corta para refresco rapido del barrido completo.
- Adquisicion inmediata (zero-delay) para no perder picos transitorios.
- Menor robustez frente a ruido, pero adecuada para señales fuertes en LoS.

Parametros activos en la rama actual:

- CFG_N_INT = 2
- CFG_COARSE_N_INT = 1
- CFG_HYST_ACQ_SWEEPS = 1
- CFG_HYST_REL_SWEEPS = 2
- CFG_REAL_DOPPLER_BIAS_BINS = -23

## Explicacion del cambio de fases en modo Tierra

Durante barridos consecutivos, la fase de correlacion ph cambia de manera casi lineal incluso cuando el satelite permanece en lock. Esto no indica corrupcion en CDC ni RAM: es la firma esperada de code Doppler.

La aproximacion de primer orden es:

$$
\Delta \phi_{chips} \approx f_{code,dopp} \cdot T_{sweep}
$$

Consecuencia practica:

- Si el barrido dura mas (perfil Tierra), el avance de fase por sweep es mayor.
- Si el barrido se acorta (perfil LEO), el avance observado por sweep se reduce y el lazo puede seguir dinamica mas agresiva.

## Evidencias experimentales incorporadas

- logs/PruebaTierraSinAjustar: comportamiento terrestre sin compensacion de sesgo.
- logs/PruebaTierraAjustada: mejora al centrar busqueda Doppler con bias.
- logs/PruebaLEO: validacion de perfil rapido y adquisicion inmediata en alta dinamica.

## Telemetria y explotacion

El monitor Python permite tres flujos operativos sobre la misma interfaz:

- Serie en vivo: adquisicion desde UART.
- Guardado de sesion: salida cruda con opcion --log.
- Replay offline: reproduccion temporal de trazas con --replay y --speed.

Este flujo permite comparar escenarios y justificar decisiones de diseno en memoria TFM con datos repetibles.
