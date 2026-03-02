## ============================================================
## Basys_3.xdc - Constraints para top_gps_system (v_fe_unified)
##
## Puertos del entity:
##   clk          -> reloj 100 MHz
##   sw[15:0]     -> bus completo de switches
##   led[2:0]     -> LEDs
##   seg[6:0]     -> segmentos del display
##   an[3:0]      -> ánodos del display
##   i1_real      -> I1 del MAX2769C (señal activa,  CMOS 3.3V, 16.368 MSPS)
##   i0_real      -> I0 del MAX2769C (señal pasiva,  CMOS 3.3V, diferencial)
##
## Mapa de switches:
##   sw[4:0]   = PRN satélite local (Satélite = sw[4:0] + 1)
##   sw[5]     = modo señal: 0 = front-end real, 1 = señal sintética
##   sw[13:6]  = Doppler offset en complemento a 2 (pasos de 320 Hz)
##   sw[14]    = activa barrido automático (flanco subida = inicia barrido)
##   sw[15]    = modo display (0 = bin Doppler, 1 = PRN actual)
##
## Conexión del MAX2769C (CLK OUT) a la Basys 3:
##   MAX2769C I1 (activa)  -> Pmod JA pin 1  (FPGA: J1)
##   MAX2769C I0 (pasiva)  -> Pmod JA pin 2  (FPGA: L2)
##   MAX2769C GND          -> Pmod JA pin 5  (GND)
##   MAX2769C VCC (3.3V)   -> Pmod JA pin 6  (VCC)
##
## Si tus cables van a pines distintos, cambia PACKAGE_PIN de
## i1_real / i0_real a continuación. Pines disponibles en Pmods:
##   JA: J1(p1) L2(p2) J2(p3) G2(p4)  H1(p7) K2(p8) H2(p9) G3(p10)
##   JB: A14(p1) A16(p2) B15(p3) B16(p4) A15(p7) A17(p8) C15(p9) C16(p10)
##   JC: K17(p1) M18(p2) N17(p3) P18(p4) L17(p7) M19(p8) P17(p9) R18(p10)
##   JXADC: J3(p1) L3(p2) M2(p3) N2(p4) K3(p7) M3(p8) M1(p9) N1(p10)
## ============================================================

## Reloj 100 MHz
set_property PACKAGE_PIN W5  [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports clk]

## Switches sw[15:0]
set_property PACKAGE_PIN V17 [get_ports {sw[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[0]}]
set_property PACKAGE_PIN V16 [get_ports {sw[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[1]}]
set_property PACKAGE_PIN W16 [get_ports {sw[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[2]}]
set_property PACKAGE_PIN W17 [get_ports {sw[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[3]}]
set_property PACKAGE_PIN W15 [get_ports {sw[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[4]}]
set_property PACKAGE_PIN V15 [get_ports {sw[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[5]}]
set_property PACKAGE_PIN W14 [get_ports {sw[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[6]}]
set_property PACKAGE_PIN W13 [get_ports {sw[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[7]}]
set_property PACKAGE_PIN V2  [get_ports {sw[8]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[8]}]
set_property PACKAGE_PIN T3  [get_ports {sw[9]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[9]}]
set_property PACKAGE_PIN T2  [get_ports {sw[10]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[10]}]
set_property PACKAGE_PIN R3  [get_ports {sw[11]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[11]}]
set_property PACKAGE_PIN W2  [get_ports {sw[12]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[12]}]
set_property PACKAGE_PIN U1  [get_ports {sw[13]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[13]}]
set_property PACKAGE_PIN T1  [get_ports {sw[14]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[14]}]
set_property PACKAGE_PIN R2  [get_ports {sw[15]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[15]}]

## LEDs
set_property PACKAGE_PIN U16 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
set_property PACKAGE_PIN E19 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property PACKAGE_PIN U19 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]

## Display 7 segmentos - segmentos (seg)
set_property PACKAGE_PIN W7  [get_ports {seg[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg[0]}]
set_property PACKAGE_PIN W6  [get_ports {seg[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg[1]}]
set_property PACKAGE_PIN U8  [get_ports {seg[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg[2]}]
set_property PACKAGE_PIN V8  [get_ports {seg[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg[3]}]
set_property PACKAGE_PIN U5  [get_ports {seg[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg[4]}]
set_property PACKAGE_PIN V5  [get_ports {seg[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg[5]}]
set_property PACKAGE_PIN U7  [get_ports {seg[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg[6]}]

## Display 7 segmentos - ánodos (an)
set_property PACKAGE_PIN U2  [get_ports {an[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {an[0]}]
set_property PACKAGE_PIN U4  [get_ports {an[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {an[1]}]
set_property PACKAGE_PIN V4  [get_ports {an[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {an[2]}]
set_property PACKAGE_PIN W4  [get_ports {an[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {an[3]}]

## UART TX hacia el FTDI (el FPGA transmite, el FTDI recibe)
set_property -dict { PACKAGE_PIN A18  IOSTANDARD LVCMOS33 } [get_ports { uart_tx_pin }]

## Opcional: UART RX desde el FTDI (si mas adelante quieres recibir comandos)
## set_property -dict { PACKAGE_PIN B18  IOSTANDARD LVCMOS33 } [get_ports { uart_rx_pin }]

## ── Entradas del front-end MAX2769C ──────────────────────────────────────
## Cambia los PACKAGE_PIN si tus cables van a otros pines del Pmod.
## IOSTANDARD LVCMOS33: el MAX2769C en Device State 2 produce salida CMOS 3.3V.
set_property PACKAGE_PIN J2  [get_ports i1_real]
set_property IOSTANDARD LVCMOS33 [get_ports i1_real]

set_property PACKAGE_PIN G2  [get_ports i0_real]
set_property IOSTANDARD LVCMOS33 [get_ports i0_real]

## Timing: las entradas del front-end son asíncronas respecto al reloj del FPGA.
## Se declaran false paths porque el diseño ya tiene doble flip-flop de sincronización.
set_false_path -from [get_ports i1_real]
set_false_path -from [get_ports i0_real]
