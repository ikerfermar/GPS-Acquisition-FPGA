## ============================================================
## Basys_3.xdc - Constraints para top_gps_system (v_fe_unified)
##
## Puertos del entity:
##   clk          -> reloj 100 MHz
##   sw_5         -> selector de modo (0=real, 1=sintética)
##   led[15:0]    -> LEDs de estado
##   i1_real      -> I1 del MAX2769C (bit de signo, CMOS 3.3V, 16.368 MSPS)
##   i0_real      -> I0 del MAX2769C (bit de magnitud, CMOS 3.3V, 16.368 MSPS)
##   fe_clkout    -> CLKOUT del MAX2769C (16.368 MHz, cable coaxial, centro->JA7:H1)
##   uart_tx_pin  -> transmisión UART a PC
##
## Mapa de switches:
##   sw_5      = modo señal: 0 = front-end real, 1 = señal sintética
##
## Conexión del MAX2769C a la Basys 3:
##   MAX2769C I1 (signo)    -> Pmod JA pin 3  (FPGA: J2)
##   MAX2769C I0 (magnitud) -> Pmod JA pin 4  (FPGA: G2)
##   MAX2769C CLKOUT        -> Pmod JA pin 7  (FPGA: H1)  [cable coaxial, centro]
##   MAX2769C GND           -> Pmod JA pin 5  (GND)
##   MAX2769C VCC (3.3V)    -> Pmod JA pin 6  (VCC)
##   Blindaje coaxial       -> Pmod JA pin 8  (FPGA: K2)  [GND potencial; sin declarar]
##
## Nota JA8/K2: el blindaje del coaxial de CLKOUT queda en K2. No se
## declara como puerto ya que esta al potencial GND del coaxial. Vivado
## lo deja como pin sin usar (pull-up/down por defecto del banco).
##
## Si tus cables van a pines distintos, cambia PACKAGE_PIN de
## i1_real / i0_real / fe_clkout a continuación. Pines disponibles en Pmods:
##   JA: J1(p1) L2(p2) J2(p3) G2(p4)  H1(p7) K2(p8) H2(p9) G3(p10)
##   JB: A14(p1) A16(p2) B15(p3) B16(p4) A15(p7) A17(p8) C15(p9) C16(p10)
##   JC: K17(p1) M18(p2) N17(p3) P18(p4) L17(p7) M19(p8) P17(p9) R18(p10)
##   JXADC: J3(p1) L3(p2) M2(p3) N2(p4) K3(p7) M3(p8) M1(p9) N1(p10)
## ============================================================

## Reloj 100 MHz
set_property PACKAGE_PIN W5  [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]

## Switches
set_property PACKAGE_PIN V15 [get_ports sw_5]
set_property IOSTANDARD LVCMOS33 [get_ports sw_5]

## LEDs
set_property PACKAGE_PIN U16 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
set_property PACKAGE_PIN E19 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property PACKAGE_PIN U19 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property PACKAGE_PIN V19 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]
set_property PACKAGE_PIN W18 [get_ports {led[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[4]}]
set_property PACKAGE_PIN U15 [get_ports {led[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[5]}]
set_property PACKAGE_PIN U14 [get_ports {led[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[6]}]
set_property PACKAGE_PIN V14 [get_ports {led[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[7]}]
set_property PACKAGE_PIN V13 [get_ports {led[8]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[8]}]
set_property PACKAGE_PIN V3 [get_ports {led[9]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[9]}]
set_property PACKAGE_PIN W3 [get_ports {led[10]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[10]}]
set_property PACKAGE_PIN U3 [get_ports {led[11]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[11]}]
set_property PACKAGE_PIN P3 [get_ports {led[12]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[12]}]
set_property PACKAGE_PIN N3 [get_ports {led[13]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[13]}]
set_property PACKAGE_PIN P1 [get_ports {led[14]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[14]}]
set_property PACKAGE_PIN L1 [get_ports {led[15]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[15]}]

## UART TX hacia el FTDI (el FPGA transmite, el FTDI recibe)
set_property -dict { PACKAGE_PIN A18  IOSTANDARD LVCMOS33 } [get_ports { uart_tx_pin }]

## ── Entradas del front-end MAX2769C ──────────────────────────────────────
## Cambia los PACKAGE_PIN si tus cables van a otros pines del Pmod.
## IOSTANDARD LVCMOS33: el MAX2769C en Device State 2 produce salida CMOS 3.3V.
set_property PACKAGE_PIN J2  [get_ports i1_real]
set_property IOSTANDARD LVCMOS33 [get_ports i1_real]

set_property PACKAGE_PIN G2  [get_ports i0_real]
set_property IOSTANDARD LVCMOS33 [get_ports i0_real]

## CLKOUT del MAX2769C (16.368 MHz) -- cable coaxial centro -> JA7:H1
## El blindaje del coaxial va a JA8 (K2); no se declara como puerto FPGA.
set_property PACKAGE_PIN H1  [get_ports fe_clkout]
set_property IOSTANDARD LVCMOS33 [get_ports fe_clkout]

## ── Propiedades de configuracion (elimina DRC CFGBVS-1) ─────────────────
## La Basys-3 usa VCCO de 3.3V en el banco de configuracion (banco 0).
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
