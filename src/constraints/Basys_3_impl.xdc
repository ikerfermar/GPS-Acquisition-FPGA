## Basys_3_impl.xdc
## Constraints de implementación (no síntesis)
## Entradas asíncronas del front-end MAX2769C ya pasan por doble FF en HDL.

# Modelo STA formal para I1/I0 asíncronos respecto al reloj interno.
# Referencia nominal del front-end: Fs = 16.368 MHz.
create_clock -add -name fe_virtual_clk -period 61.094
set_input_delay -clock [get_clocks fe_virtual_clk] -min 0.0 [get_ports {i1_real i0_real}]
set_input_delay -clock [get_clocks fe_virtual_clk] -max 0.0 [get_ports {i1_real i0_real}]
set_clock_groups -asynchronous \
	-group [get_clocks fe_virtual_clk] \
	-group [get_clocks -include_generated_clocks clk]

# Reloj real del front-end MAX2769C: CLKOUT en JA7:H1 (16.368 MHz).
# Se sincroniza al dominio 100 MHz mediante dos FF en top_gps_system.
# Declarar como reloj para STA correcto; asíncrono al reloj principal.
create_clock -add -name fe_clkout_clk -period 61.094 [get_ports fe_clkout]
set_clock_groups -asynchronous \
	-group [get_clocks fe_clkout_clk] \
	-group [get_clocks -include_generated_clocks clk]

set_false_path -from [get_ports sw_5]
