## Basys_3_impl.xdc
## Constraints de implementación (no síntesis)
## Entradas asíncronas del front-end MAX2769C ya pasan por doble FF en HDL.

set_false_path -from [get_ports i1_real]
set_false_path -from [get_ports i0_real]
set_false_path -from [get_ports {sw[*]}]
