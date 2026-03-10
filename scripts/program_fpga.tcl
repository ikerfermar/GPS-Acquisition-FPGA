# program_fpga.tcl
# Programa la Basys-3 con el bitstream en artifacts/GPS_Acquisition_FPGA.bit
# Uso desde Vivado batch:
#   vivado -mode batch -source scripts/program_fpga.tcl

set script_dir [file dirname [file normalize [info script]]]
set repo_root  [file normalize [file join $script_dir ".."]]
set bit_file   [file join $repo_root "artifacts" "GPS_Acquisition_FPGA.bit"]
set target_filter ""
set device_filter ""

if {[llength $argv] >= 1 && [string length [lindex $argv 0]] > 0} {
    set bit_file [file normalize [lindex $argv 0]]
}
if {[llength $argv] >= 2 && [string length [lindex $argv 1]] > 0} {
    set target_filter [lindex $argv 1]
}
if {[llength $argv] >= 3 && [string length [lindex $argv 2]] > 0} {
    set device_filter [lindex $argv 2]
}

if {![file exists $bit_file]} {
    error "Bitstream no encontrado: $bit_file\nEjecuta primero run_vivado_build.ps1"
}

puts "INFO: Usando bitstream: $bit_file"

open_hw_manager
connect_hw_server -allow_non_jtag

# Seleccionar target JTAG de forma determinista
set all_targets [lsort [get_hw_targets *]]
if {[llength $all_targets] == 0} {
    error "No se encontro ningun target JTAG. Comprueba que la Basys-3 esta conectada por USB."
}

if {[string length $target_filter] > 0} {
    set candidate_targets [lsort [get_hw_targets $target_filter]]
    if {[llength $candidate_targets] == 0} {
        error "No hay targets que cumplan el filtro '$target_filter'. Disponibles: $all_targets"
    }
} else {
    set candidate_targets $all_targets
}

set selected_target [lindex $candidate_targets 0]
if {[llength $candidate_targets] > 1} {
    puts "WARN: Hay [llength $candidate_targets] targets candidatos; se usara el primero en orden estable: $selected_target"
    puts "WARN: Para forzar uno concreto, define PROGRAM_HW_TARGET_FILTER."
}

open_hw_target $selected_target
puts "INFO: Target seleccionado: [current_hw_target]"

# Seleccionar dispositivo de forma determinista
refresh_hw_target [current_hw_target]
set devices [get_hw_devices]
if {[llength $devices] == 0} {
    error "No se encontro ningun dispositivo en el target seleccionado."
}

set candidate_devices {}
if {[string length $device_filter] > 0} {
    foreach d $devices {
        if {[string match $device_filter $d]} {
            lappend candidate_devices $d
        }
    }
    if {[llength $candidate_devices] == 0} {
        error "No hay dispositivos que cumplan el filtro '$device_filter'. Dispositivos: $devices"
    }
} else {
    foreach d $devices {
        set part [string tolower [get_property PART $d]]
        if {[string match "xc7a35t*" $part]} {
            lappend candidate_devices $d
        }
    }
    if {[llength $candidate_devices] == 0} {
        set candidate_devices $devices
    }
}

set candidate_devices [lsort $candidate_devices]
set dev [lindex $candidate_devices 0]
if {[llength $candidate_devices] > 1} {
    puts "WARN: Hay [llength $candidate_devices] dispositivos candidatos; se usara el primero en orden estable: $dev"
    puts "WARN: Para forzar uno concreto, define PROGRAM_HW_DEVICE_FILTER."
}

current_hw_device $dev
puts "INFO: Dispositivo seleccionado: $dev"

set_property PROGRAM.FILE $bit_file $dev
program_hw_devices $dev
refresh_hw_device $dev
puts "INFO: Programacion completada."

close_hw_target
close_hw_manager
