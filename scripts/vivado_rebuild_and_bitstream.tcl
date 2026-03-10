set script_dir [file dirname [file normalize [info script]]]
set repo_root  [file normalize [file join $script_dir ".."]]

set project_name "GPS_Acquisition_FPGA"
set part_name   "xc7a35ticpg236-1L"
set build_dir   [file join $repo_root "vivado" "${project_name}_rebuild"]

# Numero de jobs para synth/impl. Por defecto conservador para evitar cuelgues
# por presion de RAM/IO en equipos portatiles.
set run_jobs 4
if {[info exists ::env(VIVADO_RUN_JOBS)] && [string is integer -strict $::env(VIVADO_RUN_JOBS)]} {
    set run_jobs $::env(VIVADO_RUN_JOBS)
}

# ---------------------------------------------------------------------------
# Helper procs
# ---------------------------------------------------------------------------

# Verifica que un run terminó al 100%; si no, lanza error con detalles.
proc check_run_ok {run_name} {
    set prog [get_property PROGRESS [get_runs $run_name]]
    set stat [get_property STATUS   [get_runs $run_name]]
    if {$prog ne "100%"} {
        error "Run '$run_name' FAILED.\n  Status:   $stat\n  Progress: $prog\nRevisa runme.log en el directorio del run."
    }
    puts "INFO: $run_name -> $stat ($prog)"
}

# ---------------------------------------------------------------------------

file mkdir $build_dir

# If a previous Vivado process left run directories locked, create_project -force
# can fail when trying to remove *_rebuild/*. In that case, fall back to a
# timestamped rebuild directory so the build can continue.
if {[catch {create_project -force $project_name $build_dir -part $part_name} create_err]} {
    puts "WARN: create_project -force failed on '$build_dir'"
    puts "WARN: $create_err"
    set ts [clock format [clock seconds] -format "%Y%m%d_%H%M%S"]
    set build_dir [file join $repo_root "vivado" "${project_name}_rebuild_${ts}"]
    file mkdir $build_dir
    puts "INFO: Retrying in fallback build dir: $build_dir"
    create_project $project_name $build_dir -part $part_name
}
set_property target_language VHDL [current_project]

set ip_user_files_dir [file join $build_dir "${project_name}.ip_user_files"]
set ip_output_repo    [file join $build_dir "${project_name}.gen" "sources_1" "ip"]
file mkdir $ip_user_files_dir
file mkdir $ip_output_repo
set_property ip_user_files_dir $ip_user_files_dir [current_project]
set_property ip_output_repo    $ip_output_repo    [current_project]

set hdl_dir [file join $repo_root "src" "hdl"]
set xdc_dir [file join $repo_root "src" "constraints"]
set ip_dir  [file join $repo_root "src" "ip"]

set hdl_files [list \
    [file join $hdl_dir "gps_config_pkg.vhd"] \
    [file join $hdl_dir "acquisition" "acquisition_controller.vhd"] \
    [file join $hdl_dir "acquisition" "doppler_mixer.vhd"] \
    [file join $hdl_dir "acquisition" "fft_controller.vhd"] \
    [file join $hdl_dir "acquisition" "gps_ca_generator.vhd"] \
    [file join $hdl_dir "acquisition" "multi_sat_rx_gen.vhd"] \
    [file join $hdl_dir "acquisition" "peak_detector.vhd"] \
    [file join $hdl_dir "seven_seg_controller.vhd"] \
    [file join $hdl_dir "acquisition" "uart_reporter.vhd"] \
    [file join $hdl_dir "acquisition" "uart_tx.vhd"] \
    [file join $hdl_dir "top_gps_system.vhd"] \
]

set xdc_file [file join $xdc_dir "Basys_3.xdc"]
set xdc_impl_file [file join $xdc_dir "Basys_3_impl.xdc"]

set ip_files [list \
    [file join $ip_dir "clk_wiz_0.xci"] \
    [file join $ip_dir "xfft_0.xci"] \
    [file join $ip_dir "xfft_1.xci"] \
]

set local_ip_dir [file join $build_dir "${project_name}.srcs" "sources_1" "ip"]
file mkdir $local_ip_dir
set ip_files_local [list]

foreach f $hdl_files {
    if {![file exists $f]} {
        error "No existe archivo HDL: $f"
    }
}

if {![file exists $xdc_file]} {
    error "No existe constraints XDC: $xdc_file"
}

if {![file exists $xdc_impl_file]} {
    error "No existe constraints XDC: $xdc_impl_file"
}

foreach f $ip_files {
    if {![file exists $f]} {
        error "No existe archivo IP (.xci): $f"
    }

    set ip_name   [file rootname [file tail $f]]
    set ip_dstdir [file join $local_ip_dir $ip_name]
    set ip_dst    [file join $ip_dstdir [file tail $f]]
    file mkdir $ip_dstdir
    file copy -force $f $ip_dst
    lappend ip_files_local $ip_dst
}

add_files -fileset sources_1 $hdl_files
add_files -fileset constrs_1 $xdc_file
add_files -fileset constrs_1 $xdc_impl_file

# Basys_3_impl.xdc contiene restricciones de implementación únicamente.
set_property used_in_synthesis false [get_files $xdc_impl_file]

# Read each XCI without importing old generated artifacts from previous paths.
foreach f $ip_files_local {
    read_ip $f
}

set_property top top_gps_system [get_filesets sources_1]
update_compile_order -fileset sources_1

# Upgrade IPs if needed, then generate all output products.
# Nota: se evita lanzar OOC manualmente (clk_wiz/xfft) porque en algunos
# entornos Windows puede quedarse colgado sin error explicito. La sintesis
# top-level se encarga de resolver dependencias de IP de forma robusta.
set all_ips [get_ips *]
if {[llength $all_ips] > 0} {
    upgrade_ip -quiet $all_ips
    generate_target all $all_ips
    export_ip_user_files -of_objects $all_ips -no_script -sync -force -quiet
    puts "INFO: IP targets generados. OOC manual omitido; continua sintesis top-level."
}

puts "INFO: Lanzando sintesis top-level..."
launch_runs synth_1 -jobs $run_jobs
wait_on_run synth_1
check_run_ok synth_1

puts "INFO: Lanzando implementacion y generacion de bitstream..."
launch_runs impl_1 -to_step write_bitstream -jobs $run_jobs
wait_on_run impl_1
check_run_ok impl_1

open_run impl_1

# get_property BITSTREAM.FILE does not reliably return a value in batch mode.
# Construct the path directly from the known impl_1 output location.
set bit_file [file join $build_dir "${project_name}.runs" "impl_1" "top_gps_system.bit"]
if {![file exists $bit_file]} {
    error "Bitstream no encontrado en ruta esperada: $bit_file"
}
puts "BITSTREAM_GENERADO=$bit_file"

# Copy .bit to artifacts/ at the repo root for easy access
set artifacts_dir [file join $repo_root "artifacts"]
file mkdir $artifacts_dir
set dest [file join $artifacts_dir "GPS_Acquisition_FPGA.bit"]
file copy -force $bit_file $dest
puts "BITSTREAM_COPIADO=$dest"

close_project
