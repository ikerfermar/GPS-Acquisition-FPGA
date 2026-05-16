set script_dir [file dirname [file normalize [info script]]]
set repo_root  [file normalize [file join $script_dir ".."]]

set project_name "GPS_Acquisition_FPGA"
set part_name    "xc7a35ticpg236-1L"
set build_dir    [file join $repo_root "vivado" "${project_name}_rebuild"]
set xpr_file     [file join $build_dir "${project_name}.xpr"]

# Jobs: variable de entorno VIVADO_RUN_JOBS (run_vivado_build.ps1 la fija a nCores)
set run_jobs 4
if {[info exists ::env(VIVADO_RUN_JOBS)] && [string is integer -strict $::env(VIVADO_RUN_JOBS)]} {
    set run_jobs $::env(VIVADO_RUN_JOBS)
}
if {$run_jobs < 1} { set run_jobs 1 }
if {$run_jobs > 16} { set run_jobs 16 }
puts "INFO: Jobs = $run_jobs"

# Modo incremental rapido (solo proyectos existentes).
set quick_incremental 0
if {[info exists ::env(VIVADO_QUICK_INCREMENTAL)]} {
    set q [string tolower $::env(VIVADO_QUICK_INCREMENTAL)]
    if {$q in {1 true yes on}} {
        set quick_incremental 1
    }
}
puts "INFO: Quick incremental mode = $quick_incremental"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
proc check_run_ok {run_name} {
    set prog [get_property PROGRESS [get_runs $run_name]]
    set stat [get_property STATUS   [get_runs $run_name]]
    if {$prog ne "100%"} {
        error "Run '$run_name' FAILED.\n  Status:   $stat\n  Progress: $prog\nRevisa [get_property DIRECTORY [get_runs $run_name]]/runme.log"
    }
    puts "INFO: $run_name -> $stat ($prog)"
}

# Limpia marcadores residuales de ejecucion en IPs OOC.
proc clean_stale_ip_markers {runs_dir} {
    foreach d [glob -nocomplain -type d [file join $runs_dir *_synth_1]] {
        set marker [file join $d "__synthesis_is_running__"]
        if {[file exists $marker]} {
            file delete -force $marker
            set done [file join $d "__synthesis_is_complete__"]
            if {[file exists $done]} {
                puts "INFO: Marker stale limpiado en [file tail $d] (run ya completado)"
            } else {
                puts "WARN: Marker stale en [file tail $d] (run interrumpido - se relanzara)"
            }
        }
    }
}

# ---------------------------------------------------------------------------
# Flujo de creacion o apertura de proyecto
# ---------------------------------------------------------------------------
file mkdir $build_dir
set runs_dir [file join $build_dir "${project_name}.runs"]

if {[file exists $xpr_file]} {
    puts "INFO: Proyecto existente encontrado - modo incremental (IP cache reutilizada)"
    puts "INFO: Abriendo: $xpr_file"
    open_project $xpr_file

    # Limpiar marcadores antes de reset
    if {[file isdirectory $runs_dir]} {
        clean_stale_ip_markers $runs_dir
    }

    # 0) Registrar IP de ROM si no esta en el proyecto.
    if {[llength [get_ips ca_chips_rom]] == 0} {
        puts "INFO: ca_chips_rom no registrada - aniadiendo al proyecto..."
        set ip_src [file join $repo_root "src" "ip" "ca_chips_rom.xci"]
        if {![file exists $ip_src]} { error "No existe: $ip_src" }
        set ca_dst_dir [file join $build_dir "${project_name}.srcs" "sources_1" "ip" "ca_chips_rom"]
        file mkdir $ca_dst_dir
        set ca_dst [file join $ca_dst_dir "ca_chips_rom.xci"]
        file copy -force $ip_src $ca_dst
        read_ip $ca_dst
        update_compile_order -fileset sources_1
    }

    # 1) Regenerar output products de IP.
    set fft_ips [get_ips -quiet {xfft_0 xfft_1}]
    if {[llength $fft_ips] > 0} {
        puts "INFO: Optimizando uso de DSPs (cambiando a LUTs para evitar over-utilization)..."
        set_property -dict {CONFIG.butterfly_type {use_luts}} $fft_ips
        puts "INFO: Generando output products de xfft_0 y xfft_1 (siempre)..."
        generate_target all $fft_ips
    }
    if {!$quick_incremental} {
        set other_ips [get_ips -quiet -filter {NAME !~ "xfft_*"}]
        if {[llength $other_ips] > 0} {
            puts "INFO: Generando output products de [llength $other_ips] IPs adicionales..."
            generate_target all $other_ips
        }
    } else {
        puts "INFO: QUICK mode: se omite generate_target para IPs no-FFT"
    }

    # 2) Asegurar que los ficheros VHDL de utilidad esten en el proyecto.
    set hdl_dir [file join $repo_root "src" "hdl"]
    set added_hdl_files [list \
        [file join $hdl_dir "acquisition" "acquisition_utils_pkg.vhd"] \
    ]
    set added_feat1 0
    foreach f $added_hdl_files {
        if {[llength [get_files -quiet -of_objects [get_filesets sources_1] [file tail $f]]] == 0} {
            puts "INFO: Aniadiendo [file tail $f] al proyecto existente..."
            add_files -fileset sources_1 $f
            set added_feat1 1
        }
    }
    if {$added_feat1} {
        update_compile_order -fileset sources_1
    }


} else {
    puts "INFO: Proyecto no encontrado - creando desde cero (primera vez, tardara mas)"

    if {[catch {create_project -force $project_name $build_dir -part $part_name} err]} {
        puts "WARN: create_project -force fallo: $err"
        set ts [clock format [clock seconds] -format "%Y%m%d_%H%M%S"]
        set build_dir [file join $repo_root "vivado" "${project_name}_rebuild_${ts}"]
        file mkdir $build_dir
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
        [file join $hdl_dir "acquisition" "acquisition_utils_pkg.vhd"] \
        [file join $hdl_dir "acquisition" "acquisition_controller.vhd"] \
        [file join $hdl_dir "acquisition" "doppler_mixer.vhd"] \
        [file join $hdl_dir "acquisition" "fft_controller.vhd"] \
        [file join $hdl_dir "acquisition" "gps_ca_generator.vhd"] \
        [file join $hdl_dir "acquisition" "multi_sat_rx_gen.vhd"] \
        [file join $hdl_dir "acquisition" "peak_detector.vhd"] \
        [file join $hdl_dir "acquisition" "uart_reporter.vhd"] \
        [file join $hdl_dir "acquisition" "uart_tx.vhd"] \
        [file join $hdl_dir "top_gps_system.vhd"] \
    ]
    set xdc_file      [file join $xdc_dir "Basys_3.xdc"]
    set xdc_impl_file [file join $xdc_dir "Basys_3_impl.xdc"]
    set ip_files [list \
        [file join $ip_dir "ca_chips_rom.xci"] \
        [file join $ip_dir "clk_wiz_0.xci"] \
        [file join $ip_dir "xfft_0.xci"] \
        [file join $ip_dir "xfft_1.xci"] \
    ]

    foreach f $hdl_files      { if {![file exists $f]} { error "No existe HDL: $f" } }
    if {![file exists $xdc_file]}      { error "No existe XDC: $xdc_file" }
    if {![file exists $xdc_impl_file]} { error "No existe XDC: $xdc_impl_file" }

    set local_ip_dir [file join $build_dir "${project_name}.srcs" "sources_1" "ip"]
    file mkdir $local_ip_dir
    set ip_files_local [list]
    foreach f $ip_files {
        if {![file exists $f]} { error "No existe XCI: $f" }
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
    set_property used_in_synthesis false [get_files $xdc_impl_file]

    foreach f $ip_files_local { read_ip $f }
    set_property top top_gps_system [get_filesets sources_1]
    update_compile_order -fileset sources_1

    set all_ips [get_ips *]
    if {[llength $all_ips] > 0} {
        upgrade_ip -quiet $all_ips
        
        set fft_ips [get_ips -quiet {xfft_0 xfft_1}]
        if {[llength $fft_ips] > 0} {
            set_property -dict {CONFIG.butterfly_type {use_luts}} $fft_ips
        }

        generate_target all $all_ips
    }
}

# ---------------------------------------------------------------------------
# PARCHE FFT: Forzar C_ARCH=3 (Radix-2 Lite)
# ---------------------------------------------------------------------------

proc patch_fft_wrapper {gen_ip_dir ip} {
    set wrapper [file join $gen_ip_dir $ip "synth" "${ip}.vhd"]
    if {![file exists $wrapper]} {
        puts "WARN: patch_fft_wrapper: $wrapper no existe - se omite parche de $ip"
        return 0
    }
    set fh [open $wrapper r]; set txt [read $fh]; close $fh

    set arch_before "?"; set bram_before "?"
    regexp {C_ARCH\s*=>\s*(\d+)}        $txt -> arch_before
    regexp {C_BRAM_STAGES\s*=>\s*(\d+)} $txt -> bram_before

    regsub {(C_ARCH\s*=>\s*)4}       $txt {\13} txt
    regsub {(C_BRAM_STAGES\s*=>\s*)0} $txt {\16} txt

    set fh [open $wrapper w]; puts -nonewline $fh $txt; close $fh

    set fh [open $wrapper r]; set txt2 [read $fh]; close $fh
    set arch_after "?"; set bram_after "?"
    regexp {C_ARCH\s*=>\s*(\d+)}        $txt2 -> arch_after
    regexp {C_BRAM_STAGES\s*=>\s*(\d+)} $txt2 -> bram_after

    puts "INFO: FFT wrapper $ip: C_ARCH $arch_before->$arch_after  C_BRAM_STAGES $bram_before->$bram_after"
    if {$arch_after ne "3"} {
        puts "WARN: patch_fft_wrapper: parche de C_ARCH no surtio efecto en $ip - verificar formato del wrapper"
        return 0
    }
    return 1
}

set gen_ip_base [get_property ip_output_repo [current_project]]

set fft_ips [get_ips -quiet {xfft_0 xfft_1}]
if {[llength $fft_ips] > 0} {
    set_property -dict {CONFIG.butterfly_type {use_luts}} $fft_ips
    generate_target all $fft_ips
} else {
    error "No se encontraron las IPs xfft_0 / xfft_1 en el proyecto"
}

set fft_patch_ok 1
foreach fft_ip {xfft_0 xfft_1} {
    set ok [patch_fft_wrapper $gen_ip_base $fft_ip]
    if {!$ok} {
        set fft_patch_ok 0
        puts "WARN: El parche de $fft_ip falló - el diseño puede exceder recursos de la Basys 3"
    }
}

foreach fft_ip {xfft_0 xfft_1} {
    if {[llength [get_files -quiet "${fft_ip}.xci"]] > 0} {
        set_property -name {GENERATE_SYNTH_CHECKPOINT} -value {1} \
                     -objects [get_files "${fft_ip}.xci"]
        puts "INFO: OOC habilitado para $fft_ip (run OOC leerá wrapper parcheado C_ARCH=3)"
    }
}


# Buscar y desregistrar el DCP incremental del fileset utils_1
set incr_dcp_file ""
catch {
    set incr_dcp_file [get_files -quiet -of_objects \
        [get_filesets -quiet utils_1] {top_gps_system.dcp}]
}
if {$incr_dcp_file ne ""} {
    remove_files -fileset utils_1 $incr_dcp_file
    puts "INFO: DCP incremental desregistrado del fileset utils_1: $incr_dcp_file"
} else {
    puts "INFO: No se encontro DCP incremental en fileset utils_1"
}

foreach prop {
    AUTO_INCREMENTAL_CHECKPOINT
    AUTO_INCREMENTAL_CHECKPOINT_DIR
} {
    catch {
        set val [get_property $prop [get_runs synth_1]]
        if {$val ne "" && $val ne "0"} {
            set_property $prop {} [get_runs synth_1]
            puts "INFO: Propiedad $prop limpiada (era: $val)"
        }
    }
}

set incr_dcp_paths [list \
    [file join $build_dir "${project_name}.srcs" "utils_1" "imports" \
               "synth_1" "top_gps_system.dcp"] \
    [file join $build_dir "${project_name}.runs" "synth_1" \
               "top_gps_system.dcp"] \
]
foreach dcp_path $incr_dcp_paths {
    if {[file exists $dcp_path]} {
        file delete -force $dcp_path
        puts "INFO: DCP de referencia eliminado: $dcp_path"
    } else {
        puts "INFO: DCP no encontrado (ok): $dcp_path"
    }
}
set incr_dir [file join $build_dir "${project_name}.srcs" "utils_1" \
              "imports" "synth_1"]
if {[file isdirectory $incr_dir]} {
    file delete -force $incr_dir
    puts "INFO: Directorio incremental eliminado: $incr_dir"
}

puts "INFO: Reset de synth_1 e impl_1..."
catch {reset_run impl_1}
catch {reset_run synth_1}

# RE-SINTESIS OOC EXPLICITA CON WRAPPER PARCHEADO

puts "INFO: Lanzando re-sintesis OOC de FFTs con wrapper C_ARCH=3..."
foreach fft_ip {xfft_0 xfft_1} {
    set gen_ip_dir [file join $gen_ip_base $fft_ip]

    set synth_wrapper [file join $gen_ip_dir "synth" "${fft_ip}.vhd"]
    if {[file exists $synth_wrapper]} {
        set fh [open $synth_wrapper r]; set txt [read $fh]; close $fh
        set arch_val "?"
        regexp {C_ARCH\s*=>\s*(\d+)} $txt -> arch_val
        puts "INFO: Wrapper $fft_ip - C_ARCH=$arch_val"
        if {$arch_val ne "3"} {
            puts "WARN: Re-aplicando parche a $fft_ip..."
            patch_fft_wrapper $gen_ip_base $fft_ip
        }
    } else {
        puts "WARN: Wrapper $fft_ip no encontrado en disco"
    }

    set top_dcp [file join $gen_ip_dir "${fft_ip}.dcp"]
    if {[file exists $top_dcp]} {
        file delete -force $top_dcp
        puts "INFO: DCP eliminado: $top_dcp"
    }
    set ooc_run "${fft_ip}_synth_1"
    if {[llength [get_runs -quiet $ooc_run]] > 0} {
        catch {reset_run $ooc_run}
    }
    set ooc_dir [file join $runs_dir $ooc_run]
    if {[file isdirectory $ooc_dir]} {
        file delete -force $ooc_dir
        puts "INFO: Directorio OOC eliminado: $ooc_dir"
    }
}

puts "INFO: Lanzando runs OOC..."
set ooc_runs_to_wait {}
foreach fft_ip {xfft_0 xfft_1} {
    set ooc_run "${fft_ip}_synth_1"
    if {[llength [get_runs -quiet $ooc_run]] > 0} {
        launch_runs $ooc_run -jobs $run_jobs
        lappend ooc_runs_to_wait $ooc_run
        puts "INFO: Lanzado $ooc_run"
    }
}

foreach ooc_run $ooc_runs_to_wait {
    puts "INFO: Esperando $ooc_run..."
    wait_on_run $ooc_run
    set prog [get_property PROGRESS [get_runs $ooc_run]]
    set stat [get_property STATUS   [get_runs $ooc_run]]
    puts "INFO: $ooc_run -> $stat ($prog)"
    if {$prog ne "100%"} {
        error "Run OOC $ooc_run fallo. Revisa [get_property DIRECTORY [get_runs $ooc_run]]/runme.log"
    }
}
puts "INFO: Runs OOC completados con C_ARCH=3."

foreach fft_ip {xfft_0 xfft_1} {
    set top_dcp [file join $gen_ip_base $fft_ip "${fft_ip}.dcp"]
    if {[file exists $top_dcp]} {
        puts "INFO: DCP nuevo generado: $top_dcp ([file size $top_dcp] bytes)"
    } else {
        puts "WARN: DCP no encontrado despues de OOC: $top_dcp"
    }
}

puts "INFO: Reset final de synth_1 e impl_1..."
catch {reset_run impl_1}
catch {reset_run synth_1}

puts "INFO: Lanzando sintesis principal (usara DCPs OOC con C_ARCH=3)..."

# ---------------------------------------------------------------------------
# SINTESIS
# ---------------------------------------------------------------------------
set_property -name {STEPS.SYNTH_DESIGN.ARGS.DIRECTIVE} \
             -value {RuntimeOptimized} \
             -objects [get_runs synth_1]

puts "INFO: Lanzando sintesis top-level ($run_jobs jobs)..."
launch_runs synth_1 -jobs $run_jobs
wait_on_run synth_1
check_run_ok synth_1

# ---------------------------------------------------------------------------
# IMPLEMENTATION + BITSTREAM (un solo launch)
# ---------------------------------------------------------------------------
puts "INFO: Lanzando implementacion + bitstream ($run_jobs jobs)..."
launch_runs impl_1 -to_step write_bitstream -jobs $run_jobs
wait_on_run impl_1
check_run_ok impl_1

# ---------------------------------------------------------------------------
# Verificar y copiar bitstream
# ---------------------------------------------------------------------------
set bit_file [file join $build_dir "${project_name}.runs" "impl_1" "top_gps_system.bit"]
if {![file exists $bit_file]} {
    error "Bitstream no encontrado en ruta esperada: $bit_file"
}
puts "BITSTREAM_GENERADO=$bit_file"

set artifacts_dir [file join $repo_root "artifacts"]
file mkdir $artifacts_dir
set dest [file join $artifacts_dir "GPS_Acquisition_FPGA.bit"]
file copy -force $bit_file $dest
puts "BITSTREAM_COPIADO=$dest"

close_project