$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repoRoot = Split-Path -Parent $PSScriptRoot
$tclPath  = Join-Path $PSScriptRoot 'vivado_rebuild_and_bitstream.tcl'

$vivadoCmd = $null

if ($env:XILINX_VIVADO) {
    $candidate = Join-Path $env:XILINX_VIVADO 'bin/vivado.bat'
    if (Test-Path $candidate) {
        $vivadoCmd = $candidate
    }
}

if (-not $vivadoCmd) {
    $fromPath = Get-Command vivado.bat -ErrorAction SilentlyContinue
    if (-not $fromPath) {
        $fromPath = Get-Command vivado -ErrorAction SilentlyContinue
    }
    if ($fromPath) {
        $vivadoCmd = $fromPath.Source
    }
}

if (-not $vivadoCmd) {
    throw "No se encontro Vivado. Añade vivado(.bat) al PATH o define XILINX_VIVADO."
}

if (-not (Test-Path $tclPath)) {
    throw "No se encontro TCL en $tclPath"
}

# Si hay procesos Vivado ya corriendo, create_project -force puede fallar y el
# build quedar inestable. Se puede desactivar este bloqueo con:
#   $env:VIVADO_BUILD_ALLOW_RUNNING = '1'
$allowRunningVivado = ($env:VIVADO_BUILD_ALLOW_RUNNING -eq '1')
if (-not $allowRunningVivado) {
    $runningVivado = Get-Process vivado -ErrorAction SilentlyContinue
    if ($runningVivado) {
        $pids = ($runningVivado | Select-Object -ExpandProperty Id) -join ', '
        throw "Hay procesos Vivado activos (PID: $pids). Cierra esos procesos o define VIVADO_BUILD_ALLOW_RUNNING=1 para continuar."
    }
}

Push-Location $repoRoot
try {
    & $vivadoCmd -mode batch -source $tclPath
    if ($LASTEXITCODE -ne 0) {
        $vivadoLog = Join-Path $repoRoot 'vivado.log'
        $tail = ''
        if (Test-Path $vivadoLog) {
            $tail = (Get-Content $vivadoLog -Tail 80) -join [Environment]::NewLine
        }
        throw "Vivado build fallo (exit code $LASTEXITCODE). Revisa vivado.log y runme.log de runs. Ultimas lineas de vivado.log:`n$tail"
    }
}
finally {
    Pop-Location
}
