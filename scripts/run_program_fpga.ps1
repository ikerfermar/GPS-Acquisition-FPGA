<#
run_program_fpga.ps1
Programa la Basys-3 con artifacts/GPS_Acquisition_FPGA.bit sin abrir el GUI de Vivado.
Uso:
    powershell -ExecutionPolicy Bypass -File scripts/run_program_fpga.ps1
#>

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repoRoot = Split-Path -Parent $PSScriptRoot
$tclScript = Join-Path $PSScriptRoot "program_fpga.tcl"
$bitFile = Join-Path $repoRoot "artifacts\GPS_Acquisition_FPGA.bit"
$commonPs1 = Join-Path $PSScriptRoot 'vivado_common.ps1'

if (-not (Test-Path $commonPs1)) {
    throw "No se encontro utilitario comun: $commonPs1"
}
. $commonPs1
Test-BuildPreflight -RepoRoot $repoRoot -RequireBitstream

if (-not (Test-Path $tclScript)) {
    throw "No se encontro script TCL: $tclScript"
}

if (-not (Test-Path $bitFile)) {
    throw "Bitstream no encontrado: $bitFile. Ejecuta antes scripts/run_vivado_build.ps1"
}

# ---------------------------------------------------------------------------
# Metodo principal: djtgcfg (Digilent Adept Runtime)
# ---------------------------------------------------------------------------
$djtgPaths = @(
    "djtgcfg.exe",
    "C:\Program Files\Digilent\Adept2\djtgcfg.exe",
    "C:\Program Files (x86)\Digilent\Adept2\djtgcfg.exe"
)
$djtg = $null
foreach ($p in $djtgPaths) {
    if (Get-Command $p -ErrorAction SilentlyContinue) { $djtg = $p; break }
    if (Test-Path $p) { $djtg = $p; break }
}
if ($djtg) {
    Write-Host "djtgcfg detectado: $djtg. Buscando Basys-3..."
    $enumOut = & $djtg enum 2>&1
    $devLine = $enumOut | Where-Object { $_ -match 'Basys.?3' } | Select-Object -First 1
    if ($devLine -and $devLine -match 'Device:\s*(\S+)') {
        $devName = $Matches[1]
        Write-Host "Programando $devName con djtgcfg (via rapida)..."
        & $djtg prog -d $devName -i 0 -f $bitFile
        if ($LASTEXITCODE -eq 0) {
            Write-Host "FPGA programada correctamente."
            exit 0
        }
        Write-Host "WARN: djtgcfg salio con codigo $LASTEXITCODE, usando Vivado como respaldo..."
    } else {
        Write-Host "WARN: djtgcfg enum no encontro Basys-3. Salida: $($enumOut -join ' | ')"
        Write-Host "      Usando Vivado..."
    }
}

# ---------------------------------------------------------------------------
# Metodo de respaldo: Vivado batch mode
# ---------------------------------------------------------------------------

$vivadoExe = Get-VivadoCommand

Write-Host "Vivado: $vivadoExe"
Write-Host "Programando FPGA con artifacts/GPS_Acquisition_FPGA.bit ..."

$targetFilter = $env:PROGRAM_HW_TARGET_FILTER
$deviceFilter = $env:PROGRAM_HW_DEVICE_FILTER
if ($targetFilter) {
    Write-Host "Filtro target JTAG: $targetFilter"
}
if ($deviceFilter) {
    Write-Host "Filtro dispositivo HW: $deviceFilter"
}

Push-Location $repoRoot
try {
    $vivadoArgs = @(
        '-mode', 'batch',
        '-source', $tclScript,
        '-nojournal',
        '-nolog',
        '-tclargs',
        $bitFile
    )
    if ($targetFilter) { $vivadoArgs += $targetFilter }
    if ($deviceFilter) { $vivadoArgs += $deviceFilter }

    & $vivadoExe @vivadoArgs
    if ($LASTEXITCODE -ne 0) { throw "Vivado ha salido con error (code $LASTEXITCODE)" }
    Write-Host "FPGA programada correctamente."
} finally {
    Pop-Location
}
