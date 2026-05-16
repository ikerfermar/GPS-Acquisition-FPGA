param(
    [switch]$Quick,
    [int]$MaxJobs,
    [switch]$NoAutoQuick,
    [int]$TimeoutMinutes
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

# Valores por defecto
if ($TimeoutMinutes -eq 0) { $TimeoutMinutes = 60 }

$repoRoot = Split-Path -Parent $PSScriptRoot
$tclPath  = Join-Path $PSScriptRoot 'vivado_rebuild_and_bitstream.tcl'
$commonPs1 = Join-Path $PSScriptRoot 'vivado_common.ps1'
$buildDir = Join-Path $repoRoot 'vivado\GPS_Acquisition_FPGA_rebuild'
$xprPath = Join-Path $buildDir 'GPS_Acquisition_FPGA.xpr'

if (-not (Test-Path $commonPs1)) { throw "No se encontro utilitario comun: $commonPs1" }
. $commonPs1

function Invoke-VivadoBatchRun {
    param(
        [string]$VivadoCommand,
        [string]$TclFile,
        [string]$LogFile,
        [string]$JournalFile,
        [int]$TimeoutMinutes,
        [string]$AttemptLabel
    )

    if ($TimeoutMinutes -lt 1) {
        throw "TimeoutMinutes debe ser >= 1"
    }

    Write-Host "  [$AttemptLabel] Lanzando Vivado (timeout=${TimeoutMinutes} min)..." -ForegroundColor Cyan

    $procInfo = New-Object System.Diagnostics.ProcessStartInfo
    if ($VivadoCommand -match '\.bat$') {
        $procInfo.FileName = "cmd.exe"
        $procInfo.Arguments = "/c `"`"$VivadoCommand`" -mode batch -source `"$TclFile`" -log `"$LogFile`" -journal `"$JournalFile`"`""
    } else {
        $procInfo.FileName = $VivadoCommand
        $procInfo.Arguments = "-mode batch -source `"$TclFile`" -log `"$LogFile`" -journal `"$JournalFile`""
    }
    $procInfo.UseShellExecute = $false
    $procInfo.CreateNoWindow = $false

    try {
        $proc = [System.Diagnostics.Process]::Start($procInfo)
        $waitMs = $TimeoutMinutes * 60 * 1000

        if (-not $proc.WaitForExit($waitMs)) {
            Write-Warning "[$AttemptLabel] Timeout tras ${TimeoutMinutes} min. Terminando Vivado PID $($proc.Id)."
            Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
            return 124
        }

        $rawCode = $proc.ExitCode
        if ($null -eq $rawCode) { return 0 }
        return [int]$rawCode
    } catch {
        Write-Error "[$AttemptLabel] Error al lanzar proceso Vivado: $_"
        return 1
    }
}

# ---------------------------------------------------------------------------
# Localizar Vivado
# ---------------------------------------------------------------------------
$vivadoCmd = Get-VivadoCommand
if (-not (Test-Path $tclPath)) { throw "No se encontro TCL en $tclPath" }
Test-BuildPreflight -RepoRoot $repoRoot

# ---------------------------------------------------------------------------
# Terminar procesos Vivado inactivos por timeout (> 45 min)
# ---------------------------------------------------------------------------
Get-Process vivado -ErrorAction SilentlyContinue | ForEach-Object {
    $age = (Get-Date) - $_.StartTime
    if ($age.TotalMinutes -gt 45) {
        Write-Warning "Terminando proceso Vivado colgado PID $($_.Id) (arrancado hace $([int]$age.TotalMinutes) min)"
        Stop-Process -Id $_.Id -Force -ErrorAction SilentlyContinue
    } else {
        Write-Warning "Vivado activo PID $($_.Id) (arrancado hace $([int]$age.TotalMinutes) min) - puede haber conflicto"
    }
}

# ---------------------------------------------------------------------------
# Limpiar marcadores residuales de sintesis
# ---------------------------------------------------------------------------
$runsDir = Join-Path $repoRoot 'vivado\GPS_Acquisition_FPGA_rebuild\GPS_Acquisition_FPGA.runs'
if (Test-Path $runsDir) {
    $stale = @(Get-ChildItem -Path $runsDir -Filter '__synthesis_is_running__' -Recurse -ErrorAction SilentlyContinue)
    foreach ($m in $stale) {
        Remove-Item $m.FullName -Force -ErrorAction SilentlyContinue
        Write-Host "  [CLEAN] $($m.DirectoryName)\__synthesis_is_running__" -ForegroundColor DarkYellow
    }
    if ($stale.Count -gt 0) {
        Write-Host "  [CLEAN] $($stale.Count) marcadores stale eliminados" -ForegroundColor Yellow
    }
}

# ---------------------------------------------------------------------------
# Detectar nucleos fisicos para maximizar paralelismo
# ---------------------------------------------------------------------------
$cpuCores = if ($MaxJobs -gt 0) {
    [Math]::Min(16, $MaxJobs)
} else {
    try {
        $totalCores = (Get-CimInstance Win32_Processor | Measure-Object -Property NumberOfCores -Sum).Sum
        if (-not $totalCores -or $totalCores -lt 1) { $totalCores = 4 }
        [Math]::Min(8, [int]$totalCores)
    } catch { 4 }
}

# ---------------------------------------------------------------------------
# Configurar rutas de logs y modo de compilacion (Full / Incremental)
# ---------------------------------------------------------------------------
$logDir = Join-Path $repoRoot 'build_logs'
if (!(Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir | Out-Null }
$ts      = Get-Date -Format 'yyyyMMdd_HHmmss'
$logFile = Join-Path $logDir "build_$ts.log"
$jouFile = Join-Path $logDir "build_$ts.jou"
$metadataFile = Join-Path $logDir "build_$ts.json"
$latestMetadataFile = Join-Path $logDir 'latest_build.json'
$ipSignatureFile = Join-Path $logDir 'last_ip_signature.txt'

$quickMode = $Quick.IsPresent
$quickReason = ''
if (-not $quickMode -and -not $NoAutoQuick) {
    $currentIpSig = Get-IpSignature -RepoRoot $repoRoot
    $previousIpSig = if (Test-Path $ipSignatureFile) {
        (Get-Content -Path $ipSignatureFile -Raw -ErrorAction SilentlyContinue).Trim()
    } else {
        ''
    }

    if ((Test-Path $xprPath) -and $previousIpSig -and ($currentIpSig -eq $previousIpSig)) {
        $quickMode = $true
        $quickReason = 'auto: proyecto existente y firmas IP sin cambios'
    }
}

$env:VIVADO_RUN_JOBS = "$cpuCores"
if ($quickMode) {
    $env:VIVADO_QUICK_INCREMENTAL = '1'
} else {
    Remove-Item Env:VIVADO_QUICK_INCREMENTAL -ErrorAction SilentlyContinue
}

Write-Host ""
Write-Host "  ========== GPS Acquisition FPGA - BUILD ($cpuCores jobs) ==========" -ForegroundColor Cyan
if ($quickMode) {
    Write-Host "  Mode: Incremental (QUICK)" -ForegroundColor Yellow
    if ($quickReason) {
        Write-Host "  Quick reason: $quickReason" -ForegroundColor DarkYellow
    }
}
if ($MaxJobs -gt 0) {
    Write-Host "  Jobs override: $cpuCores" -ForegroundColor Yellow
}
Write-Host "  Log: $logFile" -ForegroundColor Gray
Write-Host ""

$tstart = Get-Date
$exitCode = 0
Push-Location $repoRoot
try {
    $exitCode = Invoke-VivadoBatchRun `
        -VivadoCommand $vivadoCmd `
        -TclFile $tclPath `
        -LogFile $logFile `
        -JournalFile $jouFile `
        -TimeoutMinutes $TimeoutMinutes `
        -AttemptLabel 'attempt-1'

    if ($exitCode -ne 0 -and $quickMode -and (Test-Path $logFile)) {
        $needsReset = Select-String -Path $logFile -Pattern 'needs to be reset before launching' -Quiet
        $alreadyRunning = Select-String -Path $logFile -Pattern "already running and cannot be relaunched|Run'?synth_1'? is already running" -Quiet
        if ($needsReset -or $alreadyRunning) {
            Write-Warning "Quick mode no reutilizable en este estado del run; reintentando en modo full automaticamente."
            $quickMode = $false
            if ($alreadyRunning) {
                $quickReason = 'fallback: quick encontro synth_1 en estado running stale'
            } else {
                $quickReason = 'fallback: quick requirio reset_run en synth_1'
            }
            Remove-Item Env:VIVADO_QUICK_INCREMENTAL -ErrorAction SilentlyContinue

            $exitCode = Invoke-VivadoBatchRun `
                -VivadoCommand $vivadoCmd `
                -TclFile $tclPath `
                -LogFile $logFile `
                -JournalFile $jouFile `
                -TimeoutMinutes $TimeoutMinutes `
                -AttemptLabel 'attempt-2-full'
        }
    }
} finally {
    Pop-Location
}

$elapsed = [math]::Round(((Get-Date) - $tstart).TotalMinutes, 2)
$bitFile = Join-Path $repoRoot 'vivado\GPS_Acquisition_FPGA_rebuild\GPS_Acquisition_FPGA.runs\impl_1\top_gps_system.bit'
$artifactBitFile = Join-Path $repoRoot 'artifacts\GPS_Acquisition_FPGA.bit'
$timingReportFile = Join-Path $repoRoot 'vivado\GPS_Acquisition_FPGA_rebuild\GPS_Acquisition_FPGA.runs\impl_1\top_gps_system_timing_summary_routed.rpt'
$timingMet = $false

Write-Host ""
if ($exitCode -eq 0) {
    Write-Host "  [SUCCESS] Build completado en $elapsed min" -ForegroundColor Green
} else {
    Write-Host "  [ERROR] Build fallo (exit $exitCode)" -ForegroundColor Red
}

if (($exitCode -eq 0) -and (Test-Path $bitFile)) {
    $sz = [math]::Round((Get-Item $bitFile).Length / 1MB, 2)
    Write-Host "  Bitstream ($sz MB): $bitFile" -ForegroundColor Green
    if (Test-Path $timingReportFile) {
        if (Select-String -Path $timingReportFile -Pattern 'Timing constraints are not met\.' -Quiet) {
            Write-Host "  [WARN] Timing constraints NOT MET" -ForegroundColor Yellow
        } elseif (Select-String -Path $timingReportFile -Pattern 'Timing constraints are met\.' -Quiet) {
            $timingMet = $true
            Write-Host "  [OK] Timing constraints met" -ForegroundColor Green
        } else {
            Write-Host "  [WARN] No se pudo determinar timing desde el reporte routed" -ForegroundColor Yellow
        }
    } elseif (Select-String -Path $logFile -Pattern 'Timing constraints are not met' -Quiet) {
        Write-Host "  [WARN] Timing constraints NOT MET" -ForegroundColor Yellow
    } else {
        $timingMet = $true
        Write-Host "  [OK] Timing constraints met (fallback log)" -ForegroundColor Green
    }

    if (Test-Path $artifactBitFile) {
        $artifactBytes = (Get-Item $artifactBitFile).Length
        if ($artifactBytes -lt 1000000) {
            throw "Bitstream copiado sospechosamente pequeno (<1MB): $artifactBitFile"
        }
    } else {
        Write-Host "  [WARN] Bitstream copiado no encontrado en artifacts" -ForegroundColor Yellow
    }
} elseif ($exitCode -eq 0) {
    Write-Host "  [WARN] Bitstream no encontrado en ruta esperada" -ForegroundColor Yellow
}

if ($exitCode -ne 0 -and (Test-Path $logFile)) {
    Write-Host ""
    Select-String -Path $logFile -Pattern '^ERROR:' | Select-Object -Last 8 |
        ForEach-Object { Write-Host "    $($_.Line)" -ForegroundColor Red }
}

$metadata = [ordered]@{
    timestamp = (Get-Date).ToString('o')
    mode = if ($quickMode) { 'quick' } else { 'full' }
    quickReason = $quickReason
    jobs = $cpuCores
    exitCode = $exitCode
    elapsedMinutes = $elapsed
    timingMet = $timingMet
    logFile = $logFile
    journalFile = $jouFile
    timingReportFile = $timingReportFile
    sourceProject = $xprPath
    bitstreamImpl = $bitFile
    bitstreamArtifacts = $artifactBitFile
    bitstreamImplExists = (Test-Path $bitFile)
    bitstreamArtifactsExists = (Test-Path $artifactBitFile)
}
$metadata | ConvertTo-Json -Depth 4 | Set-Content -Path $metadataFile -Encoding UTF8
$metadata | ConvertTo-Json -Depth 4 | Set-Content -Path $latestMetadataFile -Encoding UTF8

$newSig = Get-IpSignature -RepoRoot $repoRoot
Set-Content -Path $ipSignatureFile -Value $newSig -Encoding ASCII

Write-Host "  Metadata: $metadataFile" -ForegroundColor Gray
Write-Host ""

if ($exitCode -ne 0) {
    exit $exitCode
}
