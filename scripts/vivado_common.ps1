Set-StrictMode -Version Latest

function Get-VivadoCommand {
    [CmdletBinding()]
    param()

    if ($env:XILINX_VIVADO) {
        $candidate = Join-Path $env:XILINX_VIVADO 'bin/vivado.bat'
        if (Test-Path $candidate) {
            return $candidate
        }
    }

    $fromPath = Get-Command vivado.bat -ErrorAction SilentlyContinue
    if (-not $fromPath) {
        $fromPath = Get-Command vivado -ErrorAction SilentlyContinue
    }
    if ($fromPath) {
        return $fromPath.Source
    }

    throw "No se encontro Vivado. Añade vivado(.bat) al PATH o define XILINX_VIVADO."
}

function Get-VivadoBinDirectory {
    [CmdletBinding()]
    param()

    return Split-Path -Parent (Get-VivadoCommand)
}

function Test-BuildPreflight {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)]
        [string]$RepoRoot,

        [switch]$RequireBitstream
    )

    $requiredFiles = @(
        (Join-Path $RepoRoot 'scripts\vivado_rebuild_and_bitstream.tcl'),
        (Join-Path $RepoRoot 'src\hdl\top_gps_system.vhd'),
        (Join-Path $RepoRoot 'src\constraints\Basys_3.xdc'),
        (Join-Path $RepoRoot 'src\ip\ca_chips_rom.xci'),
        (Join-Path $RepoRoot 'src\ip\clk_wiz_0.xci'),
        (Join-Path $RepoRoot 'src\ip\xfft_0.xci'),
        (Join-Path $RepoRoot 'src\ip\xfft_1.xci')
    )

    foreach ($f in $requiredFiles) {
        if (-not (Test-Path $f)) {
            throw "Preflight fallo: no existe fichero requerido: $f"
        }
    }

    if ($RequireBitstream) {
        $bitFile = Join-Path $RepoRoot 'artifacts\GPS_Acquisition_FPGA.bit'
        if (-not (Test-Path $bitFile)) {
            throw "Preflight fallo: bitstream no encontrado: $bitFile"
        }
    }
}

function Get-IpSignature {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)]
        [string]$RepoRoot
    )

    $ipDir = Join-Path $RepoRoot 'src\ip'
    if (-not (Test-Path $ipDir)) {
        throw "No existe carpeta de IP: $ipDir"
    }

    $ipFiles = Get-ChildItem -Path $ipDir -File -Recurse |
        Where-Object { $_.Extension -in @('.xci', '.coe', '.mif') } |
        Sort-Object FullName

    if ($ipFiles.Count -eq 0) {
        return 'NO_IP_FILES'
    }

    $signatureText = ($ipFiles | ForEach-Object {
        "$($_.FullName)|$($_.Length)|$($_.LastWriteTimeUtc.Ticks)"
    }) -join "`n"

    $sha = [System.Security.Cryptography.SHA256]::Create()
    try {
        $bytes = [System.Text.Encoding]::UTF8.GetBytes($signatureText)
        $hash = $sha.ComputeHash($bytes)
        return ([System.BitConverter]::ToString($hash)).Replace('-', '').ToLowerInvariant()
    } finally {
        $sha.Dispose()
    }
}
