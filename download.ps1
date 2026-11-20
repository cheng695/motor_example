[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet('Debug', 'Release')]
    [string]$BuildType = 'Debug',
    [string]$JLinkExe = $env:JLINK_EXE,
    [ValidatePattern('^[A-Za-z0-9_()+.-]+$')]
    [string]$Device = $(if ($env:JLINK_DEVICE) { $env:JLINK_DEVICE } else { 'STM32F427II' }),
    [ValidateSet('SWD', 'JTAG')]
    [string]$Interface = $(if ($env:JLINK_INTERFACE) { $env:JLINK_INTERFACE } else { 'SWD' }),
    [ValidateRange(1, 50000)]
    [int]$Speed = $(if ($env:JLINK_SPEED) { [int]$env:JLINK_SPEED } else { 4000 }),
    [ValidatePattern('^\d*$')]
    [string]$SerialNumber = $env:JLINK_SN,
    [switch]$SkipBuild,
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'
if (-not $JLinkExe) {
    $jlinkCommand = Get-Command JLink.exe -CommandType Application -ErrorAction SilentlyContinue
    if ($jlinkCommand) { $JLinkExe = $jlinkCommand.Source }
    else {
        $candidates = foreach ($installRoot in @($env:ProgramFiles, ${env:ProgramFiles(x86)})) {
            if ($installRoot) {
                Get-ChildItem -Path (Join-Path $installRoot 'SEGGER/JLink*/JLink.exe') -File -ErrorAction SilentlyContinue
            }
        }
        $latest = $candidates | Sort-Object LastWriteTime -Descending | Select-Object -First 1
        if ($latest) { $JLinkExe = $latest.FullName }
    }
}
if (-not $JLinkExe -or -not (Get-Command $JLinkExe -CommandType Application -ErrorAction SilentlyContinue)) {
    throw 'J-Link Commander not found. Install SEGGER J-Link or set JLINK_EXE to the full path of JLink.exe.'
}

if (-not $SkipBuild) { & (Join-Path $PSScriptRoot 'build.ps1') -BuildType $BuildType }
$firmware = Join-Path $PSScriptRoot "build/$BuildType/motor_example.elf"
if (-not (Test-Path -LiteralPath $firmware -PathType Leaf)) {
    throw "Firmware not found: $firmware. Run build.bat $BuildType first."
}
$commands = @('ExitOnError 1', 'r', 'h', "loadfile `"$firmware`"", 'r', 'g', 'exit')
$commandFile = [IO.Path]::GetTempFileName()
try {
    # Use the Windows ANSI encoding expected by Commander, including paths with spaces.
    [IO.File]::WriteAllLines($commandFile, [string[]]$commands, [Text.Encoding]::Default)
    $jlinkArgs = @('-device', $Device, '-if', $Interface, '-speed', "$Speed",
                   '-autoconnect', '1', '-ExitOnError', '1', '-NoGui', '1', '-CommandFile', $commandFile)
    if ($SerialNumber) { $jlinkArgs += @('-USB', $SerialNumber) }
    Write-Host "J-Link: $JLinkExe"
    Write-Host "Target: $Device / $Interface / $Speed kHz"
    if ($DryRun) {
        Write-Host ('Arguments: ' + ($jlinkArgs -join ' '))
        $commands | ForEach-Object { Write-Host $_ }
        Write-Host 'Dry run complete. No connection or download performed.'
    }
    else {
        & $JLinkExe @jlinkArgs
        if ($LASTEXITCODE -ne 0) { throw "J-Link download failed (exit $LASTEXITCODE)." }
        Write-Host 'Download complete. Target reset and running.'
    }
}
finally { Remove-Item -LiteralPath $commandFile -ErrorAction SilentlyContinue }
