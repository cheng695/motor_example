[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet('Debug', 'Release')]
    [string]$BuildType = 'Debug',
    [switch]$Clean,
    [ValidateRange(1, 1024)]
    [int]$Jobs = $(if ($env:JOBS) { [int]$env:JOBS } else { [Environment]::ProcessorCount })
)

$ErrorActionPreference = 'Stop'
$originalPath = $env:PATH
try {
    # Optional overrides point to the directories containing the executables.
    foreach ($toolDir in @($env:GCC_ARM_PATH, $env:NINJA_PATH, $env:CMAKE_PATH)) {
        if ($toolDir) { $env:PATH = "$toolDir;$env:PATH" }
    }
    if (-not (Get-Command cmake.exe -ErrorAction SilentlyContinue)) {
        $cmakeDir = Join-Path $env:ProgramFiles 'CMake/bin'
        if (Test-Path -LiteralPath (Join-Path $cmakeDir 'cmake.exe')) {
            $env:PATH = "$cmakeDir;$env:PATH"
        }
    }
    foreach ($tool in @('cmake.exe', 'ninja.exe', 'arm-none-eabi-gcc.exe',
                         'arm-none-eabi-g++.exe', 'arm-none-eabi-objcopy.exe', 'arm-none-eabi-size.exe')) {
        if (-not (Get-Command $tool -CommandType Application -ErrorAction SilentlyContinue)) {
            throw "Missing tool: $tool. Install it and add its bin directory to PATH, or set GCC_ARM_PATH / NINJA_PATH / CMAKE_PATH."
        }
    }

    Push-Location -LiteralPath $PSScriptRoot
    try {
        & cmake.exe --preset $BuildType
        if ($LASTEXITCODE -ne 0) { throw "CMake configure failed (exit $LASTEXITCODE)." }
        # clangd always indexes Debug, including when the first build is Release.
        if ($BuildType -ne 'Debug' -and -not (Test-Path -LiteralPath 'build/Debug/compile_commands.json')) {
            & cmake.exe --preset Debug
            if ($LASTEXITCODE -ne 0) { throw "CMake Debug configure failed (exit $LASTEXITCODE)." }
        }
        $buildArgs = @('--build', '--preset', $BuildType, '--parallel', "$Jobs")
        if ($Clean) { $buildArgs += '--clean-first' }
        & cmake.exe @buildArgs
        if ($LASTEXITCODE -ne 0) { throw "Build failed (exit $LASTEXITCODE)." }
        Write-Host "Build complete: $PSScriptRoot/build/$BuildType/motor_example.{elf,hex,bin}"
    }
    finally { Pop-Location }
}
finally { $env:PATH = $originalPath }
