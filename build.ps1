# Build script for VEGA ARIES v3 Flight Controller Firmware (Zephyr RTOS)
[CmdletBinding()]
param(
    [switch]$Pristine
)

$ErrorActionPreference = "Stop"

$workspaceDir = "C:\dev\zephyrproject"
$sdkDir = "C:\Users\denni\zephyr-sdk-0.17.4"
$venvDir = "$workspaceDir\.venv"
$appDir = "$PSScriptRoot\zephyr\app"

$env:PATH = "$venvDir\Scripts;" + $env:PATH
$env:ZEPHYR_BASE = "$workspaceDir\zephyr"
$env:ZEPHYR_SDK_INSTALL_DIR = $sdkDir
$env:ZEPHYR_TOOLCHAIN_VARIANT = "zephyr"

Write-Host "Building VEGA ARIES v3 Flight Controller..." -ForegroundColor Cyan

$buildArgs = @("build", "-b", "aries_v3", "$appDir", "-d", "$workspaceDir\build")
if ($Pristine) {
    $buildArgs += @("-p", "always")
}

& "$venvDir\Scripts\python.exe" -m west @buildArgs
if ($LASTEXITCODE -eq 0) {
    Write-Host "`nBuild SUCCESSFUL!" -ForegroundColor Green
    Write-Host "Binary: $workspaceDir\build\zephyr\zephyr.bin" -ForegroundColor Green
    Write-Host "ELF:    $workspaceDir\build\zephyr\zephyr.elf" -ForegroundColor Green
} else {
    Write-Error "Build failed with exit code $LASTEXITCODE"
}
