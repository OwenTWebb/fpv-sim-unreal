# Compiles the FPV sim (Unreal Engine 5.8 + Visual Studio 2022). Close the sim first: a running copy locks the DLL.
#   powershell -ExecutionPolicy Bypass -File fpv\build.ps1
param([string]$Unreal = "C:\Program Files\Epic Games\UE_5.8")
$ErrorActionPreference = "Stop"
$proj = Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) "FpvSim.uproject"
& (Join-Path $Unreal "Engine\Build\BatchFiles\Build.bat") FpvSimEditor Win64 Development "-Project=$proj" -WaitMutex -NoHotReloadFromIDE
if ($LASTEXITCODE -ne 0) { throw "build failed" }
