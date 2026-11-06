# Starts the FPV sim in a window (add -Fullscreen for full screen). Extra arguments are passed to the game,
# e.g.  -FpvCam=1  (start in the chase camera)   -FpvTestFlight  (scripted flight, logged to Saved\Logs).
#   powershell -ExecutionPolicy Bypass -File fpv\run.ps1
param([string]$Unreal = "C:\Program Files\Epic Games\UE_5.8", [switch]$Fullscreen, [int]$ResX = 1600, [int]$ResY = 900)
$proj = Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) "FpvSim.uproject"
$mode = if ($Fullscreen) { "-fullscreen" } else { "-windowed", "-ResX=$ResX", "-ResY=$ResY" }
& (Join-Path $Unreal "Engine\Binaries\Win64\UnrealEditor.exe") $proj -game @mode @args
