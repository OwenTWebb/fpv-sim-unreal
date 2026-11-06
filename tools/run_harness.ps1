# Builds and runs the headless physics harness with the Visual Studio 2022 compiler.
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$vs = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath
$out = Join-Path $env:TEMP "fpv_harness"; New-Item -ItemType Directory -Force $out | Out-Null
cmd /c "`"$vs\VC\Auxiliary\Build\vcvars64.bat`" >nul 2>&1 && cl /nologo /O2 /std:c++17 /EHsc `"$here\quad_harness.cpp`" /Fe:`"$out\quad_harness.exe`" /Fo:`"$out\quad_harness.obj`""
if ($LASTEXITCODE -ne 0) { throw "harness build failed" }
& "$out\quad_harness.exe" @args
