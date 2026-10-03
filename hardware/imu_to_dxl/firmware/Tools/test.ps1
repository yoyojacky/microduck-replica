[CmdletBinding()]
param([string]$PythonExe = '')
$ErrorActionPreference = 'Stop'
$project = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if (!$PythonExe) {
    $candidate = Get-Command python -ErrorAction SilentlyContinue
    if ($candidate -and $candidate.Source -notlike '*WindowsApps*') { $PythonExe = $candidate.Source }
}
if (!$PythonExe) { throw 'Supply -PythonExe with a real Python interpreter. Firmware builds do not require Python.' }
if (!(Get-Command cl.exe -ErrorAction SilentlyContinue)) {
    throw 'Run this script from an MSVC x64 Native Tools Command Prompt or Developer PowerShell so cl.exe is on PATH.'
}
$savedTemp = $env:TEMP; $savedTmp = $env:TMP
Push-Location -LiteralPath $project
try {
    $tempDir = Join-Path $project 'Build\host-tests'
    New-Item -ItemType Directory -Path $tempDir -Force | Out-Null
    $env:TEMP = $tempDir; $env:TMP = $tempDir
    & $PythonExe -B Tools\protocol_test.py
    if ($LASTEXITCODE) { throw 'Protocol tests failed' }
    & .\Tests\imu_test.cmd
    if ($LASTEXITCODE) { throw 'IMU tests failed' }
    & .\Tests\control_table_test.cmd
    if ($LASTEXITCODE) { throw 'Control table tests failed' }
    & $PythonExe -B Tests\bus_sim_test.py
    if ($LASTEXITCODE) { throw 'Bus simulation acceptance failed' }
    Write-Host 'All host tests passed. This test run did not connect to hardware.'
} finally {
    Pop-Location
    $env:TEMP = $savedTemp; $env:TMP = $savedTmp
}
