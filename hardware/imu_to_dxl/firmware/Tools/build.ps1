[CmdletBinding()]
param(
    [string]$ToolchainRoot = 'E:\Keil_v5\ARM\ARMCLANG',
    [string[]]$ExtraDefine = @()
)

$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$buildRoot = Join-Path $projectRoot 'Build'
$objectRoot = Join-Path $buildRoot 'obj'
$temporaryRoot = Join-Path $buildRoot 'tmp'
$binRoot = Join-Path $ToolchainRoot 'bin'
$compiler = Join-Path $binRoot 'armclang.exe'
$assembler = Join-Path $binRoot 'armasm.exe'
$linker = Join-Path $binRoot 'armlink.exe'
$fromelf = Join-Path $binRoot 'fromelf.exe'
foreach ($tool in @($compiler, $assembler, $linker, $fromelf)) {
    if (!(Test-Path -LiteralPath $tool -PathType Leaf)) { throw "Missing Arm tool: $tool" }
}
New-Item -ItemType Directory -Path $buildRoot, $objectRoot, $temporaryRoot -Force | Out-Null

# Keep compiler temporary files inside the project, on its selected drive.
$previousTemp = $env:TEMP
$previousTmp = $env:TMP
$env:TEMP = $temporaryRoot
$env:TMP = $temporaryRoot
$logFile = Join-Path $buildRoot 'build.log'
[System.IO.File]::WriteAllText($logFile, "imu_to_dxl build: $([DateTimeOffset]::Now.ToString('o'))`r`n")
function Invoke-ArmTool {
    param([string]$Executable, [string[]]$ToolArguments)
    # Windows PowerShell 5 treats native stderr as ErrorRecord even for warnings.
    # Preserve native exit status while still recording all diagnostics.
    $previousErrorPreference = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $toolOutput = & $Executable @ToolArguments 2>&1
        $toolExitCode = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $previousErrorPreference
    }
    foreach ($line in $toolOutput) {
        Write-Host "$line"
        Add-Content -LiteralPath $logFile -Value "$line"
    }
    if ($toolExitCode -ne 0) { throw "$(Split-Path -Leaf $Executable) failed ($toolExitCode). See $logFile" }
}

try {
    # Remove only final products from a previous build so failures cannot leave stale firmware.
    foreach ($extension in @('axf', 'hex', 'bin', 'map', 'build.json')) {
        $oldOutput = Join-Path $buildRoot "imu_to_dxl.$extension"
        if (Test-Path -LiteralPath $oldOutput) { Remove-Item -LiteralPath $oldOutput -Force }
    }
    Invoke-ArmTool $compiler @('--version')
    $includes = @('Core/Inc', 'Drivers/CMSIS/Include', 'Drivers/CMSIS/Device/ST/STM32G0xx/Include', 'Drivers/LSM6DSV16X')
    $common = @('--target=arm-arm-none-eabi', '-mcpu=cortex-m0plus', '-mthumb', '-mfloat-abi=soft', '-std=c11', '-Oz', '-g', '-gdwarf-4', '-ffunction-sections', '-fdata-sections', '-fshort-enums', '-fshort-wchar', '-Wall', '-Wextra', '-Wno-unused-parameter', '-DSTM32G031xx', '-DHSE_VALUE=16000000', '-D__MICROLIB')
    foreach ($define in $ExtraDefine) { $common += "-D$define" }
    foreach ($directory in $includes) { $common += @('-I', (Join-Path $projectRoot $directory)) }
    $sources = @('Core/Src/main.c', 'Core/Src/board.c', 'Core/Src/control_table.c', 'Core/Src/imu.c', 'Core/Src/protocol.c', 'Drivers/LSM6DSV16X/lsm6dsv16x_reg.c', 'Drivers/CMSIS/Device/ST/STM32G0xx/Source/system_stm32g0xx.c')
    $objects = @()
    $commands = @()
    foreach ($source in $sources) {
        $sourcePath = Join-Path $projectRoot $source
        if (!(Test-Path -LiteralPath $sourcePath)) { throw "Missing source: $sourcePath" }
        $objectPath = Join-Path $objectRoot (([System.IO.Path]::GetFileNameWithoutExtension($source)) + '.o')
        $arguments = $common + @('-c', $sourcePath, '-o', $objectPath)
        Write-Host "Compiling $source"
        Invoke-ArmTool $compiler $arguments
        $objects += $objectPath
        $commands += @{directory=$projectRoot; file=$sourcePath; arguments=(@($compiler) + $arguments)}
    }
    $commands | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $buildRoot 'compile_commands.json') -Encoding utf8
    $startupObject = Join-Path $objectRoot 'startup_stm32g031xx.o'
    # ST supplies the Arm syntax startup. Suppress only the legacy-tool notice A1950W.
    Invoke-ArmTool $assembler @('--cpu=Cortex-M0plus', '--apcs=interwork', '--debug', '--diag_suppress=1950', '--pd', '__MICROLIB SETA 1', '-o', $startupObject, (Join-Path $projectRoot 'Startup/startup_stm32g031xx.s'))
    $objects += $startupObject
    $elf = Join-Path $buildRoot 'imu_to_dxl.axf'
    $map = Join-Path $buildRoot 'imu_to_dxl.map'
    $linkArguments = @('--cpu=Cortex-M0plus', '--library_type=microlib', '--entry=Reset_Handler', '--scatter', (Join-Path $projectRoot 'MDK-ARM/imu_to_dxl.sct'), '--map', '--symbols', '--xref', '--info=sizes,totals,unused,stack', '--list', $map, '--strict', '-o', $elf) + $objects
    Invoke-ArmTool $linker $linkArguments
    Invoke-ArmTool $fromelf @('--i32combined', '--output', (Join-Path $buildRoot 'imu_to_dxl.hex'), $elf)
    Invoke-ArmTool $fromelf @('--bin', '--output', (Join-Path $buildRoot 'imu_to_dxl.bin'), $elf)
    $products = @('imu_to_dxl.axf','imu_to_dxl.hex','imu_to_dxl.bin','imu_to_dxl.map') | ForEach-Object {
        $file = Get-Item -LiteralPath (Join-Path $buildRoot $_)
        @{name=$file.Name; bytes=$file.Length; sha256=(Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()}
    }
    @{createdAt=[DateTimeOffset]::Now.ToString('o'); toolchain=$ToolchainRoot; target='STM32G031F8P6'; extraDefines=$ExtraDefine; artifacts=$products} | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $buildRoot 'imu_to_dxl.build.json') -Encoding utf8
    Write-Host "Build successful. Firmware: $buildRoot\imu_to_dxl.hex"
} finally {
    $env:TEMP = $previousTemp
    $env:TMP = $previousTmp
}
