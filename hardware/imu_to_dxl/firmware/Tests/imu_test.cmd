@echo off
setlocal
if not "%~1"=="" (
  call "%~1" >nul
  if errorlevel 1 exit /b 1
)
where cl.exe >nul 2>nul
if errorlevel 1 (
  echo Run from an MSVC Developer Command Prompt, or pass the full vcvars64.bat path as the first argument.
  exit /b 1
)
cd /d "%~dp0.."
if not exist Build\Tests\imu mkdir Build\Tests\imu
cl /nologo /TC /std:c11 /utf-8 /W3 /D_CRT_SECURE_NO_WARNINGS /D__weak= /I Tests\imu_board /I Core\Inc /I Drivers\LSM6DSV16X Tests\imu_test.c Core\Src\imu.c Drivers\LSM6DSV16X\lsm6dsv16x_reg.c /FoBuild\Tests\imu\ /FeBuild\Tests\imu\imu_test.exe
if errorlevel 1 exit /b 1
Build\Tests\imu\imu_test.exe
exit /b %errorlevel%
