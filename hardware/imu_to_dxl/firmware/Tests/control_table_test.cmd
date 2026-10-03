@echo off
setlocal
cd /d "%~dp0.."
if not exist Build\host-tests mkdir Build\host-tests
set "TEMP=%CD%\Build\host-tests"
set "TMP=%TEMP%"
if not "%~1"=="" (
  call "%~1" >nul
  if errorlevel 1 exit /b 1
)
where cl.exe >nul 2>nul
if errorlevel 1 (
  echo Run from an MSVC Developer Command Prompt, or pass the full vcvars64.bat path as the first argument.
  exit /b 1
)
cl /nologo /std:c11 /W4 /WX /Od /RTC1 /I Core\Inc /FoBuild\host-tests\ /Fe:Build\host-tests\control_table_test.exe Tests\control_table_test.c Core\Src\control_table.c
if errorlevel 1 exit /b 1
Build\host-tests\control_table_test.exe
exit /b %errorlevel%
