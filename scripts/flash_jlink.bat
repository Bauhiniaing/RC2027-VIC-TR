@echo off
chcp 65001 >nul
rem J-Link 烧录脚本  用法: scripts\flash_jlink.bat [Debug|Release]
setlocal
set BUILD=%~1
if "%BUILD%"=="" set BUILD=Debug
set DEVICE=STM32H723VG
set SPEED=4000
set HEX=%~dp0..\build\%BUILD%\H7_TR.hex

rem 默认用 ST VS Code 插件自带的 J-Link（不用另外安装）
set "JLINK=%LOCALAPPDATA%\stm32cube\bundles\jlink-gdbserver\8.38.0+st.1\bin\JLink.exe"
if not exist "%JLINK%" set JLINK=JLink.exe

if not exist "%HEX%" (echo [错误] 找不到 %HEX%，请先编译 & exit /b 1)

set CMD=%TEMP%\h7tr_flash.jlink
(
echo r
echo h
echo loadfile "%HEX%"
echo r
echo g
echo qc
) > "%CMD%"

"%JLINK%" -device %DEVICE% -if SWD -speed %SPEED% -autoconnect 1 -NoGui 1 -ExitOnError 1 -CommandFile "%CMD%"
set RC=%ERRORLEVEL%
del "%CMD%" >nul 2>nul
if %RC% neq 0 (echo [失败] 烧录出错 & exit /b %RC%)
echo [成功] 烧录完成
