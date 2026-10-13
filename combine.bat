@echo off
setlocal EnableExtensions
set "BUILD_DIR=%~dp0build"
if not "%~1"=="" set "BUILD_DIR=%~1"
set "MERGE_PYTHON=python"
if defined IDF_PYTHON_ENV_PATH set "MERGE_PYTHON=%IDF_PYTHON_ENV_PATH%\Scripts\python.exe"
"%MERGE_PYTHON%" "%~dp0scripts\merge_firmware.py" --build-dir "%BUILD_DIR%" --output "%~dp0flash_all.bin"
exit /b %errorlevel%
