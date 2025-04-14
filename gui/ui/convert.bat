@echo off
setlocal enabledelayedexpansion

for %%f in (*.ui) do (
    set "filename=%%~nf"
    pyuic6 "%%f" -o "%%~nf.py"
    echo Converted %%f to %%~nf.py
)