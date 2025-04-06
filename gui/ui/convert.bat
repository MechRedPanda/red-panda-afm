@echo off
setlocal enabledelayedexpansion

for %%f in (*.ui) do (
    set "filename=%%~nf"
    pyuic6 "%%f" -o "!filename!.py"
    echo Converted %%f to !filename!.py
)
