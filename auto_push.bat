@echo off
title AUTO GITHUB BACKUP

REM === PINDAH KE FOLDER PROJECT ===
cd /d %~dp0

:loop
echo ============================
echo Checking changes...

git add .

git diff --cached --quiet
IF %ERRORLEVEL%==0 (
    echo No changes
) ELSE (
    echo Changes found, pushing...
    git commit -m "auto backup %date% %time%"
    git push
)

timeout /t 120 >nul
goto loop