@echo off
SETLOCAL
echo specified team number is %1
set teamNum=%1

echo %teamNum:~0,2%
set locat=%2
set dest=%3

set rioAddress=10.10.86.2

scp -r lvuser@%rioAddress%:%locat% /$(ssh lvuser@%rioAddress% 'ls -t %locat% | head -n 1') %dest%

ENDLOCAL