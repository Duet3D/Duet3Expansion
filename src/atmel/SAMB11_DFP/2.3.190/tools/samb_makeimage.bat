@echo off
::  Params: <patch> <application> <entry function> <stack start label> <stack end label> <application start label>
echo "%~dp0LineFetcher.exe" -m "%~dpn2.map" -l %3;%4;%5;%6
"%~dp0LineFetcher.exe" -m "%~dpn2.map" -l %3;%4;%5;%6 > tmp.txt
for /f "delims= " %%F in ('findstr /r /c:"0x[A-Za-z0-9]*.*%3" "tmp.txt"') do set address=%%F
for /f "delims= " %%F in ('findstr /r /c:"0x[A-Za-z0-9]*.*%4" "tmp.txt"') do set stack_start=%%F
for /f "delims= " %%F in ('findstr /r /c:"0x[A-Za-z0-9]*.*%5" "tmp.txt"') do set stack_end=%%F
for /f "delims= " %%F in ('findstr /r /c:"0x[A-Za-z0-9]*.*%6" "tmp.txt"') do set app_start_addr=%%F
del "tmp.txt"
echo "%~dp0SambImageCreator.exe" -f %1 -a %2 -o %~dpn2.img -r %address% -s %stack_start% -e %stack_end% -i %app_start_addr%
"%~dp0SambImageCreator.exe" -f %1 -a %2 -o "%~dpn2.img" -r %address% -s %stack_start% -e %stack_end% -i %app_start_addr%