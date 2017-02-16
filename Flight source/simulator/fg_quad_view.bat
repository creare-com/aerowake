set AUTOTESTDIR="%~dp0\..\..\..\ardupilot\Tools\autotest\aircraft"
c:
FOR /F "delims=" %%D in ('dir /b "\Program Files"\FlightGear*') DO set FGDIR=%%D
echo "Using FlightGear %FGDIR%"
echo "Using aircraft %AUTOTESTDIR%"
cd "\Program Files\%FGDIR%\bin"
REM "..\..\..\ardupilot\Tools\autotest\aircraft"
fgfs ^
    --native-fdm=socket,in,10,,5503,udp ^
    --fdm=external ^
    --aircraft=arducopter ^
    --fg-aircraft=%AUTOTESTDIR% ^
    --airport=KSFO ^
    --geometry=650x550 ^
    --bpp=32 ^
    --disable-anti-alias-hud ^
    --disable-hud-3d ^
    --disable-horizon-effect ^
    --timeofday=noon ^
    --disable-sound ^
    --disable-fullscreen ^
    --disable-random-objects ^
    --disable-ai-models ^
    --fog-disable ^
    --disable-specular-highlight ^
    --disable-anti-alias-hud ^
    --wind=0@0
pause
