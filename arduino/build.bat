@echo off
setlocal enabledelayedexpansion

set AVR_GCC_DIR=C:\Users\pma\AppData\Local\Arduino15\packages\arduino\tools\avr-gcc\7.3.0-atmel3.6.1-arduino7\bin
set CORE_DIR=C:\Users\pma\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6\cores\arduino
set VARIANT_DIR=C:\Users\pma\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6\variants\mega
set SKETCH_DIR=C:\Users\pma\AppData\Local\arduino\sketches\3B8C799CE9CA35CAD327268F4BF117AB
set LIB_WIRE=C:\Users\pma\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6\libraries\Wire\src
set CORE_A=%SKETCH_DIR%\core\core.a

set INC=-I"%CORE_DIR%" -I"%VARIANT_DIR%"
set OPTS=-g -Os -w -ffunction-sections -fdata-sections -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10607 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR

echo Compiling ArduinoFDC.ino.cpp...
"%AVR_GCC_DIR%\avr-g++" -c %OPTS% -std=gnu++11 -fpermissive -fno-exceptions -fno-threadsafe-statics -Wno-error=narrowing %INC% -I"%LIB_WIRE%" "%SKETCH_DIR%\sketch\ArduinoFDC.ino.cpp" -o "%SKETCH_DIR%\sketch\ArduinoFDC.ino.cpp.o"
if errorlevel 1 goto error

echo Compiling ArduinoFDC.cpp...
"%AVR_GCC_DIR%\avr-g++" -c %OPTS% -std=gnu++11 -fpermissive -fno-exceptions -fno-threadsafe-statics -Wno-error=narrowing %INC% -I"%LIB_WIRE%" "%SKETCH_DIR%\sketch\ArduinoFDC.cpp" -o "%SKETCH_DIR%\sketch\ArduinoFDC.cpp.o"
if errorlevel 1 goto error

echo Compiling diskio.cpp...
"%AVR_GCC_DIR%\avr-g++" -c %OPTS% -std=gnu++11 -fpermissive -fno-exceptions -fno-threadsafe-statics -Wno-error=narrowing %INC% "%SKETCH_DIR%\sketch\diskio.cpp" -o "%SKETCH_DIR%\sketch\diskio.cpp.o"
if errorlevel 1 goto error

echo Compiling ff.c...
"%AVR_GCC_DIR%\avr-gcc" -c %OPTS% -std=gnu11 %INC% "%SKETCH_DIR%\sketch\ff.c" -o "%SKETCH_DIR%\sketch\ff.c.o"
if errorlevel 1 goto error

echo Linking...
cd /d "%SKETCH_DIR%"
"%AVR_GCC_DIR%\avr-gcc" -Os -Wl,--gc-sections -flto -mmcu=atmega2560 -o "ArduinoFDC.ino.elf" "sketch\ArduinoFDC.ino.cpp.o" "sketch\ArduinoFDC.cpp.o" "sketch\diskio.cpp.o" "sketch\ff.c.o" "core\core.a" -lm
if errorlevel 1 goto error

echo Generating hex...
"%AVR_GCC_DIR%\avr-objcopy" -O ihex -R .eeprom "ArduinoFDC.ino.elf" "ArduinoFDC.ino.hex"
"%AVR_GCC_DIR%\avr-objcopy" -O binary -R .eeprom "ArduinoFDC.ino.elf" "ArduinoFDC.ino.with_bootloader.bin"
echo.
echo Build successful!
echo Output: %SKETCH_DIR%\ArduinoFDC.ino.hex
goto end

:error
echo.
echo Build FAILED!

:end
endlocal
