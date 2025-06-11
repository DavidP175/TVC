@echo off
echo Building project...

:: Create build directory if it doesn't exist
if not exist build mkdir build
cd build

:: Configure and build
cmake -G "MinGW Makefiles" ..
cmake --build .

:: Check if build was successful
if %ERRORLEVEL% EQU 0 (
    echo Build successful!
    echo Uploading to STM32...
    STM32_Programmer_CLI -c port=USB1 -w TVC1.elf
) else (
    echo Build failed!
)

cd .. 