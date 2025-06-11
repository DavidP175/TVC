@echo off
echo Building project...

:: Create build directory if it doesn't exist
if not exist build mkdir build
cd build

:: Configure with CMake
echo Configuring with CMake...
cmake -G "MinGW Makefiles" ..

:: Build the project
echo Building...
cmake --build .

:: Check if build was successful
if %ERRORLEVEL% EQU 0 (
    echo Build successful!
    
    :: Check if STM32 is in DFU mode
    echo Checking for STM32 in DFU mode...
    STM32_Programmer_CLI -l | findstr "STM32  BOOTLOADER" > nul
    if %ERRORLEVEL% EQU 0 (
        echo STM32 found in DFU mode. Uploading...
        STM32_Programmer_CLI -c port=USB1 -w TVC1.elf
        
        if %ERRORLEVEL% EQU 0 (
            echo Upload successful!
            echo Press RESET button to start the program.
        ) else (
            echo Upload failed!
        )
    ) else (
        echo STM32 not found in DFU mode.
        echo Please:
        echo 1. Hold BOOT0 button
        echo 2. Press RESET button
        echo 3. Release BOOT0 button
        echo Then run this script again.
    )
) else (
    echo Build failed!
)

pause 