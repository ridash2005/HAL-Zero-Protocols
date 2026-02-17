@echo off
setlocal

echo ==========================================================
echo  Communication Protocols - Full Verification Suite
echo ==========================================================
echo.

echo [1/2] Running Python Mock Verification...
python verification\mock_verify.py
if %errorlevel% neq 0 (
    echo [ERROR] Python tests failed!
    exit /b %errorlevel%
)
echo [PASS] Python tests passed.
echo.

echo [2/2] Generating and running Host C Tests (MSVC)...
echo Compiling test_drivers.c...

REM Check if cl is available
where cl >nul 2>nul
if %errorlevel% neq 0 (
    echo [ERROR] 'cl' compiler not found. Please run this script from a VS Developer Command Prompt.
    exit /b 1
)

REM Compile
cl /nologo /std:c11 /W4 /DUNIT_TEST /utf-8 ^
    /Idrivers/common ^
    /Iprotocol_stacks/modbus ^
    /Iprotocol_stacks/usb_cdc ^
    /Iprotocol_stacks/can_bus ^
    verification/test_drivers.c ^
    /Fe:test_drivers.exe

if %errorlevel% neq 0 (
    echo [ERROR] Compilation failed!
    exit /b %errorlevel%
)

echo Compilation successful. Running tests...
test_drivers.exe
if %errorlevel% neq 0 (
    echo [ERROR] C Unit Tests Failed!
    exit /b %errorlevel%
)

echo.
echo ==========================================================
echo  ALL VERIFICATION CHECKS PASSED SUCCESSFULLY (100%%)
echo ==========================================================
del test_drivers.obj test_drivers.exe
exit /b 0
