@echo off
echo Checking DLL dependencies...

set "DLL_PATH=C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\haptic_glove_driver\bin\win64\driver_haptic_glove_driver.dll"
set "STEAMVR_PATH=C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64"

echo.
echo === DLL Info ===
if exist "%DLL_PATH%" (
    echo DLL exists: YES
    echo Path: %DLL_PATH%
    
    echo.
    echo === DLL Dependencies ===
    dumpbin /dependents "%DLL_PATH%"
    
    echo.
    echo === Checking for openvr_api.dll ===
    if exist "%STEAMVR_PATH%\openvr_api.dll" (
        echo   ✅ openvr_api.dll found in SteamVR folder
        
        REM Check if it's copied to driver folder
        if exist "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\haptic_glove_driver\bin\win64\openvr_api.dll" (
            echo   ✅ openvr_api.dll already copied to driver folder
        ) else (
            echo   ❌ openvr_api.dll NOT in driver folder - COPYING NOW...
            copy "%STEAMVR_PATH%\openvr_api.dll" "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\haptic_glove_driver\bin\win64\"
            if %errorlevel%==0 (
                echo   ✅ Copy successful!
            ) else (
                echo   ❌ Copy failed - run as administrator
            )
        )
    ) else (
        echo   ❌ openvr_api.dll NOT found in SteamVR folder
    )
    
    echo.
    echo === Checking Visual C++ Runtime ===
    where vcruntime140.dll >nul 2>&1
    if %errorlevel%==0 (
        echo   ✅ vcruntime140.dll found
    ) else (
        echo   ❌ vcruntime140.dll NOT found
    )
    
    where msvcp140.dll >nul 2>&1
    if %errorlevel%==0 (
        echo   ✅ msvcp140.dll found
    ) else (
        echo   ❌ msvcp140.dll NOT found
    )
    
) else (
    echo ❌ DLL not found at: %DLL_PATH%
    echo.
    echo Check if driver was built and copied correctly.
)

echo.
echo === Try SteamVR again ===
echo After copying openvr_api.dll, restart SteamVR and test.
echo.

pause