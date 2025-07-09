#pragma once

#include <string>
#include <openvr_driver.h>

class DriverLog
{
public:
    static void Init();
    static void Cleanup();
    static void Log(const char* pchFormat, ...);
    
private:
    static vr::IVRDriverLog* m_pLogFile;
};
