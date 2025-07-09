#include "driverlog.h"
#include <stdio.h>
#include <stdarg.h>

vr::IVRDriverLog* DriverLog::m_pLogFile = nullptr;

void DriverLog::Init()
{
    m_pLogFile = vr::VRDriverLog();
}

void DriverLog::Cleanup()
{
    m_pLogFile = nullptr;
}

void DriverLog::Log(const char* pchFormat, ...)
{
    if (m_pLogFile)
    {
        va_list args;
        va_start(args, pchFormat);
        
        char buffer[1024];
        vsnprintf(buffer, sizeof(buffer), pchFormat, args);
        
        m_pLogFile->Log(buffer);
        
        va_end(args);
    }
}
