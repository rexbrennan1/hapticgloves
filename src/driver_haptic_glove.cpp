#include <openvr_driver.h>
#include "haptic_glove_device.h"
#include "driverlog.h"
#include <vector>
#include <thread>
#include <chrono>

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#endif

class HapticGloveServerDriver : public vr::IServerTrackedDeviceProvider
{
public:
    HapticGloveServerDriver() : m_pGloveDevice(nullptr) {}
    
    virtual vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override;
    virtual void Cleanup() override;
    virtual const char* const* GetInterfaceVersions() override;
    virtual void RunFrame() override;
    virtual bool ShouldBlockStandbyMode() override;
    virtual void EnterStandby() override;
    virtual void LeaveStandby() override;

private:
    std::unique_ptr<HapticGloveDevice> m_pGloveDevice;
};

vr::EVRInitError HapticGloveServerDriver::Init(vr::IVRDriverContext* pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
    
    DriverLog::Init();
    DriverLog::Log("HapticGloveServerDriver Init");
    
    // Create and activate the glove device
    m_pGloveDevice = std::make_unique<HapticGloveDevice>();
    vr::VRServerDriverHost()->TrackedDeviceAdded(m_pGloveDevice->GetSerialNumber().c_str(), 
                                                 vr::TrackedDeviceClass_Controller, 
                                                 m_pGloveDevice.get());
    
    return vr::VRInitError_None;
}

void HapticGloveServerDriver::Cleanup()
{
    DriverLog::Log("HapticGloveServerDriver Cleanup");
    
    if (m_pGloveDevice)
    {
        m_pGloveDevice.reset();
    }
    
    DriverLog::Cleanup();
}

const char* const* HapticGloveServerDriver::GetInterfaceVersions()
{
    return vr::k_InterfaceVersions;
}

void HapticGloveServerDriver::RunFrame()
{
    if (m_pGloveDevice)
    {
        m_pGloveDevice->RunFrame();
    }
}

bool HapticGloveServerDriver::ShouldBlockStandbyMode()
{
    return false;
}

void HapticGloveServerDriver::EnterStandby()
{
    DriverLog::Log("HapticGloveServerDriver EnterStandby");
}

void HapticGloveServerDriver::LeaveStandby()
{
    DriverLog::Log("HapticGloveServerDriver LeaveStandby");
}

static HapticGloveServerDriver g_serverDriver;

HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
    if (0 == strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName))
    {
        return &g_serverDriver;
    }
    
    if (pReturnCode)
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    
    return nullptr;
}
