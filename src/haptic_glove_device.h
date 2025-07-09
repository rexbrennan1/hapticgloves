#pragma once

#include <openvr_driver.h>
#include <string>
#include <thread>
#include <atomic>
#include <memory>

// Serial communication
#ifdef _WIN32
#include <windows.h>
#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#endif

struct GloveData {
    float quat_w, quat_x, quat_y, quat_z;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    uint16_t finger_curl[5];
    uint8_t checksum;
};

struct ForceData {
    uint8_t forces[5];
    uint8_t checksum;
};

class HapticGloveDevice : public vr::ITrackedDeviceServerDriver
{
public:
    HapticGloveDevice();
    virtual ~HapticGloveDevice();

    // ITrackedDeviceServerDriver methods
    virtual vr::EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId) override;
    virtual void Deactivate() override;
    virtual void EnterStandby() override;
    virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
    virtual vr::DriverPose_t GetPose() override;

    // Device specific methods
    void RunFrame();
    std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
    bool InitializeSerial();
    void CleanupSerial();
    bool ReadGloveData();
    void SendForceData();
    void UpdatePose();
    void ProcessFingerInputs();
    uint8_t CalculateChecksum(uint8_t* data, int length);

    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;
    std::string m_sSerialNumber;
    std::string m_sModelNumber;

    // Serial communication
#ifdef _WIN32
    HANDLE m_hSerial;
#else
    int m_serialFd;
#endif

    // Device state
    GloveData m_gloveData;
    ForceData m_forceData;
    vr::DriverPose_t m_pose;
    
    // Input handles
    vr::VRInputComponentHandle_t m_thumbCurl;
    vr::VRInputComponentHandle_t m_indexCurl;
    vr::VRInputComponentHandle_t m_middleCurl;
    vr::VRInputComponentHandle_t m_ringCurl;
    vr::VRInputComponentHandle_t m_pinkyCurl;
    
    // Haptic component
    vr::VRInputComponentHandle_t m_hapticComponent;
    
    // Threading
    std::thread m_poseThread;
    std::atomic<bool> m_bRunning;
    
    // Constants
    static const char* const k_pchSerialNumber;
    static const char* const k_pchModelNumber;
};
