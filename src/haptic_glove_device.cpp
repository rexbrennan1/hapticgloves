#include "haptic_glove_device.h"
#include "driverlog.h"
#include <algorithm>
#include <cmath>

const char* const HapticGloveDevice::k_pchSerialNumber = "HapticGlove_001";
const char* const HapticGloveDevice::k_pchModelNumber = "HapticGlove_v1";

HapticGloveDevice::HapticGloveDevice()
{
    m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
    m_sSerialNumber = k_pchSerialNumber;
    m_sModelNumber = k_pchModelNumber;
    
#ifdef _WIN32
    m_hSerial = INVALID_HANDLE_VALUE;
#else
    m_serialFd = -1;
#endif
    
    m_bRunning = false;
    
    // Initialize pose
    m_pose.poseIsValid = false;
    m_pose.deviceIsConnected = false;
    m_pose.poseTimeOffset = 0;
    m_pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
    m_pose.vecWorldFromDriverTranslation[0] = 0;
    m_pose.vecWorldFromDriverTranslation[1] = 0;
    m_pose.vecWorldFromDriverTranslation[2] = 0;
    m_pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
    m_pose.vecDriverFromHeadTranslation[0] = 0;
    m_pose.vecDriverFromHeadTranslation[1] = 0;
    m_pose.vecDriverFromHeadTranslation[2] = 0;
    m_pose.vecPosition[0] = 0;
    m_pose.vecPosition[1] = 0;
    m_pose.vecPosition[2] = 0;
    m_pose.qRotation = { 1, 0, 0, 0 };
    m_pose.vecVelocity[0] = 0;
    m_pose.vecVelocity[1] = 0;
    m_pose.vecVelocity[2] = 0;
    m_pose.vecAngularVelocity[0] = 0;
    m_pose.vecAngularVelocity[1] = 0;
    m_pose.vecAngularVelocity[2] = 0;
    m_pose.vecAcceleration[0] = 0;
    m_pose.vecAcceleration[1] = 0;
    m_pose.vecAcceleration[2] = 0;
    m_pose.vecAngularAcceleration[0] = 0;
    m_pose.vecAngularAcceleration[1] = 0;
    m_pose.vecAngularAcceleration[2] = 0;
    m_pose.result = vr::TrackingResult_Uninitialized;
    m_pose.willDriftInYaw = false;
    m_pose.shouldApplyHeadModel = false;
    
    // Initialize force data
    memset(&m_forceData, 0, sizeof(m_forceData));
    
    DriverLog::Log("HapticGloveDevice created");
}

HapticGloveDevice::~HapticGloveDevice()
{
    CleanupSerial();
    DriverLog::Log("HapticGloveDevice destroyed");
}

vr::EVRInitError HapticGloveDevice::Activate(vr::TrackedDeviceIndex_t unObjectId)
{
    m_unObjectId = unObjectId;
    m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);
    
    // Set device properties
    vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_SerialNumber_String, m_sSerialNumber.c_str());
    vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, m_sModelNumber.c_str());
    vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "Custom");
    vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");
    
    vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);
    vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_RightHand);
    
    // Input system
    vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &m_thumbCurl);
    vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/thumb", &m_thumbCurl, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/index", &m_indexCurl, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/middle", &m_middleCurl, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/ring", &m_ringCurl, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/pinky", &m_pinkyCurl, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    
    // Haptic component
    vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &m_hapticComponent);
    
    // Initialize serial communication
    if (!InitializeSerial())
    {
        DriverLog::Log("Failed to initialize serial communication");
        return vr::VRInitError_Driver_Failed;
    }
    
    // Start pose thread
    m_bRunning = true;
    m_poseThread = std::thread(&HapticGloveDevice::RunFrame, this);
    
    DriverLog::Log("HapticGloveDevice activated");
    return vr::VRInitError_None;
}

void HapticGloveDevice::Deactivate()
{
    m_bRunning = false;
    if (m_poseThread.joinable())
    {
        m_poseThread.join();
    }
    
    CleanupSerial();
    m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    DriverLog::Log("HapticGloveDevice deactivated");
}

void HapticGloveDevice::EnterStandby()
{
    DriverLog::Log("HapticGloveDevice entering standby");
}

void* HapticGloveDevice::GetComponent(const char* pchComponentNameAndVersion)
{
    return nullptr;
}

void HapticGloveDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t HapticGloveDevice::GetPose()
{
    return m_pose;
}

bool HapticGloveDevice::InitializeSerial()
{
#ifdef _WIN32
    // Try different COM ports
    for (int i = 1; i <= 20; i++)
    {
        std::string portName = "COM" + std::to_string(i);
        m_hSerial = CreateFileA(portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
        
        if (m_hSerial != INVALID_HANDLE_VALUE)
        {
            DCB dcbSerialParams = {0};
            dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
            
            if (GetCommState(m_hSerial, &dcbSerialParams))
            {
                dcbSerialParams.BaudRate = CBR_9600;
                dcbSerialParams.ByteSize = 8;
                dcbSerialParams.StopBits = ONESTOPBIT;
                dcbSerialParams.Parity = NOPARITY;
                
                if (SetCommState(m_hSerial, &dcbSerialParams))
                {
                    COMMTIMEOUTS timeouts = {0};
                    timeouts.ReadIntervalTimeout = 50;
                    timeouts.ReadTotalTimeoutConstant = 50;
                    timeouts.ReadTotalTimeoutMultiplier = 10;
                    timeouts.WriteTotalTimeoutConstant = 50;
                    timeouts.WriteTotalTimeoutMultiplier = 10;
                    
                    if (SetCommTimeouts(m_hSerial, &timeouts))
                    {
                        DriverLog::Log("Serial port %s opened successfully", portName.c_str());
                        return true;
                    }
                }
            }
            CloseHandle(m_hSerial);
            m_hSerial = INVALID_HANDLE_VALUE;
        }
    }
#else
    // Linux implementation
    const char* portNames[] = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"};
    
    for (const char* portName : portNames)
    {
        m_serialFd = open(portName, O_RDWR | O_NOCTTY);
        if (m_serialFd != -1)
        {
            struct termios tty;
            if (tcgetattr(m_serialFd, &tty) == 0)
            {
                cfsetospeed(&tty, B115200);
                cfsetispeed(&tty, B115200);
                tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
                tty.c_iflag &= ~IGNBRK;
                tty.c_lflag = 0;
                tty.c_oflag = 0;
                tty.c_cc[VMIN] = 0;
                tty.c_cc[VTIME] = 5;
                
                if (tcsetattr(m_serialFd, TCSANOW, &tty) == 0)
                {
                    DriverLog::Log("Serial port %s opened successfully", portName);
                    return true;
                }
            }
            close(m_serialFd);
            m_serialFd = -1;
        }
    }
#endif
    
    DriverLog::Log("Failed to open any serial port");
    return false;
}

void HapticGloveDevice::CleanupSerial()
{
#ifdef _WIN32
    if (m_hSerial != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_hSerial);
        m_hSerial = INVALID_HANDLE_VALUE;
    }
#else
    if (m_serialFd != -1)
    {
        close(m_serialFd);
        m_serialFd = -1;
    }
#endif
}

bool HapticGloveDevice::ReadGloveData()
{
#ifdef _WIN32
    if (m_hSerial == INVALID_HANDLE_VALUE)
        return false;
    
    DWORD bytesRead;
    if (ReadFile(m_hSerial, &m_gloveData, sizeof(m_gloveData), &bytesRead, NULL))
    {
        if (bytesRead == sizeof(m_gloveData))
        {
            uint8_t checksum = CalculateChecksum((uint8_t*)&m_gloveData, sizeof(m_gloveData) - 1);
            return checksum == m_gloveData.checksum;
        }
    }
#else
    if (m_serialFd == -1)
        return false;
    
    ssize_t bytesRead = read(m_serialFd, &m_gloveData, sizeof(m_gloveData));
    if (bytesRead == sizeof(m_gloveData))
    {
        uint8_t checksum = CalculateChecksum((uint8_t*)&m_gloveData, sizeof(m_gloveData) - 1);
        return checksum == m_gloveData.checksum;
    }
#endif
    
    return false;
}

void HapticGloveDevice::SendForceData()
{
    m_forceData.checksum = CalculateChecksum((uint8_t*)&m_forceData, sizeof(m_forceData) - 1);
    
#ifdef _WIN32
    if (m_hSerial != INVALID_HANDLE_VALUE)
    {
        DWORD bytesWritten;
        WriteFile(m_hSerial, &m_forceData, sizeof(m_forceData), &bytesWritten, NULL);
    }
#else
    if (m_serialFd != -1)
    {
        write(m_serialFd, &m_forceData, sizeof(m_forceData));
    }
#endif
}

uint8_t HapticGloveDevice::CalculateChecksum(uint8_t* data, int length)
{
    uint8_t sum = 0;
    for (int i = 0; i < length; i++)
    {
        sum += data[i];
    }
    return sum;
}

void HapticGloveDevice::UpdatePose()
{
    // Update rotation from quaternion
    m_pose.qRotation.w = m_gloveData.quat_w;
    m_pose.qRotation.x = m_gloveData.quat_x;
    m_pose.qRotation.y = m_gloveData.quat_y;
    m_pose.qRotation.z = m_gloveData.quat_z;
    
    // Set velocity from gyroscope (rad/s)
    m_pose.vecAngularVelocity[0] = m_gloveData.gyro_x;
    m_pose.vecAngularVelocity[1] = m_gloveData.gyro_y;
    m_pose.vecAngularVelocity[2] = m_gloveData.gyro_z;
    
    // Set acceleration from accelerometer (m/s²)
    m_pose.vecAcceleration[0] = m_gloveData.accel_x;
    m_pose.vecAcceleration[1] = m_gloveData.accel_y;
    m_pose.vecAcceleration[2] = m_gloveData.accel_z;
    
    // Set basic position (you may want to integrate acceleration for better tracking)
    // For now, we'll use a fixed position relative to the HMD
    m_pose.vecPosition[0] = 0.2; // 20cm to the right
    m_pose.vecPosition[1] = -0.1; // 10cm down
    m_pose.vecPosition[2] = -0.3; // 30cm forward
    
    m_pose.poseIsValid = true;
    m_pose.deviceIsConnected = true;
    m_pose.result = vr::TrackingResult_Running_OK;
    
    // Update pose in SteamVR
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_pose, sizeof(vr::DriverPose_t));
}

void HapticGloveDevice::ProcessFingerInputs()
{
    // Normalize finger curl values from ADC (0-1023) to 0.0-1.0
    float thumbCurl = static_cast<float>(m_gloveData.finger_curl[0]) / 1023.0f;
    float indexCurl = static_cast<float>(m_gloveData.finger_curl[1]) / 1023.0f;
    float middleCurl = static_cast<float>(m_gloveData.finger_curl[2]) / 1023.0f;
    float ringCurl = static_cast<float>(m_gloveData.finger_curl[3]) / 1023.0f;
    float pinkyCurl = static_cast<float>(m_gloveData.finger_curl[4]) / 1023.0f;
    
    // Update finger inputs
    vr::VRDriverInput()->UpdateScalarComponent(m_thumbCurl, thumbCurl, 0);
    vr::VRDriverInput()->UpdateScalarComponent(m_indexCurl, indexCurl, 0);
    vr::VRDriverInput()->UpdateScalarComponent(m_middleCurl, middleCurl, 0);
    vr::VRDriverInput()->UpdateScalarComponent(m_ringCurl, ringCurl, 0);
    vr::VRDriverInput()->UpdateScalarComponent(m_pinkyCurl, pinkyCurl, 0);
}

void HapticGloveDevice::RunFrame()
{
    while (m_bRunning)
    {
        if (ReadGloveData())
        {
            UpdatePose();
            ProcessFingerInputs();
            
            // Check for haptic events
            vr::VREvent_t vrEvent;
            while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent)))
            {
                if (vrEvent.eventType == vr::VREvent_Input_HapticVibration)
                {
                    if (vrEvent.trackedDeviceIndex == m_unObjectId)
                    {
                        // Convert haptic strength to force feedback
                        // This is a simple mapping - you may want to make it more sophisticated
                        float strength = vrEvent.data.hapticVibration.fAmplitude;
                        uint8_t force = static_cast<uint8_t>(strength * 255.0f);
                        
                        // Apply force to all fingers (you could make this more specific)
                        for (int i = 0; i < 5; i++)
                        {
                            m_forceData.forces[i] = force;
                        }
                        
                        SendForceData();
                    }
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100Hz update rate
    }
}
