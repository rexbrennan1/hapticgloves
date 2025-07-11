//============ Copyright (c) Valve Corporation, All rights reserved. ============
//
// OpenVR Haptic Glove Driver - Fixed for Visual Studio 2022
//
//=============================================================================

#include <openvr_driver.h>
#include <windows.h>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <sstream>
#include <cstring>

using namespace vr;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

// Serial communication class
class SerialPort {
public:
    SerialPort(const std::string& portName, int baudRate) 
        : portName_(portName), baudRate_(baudRate), hSerial_(INVALID_HANDLE_VALUE) {}
    
    ~SerialPort() {
        if (hSerial_ != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial_);
        }
    }
    
    bool Open() {
        std::string fullPortName = "\\\\.\\" + portName_;
        
        hSerial_ = CreateFileA(
            fullPortName.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL
        );
        
        if (hSerial_ == INVALID_HANDLE_VALUE) {
            return false;
        }
        
        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        
        if (!GetCommState(hSerial_, &dcbSerialParams)) {
            CloseHandle(hSerial_);
            hSerial_ = INVALID_HANDLE_VALUE;
            return false;
        }
        
        dcbSerialParams.BaudRate = static_cast<DWORD>(baudRate_);
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;
        
        if (!SetCommState(hSerial_, &dcbSerialParams)) {
            CloseHandle(hSerial_);
            hSerial_ = INVALID_HANDLE_VALUE;
            return false;
        }
        
        COMMTIMEOUTS timeout = {0};
        timeout.ReadIntervalTimeout = 50;
        timeout.ReadTotalTimeoutConstant = 50;
        timeout.ReadTotalTimeoutMultiplier = 10;
        timeout.WriteTotalTimeoutConstant = 50;
        timeout.WriteTotalTimeoutMultiplier = 10;
        
        if (!SetCommTimeouts(hSerial_, &timeout)) {
            CloseHandle(hSerial_);
            hSerial_ = INVALID_HANDLE_VALUE;
            return false;
        }
        
        return true;
    }
    
    std::string ReadLine() {
        std::string result;
        char buffer[1];
        DWORD bytesRead;
        
        while (ReadFile(hSerial_, buffer, 1, &bytesRead, NULL) && bytesRead > 0) {
            if (buffer[0] == '\n') {
                break;
            }
            if (buffer[0] != '\r') {
                result += buffer[0];
            }
        }
        
        return result;
    }
    
    bool WriteLine(const std::string& data) {
        std::string dataWithNewline = data + "\n";
        DWORD bytesWritten;
        return WriteFile(hSerial_, dataWithNewline.c_str(), static_cast<DWORD>(dataWithNewline.length()), &bytesWritten, NULL);
    }
    
    bool IsOpen() const {
        return hSerial_ != INVALID_HANDLE_VALUE;
    }

private:
    std::string portName_;
    int baudRate_;
    HANDLE hSerial_;
};

// Glove data structure
struct GloveData {
    float quat_w, quat_x, quat_y, quat_z;
    float accel_x, accel_y, accel_z;
    int thumb_curl, index_curl, middle_curl, ring_curl, pinky_curl;
    bool calibrated;
    
    GloveData() : quat_w(1.0f), quat_x(0.0f), quat_y(0.0f), quat_z(0.0f),
                  accel_x(0.0f), accel_y(0.0f), accel_z(0.0f),
                  thumb_curl(512), index_curl(512), middle_curl(512), 
                  ring_curl(512), pinky_curl(512), calibrated(false) {}
};

// Haptic Glove Device Class
class HapticGloveDevice : public vr::ITrackedDeviceServerDriver {
public:
    HapticGloveDevice()
        : objectId_(vr::k_unTrackedDeviceIndexInvalid)
        , serialPort_(nullptr)
        , isActive_(false)
        , lastUpdateTime_(0)
        , systemClickComponent_(vr::k_ulInvalidInputComponentHandle)
        , gripClickComponent_(vr::k_ulInvalidInputComponentHandle)
        , triggerClickComponent_(vr::k_ulInvalidInputComponentHandle)
        , thumbCurlComponent_(vr::k_ulInvalidInputComponentHandle)
        , indexCurlComponent_(vr::k_ulInvalidInputComponentHandle)
        , middleCurlComponent_(vr::k_ulInvalidInputComponentHandle)
        , ringCurlComponent_(vr::k_ulInvalidInputComponentHandle)
        , pinkyCurlComponent_(vr::k_ulInvalidInputComponentHandle)
        , hapticComponent_(vr::k_ulInvalidInputComponentHandle)
    {
        // Initialize pose
        memset(&pose_, 0, sizeof(pose_));
        
        pose_.qWorldFromDriverRotation.w = 1.0;
        pose_.qWorldFromDriverRotation.x = 0.0;
        pose_.qWorldFromDriverRotation.y = 0.0;
        pose_.qWorldFromDriverRotation.z = 0.0;
        
        pose_.qDriverFromHeadRotation.w = 1.0;
        pose_.qDriverFromHeadRotation.x = 0.0;
        pose_.qDriverFromHeadRotation.y = 0.0;
        pose_.qDriverFromHeadRotation.z = 0.0;
        
        pose_.qRotation.w = 1.0;
        pose_.qRotation.x = 0.0;
        pose_.qRotation.y = 0.0;
        pose_.qRotation.z = 0.0;
        
        pose_.result = TrackingResult_Running_OK;
        pose_.poseIsValid = true;
        pose_.willDriftInYaw = false;
        pose_.shouldApplyHeadModel = false;
        pose_.deviceIsConnected = true;
        pose_.poseTimeOffset = 0.0;
    }
    
    virtual ~HapticGloveDevice() {
        if (serialPort_) {
            delete serialPort_;
        }
    }
    
   virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId) override {
    objectId_ = unObjectId;
    
    vr::VRDriverLog()->Log("=== HapticGloveDevice::Activate() START ===");
    vr::VRDriverLog()->Log(("Device ID assigned: " + std::to_string(unObjectId)).c_str());
    
    // Get settings with proper error handling
    char portNameBuffer[256];
    vr::EVRSettingsError settingsError = vr::VRSettingsError_None;
    
    // Get serial port setting
    vr::VRSettings()->GetString("driver_haptic_glove", "serial_port", portNameBuffer, sizeof(portNameBuffer), &settingsError);
    
    std::string portName;
    if (settingsError == vr::VRSettingsError_None) {
        portName = std::string(portNameBuffer);
        vr::VRDriverLog()->Log(("Using serial port from settings: " + portName).c_str());
    } else {
        portName = "COM3";
        vr::VRDriverLog()->Log(("Settings error " + std::to_string(settingsError) + ", using default serial port COM3").c_str());
    }
    
    // Get baud rate setting
    vr::EVRSettingsError baudError = vr::VRSettingsError_None;
    int baudRate = vr::VRSettings()->GetInt32("driver_haptic_glove", "serial_baudrate", &baudError);
    if (baudError != vr::VRSettingsError_None) {
        baudRate = 9600;
        vr::VRDriverLog()->Log(("Baud rate settings error " + std::to_string(baudError) + ", using default 9600").c_str());
    } else {
        vr::VRDriverLog()->Log(("Using baud rate from settings: " + std::to_string(baudRate)).c_str());
    }
    
    // Attempt to open serial port
    vr::VRDriverLog()->Log(("Attempting to open serial port: " + portName).c_str());
    
    serialPort_ = new SerialPort(portName, baudRate);
    
    if (!serialPort_->Open()) {
        vr::VRDriverLog()->Log("WARNING: Failed to open serial port");
        vr::VRDriverLog()->Log("Continuing without serial connection...");
        delete serialPort_;
        serialPort_ = nullptr;
    } else {
        vr::VRDriverLog()->Log("SUCCESS: Serial port opened successfully");
        
        // Test communication
        vr::VRDriverLog()->Log("Testing serial communication...");
        for (int i = 0; i < 5; i++) {
            std::string testLine = serialPort_->ReadLine();
            if (!testLine.empty()) {
                vr::VRDriverLog()->Log(("Received: " + testLine).c_str());
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    // Set up properties
    vr::VRDriverLog()->Log("Setting up device properties...");
    vr::PropertyContainerHandle_t props = vr::VRProperties()->TrackedDeviceToPropertyContainer(objectId_);
    vr::VRDriverLog()->Log(("Property container handle: " + std::to_string(props)).c_str());
    
    // Set basic properties - only use known working properties
    vr::VRProperties()->SetStringProperty(props, Prop_ModelNumber_String, "HapticGlove_v1");
    vr::VRProperties()->SetStringProperty(props, Prop_RenderModelName_String, "generic_controller");
    vr::VRProperties()->SetStringProperty(props, Prop_SerialNumber_String, "HG001");
    vr::VRProperties()->SetStringProperty(props, Prop_ManufacturerName_String, "DIY Haptics");
    
    vr::VRProperties()->SetInt32Property(props, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
    vr::VRProperties()->SetInt32Property(props, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);
    
    // Set tracking properties
    vr::VRProperties()->SetBoolProperty(props, Prop_WillDriftInYaw_Bool, false);
    vr::VRProperties()->SetBoolProperty(props, Prop_DeviceCanPowerOff_Bool, true);
    
    vr::VRDriverLog()->Log("Basic properties set successfully");
    
    // Create input components
    vr::VRDriverLog()->Log("Creating input components...");
    
    vr::EVRInputError inputError;
    
    inputError = vr::VRDriverInput()->CreateBooleanComponent(props, "/input/system/click", &systemClickComponent_);
    vr::VRDriverLog()->Log(("System click component: " + std::to_string(inputError)).c_str());
    
    inputError = vr::VRDriverInput()->CreateBooleanComponent(props, "/input/grip/click", &gripClickComponent_);
    vr::VRDriverLog()->Log(("Grip click component: " + std::to_string(inputError)).c_str());
    
    inputError = vr::VRDriverInput()->CreateBooleanComponent(props, "/input/trigger/click", &triggerClickComponent_);
    vr::VRDriverLog()->Log(("Trigger click component: " + std::to_string(inputError)).c_str());
    
    inputError = vr::VRDriverInput()->CreateScalarComponent(props, "/input/finger/thumb", &thumbCurlComponent_, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
    vr::VRDriverLog()->Log(("Thumb curl component: " + std::to_string(inputError)).c_str());
    
    inputError = vr::VRDriverInput()->CreateScalarComponent(props, "/input/finger/index", &indexCurlComponent_, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
    vr::VRDriverLog()->Log(("Index curl component: " + std::to_string(inputError)).c_str());
    
    inputError = vr::VRDriverInput()->CreateScalarComponent(props, "/input/finger/middle", &middleCurlComponent_, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
    vr::VRDriverLog()->Log(("Middle curl component: " + std::to_string(inputError)).c_str());
    
    inputError = vr::VRDriverInput()->CreateScalarComponent(props, "/input/finger/ring", &ringCurlComponent_, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
    vr::VRDriverLog()->Log(("Ring curl component: " + std::to_string(inputError)).c_str());
    
    inputError = vr::VRDriverInput()->CreateScalarComponent(props, "/input/finger/pinky", &pinkyCurlComponent_, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
    vr::VRDriverLog()->Log(("Pinky curl component: " + std::to_string(inputError)).c_str());
    
    // Create haptic component
    inputError = vr::VRDriverInput()->CreateHapticComponent(props, "/output/haptic", &hapticComponent_);
    vr::VRDriverLog()->Log(("Haptic component: " + std::to_string(inputError)).c_str());
    
    vr::VRDriverLog()->Log("All input components created");
    
    // Initialize pose properly
    vr::VRDriverLog()->Log("Initializing pose...");
    
    // Set the pose to be valid and connected
    pose_.poseIsValid = true;
    pose_.deviceIsConnected = true;
    pose_.result = TrackingResult_Running_OK;
    
    // Initialize rotation quaternion
    pose_.qRotation.w = 1.0;
    pose_.qRotation.x = 0.0;
    pose_.qRotation.y = 0.0;
    pose_.qRotation.z = 0.0;
    
    // Initialize position (this is important!)
    pose_.vecPosition[0] = 0.0f;
    pose_.vecPosition[1] = 0.0f;
    pose_.vecPosition[2] = 0.0f;
    
    // Zero out velocities
    pose_.vecVelocity[0] = 0.0f;
    pose_.vecVelocity[1] = 0.0f;
    pose_.vecVelocity[2] = 0.0f;
    
    pose_.vecAngularVelocity[0] = 0.0f;
    pose_.vecAngularVelocity[1] = 0.0f;
    pose_.vecAngularVelocity[2] = 0.0f;
    
    // Set other pose properties
    pose_.poseTimeOffset = 0.0;
    pose_.willDriftInYaw = false;
    pose_.shouldApplyHeadModel = false;
    
    vr::VRDriverLog()->Log("Pose initialized");
    
    // Send initial pose update
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(objectId_, pose_, sizeof(DriverPose_t));
    vr::VRDriverLog()->Log("Initial pose sent to SteamVR");
    
    isActive_ = true;
    
    // Start update thread
    vr::VRDriverLog()->Log("Starting update thread...");
    updateThread_ = std::thread(&HapticGloveDevice::UpdateThread, this);
    
    vr::VRDriverLog()->Log("=== HapticGloveDevice::Activate() COMPLETED SUCCESSFULLY ===");
    return VRInitError_None;
}
    
    virtual void Deactivate() override {
        isActive_ = false;
        if (updateThread_.joinable()) {
            updateThread_.join();
        }
        objectId_ = vr::k_unTrackedDeviceIndexInvalid;
    }

    virtual void PowerOff() {
    // Clean shutdown of the device
    isActive_ = false;
    if (updateThread_.joinable()) {
        updateThread_.join();
    }
    if (serialPort_ && serialPort_->IsOpen()) {
        // Reset servos to neutral position before shutdown
        serialPort_->WriteLine("HAPTIC:90,90,90,90,90");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
    
    virtual void EnterStandby() override {}
    
    virtual void *GetComponent(const char *pchComponentNameAndVersion) override {
        return nullptr;
    }
    
    virtual void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) override {
        if (unResponseBufferSize >= 1) {
            pchResponseBuffer[0] = 0;
        }
    }
    
    virtual DriverPose_t GetPose() override {
        return pose_;
    }
    
    void UpdateThread() {
    vr::VRDriverLog()->Log("UpdateThread started");
    
    int dataPacketCount = 0;
    auto lastLogTime = std::chrono::steady_clock::now();
    
    while (isActive_) {
        if (serialPort_ && serialPort_->IsOpen()) {
            std::string line = serialPort_->ReadLine();
            
            if (line.length() > 0) {
                // Log every 100th packet to avoid spam
                if (dataPacketCount % 100 == 0) {
                    vr::VRDriverLog()->Log(("Data received: " + line).c_str());
                }
                
                if (line.substr(0, 5) == "DATA:") {
                    dataPacketCount++;
                    ParseGloveData(line.substr(5));
                    UpdatePose();
                    UpdateInputs();
                    
                    // Log status every 5 seconds
                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::seconds>(now - lastLogTime).count() >= 5) {
                        vr::VRDriverLog()->Log(("Data packets processed: " + std::to_string(dataPacketCount)).c_str());
                        lastLogTime = now;
                    }
                }
            }
        } else {
            // No serial connection - send dummy pose updates so device stays "alive"
            if (dataPacketCount % 50 == 0) { // Every second at 50Hz
                UpdatePose();
                if (dataPacketCount == 0) {
                    vr::VRDriverLog()->Log("No serial connection - sending dummy pose updates");
                }
            }
            dataPacketCount++;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    vr::VRDriverLog()->Log("UpdateThread ended");
}
    
    void ParseGloveData(const std::string& data) {
        std::stringstream ss(data);
        std::string item;
        std::vector<std::string> tokens;
        
        while (std::getline(ss, item, ',')) {
            tokens.push_back(item);
        }
        
        if (tokens.size() >= 13) {
            try {
                gloveData_.quat_w = std::stof(tokens[0]);
                gloveData_.quat_x = std::stof(tokens[1]);
                gloveData_.quat_y = std::stof(tokens[2]);
                gloveData_.quat_z = std::stof(tokens[3]);
                gloveData_.accel_x = std::stof(tokens[4]);
                gloveData_.accel_y = std::stof(tokens[5]);
                gloveData_.accel_z = std::stof(tokens[6]);
                gloveData_.thumb_curl = std::stoi(tokens[7]);
                gloveData_.index_curl = std::stoi(tokens[8]);
                gloveData_.middle_curl = std::stoi(tokens[9]);
                gloveData_.ring_curl = std::stoi(tokens[10]);
                gloveData_.pinky_curl = std::stoi(tokens[11]);
                gloveData_.calibrated = (tokens[12] == "1");
            } catch (...) {
                // Handle parsing errors gracefully
                vr::VRDriverLog()->Log("Error parsing glove data");
            }
        }
    }
    
    void UpdatePose() {
        // Update rotation from quaternion
        pose_.qRotation.w = gloveData_.quat_w;
        pose_.qRotation.x = gloveData_.quat_x;
        pose_.qRotation.y = gloveData_.quat_y;
        pose_.qRotation.z = gloveData_.quat_z;
        
        // Update tracking result based on calibration
        if (gloveData_.calibrated) {
            pose_.result = TrackingResult_Running_OK;
        } else {
            pose_.result = TrackingResult_Running_OutOfRange;
        }
        
        pose_.poseTimeOffset = 0.0;
        
        // Send pose update
        if (objectId_ != vr::k_unTrackedDeviceIndexInvalid) {
            vr::VRServerDriverHost()->TrackedDevicePoseUpdated(objectId_, pose_, sizeof(DriverPose_t));
        }
    }
    
    void UpdateInputs() {
        if (objectId_ == vr::k_unTrackedDeviceIndexInvalid) return;
        
        // Normalize finger curl values (0-1023 to 0.0-1.0)
        float thumbCurl = static_cast<float>(gloveData_.thumb_curl) / 1023.0f;
        float indexCurl = static_cast<float>(gloveData_.index_curl) / 1023.0f;
        float middleCurl = static_cast<float>(gloveData_.middle_curl) / 1023.0f;
        float ringCurl = static_cast<float>(gloveData_.ring_curl) / 1023.0f;
        float pinkyCurl = static_cast<float>(gloveData_.pinky_curl) / 1023.0f;
        
        // Update finger curl components
        vr::VRDriverInput()->UpdateScalarComponent(thumbCurlComponent_, thumbCurl, 0.0);
        vr::VRDriverInput()->UpdateScalarComponent(indexCurlComponent_, indexCurl, 0.0);
        vr::VRDriverInput()->UpdateScalarComponent(middleCurlComponent_, middleCurl, 0.0);
        vr::VRDriverInput()->UpdateScalarComponent(ringCurlComponent_, ringCurl, 0.0);
        vr::VRDriverInput()->UpdateScalarComponent(pinkyCurlComponent_, pinkyCurl, 0.0);
        
        // Simple gesture recognition
        bool gripClick = (thumbCurl > 0.7f && indexCurl > 0.7f && middleCurl > 0.7f);
        bool triggerClick = (indexCurl > 0.8f);
        bool systemClick = (thumbCurl > 0.8f && middleCurl > 0.8f && ringCurl > 0.8f && pinkyCurl > 0.8f);
        
        vr::VRDriverInput()->UpdateBooleanComponent(gripClickComponent_, gripClick, 0.0);
        vr::VRDriverInput()->UpdateBooleanComponent(triggerClickComponent_, triggerClick, 0.0);
        vr::VRDriverInput()->UpdateBooleanComponent(systemClickComponent_, systemClick, 0.0);
    }
    
    void SendHapticFeedback(float strength, float /*duration*/) {
        if (serialPort_ && serialPort_->IsOpen()) {
            // Convert strength to servo positions (90 = neutral, higher = more force)
            int servoValue = 90 + static_cast<int>(strength * 90);
            
            std::string command = "HAPTIC:" + 
                std::to_string(servoValue) + "," +
                std::to_string(servoValue) + "," +
                std::to_string(servoValue) + "," +
                std::to_string(servoValue) + "," +
                std::to_string(servoValue);
            
            serialPort_->WriteLine(command);
        }
    }

private:
    vr::TrackedDeviceIndex_t objectId_;
    vr::DriverPose_t pose_;
    SerialPort* serialPort_;
    bool isActive_;
    std::thread updateThread_;
    
    GloveData gloveData_;
    uint64_t lastUpdateTime_;
    
    // Input components
    vr::VRInputComponentHandle_t systemClickComponent_;
    vr::VRInputComponentHandle_t gripClickComponent_;
    vr::VRInputComponentHandle_t triggerClickComponent_;
    vr::VRInputComponentHandle_t thumbCurlComponent_;
    vr::VRInputComponentHandle_t indexCurlComponent_;
    vr::VRInputComponentHandle_t middleCurlComponent_;
    vr::VRInputComponentHandle_t ringCurlComponent_;
    vr::VRInputComponentHandle_t pinkyCurlComponent_;
    vr::VRInputComponentHandle_t hapticComponent_;
};

// Driver Provider Class
class HapticGloveDriverProvider : public vr::IServerTrackedDeviceProvider {
public:
    HapticGloveDriverProvider() : gloveDevice_(nullptr) {
        vr::VRDriverLog()->Log("HapticGloveDriverProvider constructor called");
    }
    
    virtual EVRInitError Init(vr::IVRDriverContext *pDriverContext) override {
        vr::VRDriverLog()->Log("=== HapticGloveDriverProvider::Init() START ===");
        
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
        vr::VRDriverLog()->Log("OpenVR driver context initialized");
        
        // Log OpenVR version info
        vr::VRDriverLog()->Log(("OpenVR Interface Version: " + std::string(vr::IVRServerDriverHost_Version)).c_str());
        
        gloveDevice_ = new HapticGloveDevice();
        vr::VRDriverLog()->Log("HapticGloveDevice created");
        
        // Add device to SteamVR
        bool deviceAdded = vr::VRServerDriverHost()->TrackedDeviceAdded("HapticGlove_001", TrackedDeviceClass_Controller, gloveDevice_);
        vr::VRDriverLog()->Log(("TrackedDeviceAdded returned: " + std::to_string(deviceAdded)).c_str());
        
        if (!deviceAdded) {
            vr::VRDriverLog()->Log("CRITICAL: Failed to add device to SteamVR!");
            return VRInitError_Driver_Failed;
        }
        
        vr::VRDriverLog()->Log("=== HapticGloveDriverProvider::Init() SUCCESS ===");
        return VRInitError_None;
    }
    
    virtual void Cleanup() override {
        vr::VRDriverLog()->Log("HapticGloveDriverProvider::Cleanup() called");
        if (gloveDevice_) {
            delete gloveDevice_;
            gloveDevice_ = nullptr;
            vr::VRDriverLog()->Log("HapticGloveDevice cleaned up");
        }
    }
    
    virtual const char * const *GetInterfaceVersions() override {
        vr::VRDriverLog()->Log("GetInterfaceVersions() called");
        return vr::k_InterfaceVersions;
    }
    
    virtual void RunFrame() override {
        // Called by OpenVR each frame - don't log here as it's too frequent
    }
    
    virtual bool ShouldBlockStandbyMode() override {
        return false;
    }
    
    virtual void EnterStandby() override {
        vr::VRDriverLog()->Log("EnterStandby() called");
    }
    
    virtual void LeaveStandby() override {
        vr::VRDriverLog()->Log("LeaveStandby() called");
    }

private:
    HapticGloveDevice* gloveDevice_;
};

static HapticGloveDriverProvider g_driverProvider;

HMD_DLL_EXPORT void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode) {
    vr::VRDriverLog()->Log("=== HmdDriverFactory() called ===");
    vr::VRDriverLog()->Log(("Requested interface: " + std::string(pInterfaceName)).c_str());
    vr::VRDriverLog()->Log(("Expected interface: " + std::string(IServerTrackedDeviceProvider_Version)).c_str());
    
    if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName)) {
        vr::VRDriverLog()->Log("Returning driver provider instance");
        if (pReturnCode) {
            *pReturnCode = VRInitError_None;
        }
        return &g_driverProvider;
    }
    
    vr::VRDriverLog()->Log("Interface not found!");
    if (pReturnCode) {
        *pReturnCode = VRInitError_Init_InterfaceNotFound;
    }
    
    return nullptr;
}