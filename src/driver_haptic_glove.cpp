//============ Copyright (c) Valve Corporation, All rights reserved. ============
//
// OpenVR Haptic Glove Driver
//
//=============================================================================

#include <openvr_driver.h>
#include <windows.h>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <sstream>

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

// Serial communication
#include <windows.h>
#include <stdio.h>

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
        
        dcbSerialParams.BaudRate = baudRate_;
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
        return WriteFile(hSerial_, dataWithNewline.c_str(), dataWithNewline.length(), &bytesWritten, NULL);
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
};

// Haptic Glove Device Class
class HapticGloveDevice : public vr::ITrackedDeviceServerDriver {
public:
    HapticGloveDevice()
        : objectId_(vr::k_unTrackedDeviceIndexInvalid)
        , serialPort_(nullptr)
        , isActive_(false)
        , lastUpdateTime_(0)
    {
        // Initialize pose
        pose_.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
        pose_.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);
        pose_.vecWorldFromDriverTranslation[0] = 0;
        pose_.vecWorldFromDriverTranslation[1] = 0;
        pose_.vecWorldFromDriverTranslation[2] = 0;
        pose_.vecDriverFromHeadTranslation[0] = 0;
        pose_.vecDriverFromHeadTranslation[1] = 0;
        pose_.vecDriverFromHeadTranslation[2] = 0;
        
        pose_.qRotation = HmdQuaternion_Init(1, 0, 0, 0);
        pose_.vecPosition[0] = 0;
        pose_.vecPosition[1] = 0;
        pose_.vecPosition[2] = 0;
        
        pose_.vecVelocity[0] = 0;
        pose_.vecVelocity[1] = 0;
        pose_.vecVelocity[2] = 0;
        
        pose_.vecAngularVelocity[0] = 0;
        pose_.vecAngularVelocity[1] = 0;
        pose_.vecAngularVelocity[2] = 0;
        
        pose_.vecAcceleration[0] = 0;
        pose_.vecAcceleration[1] = 0;
        pose_.vecAcceleration[2] = 0;
        
        pose_.vecAngularAcceleration[0] = 0;
        pose_.vecAngularAcceleration[1] = 0;
        pose_.vecAngularAcceleration[2] = 0;
        
        pose_.result = TrackingResult_Running_OK;
        pose_.poseIsValid = true;
        pose_.willDriftInYaw = false;
        pose_.shouldApplyHeadModel = false;
        pose_.deviceIsConnected = true;
        
        // Initialize glove data
        gloveData_ = {1, 0, 0, 0, 0, 0, 0, 512, 512, 512, 512, 512, false};
    }
    
    virtual ~HapticGloveDevice() {
        if (serialPort_) {
            delete serialPort_;
        }
    }
    
    virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId) override {
        objectId_ = unObjectId;
        
        // Get settings
        std::string portName = vr::VRSettings()->GetString("driver_haptic_glove", "serial_port", "COM3");
        int baudRate = vr::VRSettings()->GetInt32("driver_haptic_glove", "serial_baudrate", 9600);
        
        // Open serial port
        serialPort_ = new SerialPort(portName, baudRate);
        if (!serialPort_->Open()) {
            vr::VRDriverLog()->Log("Failed to open serial port");
            return VRInitError_Driver_Failed;
        }
        
        // Set up properties
        vr::PropertyContainerHandle_t props = vr::VRProperties()->TrackedDeviceToPropertyContainer(objectId_);
        
        vr::VRProperties()->SetStringProperty(props, Prop_ModelNumber_String, "HapticGlove_v1");
        vr::VRProperties()->SetStringProperty(props, Prop_RenderModelName_String, "generic_controller");
        vr::VRProperties()->SetStringProperty(props, Prop_SerialNumber_String, "HG001");
        vr::VRProperties()->SetStringProperty(props, Prop_ManufacturerName_String, "DIY Haptics");
        
        vr::VRProperties()->SetInt32Property(props, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
        vr::VRProperties()->SetInt32Property(props, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);
        
        // Create input components
        vr::VRDriverInput()->CreateBooleanComponent(props, "/input/system/click", &systemClickComponent_);
        vr::VRDriverInput()->CreateBooleanComponent(props, "/input/grip/click", &gripClickComponent_);
        vr::VRDriverInput()->CreateBooleanComponent(props, "/input/trigger/click", &triggerClickComponent_);
        
        vr::VRDriverInput()->CreateScalarComponent(props, "/input/finger/thumb", &thumbCurlComponent_, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
        vr::VRDriverInput()->CreateScalarComponent(props, "/input/finger/index", &indexCurlComponent_, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
        vr::VRDriverInput()->CreateScalarComponent(props, "/input/finger/middle", &middleCurlComponent_, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
        vr::VRDriverInput()->CreateScalarComponent(props, "/input/finger/ring", &ringCurlComponent_, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
        vr::VRDriverInput()->CreateScalarComponent(props, "/input/finger/pinky", &pinkyCurlComponent_, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
        
        // Create haptic component
        vr::VRDriverInput()->CreateHapticComponent(props, "/output/haptic", &hapticComponent_);
        
        isActive_ = true;
        
        // Start update thread
        updateThread_ = std::thread(&HapticGloveDevice::UpdateThread, this);
        
        return VRInitError_None;
    }
    
    virtual void Deactivate() override {
        isActive_ = false;
        if (updateThread_.joinable()) {
            updateThread_.join();
        }
        objectId_ = vr::k_unTrackedDeviceIndexInvalid;
    }
    
    virtual void EnterStandby() override {}
    
    virtual void *GetComponent(const char *pchComponentNameAndVersion) override {
        return nullptr;
    }
    
    virtual void PowerOff() override {}
    
    virtual void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) override {
        if (unResponseBufferSize >= 1) {
            pchResponseBuffer[0] = 0;
        }
    }
    
    virtual DriverPose_t GetPose() override {
        return pose_;
    }
    
    void UpdateThread() {
        while (isActive_) {
            if (serialPort_ && serialPort_->IsOpen()) {
                std::string line = serialPort_->ReadLine();
                
                if (line.length() > 0 && line.substr(0, 5) == "DATA:") {
                    ParseGloveData(line.substr(5));
                    UpdatePose();
                    UpdateInputs();
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
    
    void ParseGloveData(const std::string& data) {
        std::stringstream ss(data);
        std::string item;
        std::vector<std::string> tokens;
        
        while (std::getline(ss, item, ',')) {
            tokens.push_back(item);
        }
        
        if (tokens.size() >= 13) {
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
        
        pose_.poseTimeOffset = 0;
        
        // Send pose update
        if (objectId_ != vr::k_unTrackedDeviceIndexInvalid) {
            vr::VRServerDriverHost()->TrackedDevicePoseUpdated(objectId_, pose_, sizeof(DriverPose_t));
        }
    }
    
    void UpdateInputs() {
        if (objectId_ == vr::k_unTrackedDeviceIndexInvalid) return;
        
        // Normalize finger curl values (0-1023 to 0.0-1.0)
        float thumbCurl = (float)gloveData_.thumb_curl / 1023.0f;
        float indexCurl = (float)gloveData_.index_curl / 1023.0f;
        float middleCurl = (float)gloveData_.middle_curl / 1023.0f;
        float ringCurl = (float)gloveData_.ring_curl / 1023.0f;
        float pinkyCurl = (float)gloveData_.pinky_curl / 1023.0f;
        
        // Update finger curl components
        vr::VRDriverInput()->UpdateScalarComponent(thumbCurlComponent_, thumbCurl, 0);
        vr::VRDriverInput()->UpdateScalarComponent(indexCurlComponent_, indexCurl, 0);
        vr::VRDriverInput()->UpdateScalarComponent(middleCurlComponent_, middleCurl, 0);
        vr::VRDriverInput()->UpdateScalarComponent(ringCurlComponent_, ringCurl, 0);
        vr::VRDriverInput()->UpdateScalarComponent(pinkyCurlComponent_, pinkyCurl, 0);
        
        // Simple gesture recognition
        bool gripClick = (thumbCurl > 0.7f && indexCurl > 0.7f && middleCurl > 0.7f);
        bool triggerClick = (indexCurl > 0.8f);
        bool systemClick = (thumbCurl > 0.8f && middleCurl > 0.8f && ringCurl > 0.8f && pinkyCurl > 0.8f);
        
        vr::VRDriverInput()->UpdateBooleanComponent(gripClickComponent_, gripClick, 0);
        vr::VRDriverInput()->UpdateBooleanComponent(triggerClickComponent_, triggerClick, 0);
        vr::VRDriverInput()->UpdateBooleanComponent(systemClickComponent_, systemClick, 0);
    }
    
    void SendHapticFeedback(float strength, float duration) {
        if (serialPort_ && serialPort_->IsOpen()) {
            // Convert strength to servo positions (90 = neutral, higher = more force)
            int servoValue = 90 + (int)(strength * 90);
            
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
    virtual EVRInitError Init(vr::IVRDriverContext *pDriverContext) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
        
        gloveDevice_ = new HapticGloveDevice();
        vr::VRServerDriverHost()->TrackedDeviceAdded("HapticGlove_001", TrackedDeviceClass_Controller, gloveDevice_);
        
        return VRInitError_None;
    }
    
    virtual void Cleanup() override {
        delete gloveDevice_;
        gloveDevice_ = nullptr;
    }
    
    virtual const char * const *GetInterfaceVersions() override {
        return vr::k_InterfaceVersions;
    }
    
    virtual void RunFrame() override {
        // Called by OpenVR each frame
    }
    
    virtual bool ShouldBlockStandbyMode() override {
        return false;
    }
    
    virtual void EnterStandby() override {}
    
    virtual void LeaveStandby() override {}

private:
    HapticGloveDevice* gloveDevice_;
};

static HapticGloveDriverProvider g_driverProvider;

HMD_DLL_EXPORT void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode) {
    if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName)) {
        return &g_driverProvider;
    }
    
    if (pReturnCode) {
        *pReturnCode = VRInitError_Init_InterfaceNotFound;
    }
    
    return nullptr;
}