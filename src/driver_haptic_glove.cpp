/*
 * VR Haptic Glove OpenVR Driver
 * Simple, clean implementation for right hand glove
 */

#include <openvr_driver.h>
#include <windows.h>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <sstream>

using namespace vr;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec(dllexport)
#else
#error "Windows only"
#endif

//=============================================================================
// Serial Communication
//=============================================================================
class SerialPort {
private:
    HANDLE handle;
    std::string port;
    
public:
    SerialPort(const std::string& portName) : port(portName), handle(INVALID_HANDLE_VALUE) {}
    
    ~SerialPort() {
        if(handle != INVALID_HANDLE_VALUE) {
            CloseHandle(handle);
        }
    }
    
    bool Open() {
        std::string fullPort = "\\\\.\\" + port;
        
        handle = CreateFileA(fullPort.c_str(), GENERIC_READ | GENERIC_WRITE,
                           0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        
        if(handle == INVALID_HANDLE_VALUE) return false;
        
        // Configure serial port
        DCB dcb = {0};
        dcb.DCBlength = sizeof(dcb);
        if(!GetCommState(handle, &dcb)) return false;
        
        dcb.BaudRate = CBR_9600;
        dcb.ByteSize = 8;
        dcb.StopBits = ONESTOPBIT;
        dcb.Parity = NOPARITY;
        
        if(!SetCommState(handle, &dcb)) return false;
        
        // Set timeouts
        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;
        SetCommTimeouts(handle, &timeouts);
        
        return true;
    }
    
    std::string ReadLine() {
        std::string line;
        char c;
        DWORD bytesRead;
        
        while(ReadFile(handle, &c, 1, &bytesRead, NULL) && bytesRead > 0) {
            if(c == '\n') break;
            if(c != '\r') line += c;
        }
        return line;
    }
    
    bool WriteLine(const std::string& data) {
        std::string line = data + "\n";
        DWORD written;
        return WriteFile(handle, line.c_str(), line.length(), &written, NULL);
    }
    
    bool IsOpen() const { return handle != INVALID_HANDLE_VALUE; }
};

//=============================================================================
// Glove Data
//=============================================================================
struct GloveData {
    float qw, qx, qy, qz;      // Quaternion
    float ax, ay, az;          // Accelerometer  
    int fingers[5];            // Finger curl (0-1023)
    bool calibrated;           // IMU calibration status
    
    GloveData() : qw(1), qx(0), qy(0), qz(0), ax(0), ay(0), az(0), calibrated(false) {
        for(int i = 0; i < 5; i++) fingers[i] = 512;
    }
};

//=============================================================================
// Haptic Glove Device
//=============================================================================
class HapticGlove : public ITrackedDeviceServerDriver {
private:
    TrackedDeviceIndex_t deviceId;
    SerialPort* serial;
    std::thread updateThread;
    bool active;
    DriverPose_t pose;
    GloveData gloveData;
    
    // Input components
    VRInputComponentHandle_t inputGrip, inputTrigger, inputSystem;
    VRInputComponentHandle_t inputFingers[5];
    VRInputComponentHandle_t outputHaptic;
    
public:
    HapticGlove() : deviceId(k_unTrackedDeviceIndexInvalid), serial(nullptr), active(false) {
        // Initialize pose
        memset(&pose, 0, sizeof(pose));
        pose.qRotation.w = 1.0;
        pose.poseIsValid = true;
        pose.deviceIsConnected = true;
        pose.result = TrackingResult_Running_OK;
    }
    
    ~HapticGlove() {
        if(serial) delete serial;
    }
    
    //-------------------------------------------------------------------------
    // ITrackedDeviceServerDriver interface
    //-------------------------------------------------------------------------
    EVRInitError Activate(TrackedDeviceIndex_t unObjectId) override {
        deviceId = unObjectId;
        VRDriverLog()->Log("HapticGlove: Activating device");
        
        // Get COM port from settings
        char portBuffer[32];
        vr::EVRSettingsError settingsError = vr::VRSettingsError_None;
        VRSettings()->GetString("driver_haptic_glove", "serial_port", portBuffer, sizeof(portBuffer), &settingsError);
        
        std::string portName;
        if(settingsError == vr::VRSettingsError_None) {
            portName = std::string(portBuffer);
        } else {
            portName = "COM3";  // Default fallback
        }
        
        VRDriverLog()->Log(("HapticGlove: Using port " + portName).c_str());
        
        // Open serial connection
        serial = new SerialPort(portName);
        if(!serial->Open()) {
            VRDriverLog()->Log("HapticGlove: Failed to open serial port - continuing without");
            delete serial;
            serial = nullptr;
        } else {
            VRDriverLog()->Log("HapticGlove: Serial port opened successfully");
        }
        
        // Set device properties
        PropertyContainerHandle_t props = VRProperties()->TrackedDeviceToPropertyContainer(deviceId);
        VRProperties()->SetStringProperty(props, Prop_ModelNumber_String, "HapticGlove_v1");
        VRProperties()->SetStringProperty(props, Prop_SerialNumber_String, "HG_RIGHT_001");
        VRProperties()->SetStringProperty(props, Prop_ManufacturerName_String, "DIY Haptics");
        VRProperties()->SetInt32Property(props, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
        VRProperties()->SetInt32Property(props, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);
        
        // Create input components
        VRDriverInput()->CreateBooleanComponent(props, "/input/grip/click", &inputGrip);
        VRDriverInput()->CreateBooleanComponent(props, "/input/trigger/click", &inputTrigger);
        VRDriverInput()->CreateBooleanComponent(props, "/input/system/click", &inputSystem);
        
        // Create finger tracking components
        const char* fingerNames[] = {"/input/finger/thumb", "/input/finger/index", 
                                   "/input/finger/middle", "/input/finger/ring", "/input/finger/pinky"};
        for(int i = 0; i < 5; i++) {
            VRDriverInput()->CreateScalarComponent(props, fingerNames[i], &inputFingers[i], 
                                                 VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
        }
        
        // Create haptic output
        VRDriverInput()->CreateHapticComponent(props, "/output/haptic", &outputHaptic);
        
        // Start update thread
        active = true;
        updateThread = std::thread(&HapticGlove::UpdateLoop, this);
        
        VRDriverLog()->Log("HapticGlove: Activation complete");
        return VRInitError_None;
    }
    
    void Deactivate() override {
        VRDriverLog()->Log("HapticGlove: Deactivating");
        active = false;
        if(updateThread.joinable()) updateThread.join();
        deviceId = k_unTrackedDeviceIndexInvalid;
    }
    
    // void PowerOff() override {
    //     VRDriverLog()->Log("HapticGlove: Power off");
    //     if(serial && serial->IsOpen()) {
    //         serial->WriteLine("HAPTIC:90,90,90,90,90");  // Reset servos
    //     }
    // }
    
    void EnterStandby() override {}
    void* GetComponent(const char* pchComponentNameAndVersion) override { return nullptr; }
    void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override {
        if(unResponseBufferSize >= 1) pchResponseBuffer[0] = 0;
    }
    
    DriverPose_t GetPose() override { return pose; }
    
    //-------------------------------------------------------------------------
    // Update loop
    //-------------------------------------------------------------------------
    void UpdateLoop() {
        VRDriverLog()->Log("HapticGlove: Update thread started");
        
        while(active) {
            if(serial && serial->IsOpen()) {
                std::string line = serial->ReadLine();
                if(!line.empty() && line.substr(0, 5) == "DATA:") {
                    if(ParseData(line.substr(5))) {
                        UpdatePose();
                        UpdateInputs();
                    }
                }
            } else {
                // No serial - send dummy updates to keep device alive
                UpdatePose();
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        
        VRDriverLog()->Log("HapticGlove: Update thread ended");
    }
    
    bool ParseData(const std::string& data) {
        std::vector<std::string> tokens;
        std::stringstream ss(data);
        std::string token;
        
        while(std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        
        if(tokens.size() < 13) return false;
        
        try {
            gloveData.qw = std::stof(tokens[0]);
            gloveData.qx = std::stof(tokens[1]);
            gloveData.qy = std::stof(tokens[2]);
            gloveData.qz = std::stof(tokens[3]);
            gloveData.ax = std::stof(tokens[4]);
            gloveData.ay = std::stof(tokens[5]);
            gloveData.az = std::stof(tokens[6]);
            gloveData.fingers[0] = std::stoi(tokens[7]);
            gloveData.fingers[1] = std::stoi(tokens[8]);
            gloveData.fingers[2] = std::stoi(tokens[9]);
            gloveData.fingers[3] = std::stoi(tokens[10]);
            gloveData.fingers[4] = std::stoi(tokens[11]);
            gloveData.calibrated = (tokens[12] == "1");
            return true;
        } catch(...) {
            return false;
        }
    }
    
    void UpdatePose() {
        // Update rotation from IMU
        pose.qRotation.w = gloveData.qw;
        pose.qRotation.x = gloveData.qx;
        pose.qRotation.y = gloveData.qy;
        pose.qRotation.z = gloveData.qz;
        
        // Set tracking status based on calibration
        pose.result = gloveData.calibrated ? TrackingResult_Running_OK : TrackingResult_Running_OutOfRange;
        
        // Send pose to SteamVR
        if(deviceId != k_unTrackedDeviceIndexInvalid) {
            VRServerDriverHost()->TrackedDevicePoseUpdated(deviceId, pose, sizeof(DriverPose_t));
        }
    }
    
    void UpdateInputs() {
        if(deviceId == k_unTrackedDeviceIndexInvalid) return;
        
        // Convert finger values to 0.0-1.0 range
        float fingerValues[5];
        for(int i = 0; i < 5; i++) {
            fingerValues[i] = gloveData.fingers[i] / 1023.0f;
            VRDriverInput()->UpdateScalarComponent(inputFingers[i], fingerValues[i], 0.0);
        }
        
        // Simple gesture recognition
        bool grip = (fingerValues[0] > 0.7f && fingerValues[1] > 0.7f && fingerValues[2] > 0.7f);
        bool trigger = (fingerValues[1] > 0.8f);  // Index finger
        bool system = (fingerValues[0] > 0.8f && fingerValues[2] > 0.8f && 
                      fingerValues[3] > 0.8f && fingerValues[4] > 0.8f);  // All except index
        
        VRDriverInput()->UpdateBooleanComponent(inputGrip, grip, 0.0);
        VRDriverInput()->UpdateBooleanComponent(inputTrigger, trigger, 0.0);
        VRDriverInput()->UpdateBooleanComponent(inputSystem, system, 0.0);
    }
    
    void SendHaptic(float strength) {
        if(serial && serial->IsOpen()) {
            int value = 90 + (int)(strength * 90);  // 90-180 range
            std::string cmd = "HAPTIC:" + std::to_string(value) + "," + std::to_string(value) + "," +
                             std::to_string(value) + "," + std::to_string(value) + "," + std::to_string(value);
            serial->WriteLine(cmd);
        }
    }
};

//=============================================================================
// Driver Provider
//=============================================================================
class HapticGloveProvider : public IServerTrackedDeviceProvider {
private:
    HapticGlove* glove;
    
public:
    HapticGloveProvider() : glove(nullptr) {}
    
    EVRInitError Init(IVRDriverContext* pDriverContext) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
        VRDriverLog()->Log("HapticGloveProvider: Initializing");
        
        glove = new HapticGlove();
        VRServerDriverHost()->TrackedDeviceAdded("haptic_glove_right", TrackedDeviceClass_Controller, glove);
        
        VRDriverLog()->Log("HapticGloveProvider: Initialization complete");
        return VRInitError_None;
    }
    
    void Cleanup() override {
        VRDriverLog()->Log("HapticGloveProvider: Cleanup");
        if(glove) {
            delete glove;
            glove = nullptr;
        }
    }
    
    const char* const* GetInterfaceVersions() override { return k_InterfaceVersions; }
    void RunFrame() override {}
    bool ShouldBlockStandbyMode() override { return false; }
    void EnterStandby() override {}
    void LeaveStandby() override {}
};

//=============================================================================
// DLL Export
//=============================================================================
static HapticGloveProvider g_provider;

HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode) {
    if(0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName)) {
        return &g_provider;
    }
    
    if(pReturnCode) *pReturnCode = VRInitError_Init_InterfaceNotFound;
    return nullptr;
}