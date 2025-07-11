/*
 * VR Haptic Glove OpenVR Driver
 * Final clean version - finger tracking only, 9600 baud
 */

#include <openvr_driver.h>
#include <windows.h>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

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
        
        DCB dcb = {0};
        dcb.DCBlength = sizeof(dcb);
        if(!GetCommState(handle, &dcb)) return false;
        
        dcb.BaudRate = CBR_9600;
        dcb.ByteSize = 8;
        dcb.StopBits = ONESTOPBIT;
        dcb.Parity = NOPARITY;
        
        if(!SetCommState(handle, &dcb)) return false;
        
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
        return WriteFile(handle, line.c_str(), static_cast<DWORD>(line.length()), &written, NULL);
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
    VRInputComponentHandle_t inputGrip, inputTrigger;
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
        
        // Initialize input handles
        inputGrip = k_ulInvalidInputComponentHandle;
        inputTrigger = k_ulInvalidInputComponentHandle;
        outputHaptic = k_ulInvalidInputComponentHandle;
        for(int i = 0; i < 5; i++) {
            inputFingers[i] = k_ulInvalidInputComponentHandle;
        }
    }
    
    ~HapticGlove() {
        if(serial) delete serial;
    }
    
    //-------------------------------------------------------------------------
    // ITrackedDeviceServerDriver interface
    //-------------------------------------------------------------------------
    EVRInitError Activate(TrackedDeviceIndex_t unObjectId) override {
        deviceId = unObjectId;
        VRDriverLog()->Log("=== HapticGlove: Starting Activation ===");
        
        // Get COM port from settings
        char portBuffer[32];
        vr::EVRSettingsError settingsError = vr::VRSettingsError_None;
        VRSettings()->GetString("driver_haptic_glove", "serial_port", portBuffer, sizeof(portBuffer), &settingsError);
        
        std::string portName;
        if(settingsError == vr::VRSettingsError_None) {
            portName = std::string(portBuffer);
            VRDriverLog()->Log(("HapticGlove: Using COM port: " + portName).c_str());
        } else {
            portName = "COM3";
            VRDriverLog()->Log("HapticGlove: Using default COM port: COM3");
        }
        
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
        
        VRProperties()->SetStringProperty(props, Prop_ModelNumber_String, "Knuckles Right");
        VRProperties()->SetStringProperty(props, Prop_SerialNumber_String, "HG_RIGHT_001");
        VRProperties()->SetStringProperty(props, Prop_ManufacturerName_String, "Valve");
        VRProperties()->SetStringProperty(props, Prop_RenderModelName_String, "{indexcontroller}valve_controller_knu_1_0_right");
        VRProperties()->SetStringProperty(props, Prop_ControllerType_String, "knuckles");
        VRProperties()->SetInt32Property(props, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
        VRProperties()->SetInt32Property(props, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);
        VRProperties()->SetBoolProperty(props, Prop_WillDriftInYaw_Bool, false);
        VRProperties()->SetBoolProperty(props, Prop_DeviceCanPowerOff_Bool, true);
        
        VRDriverLog()->Log("HapticGlove: Properties set successfully");
        
        // Create input components
        VRDriverLog()->Log("HapticGlove: Creating input components");
        
        EVRInputError inputError;
        
        // Create finger tracking components
        const char* fingerNames[] = {"/input/finger/thumb", "/input/finger/index", 
                                   "/input/finger/middle", "/input/finger/ring", "/input/finger/pinky"};
        for(int i = 0; i < 5; i++) {
            inputError = VRDriverInput()->CreateScalarComponent(props, fingerNames[i], &inputFingers[i], 
                                                 VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
            if(inputError == VRInputError_None) {
                VRDriverLog()->Log(("HapticGlove: Finger " + std::to_string(i) + " component created").c_str());
            }
        }
        
        // Create gesture-based buttons
        inputError = VRDriverInput()->CreateBooleanComponent(props, "/input/grip/click", &inputGrip);
        if(inputError == VRInputError_None) {
            VRDriverLog()->Log("HapticGlove: Grip component created");
        }
        
        inputError = VRDriverInput()->CreateBooleanComponent(props, "/input/trigger/click", &inputTrigger);
        if(inputError == VRInputError_None) {
            VRDriverLog()->Log("HapticGlove: Trigger component created");
        }
        
        // Create haptic output
        inputError = VRDriverInput()->CreateHapticComponent(props, "/output/haptic", &outputHaptic);
        if(inputError == VRInputError_None) {
            VRDriverLog()->Log("HapticGlove: Haptic component created");
        }
        
        VRDriverLog()->Log("HapticGlove: All components created");
        
        // Initialize pose
        pose.poseIsValid = true;
        pose.deviceIsConnected = true;
        pose.result = TrackingResult_Running_OK;
        pose.qRotation.w = 1.0;
        pose.qRotation.x = pose.qRotation.y = pose.qRotation.z = 0.0;
        pose.vecPosition[0] = 0.3f;   // 30cm right
        pose.vecPosition[1] = 1.0f;   // 1m up
        pose.vecPosition[2] = -0.5f;  // 50cm forward
        
        // Send initial pose
        VRServerDriverHost()->TrackedDevicePoseUpdated(deviceId, pose, sizeof(DriverPose_t));
        
        // Start update thread
        active = true;
        updateThread = std::thread(&HapticGlove::UpdateLoop, this);
        
        VRDriverLog()->Log("=== HapticGlove: Activation COMPLETE ===");
        return VRInitError_None;
    }
    
    void Deactivate() override {
        VRDriverLog()->Log("HapticGlove: Deactivating");
        active = false;
        if(updateThread.joinable()) updateThread.join();
        deviceId = k_unTrackedDeviceIndexInvalid;
    }
    
    void PowerOff() {
        VRDriverLog()->Log("HapticGlove: Power off");
        if(serial && serial->IsOpen()) {
            serial->WriteLine("HAPTIC:90,90,90,90,90");
        }
    }
    
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
        
        int updateCount = 0;
        
        while(active) {
            updateCount++;
            
            if(serial && serial->IsOpen()) {
                std::string line = serial->ReadLine();
                if(!line.empty() && line.substr(0, 5) == "DATA:") {
                    if(ParseData(line.substr(5))) {
                        UpdatePose();
                        UpdateInputs();
                    }
                }
            } else {
                // No serial - send dummy updates
                UpdatePose();
                UpdateInputs();
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
        // Map IMU coordinates to SteamVR coordinates
        float qw = gloveData.qw;
        float qx = gloveData.qx;
        float qy = gloveData.qy;
        float qz = gloveData.qz;
        
        // Apply coordinate transformation for right hand
        pose.qRotation.w = qw;
        pose.qRotation.x = -qz;  // Try this mapping first
        pose.qRotation.y = qx;   
        pose.qRotation.z = qy;   
        
        // Normalize quaternion
        float magnitude = sqrt(pose.qRotation.w * pose.qRotation.w + 
                              pose.qRotation.x * pose.qRotation.x +
                              pose.qRotation.y * pose.qRotation.y + 
                              pose.qRotation.z * pose.qRotation.z);
        
        if(magnitude > 0.0f) {
            pose.qRotation.w /= magnitude;
            pose.qRotation.x /= magnitude;
            pose.qRotation.y /= magnitude;
            pose.qRotation.z /= magnitude;
        }
        
        // Simple position tracking
        static float baseX = 0.3f, baseY = 1.0f, baseZ = -0.5f;
        
        pose.vecPosition[0] = baseX + (gloveData.ax * 0.005f);
        pose.vecPosition[1] = baseY + (gloveData.ay * 0.005f);
        pose.vecPosition[2] = baseZ + (gloveData.az * 0.005f);
        
        // Set tracking status
        pose.result = gloveData.calibrated ? TrackingResult_Running_OK : TrackingResult_Running_OutOfRange;
        pose.poseIsValid = true;
        pose.deviceIsConnected = true;
        
        // Send pose to SteamVR
        if(deviceId != k_unTrackedDeviceIndexInvalid) {
            VRServerDriverHost()->TrackedDevicePoseUpdated(deviceId, pose, sizeof(DriverPose_t));
        }
    }
    
    void UpdateInputs() {
        if(deviceId == k_unTrackedDeviceIndexInvalid) return;
        
        // Update finger tracking
        for(int i = 0; i < 5; i++) {
            if(inputFingers[i] != k_ulInvalidInputComponentHandle) {
                float fingerValue = gloveData.fingers[i] / 1023.0f;
                VRDriverInput()->UpdateScalarComponent(inputFingers[i], fingerValue, 0.0);
            }
        }
        
        // Gesture recognition
        float fingerValues[5];
        for(int i = 0; i < 5; i++) {
            fingerValues[i] = gloveData.fingers[i] / 1023.0f;
        }
        
        bool makesFist = (fingerValues[0] > 0.7f && fingerValues[1] > 0.7f && 
                         fingerValues[2] > 0.7f && fingerValues[3] > 0.7f && fingerValues[4] > 0.7f);
        bool pointingGesture = (fingerValues[1] < 0.3f && fingerValues[2] > 0.7f && 
                               fingerValues[3] > 0.7f && fingerValues[4] > 0.7f);
        
        // Update gesture buttons
        if(inputGrip != k_ulInvalidInputComponentHandle) {
            VRDriverInput()->UpdateBooleanComponent(inputGrip, makesFist, 0.0);
        }
        if(inputTrigger != k_ulInvalidInputComponentHandle) {
            VRDriverInput()->UpdateBooleanComponent(inputTrigger, pointingGesture, 0.0);
        }
        
        // Occasional logging
        static int logCount = 0;
        if(++logCount % 100 == 1) {
            VRDriverLog()->Log(("HapticGlove: Fist=" + std::to_string(makesFist) + 
                               " Point=" + std::to_string(pointingGesture)).c_str());
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