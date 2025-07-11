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
    
    // Input components - only what we actually create
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
        
        // Initialize all input handles to invalid
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
        VRDriverLog()->Log(("HapticGlove: Device ID assigned: " + std::to_string(unObjectId)).c_str());
        
        // Get COM port from settings
        char portBuffer[32];
        vr::EVRSettingsError settingsError = vr::VRSettingsError_None;
        VRSettings()->GetString("driver_haptic_glove", "serial_port", portBuffer, sizeof(portBuffer), &settingsError);
        
        std::string portName;
        if(settingsError == vr::VRSettingsError_None) {
            portName = std::string(portBuffer);
            VRDriverLog()->Log(("HapticGlove: Using COM port from settings: " + portName).c_str());
        } else {
            portName = "COM3";  // Default fallback
            VRDriverLog()->Log(("HapticGlove: Settings error " + std::to_string(settingsError) + ", using default: " + portName).c_str());
        }
        
        // Open serial connection
        VRDriverLog()->Log(("HapticGlove: Attempting to open " + portName).c_str());
        serial = new SerialPort(portName);
        if(!serial->Open()) {
            VRDriverLog()->Log("HapticGlove: FAILED to open serial port - continuing without serial");
            delete serial;
            serial = nullptr;
        } else {
            VRDriverLog()->Log("HapticGlove: Serial port opened successfully");
            
            // Test reading a few lines
            for(int i = 0; i < 5; i++) {
                std::string testLine = serial->ReadLine();
                if(!testLine.empty()) {
                    VRDriverLog()->Log(("HapticGlove: Received: " + testLine).c_str());
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        // Set device properties - Make it appear as a simple controller SteamVR recognizes
        VRDriverLog()->Log("HapticGlove: Setting device properties");
        PropertyContainerHandle_t props = VRProperties()->TrackedDeviceToPropertyContainer(deviceId);
        
        VRProperties()->SetStringProperty(props, Prop_ModelNumber_String, "Vive Controller");
        VRProperties()->SetStringProperty(props, Prop_SerialNumber_String, "LHR-CB001001");
        VRProperties()->SetStringProperty(props, Prop_ManufacturerName_String, "HTC");
        VRProperties()->SetStringProperty(props, Prop_RenderModelName_String, "vr_controller_vive_1_5");
        VRProperties()->SetStringProperty(props, Prop_ControllerType_String, "vive_controller");
        VRProperties()->SetInt32Property(props, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
        VRProperties()->SetInt32Property(props, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);
        VRProperties()->SetBoolProperty(props, Prop_WillDriftInYaw_Bool, false);
        VRProperties()->SetBoolProperty(props, Prop_DeviceCanPowerOff_Bool, true);
        
        // Enable hand tracking
        VRProperties()->SetBoolProperty(props, Prop_DeviceProvidesBatteryStatus_Bool, false);
        VRProperties()->SetBoolProperty(props, Prop_DeviceCanPowerOff_Bool, true);
        VRProperties()->SetInt32Property(props, Prop_Axis0Type_Int32, k_eControllerAxis_Trigger);
        VRProperties()->SetInt32Property(props, Prop_Axis1Type_Int32, k_eControllerAxis_TrackPad);
        
        // CRITICAL: Set the input profile path
        VRProperties()->SetStringProperty(props, Prop_InputProfilePath_String, "{haptic_glove_driver}/input/haptic_glove_input_profile.json");
        
        VRDriverLog()->Log("HapticGlove: Properties set successfully");
        
        // Create only the essential input components that Vive controller expects
        VRDriverLog()->Log("HapticGlove: Creating input components");
        
        EVRInputError inputError;
        
        // Create basic Vive controller inputs
        inputError = VRDriverInput()->CreateBooleanComponent(props, "/input/trigger/click", &inputTrigger);
        if(inputError == VRInputError_None) {
            VRDriverLog()->Log("HapticGlove: Trigger component created successfully");
        } else {
            VRDriverLog()->Log(("HapticGlove: Trigger component FAILED: " + std::to_string(inputError)).c_str());
            inputTrigger = k_ulInvalidInputComponentHandle;
        }
        
        inputError = VRDriverInput()->CreateBooleanComponent(props, "/input/grip/click", &inputGrip);
        if(inputError == VRInputError_None) {
            VRDriverLog()->Log("HapticGlove: Grip component created successfully");
        } else {
            VRDriverLog()->Log(("HapticGlove: Grip component FAILED: " + std::to_string(inputError)).c_str());
            inputGrip = k_ulInvalidInputComponentHandle;
        }
        
        // Create finger tracking components (these are extra)
        const char* fingerNames[] = {"/input/finger/thumb", "/input/finger/index", 
                                   "/input/finger/middle", "/input/finger/ring", "/input/finger/pinky"};
        for(int i = 0; i < 5; i++) {
            inputError = VRDriverInput()->CreateScalarComponent(props, fingerNames[i], &inputFingers[i], 
                                                 VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
            if(inputError != VRInputError_None) {
                inputFingers[i] = k_ulInvalidInputComponentHandle;
            }
        }
        
        // Create haptic output
        inputError = VRDriverInput()->CreateHapticComponent(props, "/output/haptic", &outputHaptic);
        if(inputError != VRInputError_None) {
            outputHaptic = k_ulInvalidInputComponentHandle;
        }
        
        VRDriverLog()->Log("HapticGlove: All components created");
        
        // Initialize pose properly
        VRDriverLog()->Log("HapticGlove: Initializing pose");
        pose.poseIsValid = true;
        pose.deviceIsConnected = true;
        pose.result = TrackingResult_Running_OK;
        pose.qRotation.w = 1.0;
        pose.qRotation.x = pose.qRotation.y = pose.qRotation.z = 0.0;
        pose.vecPosition[0] = pose.vecPosition[1] = pose.vecPosition[2] = 0.0f;
        pose.vecVelocity[0] = pose.vecVelocity[1] = pose.vecVelocity[2] = 0.0f;
        pose.vecAngularVelocity[0] = pose.vecAngularVelocity[1] = pose.vecAngularVelocity[2] = 0.0f;
        pose.willDriftInYaw = false;
        pose.shouldApplyHeadModel = false;
        pose.poseTimeOffset = 0.0;
        
        // Send initial pose
        VRServerDriverHost()->TrackedDevicePoseUpdated(deviceId, pose, sizeof(DriverPose_t));
        VRDriverLog()->Log("HapticGlove: Initial pose sent");
        
        // Start update thread
        VRDriverLog()->Log("HapticGlove: Starting update thread");
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
            serial->WriteLine("HAPTIC:90,90,90,90,90");  // Reset servos
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
        int dataPackets = 0;
        
        while(active) {
            updateCount++;
            
            if(serial && serial->IsOpen()) {
                std::string line = serial->ReadLine();
                if(!line.empty() && line.substr(0, 5) == "DATA:") {
                    dataPackets++;
                    if(ParseData(line.substr(5))) {
                        UpdatePose();
                        UpdateInputs();
                        
                        // Log every 50 packets to avoid spam
                        if(dataPackets % 50 == 1) {
                            VRDriverLog()->Log(("HapticGlove: Processed " + std::to_string(dataPackets) + " data packets").c_str());
                        }
                    }
                } else if(!line.empty()) {
                    // Log non-data messages
                    VRDriverLog()->Log(("HapticGlove: Arduino says: " + line).c_str());
                }
            } else {
                // No serial - send dummy updates to keep device alive
                UpdatePose();
                UpdateInputs();  // Update with dummy data
                
                if(updateCount % 250 == 1) {  // Every 5 seconds
                    VRDriverLog()->Log("HapticGlove: No serial connection - sending dummy updates");
                }
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
        
        // Changed: Now expecting 12 tokens instead of 13 (no calibration)
        if(tokens.size() < 12) return false;
        
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
            gloveData.calibrated = true;  // Always assume calibrated in IMUPLUS mode
            return true;
        } catch(...) {
            return false;
        }
    }
    
    void UpdatePose() {
        // Debug logging to see if we're getting data
        static int poseCount = 0;
        poseCount++;
        
        if(poseCount % 100 == 1) {
            VRDriverLog()->Log(("UpdatePose called " + std::to_string(poseCount) + " times").c_str());
            VRDriverLog()->Log(("Current quat: w=" + std::to_string(gloveData.qw) + 
                               " x=" + std::to_string(gloveData.qx) + 
                               " y=" + std::to_string(gloveData.qy) + 
                               " z=" + std::to_string(gloveData.qz)).c_str());
        }
        
        // Apply rotation from IMU - try direct mapping first
        pose.qRotation.w = gloveData.qw;
        pose.qRotation.x = gloveData.qx;
        pose.qRotation.y = gloveData.qy;
        pose.qRotation.z = gloveData.qz;
        
        // If rotation values are clearly changing, log it
        static float lastQw = 1.0f;
        if(abs(gloveData.qw - lastQw) > 0.01f) {
            VRDriverLog()->Log("Rotation detected - quaternion changed!");
            lastQw = gloveData.qw;
        }
        
        // Keep position fixed and visible
        pose.vecPosition[0] = 0.3f;   // 30cm to the right
        pose.vecPosition[1] = 1.2f;   // 1.2m up from floor (higher)
        pose.vecPosition[2] = -0.5f;  // 50cm forward
        
        // Force the device to be tracked and valid
        pose.result = TrackingResult_Running_OK;
        pose.poseIsValid = true;
        pose.deviceIsConnected = true;
        pose.poseTimeOffset = 0.0;
        
        // Send pose to SteamVR
        if(deviceId != k_unTrackedDeviceIndexInvalid) {
            VRServerDriverHost()->TrackedDevicePoseUpdated(deviceId, pose, sizeof(DriverPose_t));
        }
    }
    
    void UpdateInputs() {
        if(deviceId == k_unTrackedDeviceIndexInvalid) return;
        
        // Convert finger values to 0.0-1.0 range
        float fingerValues[5];
        static int logCounter = 0;
        logCounter++;
        
        for(int i = 0; i < 5; i++) {
            fingerValues[i] = gloveData.fingers[i] / 1023.0f;
            
            // Update the component
            EVRInputError error = VRDriverInput()->UpdateScalarComponent(inputFingers[i], fingerValues[i], 0.0);
            
            // Log occasionally to debug
            if(logCounter % 100 == 1 && i == 0) {  // Log every 100 updates for thumb only
                VRDriverLog()->Log(("HapticGlove: Thumb value: " + std::to_string(fingerValues[i]) + 
                                   " (raw: " + std::to_string(gloveData.fingers[i]) + "), error: " + std::to_string(error)).c_str());
            }
        }
        
        // Simple gesture recognition
        bool grip = (fingerValues[0] > 0.7f && fingerValues[1] > 0.7f && fingerValues[2] > 0.7f);
        bool trigger = (fingerValues[1] > 0.8f);  // Index finger
        bool system = (fingerValues[0] > 0.8f && fingerValues[2] > 0.8f && 
                      fingerValues[3] > 0.8f && fingerValues[4] > 0.8f);  // All except index
        
        // Update button components
        VRDriverInput()->UpdateBooleanComponent(inputGrip, grip, 0.0);
        VRDriverInput()->UpdateBooleanComponent(inputTrigger, trigger, 0.0);
        
        // Log button states occasionally
        if(logCounter % 100 == 1) {
            VRDriverLog()->Log(("HapticGlove: Buttons - Grip: " + std::to_string(grip) + 
                               ", Trigger: " + std::to_string(trigger) + 
                               ", System: " + std::to_string(system)).c_str());
        }
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