/*
 * VR Haptic Glove OpenVR Driver - Index Controller Emulation
 * Fixed version with proper Index controller compatibility
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
    
    GloveData() : qw(1), qx(0), qy(0), qz(0), ax(0), ay(0), az(0), calibrated(true) {
        for(int i = 0; i < 5; i++) fingers[i] = 512;
    }
};

//=============================================================================
// Haptic Glove Device - Index Controller Emulation
//=============================================================================
class HapticGlove : public ITrackedDeviceServerDriver {
private:
    TrackedDeviceIndex_t deviceId;
    SerialPort* serial;
    std::thread updateThread;
    bool active;
    DriverPose_t pose;
    GloveData gloveData;
    
    // Index Controller Input Components
    VRInputComponentHandle_t inputA, inputB, inputTrigger, inputGrip;
    VRInputComponentHandle_t inputSystem, inputTrackpadClick, inputTrackpadTouch;
    VRInputComponentHandle_t inputThumbstickClick, inputThumbstickTouch;
    VRInputComponentHandle_t inputTriggerValue, inputGripValue;
    VRInputComponentHandle_t inputTrackpadX, inputTrackpadY;
    VRInputComponentHandle_t inputThumbstickX, inputThumbstickY;
    VRInputComponentHandle_t inputFingers[5];  // Finger curl values
    VRInputComponentHandle_t inputFingerSplay[4];  // Finger splay
    VRInputComponentHandle_t outputHaptic;
    
public:
    HapticGlove() : deviceId(k_unTrackedDeviceIndexInvalid), serial(nullptr), active(false) {
        // Initialize pose
        memset(&pose, 0, sizeof(pose));
        pose.qRotation.w = 1.0;
        pose.poseIsValid = true;
        pose.deviceIsConnected = true;
        pose.result = TrackingResult_Running_OK;
        pose.willDriftInYaw = false;
        pose.shouldApplyHeadModel = false;
        
        // Initialize all input handles
        inputA = inputB = inputTrigger = inputGrip = k_ulInvalidInputComponentHandle;
        inputSystem = inputTrackpadClick = inputTrackpadTouch = k_ulInvalidInputComponentHandle;
        inputThumbstickClick = inputThumbstickTouch = k_ulInvalidInputComponentHandle;
        inputTriggerValue = inputGripValue = k_ulInvalidInputComponentHandle;
        inputTrackpadX = inputTrackpadY = k_ulInvalidInputComponentHandle;
        inputThumbstickX = inputThumbstickY = k_ulInvalidInputComponentHandle;
        outputHaptic = k_ulInvalidInputComponentHandle;
        
        for(int i = 0; i < 5; i++) {
            inputFingers[i] = k_ulInvalidInputComponentHandle;
        }
        for(int i = 0; i < 4; i++) {
            inputFingerSplay[i] = k_ulInvalidInputComponentHandle;
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
        VRDriverLog()->Log("=== HapticGlove: Starting Activation (Index Controller Mode) ===");
        
        // Get COM port from settings
        char portBuffer[32];
        vr::EVRSettingsError settingsError = vr::VRSettingsError_None;
        VRSettings()->GetString("driver_haptic_glove", "serial_port", portBuffer, sizeof(portBuffer), &settingsError);
        
        std::string portName = (settingsError == vr::VRSettingsError_None) ? std::string(portBuffer) : "COM3";
        VRDriverLog()->Log(("HapticGlove: Using COM port: " + portName).c_str());
        
        // Open serial connection
        serial = new SerialPort(portName);
        if(!serial->Open()) {
            VRDriverLog()->Log("HapticGlove: Failed to open serial port - continuing without");
            delete serial;
            serial = nullptr;
        } else {
            VRDriverLog()->Log("HapticGlove: Serial port opened successfully");
        }
        
        // Set Index Controller properties
        PropertyContainerHandle_t props = VRProperties()->TrackedDeviceToPropertyContainer(deviceId);
        
        VRProperties()->SetStringProperty(props, Prop_ModelNumber_String, "Knuckles Right");
        VRProperties()->SetStringProperty(props, Prop_SerialNumber_String, "LHR-E217CD00");
        VRProperties()->SetStringProperty(props, Prop_ManufacturerName_String, "Valve");
        VRProperties()->SetStringProperty(props, Prop_RenderModelName_String, "{indexcontroller}valve_controller_knu_1_0_right");
        VRProperties()->SetStringProperty(props, Prop_ControllerType_String, "knuckles");
        VRProperties()->SetInt32Property(props, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
        VRProperties()->SetInt32Property(props, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);
        VRProperties()->SetBoolProperty(props, Prop_WillDriftInYaw_Bool, false);
        VRProperties()->SetBoolProperty(props, Prop_DeviceCanPowerOff_Bool, true);
        VRProperties()->SetBoolProperty(props, Prop_DeviceProvidesBatteryStatus_Bool, false);
        
        // Input profile path - use Index controller profile
        VRProperties()->SetStringProperty(props, Prop_InputProfilePath_String, "{indexcontroller}/input/index_controller_profile.json");
        
        VRDriverLog()->Log("HapticGlove: Index Controller properties set");
        
        // Create Index Controller input components
        CreateInputComponents(props);
        
        // Initialize pose at a visible position
        pose.vecPosition[0] = 0.3f;   // 30cm right
        pose.vecPosition[1] = 1.0f;   // 1m up
        pose.vecPosition[2] = -0.5f;  // 50cm forward
        pose.result = TrackingResult_Running_OK;
        pose.poseIsValid = true;
        pose.deviceIsConnected = true;
        
        // Send initial pose
        VRServerDriverHost()->TrackedDevicePoseUpdated(deviceId, pose, sizeof(DriverPose_t));
        
        // Start update thread
        active = true;
        updateThread = std::thread(&HapticGlove::UpdateLoop, this);
        
        VRDriverLog()->Log("=== HapticGlove: Activation COMPLETE ===");
        return VRInitError_None;
    }
    
    void CreateInputComponents(PropertyContainerHandle_t props) {
        VRDriverLog()->Log("HapticGlove: Creating Index Controller input components");
        
        // Create all Index Controller inputs to maintain compatibility
        VRDriverInput()->CreateBooleanComponent(props, "/input/a/click", &inputA);
        VRDriverInput()->CreateBooleanComponent(props, "/input/b/click", &inputB);
        VRDriverInput()->CreateBooleanComponent(props, "/input/trigger/click", &inputTrigger);
        VRDriverInput()->CreateBooleanComponent(props, "/input/grip/click", &inputGrip);
        VRDriverInput()->CreateBooleanComponent(props, "/input/system/click", &inputSystem);
        VRDriverInput()->CreateBooleanComponent(props, "/input/trackpad/click", &inputTrackpadClick);
        VRDriverInput()->CreateBooleanComponent(props, "/input/trackpad/touch", &inputTrackpadTouch);
        VRDriverInput()->CreateBooleanComponent(props, "/input/thumbstick/click", &inputThumbstickClick);
        VRDriverInput()->CreateBooleanComponent(props, "/input/thumbstick/touch", &inputThumbstickTouch);
        
        // Analog inputs
        VRDriverInput()->CreateScalarComponent(props, "/input/trigger/value", &inputTriggerValue, 
                                              VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
        VRDriverInput()->CreateScalarComponent(props, "/input/grip/value", &inputGripValue,
                                              VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
        VRDriverInput()->CreateScalarComponent(props, "/input/trackpad/x", &inputTrackpadX,
                                              VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
        VRDriverInput()->CreateScalarComponent(props, "/input/trackpad/y", &inputTrackpadY,
                                              VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
        VRDriverInput()->CreateScalarComponent(props, "/input/thumbstick/x", &inputThumbstickX,
                                              VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
        VRDriverInput()->CreateScalarComponent(props, "/input/thumbstick/y", &inputThumbstickY,
                                              VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
        
        // Finger tracking - Index Controller style
        const char* fingerPaths[] = {
            "/input/finger/thumb",
            "/input/finger/index", 
            "/input/finger/middle",
            "/input/finger/ring",
            "/input/finger/pinky"
        };
        
        for(int i = 0; i < 5; i++) {
            VRDriverInput()->CreateScalarComponent(props, fingerPaths[i], &inputFingers[i],
                                                  VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
        }
        
        // Finger splay (separation between fingers)
        const char* splayPaths[] = {
            "/input/finger/index_middle_splay",
            "/input/finger/middle_ring_splay", 
            "/input/finger/ring_pinky_splay",
            "/input/finger/thumb_index_splay"
        };
        
        for(int i = 0; i < 4; i++) {
            VRDriverInput()->CreateScalarComponent(props, splayPaths[i], &inputFingerSplay[i],
                                                  VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
        }
        
        // Haptic output
        VRDriverInput()->CreateHapticComponent(props, "/output/haptic", &outputHaptic);
        
        VRDriverLog()->Log("HapticGlove: All Index Controller components created");
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
                // No serial - send dummy updates with reasonable finger positions
                UpdatePose();
                UpdateInputs();
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz
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
            gloveData.calibrated = true;
            return true;
        } catch(...) {
            return false;
        }
    }
    
    void UpdatePose() {
        // Transform IMU quaternion to SteamVR coordinate system
        // BNO055 coordinate system to SteamVR coordinate system transformation
        // This mapping may need adjustment based on your IMU mounting orientation
        
        float qw = gloveData.qw;
        float qx = gloveData.qx;
        float qy = gloveData.qy;
        float qz = gloveData.qz;
        
        // Apply coordinate transformation (adjust these based on your IMU orientation)
        // This is for a right-hand glove with IMU mounted in standard orientation
        pose.qRotation.w = qw;
        pose.qRotation.x = -qy;  // Swap and negate to align with SteamVR
        pose.qRotation.y = qz;
        pose.qRotation.z = -qx;
        
        // Normalize quaternion
        float magnitude = sqrt(pose.qRotation.w * pose.qRotation.w + 
                              pose.qRotation.x * pose.qRotation.x +
                              pose.qRotation.y * pose.qRotation.y + 
                              pose.qRotation.z * pose.qRotation.z);
        
        if(magnitude > 0.001f) {
            pose.qRotation.w /= magnitude;
            pose.qRotation.x /= magnitude;
            pose.qRotation.y /= magnitude;
            pose.qRotation.z /= magnitude;
        } else {
            // Fallback to identity quaternion
            pose.qRotation.w = 1.0f;
            pose.qRotation.x = pose.qRotation.y = pose.qRotation.z = 0.0f;
        }
        
        // Simple position tracking using accelerometer
        static float baseX = 0.3f, baseY = 1.0f, baseZ = -0.5f;
        static float velX = 0.0f, velY = 0.0f, velZ = 0.0f;
        
        // Integrate acceleration for position (very basic - you may want to improve this)
        float dt = 0.02f; // 20ms
        velX += gloveData.ax * dt * 0.001f;
        velY += gloveData.ay * dt * 0.001f;
        velZ += gloveData.az * dt * 0.001f;
        
        // Apply damping to prevent drift
        velX *= 0.95f;
        velY *= 0.95f;
        velZ *= 0.95f;
        
        pose.vecPosition[0] = baseX + velX;
        pose.vecPosition[1] = baseY + velY;
        pose.vecPosition[2] = baseZ + velZ;
        
        // Set velocities for SteamVR prediction
        pose.vecVelocity[0] = velX / dt;
        pose.vecVelocity[1] = velY / dt;
        pose.vecVelocity[2] = velZ / dt;
        
        // Set tracking result
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
        
        // Update finger tracking - map potentiometer values to curl
        for(int i = 0; i < 5; i++) {
            if(inputFingers[i] != k_ulInvalidInputComponentHandle) {
                // Convert 0-1023 range to 0.0-1.0 curl (0 = open, 1 = closed)
                float curl = gloveData.fingers[i] / 1023.0f;
                VRDriverInput()->UpdateScalarComponent(inputFingers[i], curl, 0.0);
            }
        }
        
        // Update finger splay (set to neutral for now)
        for(int i = 0; i < 4; i++) {
            if(inputFingerSplay[i] != k_ulInvalidInputComponentHandle) {
                VRDriverInput()->UpdateScalarComponent(inputFingerSplay[i], 0.0f, 0.0);
            }
        }
        
        // Generate button presses based on finger positions
        float fingerCurls[5];
        for(int i = 0; i < 5; i++) {
            fingerCurls[i] = gloveData.fingers[i] / 1023.0f;
        }
        
        // Gesture recognition
        bool triggerPress = fingerCurls[1] > 0.7f; // Index finger curl
        bool gripPress = (fingerCurls[2] > 0.7f && fingerCurls[3] > 0.7f && fingerCurls[4] > 0.7f); // Middle+Ring+Pinky
        bool aPress = fingerCurls[0] > 0.8f; // Thumb curl
        bool bPress = (fingerCurls[0] > 0.7f && fingerCurls[1] > 0.7f); // Thumb+Index
        
        // Update button states
        VRDriverInput()->UpdateBooleanComponent(inputTrigger, triggerPress, 0.0);
        VRDriverInput()->UpdateBooleanComponent(inputGrip, gripPress, 0.0);
        VRDriverInput()->UpdateBooleanComponent(inputA, aPress, 0.0);
        VRDriverInput()->UpdateBooleanComponent(inputB, bPress, 0.0);
        
        // Update analog trigger and grip values
        VRDriverInput()->UpdateScalarComponent(inputTriggerValue, fingerCurls[1], 0.0);
        VRDriverInput()->UpdateScalarComponent(inputGripValue, (fingerCurls[2] + fingerCurls[3] + fingerCurls[4]) / 3.0f, 0.0);
        
        // Set other inputs to neutral/false
        VRDriverInput()->UpdateBooleanComponent(inputSystem, false, 0.0);
        VRDriverInput()->UpdateBooleanComponent(inputTrackpadClick, false, 0.0);
        VRDriverInput()->UpdateBooleanComponent(inputTrackpadTouch, false, 0.0);
        VRDriverInput()->UpdateBooleanComponent(inputThumbstickClick, false, 0.0);
        VRDriverInput()->UpdateBooleanComponent(inputThumbstickTouch, false, 0.0);
        
        VRDriverInput()->UpdateScalarComponent(inputTrackpadX, 0.0f, 0.0);
        VRDriverInput()->UpdateScalarComponent(inputTrackpadY, 0.0f, 0.0);
        VRDriverInput()->UpdateScalarComponent(inputThumbstickX, 0.0f, 0.0);
        VRDriverInput()->UpdateScalarComponent(inputThumbstickY, 0.0f, 0.0);
    }
    
    void ProcessHapticEvent(const VREvent_HapticVibration_t& hapticData) {
        if(serial && serial->IsOpen()) {
            // Convert haptic data to servo positions
            int intensity = 90 + (int)(hapticData.fAmplitude * 90); // 90-180 range
            std::string cmd = "HAPTIC:" + std::to_string(intensity) + "," + 
                             std::to_string(intensity) + "," + std::to_string(intensity) + "," +
                             std::to_string(intensity) + "," + std::to_string(intensity);
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
        VRDriverLog()->Log("HapticGloveProvider: Initializing Index Controller Mode");
        
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