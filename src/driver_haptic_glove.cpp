/*
 * Simple VR Hand Tracker - Just rotation tracking, no buttons
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
#endif

//=============================================================================
// Serial Communication (same as before)
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
    
    bool IsOpen() const { return handle != INVALID_HANDLE_VALUE; }
};

//=============================================================================
// Simple Hand Tracker - Just rotation tracking
//=============================================================================
class SimpleHandTracker : public ITrackedDeviceServerDriver {
private:
    TrackedDeviceIndex_t deviceId;
    SerialPort* serial;
    std::thread updateThread;
    bool active;
    DriverPose_t pose;
    
    // Hand data
    struct {
        float qw, qx, qy, qz;
        float fingers[5];
    } handData;
    
public:
    SimpleHandTracker() : deviceId(k_unTrackedDeviceIndexInvalid), serial(nullptr), active(false) {
        // Initialize pose
        memset(&pose, 0, sizeof(pose));
        pose.qRotation.w = 1.0;
        pose.poseIsValid = true;
        pose.deviceIsConnected = true;
        pose.result = TrackingResult_Running_OK;
        
        // Initialize hand data
        handData.qw = 1.0f;
        handData.qx = handData.qy = handData.qz = 0.0f;
        for(int i = 0; i < 5; i++) handData.fingers[i] = 0.5f;
    }
    
    ~SimpleHandTracker() {
        if(serial) delete serial;
    }
    
    EVRInitError Activate(TrackedDeviceIndex_t unObjectId) override {
        deviceId = unObjectId;
        VRDriverLog()->Log("SimpleHandTracker: Activating");
        
        // Get COM port
        char portBuffer[32];
        vr::EVRSettingsError settingsError = vr::VRSettingsError_None;
        VRSettings()->GetString("driver_haptic_glove", "serial_port", portBuffer, sizeof(portBuffer), &settingsError);
        std::string portName = (settingsError == vr::VRSettingsError_None) ? std::string(portBuffer) : "COM3";
        
        VRDriverLog()->Log(("SimpleHandTracker: Using port " + portName).c_str());
        
        // Open serial
        serial = new SerialPort(portName);
        if(!serial->Open()) {
            VRDriverLog()->Log("SimpleHandTracker: Failed to open serial - continuing anyway");
            delete serial;
            serial = nullptr;
        } else {
            VRDriverLog()->Log("SimpleHandTracker: Serial opened successfully");
        }
        
        // Set properties - make it a basic tracker
        PropertyContainerHandle_t props = VRProperties()->TrackedDeviceToPropertyContainer(deviceId);
        VRProperties()->SetStringProperty(props, Prop_ModelNumber_String, "Hand Tracker");
        VRProperties()->SetStringProperty(props, Prop_SerialNumber_String, "HAND_001");
        VRProperties()->SetStringProperty(props, Prop_ManufacturerName_String, "Custom");
        VRProperties()->SetInt32Property(props, Prop_DeviceClass_Int32, TrackedDeviceClass_GenericTracker);
        
        VRDriverLog()->Log("SimpleHandTracker: Properties set");
        
        // Start update thread
        active = true;
        updateThread = std::thread(&SimpleHandTracker::UpdateLoop, this);
        
        VRDriverLog()->Log("SimpleHandTracker: Activation complete");
        return VRInitError_None;
    }
    
    void Deactivate() override {
        VRDriverLog()->Log("SimpleHandTracker: Deactivating");
        active = false;
        if(updateThread.joinable()) updateThread.join();
        deviceId = k_unTrackedDeviceIndexInvalid;
    }
    
    void EnterStandby() override {}
    void* GetComponent(const char* pchComponentNameAndVersion) override { return nullptr; }
    void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override {
        if(unResponseBufferSize >= 1) pchResponseBuffer[0] = 0;
    }
    
    DriverPose_t GetPose() override { return pose; }
    
    void UpdateLoop() {
        VRDriverLog()->Log("SimpleHandTracker: Update loop started");
        
        int updateCount = 0;
        
        while(active) {
            updateCount++;
            
            if(serial && serial->IsOpen()) {
                std::string line = serial->ReadLine();
                if(!line.empty() && line.substr(0, 5) == "DATA:") {
                    ParseData(line.substr(5));
                }
            }
            
            UpdatePose();
            
            // Log every 5 seconds
            if(updateCount % 250 == 1) {
                VRDriverLog()->Log(("SimpleHandTracker: Update " + std::to_string(updateCount) + 
                                   ", Quat: " + std::to_string(handData.qw) + "," + 
                                   std::to_string(handData.qx) + "," + std::to_string(handData.qy) + "," + 
                                   std::to_string(handData.qz)).c_str());
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        
        VRDriverLog()->Log("SimpleHandTracker: Update loop ended");
    }
    
    void ParseData(const std::string& data) {
        std::vector<std::string> tokens;
        std::stringstream ss(data);
        std::string token;
        
        while(std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        
        if(tokens.size() >= 12) {
            try {
                handData.qw = std::stof(tokens[0]);
                handData.qx = std::stof(tokens[1]);
                handData.qy = std::stof(tokens[2]);
                handData.qz = std::stof(tokens[3]);
                // Skip accelerometer (tokens 4-6)
                handData.fingers[0] = std::stof(tokens[7]) / 1023.0f;
                handData.fingers[1] = std::stof(tokens[8]) / 1023.0f;
                handData.fingers[2] = std::stof(tokens[9]) / 1023.0f;
                handData.fingers[3] = std::stof(tokens[10]) / 1023.0f;
                handData.fingers[4] = std::stof(tokens[11]) / 1023.0f;
            } catch(...) {
                // Ignore parse errors
            }
        }
    }
    
    void UpdatePose() {
        // Apply rotation directly
        pose.qRotation.w = handData.qw;
        pose.qRotation.x = handData.qx;
        pose.qRotation.y = handData.qy;
        pose.qRotation.z = handData.qz;
        
        // Fixed position - visible in front of user
        pose.vecPosition[0] = 0.3f;   // Right
        pose.vecPosition[1] = 1.2f;   // Up
        pose.vecPosition[2] = -0.5f;  // Forward
        
        // Always tracked
        pose.result = TrackingResult_Running_OK;
        pose.poseIsValid = true;
        pose.deviceIsConnected = true;
        
        // Send to SteamVR
        if(deviceId != k_unTrackedDeviceIndexInvalid) {
            VRServerDriverHost()->TrackedDevicePoseUpdated(deviceId, pose, sizeof(DriverPose_t));
        }
    }
};

//=============================================================================
// Simple Provider
//=============================================================================
class SimpleProvider : public IServerTrackedDeviceProvider {
private:
    SimpleHandTracker* tracker;
    
public:
    SimpleProvider() : tracker(nullptr) {}
    
    EVRInitError Init(IVRDriverContext* pDriverContext) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
        VRDriverLog()->Log("SimpleProvider: Initializing");
        
        tracker = new SimpleHandTracker();
        VRServerDriverHost()->TrackedDeviceAdded("simple_hand_tracker", TrackedDeviceClass_GenericTracker, tracker);
        
        VRDriverLog()->Log("SimpleProvider: Hand tracker added");
        return VRInitError_None;
    }
    
    void Cleanup() override {
        if(tracker) {
            delete tracker;
            tracker = nullptr;
        }
    }
    
    const char* const* GetInterfaceVersions() override { return k_InterfaceVersions; }
    void RunFrame() override {}
    bool ShouldBlockStandbyMode() override { return false; }
    void EnterStandby() override {}
    void LeaveStandby() override {}
};

//=============================================================================
// Export
//=============================================================================
static SimpleProvider g_provider;

HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode) {
    if(0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName)) {
        return &g_provider;
    }
    
    if(pReturnCode) *pReturnCode = VRInitError_Init_InterfaceNotFound;
    return nullptr;
}