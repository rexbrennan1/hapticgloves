/*
 * OpenVR Skeletal Hand Tracking Driver
 * Complete implementation for haptic glove with BNO055 IMU and finger potentiometers
 * 
 * Key Concepts Explained:
 * - Uses 31-bone hand skeleton with proper parent-child relationships
 * - Implements dual motion ranges (WithController/WithoutController) 
 * - Transforms BNO055 quaternions to OpenVR coordinate system
 * - Maintains 90Hz update rate for smooth VR tracking
 */

#include <openvr_driver.h>
#include <windows.h>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <array>

using namespace vr;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec(dllexport)
#else
#error "Windows only implementation"
#endif

//=============================================================================
// Serial Communication Helper
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
        
        // Configure serial port for optimal performance
        DCB dcb = {0};
        dcb.DCBlength = sizeof(dcb);
        if(!GetCommState(handle, &dcb)) return false;
        
        dcb.BaudRate = CBR_115200;  // Higher baud rate for 90Hz updates
        dcb.ByteSize = 8;
        dcb.StopBits = ONESTOPBIT;
        dcb.Parity = NOPARITY;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        
        if(!SetCommState(handle, &dcb)) return false;
        
        // Aggressive timeouts for real-time performance
        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 10;
        timeouts.ReadTotalTimeoutConstant = 10;
        timeouts.ReadTotalTimeoutMultiplier = 1;
        timeouts.WriteTotalTimeoutConstant = 10;
        timeouts.WriteTotalTimeoutMultiplier = 1;
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
            if(line.length() > 256) break; // Prevent buffer overflow
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
// Hand Skeleton Data Structures
//=============================================================================

// OpenVR uses a specific 31-bone hand skeleton hierarchy
// Understanding this structure is crucial for proper hand tracking
enum HandBones {
    // Root and wrist
    eBone_Root = 0,
    eBone_Wrist = 1,
    
    // Thumb chain (4 bones)
    eBone_Thumb0 = 2,  // Metacarpal
    eBone_Thumb1 = 3,  // Proximal phalanx
    eBone_Thumb2 = 4,  // Distal phalanx
    eBone_Thumb3 = 5,  // Tip
    
    // Index finger chain (5 bones)
    eBone_IndexFinger0 = 6,   // Metacarpal
    eBone_IndexFinger1 = 7,   // Proximal phalanx
    eBone_IndexFinger2 = 8,   // Middle phalanx
    eBone_IndexFinger3 = 9,   // Distal phalanx
    eBone_IndexFinger4 = 10,  // Tip
    
    // Middle finger chain (5 bones)
    eBone_MiddleFinger0 = 11,
    eBone_MiddleFinger1 = 12,
    eBone_MiddleFinger2 = 13,
    eBone_MiddleFinger3 = 14,
    eBone_MiddleFinger4 = 15,
    
    // Ring finger chain (5 bones)
    eBone_RingFinger0 = 16,
    eBone_RingFinger1 = 17,
    eBone_RingFinger2 = 18,
    eBone_RingFinger3 = 19,
    eBone_RingFinger4 = 20,
    
    // Pinky chain (5 bones)
    eBone_PinkyFinger0 = 21,
    eBone_PinkyFinger1 = 22,
    eBone_PinkyFinger2 = 23,
    eBone_PinkyFinger3 = 24,
    eBone_PinkyFinger4 = 25,
    
    // Auxiliary bones for advanced tracking
    eBone_Aux_Thumb = 26,
    eBone_Aux_IndexFinger = 27,
    eBone_Aux_MiddleFinger = 28,
    eBone_Aux_RingFinger = 29,
    eBone_Aux_PinkyFinger = 30,
    
    eBone_Count = 31
};

// Sensor data from our haptic glove
struct GloveData {
    float qw, qx, qy, qz;      // IMU quaternion (world orientation)
    float ax, ay, az;          // Accelerometer data
    float fingers[5];          // Finger curl values (0.0-1.0, normalized from potentiometers)
    unsigned long timestamp;   // For data validation and interpolation
    bool dataValid;           // Data integrity flag
    
    GloveData() : qw(1), qx(0), qy(0), qz(0), ax(0), ay(0), az(0), 
                  timestamp(0), dataValid(false) {
        for(int i = 0; i < 5; i++) fingers[i] = 0.0f;
    }
};

//=============================================================================
// Skeletal Hand Controller - The Core Implementation
//=============================================================================
class SkeletalHandController : public ITrackedDeviceServerDriver {
private:
    // OpenVR device tracking
    TrackedDeviceIndex_t m_deviceId;
    PropertyContainerHandle_t m_propertyContainer;
    
    // Hardware communication
    SerialPort* m_serial;
    
    // Threading and lifecycle
    std::thread m_updateThread;
    bool m_isActive;
    
    // Hand tracking data
    GloveData m_gloveData;
    DriverPose_t m_pose;
    
    // Skeletal input components - this is the heart of hand tracking
    VRInputComponentHandle_t m_skeletalComponent;      // Main skeletal input
    VRInputComponentHandle_t m_hapticComponent;        // Haptic feedback output
    
    // Bone transforms - 31 bones as required by OpenVR
    std::array<VRBoneTransform_t, eBone_Count> m_boneTransforms;
    
    // Controller role (left or right hand)
    ETrackedControllerRole m_controllerRole;
    
    // Performance monitoring
    std::chrono::high_resolution_clock::time_point m_lastUpdate;
    int m_updateCount;
    
public:
    SkeletalHandController(ETrackedControllerRole role) 
        : m_deviceId(k_unTrackedDeviceIndexInvalid)
        , m_propertyContainer(k_ulInvalidPropertyContainer)
        , m_serial(nullptr)
        , m_isActive(false)
        , m_controllerRole(role)
        , m_skeletalComponent(k_ulInvalidInputComponentHandle)
        , m_hapticComponent(k_ulInvalidInputComponentHandle)
        , m_updateCount(0) {
        
        // Initialize pose structure for 6DOF tracking
        memset(&m_pose, 0, sizeof(m_pose));
        m_pose.qRotation.w = 1.0;  // Identity quaternion
        m_pose.poseIsValid = true;
        m_pose.deviceIsConnected = true;
        m_pose.result = TrackingResult_Running_OK;
        m_pose.willDriftInYaw = false;
        m_pose.shouldApplyHeadModel = false;
        
        // Initialize bone transforms to reference pose
        InitializeReferencePose();
    }
    
    ~SkeletalHandController() {
        if(m_serial) delete m_serial;
    }
    
    //-------------------------------------------------------------------------
    // ITrackedDeviceServerDriver Implementation
    //-------------------------------------------------------------------------
    
    EVRInitError Activate(TrackedDeviceIndex_t deviceId) override {
        m_deviceId = deviceId;
        m_propertyContainer = VRProperties()->TrackedDeviceToPropertyContainer(deviceId);
        
        std::string handSide = (m_controllerRole == TrackedControllerRole_LeftHand) ? "left" : "right";
        VRDriverLog()->Log(("SkeletalHandController: Activating " + handSide + " hand").c_str());
        
        // Set up device properties - this tells OpenVR what kind of device we are
        SetupDeviceProperties();
        
        // Initialize serial communication
        if(!InitializeSerial()) {
            VRDriverLog()->Log("SkeletalHandController: Warning - no serial connection, using simulation mode");
        }
        
        // Create skeletal input components - this is where the magic happens
        if(!CreateSkeletalComponents()) {
            VRDriverLog()->Log("SkeletalHandController: Failed to create skeletal components");
            return VRInitError_Driver_Failed;
        }
        
        // Set initial pose - place hand in visible location for testing
        SetInitialPose();
        
        // Start the update thread for 90Hz tracking
        m_isActive = true;
        m_updateThread = std::thread(&SkeletalHandController::UpdateLoop, this);
        
        VRDriverLog()->Log("SkeletalHandController: Activation complete");
        return VRInitError_None;
    }
    
    void Deactivate() override {
        VRDriverLog()->Log("SkeletalHandController: Deactivating");
        m_isActive = false;
        if(m_updateThread.joinable()) {
            m_updateThread.join();
        }
        m_deviceId = k_unTrackedDeviceIndexInvalid;
    }
    
    void EnterStandby() override {}
    
    void* GetComponent(const char* componentNameAndVersion) override {
        return nullptr;
    }
    
    void DebugRequest(const char* request, char* responseBuffer, uint32_t responseBufferSize) override {
        if(responseBufferSize >= 1) {
            responseBuffer[0] = 0;
        }
    }
    
    DriverPose_t GetPose() override {
        return m_pose;
    }
    
private:
    //-------------------------------------------------------------------------
    // Device Configuration
    //-------------------------------------------------------------------------
    
    void SetupDeviceProperties() {
        // Critical: Device class must be Controller for skeletal input to work
        VRProperties()->SetInt32Property(m_propertyContainer, 
            Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
        
        // Set controller role - this determines which hand we're tracking
        VRProperties()->SetInt32Property(m_propertyContainer,
            Prop_ControllerRoleHint_Int32, m_controllerRole);
        
        // Device identification
        std::string handSide = (m_controllerRole == TrackedControllerRole_LeftHand) ? "left" : "right";
        VRProperties()->SetStringProperty(m_propertyContainer,
            Prop_ModelNumber_String, ("SkeletalGlove_" + handSide).c_str());
        VRProperties()->SetStringProperty(m_propertyContainer,
            Prop_SerialNumber_String, ("HapticGlove_" + handSide + "_001").c_str());
        VRProperties()->SetStringProperty(m_propertyContainer,
            Prop_ManufacturerName_String, "CustomVR");
        
        // Render model - tells SteamVR how to visualize our device
        VRProperties()->SetStringProperty(m_propertyContainer,
            Prop_RenderModelName_String, ("{skeletal_hand_tracker}hand_" + handSide).c_str());
        
        // Controller type for input binding system
        VRProperties()->SetStringProperty(m_propertyContainer,
            Prop_ControllerType_String, "skeletal_hand_tracker");
        
        // Input profile path - critical for proper input handling
        VRProperties()->SetStringProperty(m_propertyContainer,
            Prop_InputProfilePath_String, "{haptic_glove_driver}/input/hand_tracking_profile.json");
        
        // Performance and capability flags
        VRProperties()->SetBoolProperty(m_propertyContainer,
            Prop_WillDriftInYaw_Bool, false);
        VRProperties()->SetBoolProperty(m_propertyContainer,
            Prop_DeviceCanPowerOff_Bool, true);
        VRProperties()->SetBoolProperty(m_propertyContainer,
            Prop_DeviceProvidesBatteryStatus_Bool, false);
        
        // Enable skeletal tracking capability
        VRProperties()->SetInt32Property(m_propertyContainer,
            Prop_ControllerHandSelectionPriority_Int32, 0);
    }
    
    bool InitializeSerial() {
        // Get COM port from settings
        char portBuffer[32] = "COM3";  // Default fallback
        vr::EVRSettingsError settingsError = vr::VRSettingsError_None;
        VRSettings()->GetString("driver_haptic_glove", "serial_port", 
                               portBuffer, sizeof(portBuffer), &settingsError);
        
        std::string portName = std::string(portBuffer);
        VRDriverLog()->Log(("SkeletalHandController: Attempting to open " + portName).c_str());
        
        m_serial = new SerialPort(portName);
        if(!m_serial->Open()) {
            delete m_serial;
            m_serial = nullptr;
            return false;
        }
        
        // Test communication
        m_serial->WriteLine("STATUS");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::string response = m_serial->ReadLine();
        
        VRDriverLog()->Log(("SkeletalHandController: Serial initialized, response: " + response).c_str());
        return true;
    }
    
    bool CreateSkeletalComponents() {
        // Create the main skeletal input component
        // This is the core of hand tracking - it tells OpenVR we can provide bone data
        std::string skeletonPath = (m_controllerRole == TrackedControllerRole_LeftHand) ? 
            "/input/skeleton/left" : "/input/skeleton/right";
        std::string skeletonReference = (m_controllerRole == TrackedControllerRole_LeftHand) ?
            "/skeleton/hand/left" : "/skeleton/hand/right";
        
        // Create skeletal component with reference pose
        EVRInputError skeletalError = VRDriverInput()->CreateSkeletonComponent(
            m_propertyContainer,                    // Property container
            skeletonPath.c_str(),                  // Component path
            skeletonReference.c_str(),             // Skeleton reference
            "/pose/raw",                           // Base pose path
            VRSkeletalTracking_Full,              // Full finger tracking
            nullptr,                              // Grip limit transforms (use defaults)
            0,                                    // Grip limit count
            &m_skeletalComponent                  // Output handle
        );
        
        if(skeletalError != VRInputError_None) {
            VRDriverLog()->Log(("SkeletalHandController: Failed to create skeletal component: " + std::to_string(skeletalError)).c_str());
            return false;
        }
        
        // Create haptic output component for force feedback
        EVRInputError hapticError = VRDriverInput()->CreateHapticComponent(
            m_propertyContainer,
            "/output/haptic",
            &m_hapticComponent
        );
        
        if(hapticError != VRInputError_None) {
            VRDriverLog()->Log(("SkeletalHandController: Failed to create haptic component: " + std::to_string(hapticError)).c_str());
            // Non-critical - continue without haptics
        }
        
        VRDriverLog()->Log("SkeletalHandController: Skeletal components created successfully");
        return true;
    }
    
    //-------------------------------------------------------------------------
    // Hand Skeleton Mathematics
    //-------------------------------------------------------------------------
    
    void InitializeReferencePose() {
        // Set up reference pose - this represents a relaxed, open hand
        // All bone transforms are in parent space, not world space!
        
        // Root bone (always identity)
        m_boneTransforms[eBone_Root] = CreateBoneTransform(0, 0, 0, 1, 0, 0, 0);
        
        // Wrist relative to root
        m_boneTransforms[eBone_Wrist] = CreateBoneTransform(0, 0, 0, 1, 0, 0, 0);
        
        // Initialize all finger chains with natural poses
        InitializeFingerChain(eBone_Thumb0, 4);      // Thumb has 4 bones
        InitializeFingerChain(eBone_IndexFinger0, 5); // Other fingers have 5 bones
        InitializeFingerChain(eBone_MiddleFinger0, 5);
        InitializeFingerChain(eBone_RingFinger0, 5);
        InitializeFingerChain(eBone_PinkyFinger0, 5);
        
        // Auxiliary bones (used for advanced hand poses)
        for(int i = eBone_Aux_Thumb; i <= eBone_Aux_PinkyFinger; i++) {
            m_boneTransforms[i] = CreateBoneTransform(0, 0, 0, 1, 0, 0, 0);
        }
    }
    
    void InitializeFingerChain(int startBone, int boneCount) {
        // Each finger has a natural rest pose with slight curvature
        // Bones are positioned relative to their parent
        
        for(int i = 0; i < boneCount; i++) {
            float x = 0.0f, y = 0.0f, z = 0.0f;
            float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
            
            if(i == 0) {
                // Metacarpal bones - position varies by finger
                int fingerIndex = (startBone - eBone_Thumb0) / 5;
                if(startBone == eBone_Thumb0) fingerIndex = 0; // Special case for thumb
                
                // Position metacarpals across the hand
                x = (fingerIndex - 2.0f) * 0.02f;  // Spread across hand width
                z = (fingerIndex == 0) ? 0.01f : 0.03f; // Thumb is closer to palm
            } else {
                // Phalanx bones - positioned along finger length
                z = 0.025f + (i - 1) * 0.02f;  // Progressive length down finger
            }
            
            m_boneTransforms[startBone + i] = CreateBoneTransform(x, y, z, qw, qx, qy, qz);
        }
    }
    
    VRBoneTransform_t CreateBoneTransform(float x, float y, float z, 
                                         float qw, float qx, float qy, float qz) {
        VRBoneTransform_t transform;
        transform.position.v[0] = x;
        transform.position.v[1] = y;
        transform.position.v[2] = z;
        transform.position.v[3] = 1.0f;
        
        transform.orientation.w = qw;
        transform.orientation.x = qx;
        transform.orientation.y = qy;
        transform.orientation.z = qz;
        
        return transform;
    }
    
    void SetInitialPose() {
        // Position hand in a visible location for testing
        if(m_controllerRole == TrackedControllerRole_LeftHand) {
            m_pose.vecPosition[0] = -0.3f;  // 30cm to the left
        } else {
            m_pose.vecPosition[0] = 0.3f;   // 30cm to the right
        }
        m_pose.vecPosition[1] = 1.0f;       // 1m up from floor
        m_pose.vecPosition[2] = -0.5f;      // 50cm forward
        
        // Send initial pose to SteamVR
        VRServerDriverHost()->TrackedDevicePoseUpdated(m_deviceId, m_pose, sizeof(DriverPose_t));
    }
    
    //-------------------------------------------------------------------------
    // Real-time Update Loop
    //-------------------------------------------------------------------------
    
    void UpdateLoop() {
        VRDriverLog()->Log("SkeletalHandController: Update thread started");
        
        m_lastUpdate = std::chrono::high_resolution_clock::now();
        
        // Target 90Hz for smooth VR tracking
        const auto targetInterval = std::chrono::microseconds(11111); // ~90Hz
        
        while(m_isActive) {
            auto frameStart = std::chrono::high_resolution_clock::now();
            
            // Read sensor data from Arduino
            UpdateSensorData();
            
            // Update hand pose from IMU
            UpdateHandPose();
            
            // Calculate finger bone transforms
            UpdateFingerBones();
            
            // Send skeletal data to OpenVR - this is the critical step!
            UpdateSkeletalInput();
            
            // Performance monitoring
            m_updateCount++;
            if(m_updateCount % 450 == 0) { // Log every 5 seconds at 90Hz
                VRDriverLog()->Log(("SkeletalHandController:" + std::to_string(m_updateCount) + " updates completed").c_str());
            }
            
            // Maintain precise timing for 90Hz updates
            auto frameEnd = std::chrono::high_resolution_clock::now();
            auto frameTime = frameEnd - frameStart;
            
            if(frameTime < targetInterval) {
                std::this_thread::sleep_for(targetInterval - frameTime);
            }
        }
        
        VRDriverLog()->Log("SkeletalHandController: Update thread ended");
    }
    
    void UpdateSensorData() {
        if(!m_serial || !m_serial->IsOpen()) {
            // Simulation mode for testing without hardware
            UpdateSimulatedData();
            return;
        }
        
        // Read data from Arduino
        std::string line = m_serial->ReadLine();
        if(!line.empty() && line.substr(0, 5) == "DATA:") {
            ParseSerialData(line.substr(5));
        }
    }
    
    void UpdateSimulatedData() {
        // Generate smooth hand motion for testing
        static float time = 0.0f;
        time += 0.011f; // 90Hz increment
        
        // Simulate gentle hand rotation
        float angle = sin(time * 0.5f) * 0.3f;
        m_gloveData.qw = cos(angle * 0.5f);
        m_gloveData.qx = 0.0f;
        m_gloveData.qy = sin(angle * 0.5f);
        m_gloveData.qz = 0.0f;
        
        // Simulate finger curl animation
        for(int i = 0; i < 5; i++) {
            m_gloveData.fingers[i] = (sin(time + i * 0.5f) + 1.0f) * 0.3f; // 0-0.6 range
        }
        
        m_gloveData.dataValid = true;
        m_gloveData.timestamp = GetTickCount();
    }
    
    bool ParseSerialData(const std::string& data) {
        // Parse: qw,qx,qy,qz,ax,ay,az,f0,f1,f2,f3,f4
        std::vector<std::string> tokens;
        std::stringstream ss(data);
        std::string token;
        
        while(std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        
        if(tokens.size() < 12) return false;
        
        try {
            m_gloveData.qw = std::stof(tokens[0]);
            m_gloveData.qx = std::stof(tokens[1]);
            m_gloveData.qy = std::stof(tokens[2]);
            m_gloveData.qz = std::stof(tokens[3]);
            m_gloveData.ax = std::stof(tokens[4]);
            m_gloveData.ay = std::stof(tokens[5]);
            m_gloveData.az = std::stof(tokens[6]);
            
            // Convert potentiometer values to 0-1 range
            for(int i = 0; i < 5; i++) {
                int rawValue = std::stoi(tokens[7 + i]);
                m_gloveData.fingers[i] = constrain(rawValue / 1023.0f, 0.0f, 1.0f);
            }
            
            m_gloveData.dataValid = true;
            m_gloveData.timestamp = GetTickCount();
            return true;
            
        } catch(const std::exception& e) {
            VRDriverLog()->Log(("SkeletalHandController: Parse error: " + std::string(e.what())).c_str());
            return false;
        }
    }
    
    float constrain(float value, float min, float max) {
        return (value < min) ? min : (value > max) ? max : value;
    }
    
    void UpdateHandPose() {
        if(!m_gloveData.dataValid) return;
        
        // Transform BNO055 quaternion to OpenVR coordinate system
        // This is critical - coordinate systems must align perfectly!
        
        // BNO055 uses different coordinate convention than OpenVR
        // Adjust these transforms based on your IMU mounting orientation
        float qw = m_gloveData.qw;
        float qx = m_gloveData.qx;
        float qy = m_gloveData.qy;
        float qz = m_gloveData.qz;
        
        // Apply coordinate transformation for right-hand glove
        // (left-hand glove may need different transformation)
        if(m_controllerRole == TrackedControllerRole_RightHand) {
            m_pose.qRotation.w = qw;
            m_pose.qRotation.x = -qy;  // IMU Y becomes OpenVR -X
            m_pose.qRotation.y = qz;   // IMU Z becomes OpenVR Y  
            m_pose.qRotation.z = -qx;  // IMU X becomes OpenVR -Z
        } else {
            // Mirror for left hand
            m_pose.qRotation.w = qw;
            m_pose.qRotation.x = qy;   
            m_pose.qRotation.y = qz;   
            m_pose.qRotation.z = qx;   
        }
        
        // Normalize quaternion to prevent drift
        float magnitude = sqrt(m_pose.qRotation.w * m_pose.qRotation.w + 
                              m_pose.qRotation.x * m_pose.qRotation.x +
                              m_pose.qRotation.y * m_pose.qRotation.y + 
                              m_pose.qRotation.z * m_pose.qRotation.z);
        
        if(magnitude > 0.001f) {
            m_pose.qRotation.w /= magnitude;
            m_pose.qRotation.x /= magnitude;
            m_pose.qRotation.y /= magnitude;
            m_pose.qRotation.z /= magnitude;
        }
        
        // Update tracking status
        m_pose.poseIsValid = true;
        m_pose.deviceIsConnected = true;
        m_pose.result = TrackingResult_Running_OK;
        
        // Send pose update to SteamVR
        VRServerDriverHost()->TrackedDevicePoseUpdated(m_deviceId, m_pose, sizeof(DriverPose_t));
    }
    
    void UpdateFingerBones() {
        if(!m_gloveData.dataValid) return;
        
        // Calculate bone transforms for each finger based on sensor data
        UpdateThumbBones(m_gloveData.fingers[0]);
        UpdateIndexFingerBones(m_gloveData.fingers[1]);
        UpdateMiddleFingerBones(m_gloveData.fingers[2]);
        UpdateRingFingerBones(m_gloveData.fingers[3]);
        UpdatePinkyBones(m_gloveData.fingers[4]);
    }
    
    void UpdateFingerBoneChain(int startBone, int boneCount, float curl, float splay = 0.0f) {
        // Apply progressive curl along finger bones
        // Each segment bends more than the previous one for natural finger motion
        
        for(int i = 1; i < boneCount; i++) { // Skip metacarpal (bone 0)
            float segmentCurl = curl * GetFingerSegmentMultiplier(i);
            float rotationAngle = segmentCurl * 1.5708f; // 90 degrees max curl
            
            // Create rotation quaternion around X-axis (finger curl direction)
            float halfAngle = rotationAngle * 0.5f;
            float sinHalf = sin(halfAngle);
            float cosHalf = cos(halfAngle);
            
            m_boneTransforms[startBone + i].orientation.w = cosHalf; // w
            m_boneTransforms[startBone + i].orientation.x = sinHalf; // x
            m_boneTransforms[startBone + i].orientation.y = 0.0f;    // y
            m_boneTransforms[startBone + i].orientation.z = 0.0f;    // z
        }
        
        // Apply splay to metacarpal if provided
        if(splay != 0.0f && boneCount > 0) {
            float splayAngle = splay * 0.5f; // Limit splay range
            float halfAngle = splayAngle * 0.5f;
            
            m_boneTransforms[startBone].orientation.w = cos(halfAngle); // w
            m_boneTransforms[startBone].orientation.x = 0.0f;          // x
            m_boneTransforms[startBone].orientation.y = 0.0f;          // y
            m_boneTransforms[startBone].orientation.z = sin(halfAngle); // z (splay around Z)
        }
    }
    
    float GetFingerSegmentMultiplier(int segment) {
        // Progressive curl - later segments curl more than earlier ones
        switch(segment) {
            case 1: return 0.3f;  // Proximal phalanx
            case 2: return 0.6f;  // Middle phalanx
            case 3: return 1.0f;  // Distal phalanx
            case 4: return 1.0f;  // Tip (follows distal)
            default: return 0.0f;
        }
    }
    
    void UpdateThumbBones(float curl) {
        // Thumb has special kinematics - it opposes other fingers
        UpdateFingerBoneChain(eBone_Thumb0, 4, curl, 0.0f);
        
        // Add thumb opposition (movement toward palm)
        float oppositionAngle = curl * 0.8f; // 80% of curl becomes opposition
        float halfAngle = oppositionAngle * 0.5f;
        
        // Combine curl with opposition for metacarpal
        m_boneTransforms[eBone_Thumb0].orientation.w = cos(halfAngle); // w
        m_boneTransforms[eBone_Thumb0].orientation.x = sin(halfAngle) * 0.5f; // x (some curl)
        m_boneTransforms[eBone_Thumb0].orientation.y = sin(halfAngle) * 0.866f; // y (main opposition)
        m_boneTransforms[eBone_Thumb0].orientation.z = 0.0f; // z
    }
    
    void UpdateIndexFingerBones(float curl) {
        UpdateFingerBoneChain(eBone_IndexFinger0, 5, curl, 0.0f);
    }
    
    void UpdateMiddleFingerBones(float curl) {
        UpdateFingerBoneChain(eBone_MiddleFinger0, 5, curl, 0.0f);
    }
    
    void UpdateRingFingerBones(float curl) {
        UpdateFingerBoneChain(eBone_RingFinger0, 5, curl, 0.0f);
    }
    
    void UpdatePinkyBones(float curl) {
        UpdateFingerBoneChain(eBone_PinkyFinger0, 5, curl, 0.0f);
    }
    
    void UpdateSkeletalInput() {
        if(m_skeletalComponent == k_ulInvalidInputComponentHandle) return;
        
        // Update skeletal component with both motion ranges
        // This is CRITICAL - many applications require both ranges to work properly!
        
        // WithController range - hand constrained as if holding a controller
        EVRInputError error1 = VRDriverInput()->UpdateSkeletonComponent(
            m_skeletalComponent,
            VRSkeletalMotionRange_WithController,
            m_boneTransforms.data(),
            eBone_Count
        );
        
        // WithoutController range - full natural hand movement
        EVRInputError error2 = VRDriverInput()->UpdateSkeletonComponent(
            m_skeletalComponent,
            VRSkeletalMotionRange_WithoutController,
            m_boneTransforms.data(),
            eBone_Count
        );
        
        // Log errors (but don't spam the log)
        if((error1 != VRInputError_None || error2 != VRInputError_None) && 
           m_updateCount % 900 == 0) { // Every 10 seconds
            std::string errorMsg = "SkeletalHandController: Skeletal update errors: " + 
                                 std::to_string(error1) + ", " + std::to_string(error2);
            VRDriverLog()->Log(errorMsg.c_str());
        }
    }
    
    //-------------------------------------------------------------------------
    // Haptic Feedback
    //-------------------------------------------------------------------------
    
    void ProcessHapticEvent(const VREvent_HapticVibration_t& hapticData) {
        if(!m_serial || !m_serial->IsOpen()) return;
        
        // Convert haptic strength to servo positions
        int intensity = 90 + (int)(hapticData.fAmplitude * 90); // 90-180 range for servos
        
        std::string command = "HAPTIC:" + 
            std::to_string(intensity) + "," + std::to_string(intensity) + "," +
            std::to_string(intensity) + "," + std::to_string(intensity) + "," +
            std::to_string(intensity);
            
        m_serial->WriteLine(command);
    }
};

//=============================================================================
// Device Provider - Manages Device Lifecycle
//=============================================================================
class HapticGloveProvider : public IServerTrackedDeviceProvider {
private:
    SkeletalHandController* m_leftHand;
    SkeletalHandController* m_rightHand;
    
public:
    HapticGloveProvider() : m_leftHand(nullptr), m_rightHand(nullptr) {}
    
    EVRInitError Init(IVRDriverContext* driverContext) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(driverContext);
        VRDriverLog()->Log("HapticGloveProvider: Initializing skeletal hand tracking");
        
        // Check settings to see which hands to enable
        bool enableLeft = true;   // Default to enabling both hands
        bool enableRight = true;
        
        // You can add settings to control which hands are active:
        // VRSettings()->GetBool("driver_haptic_glove", "enable_left_hand", &enableLeft);
        // VRSettings()->GetBool("driver_haptic_glove", "enable_right_hand", &enableRight);
        
        if(enableLeft) {
            m_leftHand = new SkeletalHandController(TrackedControllerRole_LeftHand);
            VRServerDriverHost()->TrackedDeviceAdded("haptic_glove_left", 
                                                   TrackedDeviceClass_Controller, m_leftHand);
        }
        
        if(enableRight) {
            m_rightHand = new SkeletalHandController(TrackedControllerRole_RightHand);
            VRServerDriverHost()->TrackedDeviceAdded("haptic_glove_right", 
                                                    TrackedDeviceClass_Controller, m_rightHand);
        }
        
        VRDriverLog()->Log("HapticGloveProvider: Initialization complete");
        return VRInitError_None;
    }
    
    void Cleanup() override {
        VRDriverLog()->Log("HapticGloveProvider: Cleanup");
        if(m_leftHand) {
            delete m_leftHand;
            m_leftHand = nullptr;
        }
        if(m_rightHand) {
            delete m_rightHand;
            m_rightHand = nullptr;
        }
    }
    
    const char* const* GetInterfaceVersions() override {
        return k_InterfaceVersions;
    }
    
    void RunFrame() override {
        // Called once per frame - can be used for global updates
    }
    
    bool ShouldBlockStandbyMode() override {
        return false;
    }
    
    void EnterStandby() override {}
    void LeaveStandby() override {}
};

//=============================================================================
// Driver Factory - Entry Point for OpenVR
//=============================================================================
static HapticGloveProvider g_serverProvider;

HMD_DLL_EXPORT void* HmdDriverFactory(const char* interfaceName, int* returnCode) {
    if(0 == strcmp(IServerTrackedDeviceProvider_Version, interfaceName)) {
        return &g_serverProvider;
    }
    
    if(returnCode) {
        *returnCode = VRInitError_Init_InterfaceNotFound;
    }
    
    return nullptr;
}