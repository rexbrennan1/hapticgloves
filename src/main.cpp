#include "../openvr/headers/openvr_driver.h"
#include <string>
#include <memory>
#include <vector>
#include <chrono>
#include <cmath>

#ifdef _WIN32
#define DRIVER_EXPORT extern "C" __declspec(dllexport)
#else
#define DRIVER_EXPORT extern "C" __attribute__((visibility("default")))
#endif

//Skeletondevice is individual controllers
class SkeletonDevice : public vr::ITrackedDeviceServerDriver {
private:
    // Core device identification and state
    vr::TrackedDeviceIndex_t m_deviceIndex;
    vr::PropertyContainerHandle_t m_propertyContainer;
    std::string m_serialNumber;
    vr::ETrackedControllerRole m_role;
    
    // Skeletal input system components
    vr::VRInputComponentHandle_t m_skeletalComponent;
    vr::VRInputComponentHandle_t m_poseComponent;
    
    float m_fingerCurl[5];     
    bool m_isTracking;
    
    // Performance tracking for maintaining 90Hz updates
    std::chrono::high_resolution_clock::time_point m_lastUpdateTime;

public:
    SkeletonDevice(vr::ETrackedControllerRole role) : 
        m_deviceIndex(vr::k_unTrackedDeviceIndexInvalid),
        m_propertyContainer(vr::k_ulInvalidPropertyContainer),
        m_role(role),
        m_skeletalComponent(vr::k_ulInvalidInputComponentHandle),
        m_poseComponent(vr::k_ulInvalidInputComponentHandle),
        m_isTracking(true) {
        
        m_serialNumber = "HapticGloveRight";
        
        for (int i = 0; i < 5; i++) { m_fingerCurl[i] = 0.1f; } // Relaxed start pose, Slightly curved
        
        m_lastUpdateTime = std::chrono::high_resolution_clock::now();
    }

    // Device Lifecycle: Activation Phase
    virtual vr::EVRInitError Activate(vr::TrackedDeviceIndex_t deviceIndex) override {
        m_deviceIndex = deviceIndex;
        m_propertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(deviceIndex);
         
        // Basic identification
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_SerialNumber_String, m_serialNumber.c_str());
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_ModelNumber_String, "Haptic Gloves v1.0");
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_ManufacturerName_String, "Rex Brennan");
        vr::VRProperties()->SetInt32Property(m_propertyContainer, vr::Prop_ControllerRoleHint_Int32, m_role);
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_ControllerType_String, "hapticglove");
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_InputProfilePath_String, "resources/input/input_profile.json");
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_WillDriftInYaw_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_DeviceIsWireless_Bool, true);
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_DeviceIsCharging_Bool, false);
        vr::VRProperties()->SetFloatProperty(m_propertyContainer, vr::Prop_DeviceBatteryPercentage_Float, 1.0f);
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_DeviceProvidesBatteryStatus_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_DeviceCanPowerOff_Bool, false);
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_TrackingSystemName_String, "hapticgloves");

        CreateSkeletalComponent();
        
        vr::VRDriverInput()->CreateHapticComponent(m_propertyContainer, "/output/haptic", &m_poseComponent);
        
        return vr::VRInitError_None;
    }

    void CreateSkeletalComponent() {

        vr::VRBoneTransform_t bindPose[15];
        ComputeBindPose(bindPose);
        
        vr::VRBoneTransform_t gripLimitPose[15];
        ComputeGripLimitPose(gripLimitPose);
        
        std::string skeletonPath = "/input/skeleton/right";
        
        vr::EVRInputError inputError = vr::VRDriverInput()->CreateSkeletonComponent(
            m_propertyContainer,             // Property container for this device
            skeletonPath.c_str(),            // Input path (how applications reference this)
            "/skeleton/hand/right",          // Skeleton path 
            "/pose/raw",                     // Base pose path
            vr::VRSkeletalTracking_Partial,  // We provide full finger tracking
            bindPose,                        // Reference pose for the skeleton
            15,                              // Number of bones (fixed for hands)
            &m_skeletalComponent             // Handle for future updates
        );
        
        if (inputError != vr::VRInputError_None) { vr::VRDriverLog()->Log("Failed to create skeletal component"); }
    }

    // Real-Time Update Loop
    virtual void RunFrame() {
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto deltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - m_lastUpdateTime).count();
        m_lastUpdateTime = currentTime;
        
        if (m_deviceIndex != vr::k_unTrackedDeviceIndexInvalid) {
            vr::DriverPose_t pose = GetPose();
            vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_deviceIndex, pose, sizeof(pose));
            
            UpdateSkeletalPose();
            
            // In a real implementation, you might also:
            // - Process input from your tracking system
            // - Apply filtering or prediction algorithms
        }
    }

    // Skeletal Pose Updates: The Core of Hand Tracking
    void UpdateSkeletalPose() {
        SimulateHandMovement();
        
        vr::VRBoneTransform_t boneTransforms[15];
        ComputeBoneTransforms(boneTransforms);
        
        vr::VRDriverInput()->UpdateSkeletonComponent(
            m_skeletalComponent,
            vr::VRSkeletalMotionRange_WithoutController,
            boneTransforms,
            15
        );
    }

    void SimulateHandMovement() {
        static float time = 0.0f;
        time += 0.016f;  // ~60fps
        
        // Simple wave motion for all fingers
        float wave = (sin(time * 2.0f) + 1.0f) * 0.5f;  // 0 to 1
        
        // Apply to all fingers with slight variation
        m_fingerCurl[0] = wave * 0.7f;  // Thumb
        m_fingerCurl[1] = wave * 0.9f;  // Index
        m_fingerCurl[2] = wave * 1.0f;  // Middle (most curl)
        m_fingerCurl[3] = wave * 0.9f;  // Ring
        m_fingerCurl[4] = wave * 0.8f;  // Pinky
    }


    // Bone Transform Computation: The Mathematical Core
    void ComputeBoneTransforms(vr::VRBoneTransform_t* transforms) {
        // Initialize all transforms to identity
        for (int i = 0; i < 15; i++) {
            transforms[i].position = { 0.0f, 0.0f, 0.0f, 1.0f };
            transforms[i].orientation = { 0.0f, 0.0f, 0.0f, 1.0f };
        }
        
        // Root and wrist (always at origin)
        transforms[0] = { {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f} }; // Root
        transforms[1] = { {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f} }; // Wrist
        
        // Simplified finger calculations
        ComputeSimpleThumb(transforms);        // Bones 2-4
        ComputeSimpleFinger(transforms, 5, 0); // Index: Bones 5-7
        ComputeSimpleFinger(transforms, 8, 1); // Middle: Bones 8-10  
        ComputeSimpleFinger(transforms, 11, 2); // Ring: Bones 11-13
        ComputeSimplePinky(transforms);        // Bone 14
    }

    void ComputeSimpleThumb(vr::VRBoneTransform_t* transforms) {
        const float thumbLengths[] = { 0.04f, 0.03f, 0.025f };
        float curl = m_fingerCurl[0]; // Thumb curl (0-1)
        
        for (int i = 0; i < 3; i++) {
            int boneIndex = 2 + i;
            float jointCurl = curl * (i + 1) * 0.4f; // Progressive curl
            
            transforms[boneIndex].position = {
                thumbLengths[i] * cos(jointCurl - 0.5f), // Thumb angle offset
                thumbLengths[i] * sin(jointCurl - 0.5f),
                0.02f * i, // Slight Z offset
                1.0f
            };
            
            // Simple rotation around Z axis for thumb
            float halfAngle = jointCurl * 0.5f;
            transforms[boneIndex].orientation = { 0.0f, 0.0f, sin(halfAngle), cos(halfAngle) };
        }
    }

    void ComputeSimpleFinger(vr::VRBoneTransform_t* transforms, int startBone, int fingerIndex) {
        const float boneLengths[] = { 0.05f, 0.04f, 0.035f }; // 3 bone segments
        const float fingerScales[] = { 1.0f, 1.1f, 1.05f, 0.9f }; // Index, middle, ring, pinky
        const float fingerPositions[] = { -0.03f, 0.0f, 0.03f, 0.06f }; // X positions across palm
        
        float curl = m_fingerCurl[fingerIndex + 1]; // +1 because thumb is index 0
        float scale = fingerScales[fingerIndex];
        float baseX = fingerPositions[fingerIndex];
        
        for (int i = 0; i < 3; i++) {
            int boneIndex = startBone + i;
            float jointCurl = curl * (i + 1) * 0.5f; // Progressive curl
            float length = boneLengths[i] * scale;
            
            transforms[boneIndex].position = {
                baseX,                        // Fixed X position for each finger
                length * cos(jointCurl),      // Y: length adjusted for curl
                -length * sin(jointCurl),     // Z: curl direction
                1.0f
            };
            
            // Simple rotation around X axis for finger curl
            float halfAngle = jointCurl * 0.5f;
            transforms[boneIndex].orientation = { sin(halfAngle), 0.0f, 0.0f, cos(halfAngle) };
        }
    }

    void ComputeSimplePinky(vr::VRBoneTransform_t* transforms) {
        float curl = m_fingerCurl[4]; // Pinky curl
        
        transforms[14].position = {
            0.08f,                    // Pinky X position
            0.04f * cos(curl * 0.5f), // Y with curl
            -0.04f * sin(curl * 0.5f), // Z with curl
            1.0f
        };
        
        // Simple curl rotation
        float halfAngle = curl * 0.25f;
        transforms[14].orientation = { sin(halfAngle), 0.0f, 0.0f, cos(halfAngle) };
    }

    // Reference Pose Computation: Bind Pose and Grip Limits
    void ComputeBindPose(vr::VRBoneTransform_t* bindPose) {
        for (int i = 0; i < 15; i++) {  
            bindPose[i].position = { 0.0f, 0.0f, 0.0f, 1.0f };
            bindPose[i].orientation = { 0.0f, 0.0f, 0.0f, 1.0f };
        }
    }
    
    void ComputeGripLimitPose(vr::VRBoneTransform_t* gripPose) {
        // Grip limit pose: Maximum finger closure when holding a controller
        for (int i = 0; i < 15; i++) {
            gripPose[i].position = { 0.0f, 0.0f, 0.0f, 1.0f };
            gripPose[i].orientation = { 0.0f, 0.0f, 0.0f, 1.0f };
        }
    }

    // Required interface method - return current device pose
    virtual vr::DriverPose_t GetPose() override {
        vr::DriverPose_t pose = { 0 };
        
        pose.poseIsValid = true;
        pose.result = vr::TrackingResult_Running_OK;
        pose.deviceIsConnected = true;
        
        pose.vecPosition[0] = 0.3;       // 30cm to the right
        pose.vecPosition[1] = 1.0;       // 1 meter up (chest level)
        pose.vecPosition[2] = -0.5;      // 50cm forward
        
        pose.qRotation.w = 1.0;
        pose.qRotation.x = 0.0;
        pose.qRotation.y = 0.0;
        pose.qRotation.z = 0.0;
        
        for (int i = 0; i < 3; i++) {
            pose.vecVelocity[i] = 0.0;
            pose.vecAngularVelocity[i] = 0.0;
            pose.vecAcceleration[i] = 0.0;
           pose.vecAngularAcceleration[i] = 0.0;
        }
    
        pose.poseTimeOffset = 0.0;
    
        return pose;
    }

    // Device Lifecycle: Deactivation and Cleanup
    virtual void Deactivate() override {
        m_deviceIndex = vr::k_unTrackedDeviceIndexInvalid;
        m_propertyContainer = vr::k_ulInvalidPropertyContainer;
        m_skeletalComponent = vr::k_ulInvalidInputComponentHandle;
        m_poseComponent = vr::k_ulInvalidInputComponentHandle;
    }

    // Interface requirements
    virtual void EnterStandby() override {}
    virtual void* GetComponent(const char* pchComponentNameAndVersion) override { return nullptr; }
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override {}
    
    // Accessors for the driver provider
    std::string GetSerialNumber() const { return m_serialNumber; }
};

// Driver Provider: The Main Driver Class
class SkeletonDriverProvider : public vr::IServerTrackedDeviceProvider {
private:
    std::unique_ptr<SkeletonDevice> hapticglove;

public:
    // Driver Initialization: Setting Up the VR Environment
    virtual vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override {
        // Initialize the driver context - this MUST be the first call
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
        
        vr::VRDriverLog()->Log("Skeleton Controller Driver: Initializing");
        
        // Create our hand tracking devices
        hapticglove = std::make_unique<SkeletonDevice>(vr::TrackedControllerRole_RightHand);
        
        // This tells SteamVR that these devices exist and can be activated 
        vr::VRServerDriverHost()->TrackedDeviceAdded(
            hapticglove->GetSerialNumber().c_str(),
            vr::TrackedDeviceClass_GenericTracker,
            hapticglove.get()
        );
        
        vr::VRDriverLog()->Log("Skeleton Controller Driver: Initialization complete");
        return vr::VRInitError_None;
    }

    // Frame Updates: Maintaining Real-Time Performance
    virtual void RunFrame() override { hapticglove->RunFrame(); }

    // Driver Shutdown
    virtual void Cleanup() override {
        vr::VRDriverLog()->Log("Skeleton Controller Driver: Shutting down");
        hapticglove.reset();
        VR_CLEANUP_SERVER_DRIVER_CONTEXT();
    }

    // Interface requirements - not needed for basic operation
    virtual const char* const* GetInterfaceVersions() override { return vr::k_InterfaceVersions; }
    virtual bool ShouldBlockStandbyMode() override { return false; }
    virtual void EnterStandby() override {}
    virtual void LeaveStandby() override {}
};

// Driver Factory Function: The Entry Point
DRIVER_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode) {
    // Check if SteamVR is asking for our driver provider interface then create an instance of our driver provider
    if (0 == strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName)) { 
        static SkeletonDriverProvider provider;
        
        if (pReturnCode) { *pReturnCode = vr::VRInitError_None; }
        
        return &provider;
    }
    // SteamVR asked for an interface we don't provide
    if (pReturnCode) { *pReturnCode = vr::VRInitError_Init_InterfaceNotFound; }

    return nullptr;
}