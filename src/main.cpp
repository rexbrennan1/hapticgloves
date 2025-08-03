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
    float m_fingerSplay[4];      
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
        
        // Create unique serial numbers for left and right hands
        m_serialNumber = (role == vr::TrackedControllerRole_LeftHand) ? 
            "SkeletonController_Left_001" : "SkeletonController_Right_001";
        
        // This represents a relaxed hand pose that serves as our starting point
        for (int i = 0; i < 5; i++) {
            m_fingerCurl[i] = 0.1f;  // Slightly curved, not perfectly straight
        }
        for (int i = 0; i < 4; i++) {
            m_fingerSplay[i] = 0.0f; // Fingers close together
        }
        
        m_lastUpdateTime = std::chrono::high_resolution_clock::now();
    }

    // Device Lifecycle: Activation Phase
    virtual vr::EVRInitError Activate(vr::TrackedDeviceIndex_t deviceIndex) override {
        m_deviceIndex = deviceIndex;
        m_propertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(deviceIndex);
        
        // Device Properties: These tell SteamVR and applications what kind of device this is
        
        // Basic identification - these appear in SteamVR's device list
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_SerialNumber_String, m_serialNumber.c_str());
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_ModelNumber_String, "SkeletonController v1.0");
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_ManufacturerName_String, "OpenVR Tutorial");
        
        // Controller-specific properties that affect behavior
        vr::VRProperties()->SetInt32Property(m_propertyContainer, vr::Prop_ControllerRoleHint_Int32, m_role);
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_ControllerType_String, "skeleton_hand");
        
        // Input profile tells SteamVR how to present binding options to users
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_InputProfilePath_String, 
            "{skeleton}/input/skeleton_hand_profile.json");
        
        // Hardware and tracking properties
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_WillDriftInYaw_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_DeviceIsWireless_Bool, true);
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_DeviceIsCharging_Bool, false);
        vr::VRProperties()->SetFloatProperty(m_propertyContainer, vr::Prop_DeviceBatteryPercentage_Float, 1.0f);

        CreateSkeletalComponent();
        
        // Pose component: Basic position and orientation tracking
        vr::VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/system/click", &m_poseComponent);
        vr::VRDriverInput()->CreateHapticComponent(m_propertyContainer, "/output/haptic", &m_poseComponent);
        
        return vr::VRInitError_None;
    }

    void CreateSkeletalComponent() {

        vr::VRBoneTransform_t bindPose[31];
        ComputeBindPose(bindPose);
        
        vr::VRBoneTransform_t gripLimitPose[31];
        ComputeGripLimitPose(gripLimitPose);
        
        std::string skeletonPath = (m_role == vr::TrackedControllerRole_LeftHand) ? 
            "/input/skeleton/left" : "/input/skeleton/right";
        
        vr::EVRInputError inputError = vr::VRDriverInput()->CreateSkeletonComponent(
            m_propertyContainer,              // Property container for this device
            skeletonPath.c_str(),            // Input path (how applications reference this)
            "/skeleton/hand/left",            // Skeleton path (defines bone hierarchy)
            "/pose/raw",                     // Base pose path
            vr::VRSkeletalTracking_Full,     // We provide full finger tracking
            bindPose,                        // Reference pose for the skeleton
            31,                              // Number of bones (fixed for hands)
            &m_skeletalComponent             // Handle for future updates
        );
        
        if (inputError != vr::VRInputError_None) {
            vr::VRDriverLog()->Log("Failed to create skeletal component");
        }
    }

    // Real-Time Update Loop
    virtual void RunFrame() {
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto deltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - m_lastUpdateTime).count();
        m_lastUpdateTime = currentTime;
        
        if (m_deviceIndex != vr::k_unTrackedDeviceIndexInvalid) {
            UpdateDevicePose();
            
            UpdateSkeletalPose();
            
            // In a real implementation, you might also:
            // - Process input from your tracking system
            // - Apply filtering or prediction algorithms
            // - Handle tracking confidence and error states
            // - Update other input components (buttons, triggers, etc.)
        }
    }

    // Device Pose Updates: Position in Virtual Space
    void UpdateDevicePose() {

        vr::DriverPose_t pose = { 0 };
        
        // Basic pose setup - device is present and tracking
        pose.poseIsValid = true;
        pose.result = vr::TrackingResult_Running_OK;
        pose.deviceIsConnected = true;
        
        // Position: Where the controller is in 3D space (in meters)
        // For demonstration, we'll place it at a reasonable hand position
        if (m_role == vr::TrackedControllerRole_LeftHand) {
            pose.vecPosition[0] = -0.3;  // 30cm to the left
        } else {
            pose.vecPosition[0] = 0.3;   // 30cm to the right
        }
        pose.vecPosition[1] = 1.0;       // 1 meter up (chest level)
        pose.vecPosition[2] = -0.5;      // 50cm forward
        
        // Orientation: Which way the controller is pointing
        // Quaternion representing no rotation (identity)
        pose.qRotation.w = 1.0;
        pose.qRotation.x = 0.0;
        pose.qRotation.y = 0.0;
        pose.qRotation.z = 0.0;
        
        // Velocity and acceleration (for prediction algorithms)
        // Set to zero for stationary pose
        for (int i = 0; i < 3; i++) {
            pose.vecVelocity[i] = 0.0;
            pose.vecAngularVelocity[i] = 0.0;
            pose.vecAcceleration[i] = 0.0;
            pose.vecAngularAcceleration[i] = 0.0;
        }
        
        // Timing information for prediction
        pose.poseTimeOffset = 0.0;  // No prediction offset for this example
        
        // Send the pose to SteamVR
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_deviceIndex, pose, sizeof(pose));
    }

    // Skeletal Pose Updates: The Core of Hand Tracking
    void UpdateSkeletalPose() {
        // In a real driver, this data would come from your tracking system
        SimulateHandMovement();
        
        // Create arrays to hold our bone transforms
        vr::VRBoneTransform_t boneTransforms[31];
        
        // This represents how the hand moves when holding a physical controller
        ComputeBoneTransforms(boneTransforms, true);
        vr::VRDriverInput()->UpdateSkeletonComponent(
            m_skeletalComponent,
            vr::VRSkeletalMotionRange_WithController,
            boneTransforms,
            31
        );
        
        // This represents natural hand movement without physical constraints
        ComputeBoneTransforms(boneTransforms, false);
        vr::VRDriverInput()->UpdateSkeletonComponent(
            m_skeletalComponent,
            vr::VRSkeletalMotionRange_WithoutController,
            boneTransforms,
            31
        );
    }


    // Hand Movement Simulation: Demonstration Data
    void SimulateHandMovement() {
        static float time = 0.0f;
        time += 0.016f;  // Assume ~60fps update rate
        
        // Create a gentle opening and closing motion
        float wave = (sin(time * 2.0f) + 1.0f) * 0.5f;  // 0 to 1
        
        // Apply wave to finger curl with some variation per finger
        m_fingerCurl[0] = wave * 0.8f;         // Thumb curls less
        m_fingerCurl[1] = wave * 0.9f;         // Index finger
        m_fingerCurl[2] = wave * 1.0f;         // Middle finger curls most
        m_fingerCurl[3] = wave * 0.95f;        // Ring finger
        m_fingerCurl[4] = wave * 0.85f;        // Pinky curls less
        
        // Add some gentle splay variation
        float splayWave = sin(time * 1.5f) * 0.3f;
        for (int i = 0; i < 4; i++) {
            m_fingerSplay[i] = (splayWave + 1.0f) * 0.2f;  // Gentle spreading
        }
    }


    // Bone Transform Computation: The Mathematical Core
    void ComputeBoneTransforms(vr::VRBoneTransform_t* transforms, bool withController) {
        // Initialize all transforms to identity (no transformation)
        for (int i = 0; i < 31; i++) {
            transforms[i].position = { 0.0f, 0.0f, 0.0f, 1.0f };  // w=1 for points
            transforms[i].orientation = { 0.0f, 0.0f, 0.0f, 1.0f }; // Identity quaternion
        }
        
        // Root and wrist bones: These establish the base coordinate system
        // Root bone is the origin, wrist bone is slightly offset
        transforms[0] = { {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f} }; // Root
        transforms[1] = { {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f} }; // Wrist
        
        // Compute each finger: Thumb has 4 bones, others have 5
        ComputeThumbTransforms(transforms, withController);
        ComputeFingerTransforms(transforms, 2, 5, 0, withController);   // Index finger (bone 2-6)
        ComputeFingerTransforms(transforms, 7, 5, 1, withController);   // Middle finger (bone 7-11)
        ComputeFingerTransforms(transforms, 12, 5, 2, withController);  // Ring finger (bone 12-16)
        ComputeFingerTransforms(transforms, 17, 5, 3, withController);  // Pinky finger (bone 17-21)
        
    }

    // Thumb Computation: Special Case Due to Different Anatomy
    void ComputeThumbTransforms(vr::VRBoneTransform_t* transforms, bool withController) {
        // Thumb bone indices: 2, 3, 4, 5 (4 bones total)
        const float thumbLength[] = { 0.04f, 0.03f, 0.025f, 0.02f }; // Bone lengths in meters
        
        float curl = m_fingerCurl[0];
        if (withController) {
            curl *= 0.7f;  // Constrain thumb movement when holding controller
        }
        
        // Thumb moves in a different plane than other fingers
        float thumbAngleBase = -0.5f;  // Base angle relative to palm
        float thumbCurlAngle = curl * 1.2f;  // Curl amount
        
        for (int i = 0; i < 4; i++) {
            int boneIndex = 2 + i;  // Thumb bones start at index 2
            
            // Each joint adds to the total curl
            float jointAngle = thumbAngleBase + (thumbCurlAngle * (i + 1) / 4.0f);
            
            // Position: Each bone extends from the previous one
            transforms[boneIndex].position = {
                thumbLength[i] * cos(jointAngle),
                thumbLength[i] * sin(jointAngle),
                0.1f * i,  // Slight Z offset for natural thumb positioning
                1.0f
            };
            
            // Orientation: Rotate around Z axis for thumb curl
            float halfAngle = jointAngle * 0.5f;
            transforms[boneIndex].orientation = {
                0.0f, 0.0f, sin(halfAngle), cos(halfAngle)
            };
        }
    }

    // Finger Transform Computation: Standard Four-Finger Algorithm
    void ComputeFingerTransforms(vr::VRBoneTransform_t* transforms, int startBone, 
                                int numBones, int fingerIndex, bool withController) {
        // Standard finger bone lengths (approximate ratios)
        const float baseLengths[] = { 0.05f, 0.04f, 0.035f, 0.03f, 0.025f };
        
        // Finger-specific scaling factors
        const float fingerScales[] = { 1.0f, 1.1f, 1.05f, 0.9f };  // Index, middle, ring, pinky
        float scale = fingerScales[fingerIndex];
        
        float curl = m_fingerCurl[fingerIndex + 1];  // +1 because thumb is index 0
        float splay = (fingerIndex < 4) ? m_fingerSplay[fingerIndex] : 0.0f;
        
        if (withController) {
            curl *= 0.8f;  // Reduce curl when constrained by controller
            splay *= 0.5f; // Reduce splay when gripping controller
        }
        
        // Base position and orientation for this finger
        float baseX = (fingerIndex - 1.5f) * 0.025f;  // Spread fingers across palm
        float splayAngle = (fingerIndex - 1.5f) * splay * 0.3f;  // Splay between fingers
        
        for (int i = 0; i < numBones; i++) {
            int boneIndex = startBone + i;
            
            // Calculate curl angle for this joint
            // Joints curl progressively more towards fingertips
            float jointCurl = curl * (i + 1) / numBones * 1.5f;
            
            // Position calculation: Each bone extends from previous
            float length = baseLengths[i] * scale;
            transforms[boneIndex].position = {
                baseX + splayAngle * i * 0.01f,  // X: finger spread + splay
                length * cos(jointCurl),          // Y: length adjusted for curl
                -length * sin(jointCurl),         // Z: curl direction
                1.0f
            };
            
            // Orientation: Rotation around X axis for finger curl
            float halfAngle = jointCurl * 0.5f;
            transforms[boneIndex].orientation = {
                sin(halfAngle), 0.0f, 0.0f, cos(halfAngle)
            };
        }
    }

    // Reference Pose Computation: Bind Pose and Grip Limits
    void ComputeBindPose(vr::VRBoneTransform_t* bindPose) {
        // Bind pose: Natural hand position pointing forward, palm inward
        for (int i = 0; i < 31; i++) {
            bindPose[i].position = { 0.0f, 0.0f, 0.0f, 1.0f };
            bindPose[i].orientation = { 0.0f, 0.0f, 0.0f, 1.0f };
        }   
    }
    
    void ComputeGripLimitPose(vr::VRBoneTransform_t* gripPose) {
        // Grip limit pose: Maximum finger closure when holding a controller
        for (int i = 0; i < 31; i++) {
            gripPose[i].position = { 0.0f, 0.0f, 0.0f, 1.0f };
            gripPose[i].orientation = { 0.0f, 0.0f, 0.0f, 1.0f };
        }
    }

    // Required interface method - return current device pose
    virtual vr::DriverPose_t GetPose() override {
        vr::DriverPose_t pose = { 0 };
        
        // Basic pose setup - device is present and tracking
        pose.poseIsValid = true;
        pose.result = vr::TrackingResult_Running_OK;
        pose.deviceIsConnected = true;
        
        // Position: Where the controller is in 3D space (in meters)
        if (m_role == vr::TrackedControllerRole_LeftHand) {
            pose.vecPosition[0] = -0.3;  // 30cm to the left
        } else {
            pose.vecPosition[0] = 0.3;   // 30cm to the right
        }
        pose.vecPosition[1] = 1.0;       // 1 meter up (chest level)
        pose.vecPosition[2] = -0.5;      // 50cm forward
        
        // Orientation: Which way the controller is pointing
        pose.qRotation.w = 1.0;
        pose.qRotation.x = 0.0;
        pose.qRotation.y = 0.0;
        pose.qRotation.z = 0.0;
        
        // Velocity and acceleration (for prediction algorithms)
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
    std::unique_ptr<SkeletonDevice> m_leftHand;
    std::unique_ptr<SkeletonDevice> m_rightHand;

public:
    // Driver Initialization: Setting Up the VR Environment
    virtual vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override {
        // Initialize the driver context - this MUST be the first call
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
        
        vr::VRDriverLog()->Log("Skeleton Controller Driver: Initializing");
        
        // Create our hand tracking devices
        m_leftHand = std::make_unique<SkeletonDevice>(vr::TrackedControllerRole_LeftHand);
        m_rightHand = std::make_unique<SkeletonDevice>(vr::TrackedControllerRole_RightHand);
        
        // This tells SteamVR that these devices exist and can be activated
        vr::VRServerDriverHost()->TrackedDeviceAdded(
            m_leftHand->GetSerialNumber().c_str(),
            vr::TrackedDeviceClass_Controller,
            m_leftHand.get()
        );
        
        vr::VRServerDriverHost()->TrackedDeviceAdded(
            m_rightHand->GetSerialNumber().c_str(),
            vr::TrackedDeviceClass_Controller,
            m_rightHand.get()
        );
        
        vr::VRDriverLog()->Log("Skeleton Controller Driver: Initialization complete");
        return vr::VRInitError_None;
    }

    // Frame Updates: Maintaining Real-Time Performance
    virtual void RunFrame() override {
        if (m_leftHand) {
            m_leftHand->RunFrame();
        }
        if (m_rightHand) {
            m_rightHand->RunFrame();
        }
    }

    // Driver Shutdown: Clean Resource Management
    virtual void Cleanup() override {
        vr::VRDriverLog()->Log("Skeleton Controller Driver: Shutting down");
        
        // Clean up our devices
        m_leftHand.reset();
        m_rightHand.reset();
        
        // Clear the driver context
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
        
        if (pReturnCode) {
            *pReturnCode = vr::VRInitError_None;
        }
        
        return &provider;
    }
    // SteamVR asked for an interface we don't provide
    if (pReturnCode) {
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    }
    return nullptr;
}