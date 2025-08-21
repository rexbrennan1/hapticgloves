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

// Utility: normalize a quaternion in-place
static inline void NormalizeQuat(vr::HmdQuaternionf_t &q) {
    double m = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (m > 1e-8) { q.w/=m; q.x/=m; q.y/=m; q.z/=m; }
    else { q.w = 1.0; q.x = q.y = q.z = 0.0; }
}

// SkeletonDevice represents a single controller-hand device
class SkeletonDevice : public vr::ITrackedDeviceServerDriver {
private:
    // Identity & state
    vr::TrackedDeviceIndex_t m_deviceIndex;
    vr::PropertyContainerHandle_t m_propertyContainer;
    std::string m_serialNumber;
    vr::ETrackedControllerRole m_role;

    // Input components
    vr::VRInputComponentHandle_t m_skeletalComponent;
    vr::VRInputComponentHandle_t m_poseComponent; // haptic component; keeping original name

    float m_fingerCurl[5];
    bool m_isTracking;

    // Timing
    std::chrono::high_resolution_clock::time_point m_lastUpdateTime;

public:
    SkeletonDevice(vr::ETrackedControllerRole role)
        : m_deviceIndex(vr::k_unTrackedDeviceIndexInvalid)
        , m_propertyContainer(vr::k_ulInvalidPropertyContainer)
        , m_serialNumber("HapticGloveRight")
        , m_role(role)
        , m_skeletalComponent(vr::k_ulInvalidInputComponentHandle)
        , m_poseComponent(vr::k_ulInvalidInputComponentHandle)
        , m_isTracking(true) {
        for (int i = 0; i < 5; ++i) m_fingerCurl[i] = 0.1f; // relaxed start
        m_lastUpdateTime = std::chrono::high_resolution_clock::now();
    }

    // --- Activation ---
    virtual vr::EVRInitError Activate(vr::TrackedDeviceIndex_t deviceIndex) override {
        m_deviceIndex = deviceIndex;
        m_propertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(deviceIndex);

        // Identity
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_SerialNumber_String, m_serialNumber.c_str());
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_ModelNumber_String, "Haptic Gloves v1.0");
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_ManufacturerName_String, "Rex Brennan");

        // Pretend to be Index controller so apps show hands
        vr::VRProperties()->SetInt32Property(m_propertyContainer, vr::Prop_ControllerRoleHint_Int32, m_role);
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_InputProfilePath_String, "{hapticgloves}/input/input_profile.json");
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_ControllerType_String, "hapticglove");
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_RegisteredDeviceType_String, "hapticgloves");
        vr::VRProperties()->SetInt32Property(m_propertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);

        // Status
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_WillDriftInYaw_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_DeviceIsWireless_Bool, true);
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_DeviceIsCharging_Bool, false);
        vr::VRProperties()->SetFloatProperty(m_propertyContainer, vr::Prop_DeviceBatteryPercentage_Float, 1.0f);
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_DeviceProvidesBatteryStatus_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_DeviceCanPowerOff_Bool, false);
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_TrackingSystemName_String, "hapticgloves");

        // Icons/visuals
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{hapticgloves}/icons/controller_status_off.png");
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{hapticgloves}/icons/controller_status_searching.gif");
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_ContainsProximitySensor_Bool, false);
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_RenderModelName_String, "vr_controller_vive_knu_ev3");
        vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{hapticgloves}/icons/hand_ready.png");
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_NeverTracked_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_HasDisplayComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_HasCameraComponent_Bool, false);

        CreateSkeletalComponent();
        vr::VRDriverInput()->CreateHapticComponent(m_propertyContainer, "/output/haptic", &m_poseComponent);
        return vr::VRInitError_None;
    }

    // Build skeleton component and reference poses
    void CreateSkeletalComponent() {
        vr::VRBoneTransform_t bindPose[31];
        vr::VRBoneTransform_t gripLimitPose[31];
        ComputeBindPose(bindPose);
        ComputeGripLimitPose(gripLimitPose);

        const std::string skeletonPath = "/input/skeleton/right"; // right hand only

        vr::EVRInputError err = vr::VRDriverInput()->CreateSkeletonComponent(
            m_propertyContainer,
            skeletonPath.c_str(),
            "/skeleton/hand/right",         // SteamVR hand skeleton
            "/pose/raw",                    // follows our raw device pose
            vr::VRSkeletalTracking_Full,
            gripLimitPose,                    // IMPORTANT: grip limits, not bind pose
            31,                               // bone count
            &m_skeletalComponent);

        if (err != vr::VRInputError_None) {
            vr::VRDriverLog()->Log("[HapticGlove] CreateSkeletonComponent failed");
        }
    }

    // --- Per-frame update ---
    virtual void RunFrame() {
        auto now = std::chrono::high_resolution_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastUpdateTime).count();
        m_lastUpdateTime = now;

        // Convert to seconds with clamp to handle long stalls
        float dt = (ms > 200 ? 0.2f : float(ms) * 0.001f);

        if (m_deviceIndex == vr::k_unTrackedDeviceIndexInvalid) return;

        vr::DriverPose_t pose = GetPose();
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_deviceIndex, pose, sizeof(pose));

        UpdateSkeletalPose(dt);
    }

    // Update skeleton (both motion ranges)
    void UpdateSkeletalPose(float dt) {
        SimulateHandMovement(dt);

        vr::VRBoneTransform_t bones[31];
        ComputeBoneTransforms(bones);

        // Normalize quaternions to avoid odd rotations
        for (int i = 0; i < 31; ++i) NormalizeQuat(bones[i].orientation);

        // Publish both motion ranges for compatibility
        auto e1 = vr::VRDriverInput()->UpdateSkeletonComponent(m_skeletalComponent, vr::VRSkeletalMotionRange_WithController, bones, 31);
        auto e2 = vr::VRDriverInput()->UpdateSkeletonComponent(m_skeletalComponent, vr::VRSkeletalMotionRange_WithoutController, bones, 31);
        (void)e1; (void)e2; // optionally add logging if non-zero
    }

    // Very simple procedural animation using dt
    void SimulateHandMovement(float dt) {
        static float t = 0.0f;
        t += dt; // advance by real frame time

        float wave = (std::sin(t * 2.0f) + 1.0f) * 0.5f; // 0..1
        m_fingerCurl[0] = wave * 0.7f;  // thumb
        m_fingerCurl[1] = wave * 0.9f;  // index
        m_fingerCurl[2] = wave * 1.0f;  // middle
        m_fingerCurl[3] = wave * 0.9f;  // ring
        m_fingerCurl[4] = wave * 0.8f;  // pinky
    }

    // Compute local bone transforms. Keep positions at identity so SteamVR uses reference skeleton for bone lengths; only drive rotations for curl.
    void ComputeBoneTransforms(vr::VRBoneTransform_t* tr) {
        for (int i = 0; i < 31; ++i) {
            tr[i].position = {0.0f, 0.0f, 0.0f, 1.0f};
            tr[i].orientation = {0.0f, 0.0f, 0.0f, 1.0f};
        }

        // 0 Root, 1 Wrist are identity

        // Thumb bones 2..5
        ComputeFullThumb(tr);
        // Index 6..10, Middle 11..15, Ring 16..20, Pinky 21..25
        ComputeFullFinger(tr, 6, 0);   // index
        ComputeFullFinger(tr, 11, 1);  // middle
        ComputeFullFinger(tr, 16, 2);  // ring
        ComputeFullFinger(tr, 21, 3);  // pinky
        // 26..30 aux bones remain identity
    }

    // Four thumb joints (metacarpal through distal)
    void ComputeFullThumb(vr::VRBoneTransform_t* tr) {
        float curl = m_fingerCurl[0];
        for (int i = 0; i < 4; ++i) {
            int idx = 2 + i; // 2..5
            float joint = curl * (i + 1) * 0.35f; // progressive curl
            float half = joint * 0.5f;
            tr[idx].orientation = { 0.0, 0.0, std::sin(half), std::cos(half) }; // rotate about Z
        }
    }

    // Five joints per finger (metacarpal through tip)
    void ComputeFullFinger(vr::VRBoneTransform_t* tr, int startBone, int fingerIndex) {
        float curl = m_fingerCurl[fingerIndex + 1]; // +1 because thumb is index 0
        for (int i = 0; i < 5; ++i) {
            int idx = startBone + i; // 5 bones
            float joint = curl * (i + 1) * 0.4f; // progressive curl
            float half = joint * 0.5f;
            tr[idx].orientation = { std::sin(half), 0.0, 0.0, std::cos(half) }; // rotate about X
        }
    }

    // Reference poses (keep identity to let SteamVR provide bone lengths/placements)
    void ComputeBindPose(vr::VRBoneTransform_t* bindPose) {
        for (int i = 0; i < 31; ++i) {
            bindPose[i].position = {0.0f, 0.0f, 0.0f, 1.0f};
            bindPose[i].orientation = {0.0f, 0.0f, 0.0f, 1.0f};
        }
    }

    void ComputeGripLimitPose(vr::VRBoneTransform_t* gripPose) {
        for (int i = 0; i < 31; ++i) {
            gripPose[i].position = {0.0f, 0.0f, 0.0f, 1.0f};
            gripPose[i].orientation = {0.0f, 0.0f, 0.0f, 1.0f};
        }
    }

    // --- Device pose ---
    virtual vr::DriverPose_t GetPose() override {
        vr::DriverPose_t pose = {0};
        pose.poseIsValid = true;
        pose.result = vr::TrackingResult_Running_OK;
        pose.deviceIsConnected = true;

        pose.vecPosition[0] = 0.0; // 30cm right
        pose.vecPosition[1] = 1.6; // 1m up
        pose.vecPosition[2] = -0.5; // 50cm forward

        pose.qRotation = {1.0, 0.0, 0.0, 0.0};

        for (int i = 0; i < 3; ++i) {
            pose.vecVelocity[i] = 0.0;
            pose.vecAngularVelocity[i] = 0.0;
            pose.vecAcceleration[i] = 0.0;
            pose.vecAngularAcceleration[i] = 0.0;
        }

        pose.poseTimeOffset = 0.0;
        return pose;
    }

    // --- Lifecycle ---
    virtual void Deactivate() override {
        m_deviceIndex = vr::k_unTrackedDeviceIndexInvalid;
        m_propertyContainer = vr::k_ulInvalidPropertyContainer;
        m_skeletalComponent = vr::k_ulInvalidInputComponentHandle;
        m_poseComponent = vr::k_ulInvalidInputComponentHandle;
    }

    virtual void EnterStandby() override {}
    virtual void* GetComponent(const char* pchComponentNameAndVersion) override { return nullptr; }
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override {}

    std::string GetSerialNumber() const { return m_serialNumber; }
};

// --- Provider ---
class SkeletonDriverProvider : public vr::IServerTrackedDeviceProvider {
private:
    std::unique_ptr<SkeletonDevice> hapticglove;

public:
    virtual vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
        vr::VRDriverLog()->Log("Skeleton Controller Driver: Initializing");

        hapticglove = std::make_unique<SkeletonDevice>(vr::TrackedControllerRole_RightHand);

        vr::VRServerDriverHost()->TrackedDeviceAdded(
            hapticglove->GetSerialNumber().c_str(),
            vr::TrackedDeviceClass_Controller,
            hapticglove.get());

        vr::VRDriverLog()->Log("Skeleton Controller Driver: Initialization complete");
        return vr::VRInitError_None;
    }

    virtual void RunFrame() override { if (hapticglove) hapticglove->RunFrame(); }

    virtual void Cleanup() override {
        vr::VRDriverLog()->Log("Skeleton Controller Driver: Shutting down");
        hapticglove.reset();
        VR_CLEANUP_SERVER_DRIVER_CONTEXT();
    }

    virtual const char* const* GetInterfaceVersions() override { return vr::k_InterfaceVersions; }
    virtual bool ShouldBlockStandbyMode() override { return false; }
    virtual void EnterStandby() override {}
    virtual void LeaveStandby() override {}
};

// --- Factory ---
DRIVER_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode) {
    if (0 == strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName)) {
        static SkeletonDriverProvider provider;
        if (pReturnCode) *pReturnCode = vr::VRInitError_None;
        return &provider;
    }
    if (pReturnCode) *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    return nullptr;
}