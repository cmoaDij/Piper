// Piper LOCAL-space sender. Meta SampleXrFramework supplies the OpenXR lifecycle,
// default Touch action bindings and rendering; the original teleop APK is separate.
#include "XrApp.h"
#include <android/log.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <time.h>

class PiperQuestWorld : public OVRFW::XrApp {
    uint64_t session_ = std::chrono::steady_clock::now().time_since_epoch().count();
    uint64_t generation_ = 0;
    uint64_t sequence_ = 0;
    XrTime pendingChangeTime_ = 0;

    static void PoseJson(std::ostream& out, const XrPosef& p) {
        out << '[' << p.position.x << ',' << p.position.y << ',' << p.position.z
            << ',' << p.orientation.x << ',' << p.orientation.y << ','
            << p.orientation.z << ',' << p.orientation.w << ']';
    }

    bool Locate(XrSpace space, XrTime time, XrPosef& pose) {
        XrSpaceLocation location{XR_TYPE_SPACE_LOCATION};
        const auto result = xrLocateSpace(space, LocalSpace, time, &location);
        constexpr XrSpaceLocationFlags required = XR_SPACE_LOCATION_POSITION_VALID_BIT |
            XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_TRACKED_BIT |
            XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
        pose = location.pose;
        return XR_SUCCEEDED(result) && (location.locationFlags & required) == required;
    }

public:
    void GetInitialSceneUri(std::string& uri) const override { uri.clear(); }

    PiperQuestWorld() {
        BackgroundColor = OVR::Vector4f(0.03f, 0.06f, 0.08f, 1.0f);
        FramebufferResolutionScaleFactor = 0.5f;
    }

    bool SessionInit() override {
        CurrentSpace = LocalSpace;
        ++generation_;
        return true;
    }

    void SessionStateChanged(XrSessionState state) override {
        if (state == XR_SESSION_STATE_FOCUSED) {
            ++generation_;
        }
    }

    void AppHandleEvent(XrEventDataBaseHeader* event) override {
        if (event->type == XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING) {
            const auto* change = reinterpret_cast<XrEventDataReferenceSpaceChangePending*>(event);
            if (change->referenceSpaceType == XR_REFERENCE_SPACE_TYPE_LOCAL) {
                pendingChangeTime_ = change->changeTime;
                ++generation_;
            }
        }
    }

    void Update(const OVRFW::ovrApplFrameIn& in) override {
        const XrTime sampleTime = ToXrTime(in.PredictedDisplayTime);
        XrPosef grip{}, aim{};
        const bool gripValid = Locate(RightControllerGripSpace, sampleTime, grip);
        const bool aimValid = Locate(RightControllerAimSpace, sampleTime, aim);
        const bool gripActive = ActionPoseIsActive(GripPoseAction, RightHandPath);
        const bool aimActive = ActionPoseIsActive(AimPoseAction, RightHandPath);
        const bool valid = Focused && sampleTime >= pendingChangeTime_ && gripValid && aimValid &&
            gripActive && aimActive;
        // BOOTTIME continues while Quest suspends, avoiding a permanent stale
        // offset after the headset sleeps while the workstation remains awake.
        timespec clock{};
        clock_gettime(CLOCK_BOOTTIME, &clock);
        const int64_t now = int64_t(clock.tv_sec) * 1000000000LL + clock.tv_nsec;
        std::ostringstream out;
        out << std::setprecision(9) << std::boolalpha;
        out << "{\"protocol\":\"piper.quest.local.v1\",\"space\":\"LOCAL\",\"session\":"
            << session_ << ",\"generation\":" << generation_ << ",\"seq\":" << ++sequence_
            << ",\"device_monotonic_ns\":" << now << ",\"xr_sample_time_ns\":" << sampleTime
            << ",\"valid\":" << valid
            << ",\"tracking\":{\"focused\":" << Focused << ",\"grip\":" << gripValid
            << ",\"aim\":" << aimValid << ",\"grip_active\":" << gripActive
            << ",\"aim_active\":" << aimActive << ",\"reference_ready\":"
            << (sampleTime >= pendingChangeTime_) << "},\"grip\":";
        PoseJson(out, grip);
        out << ",\"aim\":";
        PoseJson(out, aim);
        using Frame = OVRFW::ovrApplFrameIn;
        out << ",\"buttons\":{\"A\":" << (Focused && (in.AllButtons & Frame::kButtonA) != 0)
            << ",\"B\":" << (Focused && (in.AllButtons & Frame::kButtonB) != 0)
            << ",\"RJ\":" << (Focused && (in.AllButtons & Frame::kButtonRightThumbStick) != 0)
            << ",\"RG\":" << (Focused && in.RightRemoteGripTrigger > 0.25f)
            << ",\"rightTrig\":" << (Focused ? in.RightRemoteIndexTrigger : 0.0f) << "}}";
        __android_log_write(ANDROID_LOG_INFO, "PiperQuestWorld", out.str().c_str());
    }
};

ENTRY_POINT(PiperQuestWorld)
