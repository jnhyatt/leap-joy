#include <chrono>
#include <functional>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <iostream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>

// VJoy
#include <Windows.h>
#include <public.h>
#include <vjoyinterface.h>

// Leap
#include <LeapC.h>

#undef min
#undef max

using glm::quat;
using glm::uvec3;
using glm::vec3;
namespace ch = std::chrono;
typedef ch::high_resolution_clock clk;

const static float pi = 3.141592653589793f;

std::unordered_map<VjdStat, std::optional<std::string>> vjdStatus{
    {VJD_STAT_OWN, "Owned"},
    {VJD_STAT_FREE, "Free"},
    {VJD_STAT_BUSY, "Busy"},
    {VJD_STAT_MISS, "Miss"},
};

class VjoyDevice {
public:
    enum class Axis { X, Y, Z, Rx, Ry, Rz };

    VjoyDevice() {
        if (!AcquireVJD(1))
            throw std::runtime_error("Error acquiring device");
        ResetVJD(1);
    }

    ~VjoyDevice() { RelinquishVJD(1); }

    void setAxis(Axis axis, uint32_t value) { SetAxis(value, 1, axes[axis]); }

    void setButton(size_t index, bool down) {
        SetBtn(down, 1, static_cast<UCHAR>(index));
    }

private:
    static std::unordered_map<Axis, UINT> axes;
};

std::unordered_map<VjoyDevice::Axis, UINT> VjoyDevice::axes = {
    {Axis::X, HID_USAGE_X},   {Axis::Y, HID_USAGE_Y},
    {Axis::Z, HID_USAGE_Z},   {Axis::Rx, HID_USAGE_RX},
    {Axis::Ry, HID_USAGE_RY}, {Axis::Rz, HID_USAGE_RZ},
};

vec3 toGlm(const LEAP_VECTOR& x) { return vec3(x.x, x.y, x.z); }

quat toGlm(const LEAP_QUATERNION& x) { return quat(x.w, x.x, x.y, x.z); }

std::ostream& operator<<(std::ostream& out, const vec3& x) {
    return out << "<" << x.x << ", " << x.y << ", " << x.z << ">";
}

std::ostream& operator<<(std::ostream& out, const quat& x) {
    return out << "(" << x.w << ", " << x.x << ", " << x.y << ", " << x.z
               << ")";
}

class Hands {
public:
    class It {
    public:
        It(const LEAP_HAND* element) : m_element(element) {}

        const LEAP_HAND& operator*() const { return *m_element; }

        const LEAP_HAND* operator->() const { return m_element; }

        It& operator++() {
            ++m_element;
            return *this;
        }

        bool operator==(const It& rhs) const {
            return m_element == rhs.m_element;
        }

        bool operator!=(const It& rhs) const {
            return m_element != rhs.m_element;
        }

    private:
        const LEAP_HAND* m_element;
    };

    Hands(const LEAP_TRACKING_EVENT& trackingEvent)
        : m_begin(trackingEvent.pHands), m_end(m_begin + trackingEvent.nHands) {
    }

    It begin() const { return It(m_begin); }
    It end() const { return It(m_end); }

    size_t count() const { return m_end - m_begin; }

    It firstWhere(const std::function<bool(const LEAP_HAND&)>& pred) const {
        for (auto it = begin(); it != end(); ++it) {
            if (pred(*it)) {
                return it;
            }
        }
        return end();
    }

private:
    const LEAP_HAND* m_begin;
    const LEAP_HAND* m_end;
};

float normalizeAngleTo(float angle, float center) {
    while (angle > center + pi) {
        angle -= 2.0f * pi;
    }
    while (angle < center - pi) {
        angle += 2.0f * pi;
    }
    return angle;
}

template <typename T>
concept arithmetic = std::is_arithmetic_v<T>;

template <typename T> class Range {
public:
    Range(T min, T max) : min(min), max(max) {}

    T min;
    T max;

    T width() const { return max - min; }

    void setWidth(T newWidth) {
        T halfDelta = (newWidth - width()) / static_cast<T>(2);
        min -= halfDelta;
        max += halfDelta;
    }

    T center() const { return (max + min) / static_cast<T>(2); }

    void setCenter(T newCenter) {
        T delta = newCenter - center();
        min += delta;
        max += delta;
        vec3 dog = uvec3(1);
    }
};

vec3 operator/(const vec3& lhs, const vec3& rhs) {
    return vec3(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z);
}

vec3 operator+(const vec3& lhs, const uvec3& rhs) {
    return vec3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

namespace std {
    vec3 clamp(const vec3& x, const vec3& min, const vec3& max) {
        return vec3(std::clamp(x.x, min.x, max.x),
                    std::clamp(x.y, min.y, max.y),
                    std::clamp(x.z, min.z, max.z));
    }
} // namespace std

template <arithmetic K, arithmetic V>
class LinearKeys {
public:
    V transform(K x) const {
        auto low = keys.lower_bound(x);
        if (low == keys.begin()) {
            return low->second;
        }
        auto high = low--;
        return low->second + (x - low->first) * (high->second - low->second) /
                                 (high->first - low->first);
    }

    std::map<K, V> keys;
};

template <typename X, typename Y>
Y convertClamped(X x, const Range<X>& rangeX, const Range<Y>& rangeY) {
    return (std::clamp(x, rangeX.min, rangeX.max) - rangeX.min) *
               (rangeY.width() / rangeX.width()) +
           rangeY.min;
}

int main() {
    Range<vec3> positionBounds(vec3(-200.0f, -200.0f, 120.0f),
                               vec3(200.0f, 200.0f, 300.0f));
    Range<vec3> rotationBounds(vec3(-1.0f, -0.5f, -1.0f),
                               vec3(1.0f, 0.5f, 1.0f));
    Range<uvec3> joy(uvec3(0), uvec3(32767));
    LinearKeys<uint32_t, uint32_t> positionDeadzone;
    positionDeadzone.keys = {
        {0, 0},
        {14746, 16383},
        {18022, 16384},
        {32767, 32767},
    };

    uint32_t activeHandId = 0;

    VjoyDevice device;
    LEAP_CONNECTION connection;
    LeapCreateConnection(NULL, &connection);
    LeapOpenConnection(connection);

    clk::time_point start = clk::now();
    bool setTrackingMode = true;
    while (true) {
        LEAP_CONNECTION_MESSAGE msg;

        if (setTrackingMode) {
            LeapSetTrackingMode(connection, eLeapTrackingMode_Desktop);
        }

        eLeapRS result = LeapPollConnection(connection, 1000, &msg);
        switch (result) {
        case eLeapRS_Success:
            break;
        case eLeapRS_Timeout:
            std::cout << "Poll device timed out" << std::endl;
            continue;
        default:
            std::cout << "Poll device returned error code " << std::hex
                      << result << std::endl;
            continue;
        }

        switch (msg.type) {
        case eLeapEventType_Tracking: {
            Hands hands(*msg.tracking_event);
            auto primaryHand = hands.firstWhere(
                [&](const LEAP_HAND& x) { return x.id == activeHandId; });

            if (primaryHand == hands.end()) { // Previously active hand lost
                primaryHand = hands.begin();
            }
            if (primaryHand == hands.end()) { // No hands
                device.setAxis(VjoyDevice::Axis::X, 16383);
                device.setAxis(VjoyDevice::Axis::Y, 16383);
                device.setAxis(VjoyDevice::Axis::Z, 16383);
                device.setAxis(VjoyDevice::Axis::Rx, 16383);
                device.setAxis(VjoyDevice::Axis::Ry, 16383);
                device.setAxis(VjoyDevice::Axis::Rz, 16383);
                device.setButton(1, false);
                device.setButton(2, false);
                continue;
            }

            vec3 pos = toGlm(primaryHand->palm.position);
            vec3 euler = glm::eulerAngles(toGlm(primaryHand->palm.orientation));
            euler.x = normalizeAngleTo(euler.x, rotationBounds.center().x);
            euler.y = normalizeAngleTo(euler.y, rotationBounds.center().y);
            euler.z = normalizeAngleTo(euler.z, rotationBounds.center().z);

            vec3 middleDir =
                normalize(toGlm(primaryHand->middle.distal.next_joint) -
                          toGlm(primaryHand->middle.proximal.prev_joint));
            vec3 pinkyDir =
                normalize(toGlm(primaryHand->pinky.distal.next_joint) -
                          toGlm(primaryHand->pinky.proximal.prev_joint));
            
            float spreadAngle = acosf(glm::dot(middleDir, pinkyDir));

            bool recentering = spreadAngle > 0.8f;
            if (recentering) {
                positionBounds.setCenter(pos);
                rotationBounds.setCenter(euler);
            }
            uvec3 joyPosition = convertClamped(pos, positionBounds, joy);
            uvec3 joyRotation = convertClamped(euler, rotationBounds, joy);
            device.setAxis(VjoyDevice::Axis::X, positionDeadzone.transform(joyPosition.x));
            device.setAxis(VjoyDevice::Axis::Y, positionDeadzone.transform(joyPosition.y));
            device.setAxis(VjoyDevice::Axis::Z, positionDeadzone.transform(joyPosition.z));
            device.setAxis(VjoyDevice::Axis::Rx, joyRotation.x);
            device.setAxis(VjoyDevice::Axis::Ry, joyRotation.y);
            device.setAxis(VjoyDevice::Axis::Rz, joyRotation.z);
            device.setButton(1, !primaryHand->index.is_extended);
            device.setButton(2, !primaryHand->thumb.is_extended);
            break;
        }
        case eLeapEventType_TrackingMode:
            if (msg.tracking_mode_event->current_tracking_mode ==
                eLeapTrackingMode_Desktop) {
                setTrackingMode = false;
            }
            break;
        default:
            std::cout << "Ignoring message: " << msg.type << std::endl;
        }
    }

    LeapCloseConnection(connection);
    LeapDestroyConnection(connection);

    return 0;
}
