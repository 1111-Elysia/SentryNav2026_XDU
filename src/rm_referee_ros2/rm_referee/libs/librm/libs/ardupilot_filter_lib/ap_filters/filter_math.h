#pragma once

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_2PI
#define M_2PI (2.0 * M_PI)
#endif

#include <chrono>

// Disable copy constructor and assignment operator
#define CLASS_NO_COPY(classname) \
    classname(const classname &) = delete; \
    classname &operator=(const classname &) = delete

// Simple 2D vector class
template <typename T>
struct Vector2 {
    T x, y;

    Vector2() : x(0), y(0) {}
    Vector2(T x_, T y_) : x(x_), y(y_) {}

    Vector2 operator+(const Vector2 &v) const { return Vector2(x + v.x, y + v.y); }
    Vector2 operator-(const Vector2 &v) const { return Vector2(x - v.x, y - v.y); }
    Vector2 operator*(T scalar) const { return Vector2(x * scalar, y * scalar); }
    Vector2 operator/(T scalar) const { return Vector2(x / scalar, y / scalar); }

    Vector2 &operator+=(const Vector2 &v) { x += v.x; y += v.y; return *this; }
    Vector2 &operator-=(const Vector2 &v) { x -= v.x; y -= v.y; return *this; }
    Vector2 &operator*=(T scalar) { x *= scalar; y *= scalar; return *this; }
    Vector2 &operator/=(T scalar) { x /= scalar; y /= scalar; return *this; }

    T length() const { return std::sqrt(x * x + y * y); }
    T length_squared() const { return x * x + y * y; }
};

// Simple 3D vector class
template <typename T>
struct Vector3 {
    T x, y, z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3 &v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
    Vector3 operator-(const Vector3 &v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
    Vector3 operator*(T scalar) const { return Vector3(x * scalar, y * scalar, z * scalar); }
    Vector3 operator/(T scalar) const { return Vector3(x / scalar, y / scalar, z / scalar); }

    Vector3 &operator+=(const Vector3 &v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vector3 &operator-=(const Vector3 &v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vector3 &operator*=(T scalar) { x *= scalar; y *= scalar; z *= scalar; return *this; }
    Vector3 &operator/=(T scalar) { x /= scalar; y /= scalar; z /= scalar; return *this; }

    T length() const { return std::sqrt(x * x + y * y + z * z); }
    T length_squared() const { return x * x + y * y + z * z; }
};

// Type aliases
using Vector2f = Vector2<float>;
using Vector3f = Vector3<float>;

// Utility functions
inline bool is_positive(float f) {
    return f > 0.0f;
}

inline bool is_zero(float f) {
    return std::fabs(f) < 1e-6f;
}

inline bool is_equal(float a, float b) {
    return std::fabs(a - b) < 1e-6f;
}

inline float constrain_float(float val, float min_val, float max_val) {
    return std::max(min_val, std::min(max_val, val));
}

inline int32_t constrain_int32(int32_t val, int32_t min_val, int32_t max_val) {
    return std::max(min_val, std::min(max_val, val));
}

inline float sq(float v) {
    return v * v;
}

inline float safe_sqrt(float v) {
    return std::sqrt(std::max(0.0f, v));
}

// Calculate low pass filter alpha (discrete time)
inline float calc_lowpass_alpha_dt(float dt, float cutoff_freq) {
    if (dt <= 0.0f || cutoff_freq <= 0.0f) {
        return 1.0f;
    }
    const float rc = 1.0f / (M_2PI * cutoff_freq);
    return dt / (dt + rc);
}

// Time utilities (for standalone use)
namespace filter_hal {
    // Weak symbol - user can override this with their own implementation
    uint32_t millis() __attribute__((weak));

    inline uint32_t millis() {
        // Default implementation - returns time in ms since epoch
        // User should override this with their platform-specific implementation
        return static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()
            ).count()
        );
    }
}



