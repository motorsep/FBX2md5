#pragma once

#include <cmath>
#include <algorithm>

namespace fbx2md5 {

struct Vec2 {
    double x = 0, y = 0;
    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
};

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& b) const { return {x + b.x, y + b.y, z + b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x - b.x, y - b.y, z - b.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }

    double Length() const { return std::sqrt(x * x + y * y + z * z); }

    Vec3 Normalized() const {
        double len = Length();
        if (len < 1e-12) return {0, 0, 0};
        return *this * (1.0 / len);
    }

    static Vec3 Cross(const Vec3& a, const Vec3& b) {
        return {a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
    }

    static double Dot(const Vec3& a, const Vec3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
};

struct Quat {
    double x = 0, y = 0, z = 0, w = 1;
    Quat() = default;
    Quat(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}

    Quat Normalized() const {
        double len = std::sqrt(x * x + y * y + z * z + w * w);
        if (len < 1e-12) return {0, 0, 0, 1};
        double inv = 1.0 / len;
        return {x * inv, y * inv, z * inv, w * inv};
    }

    // md5 format reconstructs w = -sqrt(1 - x² - y² - z²), always ≤ 0.
    // To ensure the stored quaternion is compatible, we need w ≤ 0.
    // If w > 0, negate entire quaternion (q and -q are same rotation).
    Quat EnsurePositiveW() const {
        if (w < 0) return {-x, -y, -z, -w};
        return *this;
    }

    // For md5 format: ensure w ≤ 0 (engine computes w = -sqrt(...))
    Quat EnsureW() const {
        if (w > 0) return {-x, -y, -z, -w};
        return *this;
    }

    Quat Conjugate() const { return {-x, -y, -z, w}; }

    Quat operator*(const Quat& b) const {
        return {
            w * b.x + x * b.w + y * b.z - z * b.y,
            w * b.y - x * b.z + y * b.w + z * b.x,
            w * b.z + x * b.y - y * b.x + z * b.w,
            w * b.w - x * b.x - y * b.y - z * b.z
        };
    }

    Vec3 RotatePoint(const Vec3& v) const {
        // q * v * q^-1
        Quat qv(v.x, v.y, v.z, 0);
        Quat result = *this * qv * Conjugate();
        return {result.x, result.y, result.z};
    }
};

} // namespace fbx2md5
