#pragma once

#include "fpm/fixed.hpp"
#include "fpm/math.hpp"   

namespace GekkoMath {
    using Unit = fpm::fixed_16_16;

    // VISUALIZATION ONLY
    struct Vec3F {
        float x, y, z;
        Vec3F(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
    };

    struct Vec3 {
        Unit x, y, z;

        Vec3() : x{0}, y{0}, z{0} {}
        Vec3(const Vec3& v) = default;
        Vec3(const Unit& xx, const Unit& yy, const Unit& zz) : x(xx), y(yy), z(zz) {}

        Unit Dot(const Vec3& other) const {
            return (x * other.x) + (y * other.y) + (z * other.z);
        }

        Vec3 Cross(const Vec3& other) const {
            return Vec3(
                y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x
            );
        }

        Vec3 operator+(const Vec3& other) const {
            return Vec3(x + other.x, y + other.y, z + other.z);
        }

        Vec3& operator+=(const Vec3& other) {
            *this = *this + other;
            return *this;
        }

        Vec3 operator+(const Unit& other) const {
            return Vec3(x + other, y + other, z + other);
        }

        Vec3& operator+=(const Unit& other) {
            *this = *this + other;
            return *this;
        }

        Vec3 operator-(const Vec3& other) const {
            return Vec3(x - other.x, y - other.y, z - other.z);
        }

        Vec3& operator-=(const Vec3& other) {
            *this = *this - other;
            return *this;
        }

        Vec3 operator-(const Unit& other) const {
            return Vec3(x - other, y - other, z - other);
        }

        Vec3& operator-=(const Unit& other) {
            *this = *this - other;
            return *this;
        }

        Vec3 operator/(const Vec3& other) const {
            return Vec3(x / other.x, y / other.y, z / other.z);
        }

        Vec3 operator/(const Unit& other) const {
            return Vec3(x / other, y / other, z / other);
        }

        Vec3& operator/=(const Vec3& other) {
            *this = *this / other;
            return *this;
        }

        Vec3& operator/=(const Unit& other) {
            *this = *this / other;
            return *this;
        }

        Vec3 operator*(const Vec3& other) const {
            return Vec3(x * other.x, y * other.y, z * other.z);
        }

        Vec3 operator*(const Unit& other) const {
            return Vec3(x * other, y * other, z * other);
        }

        Vec3& operator*=(const Vec3& other) {
            *this = *this * other;
            return *this;
        }

        Vec3& operator*=(const Unit& other) {
            *this = *this * other;
            return *this;
        }

        bool operator==(const Vec3& other) const {
            return x == other.x && y == other.y && z == other.z;
        }

        bool operator!=(const Vec3& other) const {
            return !(*this == other);
        }

        // VISUALIZATION ONLY
        Vec3F AsFloat() const {
            return Vec3F(
                static_cast<float>(x), 
                static_cast<float>(y), 
                static_cast<float>(z));
        }
    };

    inline Unit cosdeg(int deg) {
        int a = ((deg % 360) + 360) % 360;
        switch (a) {
            case 0:   return Unit{1};
            case 90:  return Unit{0};
            case 180: return Unit{-1};
            case 270: return Unit{0};
            default:  return fpm::cos(Unit::pi() * Unit{deg} / Unit{180});
        }
    }

    inline Unit sindeg(int deg) {
        int a = ((deg % 360) + 360) % 360;
        switch (a) {
            case 0:   return Unit{0};
            case 90:  return Unit{1};
            case 180: return Unit{0};
            case 270: return Unit{-1};
            default:  return fpm::sin(Unit::pi() * Unit{deg} / Unit{180});
        }
    }

    struct Mat3 {
        Vec3 cols[3]; // column-major: [right, up, forward]

        Mat3() : cols{
            Vec3(Unit{1}, Unit{0}, Unit{0}),
            Vec3(Unit{0}, Unit{1}, Unit{0}),
            Vec3(Unit{0}, Unit{0}, Unit{1})
        } {
        }

        Mat3(const Vec3& x, const Vec3& y, const Vec3& z) : cols{ x, y, z } {}

        Vec3 operator*(const Vec3& v) const {
            return cols[0] * v.x + cols[1] * v.y + cols[2] * v.z;
        }

        Vec3 Transform(const Vec3& v) const { return *this * v; }

        // Transform point from local to world: world = origin + rotation * local
        Vec3 TransformPoint(const Vec3& local, const Vec3& origin, const Mat3& rotation) {
            return origin + rotation.Transform(local);
        }

        Mat3 operator*(const Mat3& o) const {
            return Mat3(
                *this * o.cols[0],
                *this * o.cols[1],
                *this * o.cols[2]
            );
        }

        bool operator==(const Mat3& o) const {
            return cols[0] == o.cols[0] && cols[1] == o.cols[1] && cols[2] == o.cols[2];
        }

        static Mat3 RotateX(int deg) {
            Unit c = cosdeg(deg), s = sindeg(deg);
            return Mat3(
                Vec3(Unit{1}, Unit{0}, Unit{0}),
                Vec3(Unit{0}, c, s),
                Vec3(Unit{0}, Unit{0} - s, c)
            );
        }

        static Mat3 RotateY(int deg) {
            Unit c = cosdeg(deg), s = sindeg(deg);
            return Mat3(
                Vec3(c, Unit{0}, Unit{0} - s),
                Vec3(Unit{0}, Unit{1}, Unit{0}),
                Vec3(s, Unit{0}, c)
            );
        }

        static Mat3 RotateZ(int deg) {
            Unit c = cosdeg(deg), s = sindeg(deg);
            return Mat3(
                Vec3(c, s, Unit{0}),
                Vec3(Unit{0} - s, c, Unit{0}),
                Vec3(Unit{0}, Unit{0}, Unit{1})
            );
        }
    };

    // util funcs
    inline Unit abs(Unit num) {
        return fpm::abs(num);
    }

    inline Unit sqrt(Unit num) {
        return fpm::sqrt(num);
    }

    inline Unit clamp(Unit num, Unit lo, Unit hi) {
        return (num < lo) ? lo : (num > hi ? hi : num);
    }

    inline Unit length(const Vec3& vector) {
        return fpm::sqrt(vector.Dot(vector));
    }

    inline Vec3 normalize(const Vec3& v) {
        Unit len = length(v);
        if (len == Unit{0}) return Vec3();
        return v / len;
    }

}