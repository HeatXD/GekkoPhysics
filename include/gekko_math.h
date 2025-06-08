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

        Vec3() = default;
        Vec3(const Vec3& v) = default;
        Vec3(const Unit& xx, const Unit& yy, const Unit& zz) : x(xx), y(yy), z(zz) {}

        Unit Dot(const Vec3& other) const {
            return (x * other.x) + (y * other.y) + (z * other.z);
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
}