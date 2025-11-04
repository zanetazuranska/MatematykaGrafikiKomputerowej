#pragma once
#include "Vector.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

class Quaternion {
public:
    float w; 
    float x;
    float y;
    float z;

    Quaternion();                        
    Quaternion(float w, float x, float y, float z);
    Quaternion(float w, const Vector& v);

    static Quaternion fromAxisAngle(const Vector& axis, float angleRad);

    // --- operatory ---
    Quaternion operator+(const Quaternion& q) const;
    Quaternion operator-(const Quaternion& q) const;
    Quaternion operator*(const Quaternion& q) const; 
    Quaternion operator*(float s) const;
    Quaternion operator/(float s) const;
    Quaternion operator/(const Quaternion& q) const;
    Quaternion& operator+=(const Quaternion& q);
    Quaternion& operator-=(const Quaternion& q);
    Quaternion& operator*=(const Quaternion& q);
    Quaternion& operator*=(float s);
    Quaternion& operator/=(float s);

    bool operator==(const Quaternion& q) const;
    bool operator!=(const Quaternion& q) const;

    void normalize();
    Quaternion conjugate() const;
    Quaternion inverse() const;

    Vector rotate(const Vector& v) const; 

    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q);
};
