#pragma once
#include <iosfwd>

class Vector {
public:
    float x = 0;
    float y = 0;
    float z = 0;

    Vector();
    Vector(float x, float y, float z);
    Vector(Vector p1, Vector p2);
    Vector(const Vector& v);

    void operator+=(const Vector& v);
    Vector operator+(const Vector& v) const;
    void operator-=(const Vector& v);
    Vector operator-(const Vector& v);
    void operator/=(float f);
    void operator*=(float f);
    Vector operator*(float f) const;

    float length() const;
    void normalize();
    Vector dot(Vector v) const;
    float dotProduct(Vector v) const;
    Vector cross(Vector v) const;

    ~Vector();
};

std::ostream& operator<<(std::ostream& os, const Vector& v);
