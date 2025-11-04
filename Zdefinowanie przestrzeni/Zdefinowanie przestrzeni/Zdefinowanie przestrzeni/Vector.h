#pragma once

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
    Vector operator+(const Vector& v);
    void operator-=(const Vector& v);
    void operator/=(float f);
    void operator*=(float f);
    Vector operator*(float f);

    float length();
    void normalize();
    Vector dot(Vector v);
    float dotProduct(Vector v);
    Vector cross(Vector v);

    ~Vector();
};

