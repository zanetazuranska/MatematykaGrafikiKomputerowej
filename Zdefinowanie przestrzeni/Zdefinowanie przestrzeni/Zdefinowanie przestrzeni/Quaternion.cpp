#include "Quaternion.h"

Quaternion::Quaternion() : w(1), x(0), y(0), z(0) {}

Quaternion::Quaternion(float w, float x, float y, float z)
    : w(w), x(x), y(y), z(z) {
}

Quaternion::Quaternion(float w, const Vector& v)
    : w(w), x(v.x), y(v.y), z(v.z) {
}

Quaternion Quaternion::fromAxisAngle(const Vector& axis, float angleRad) {
    Vector n = axis;
    n.normalize();
    float half = angleRad * 0.5f;
    float s = std::sin(half);
    return Quaternion(std::cos(half), n.x * s, n.y * s, n.z * s);
}


Quaternion Quaternion::operator+(const Quaternion& q) const {
    return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
}

Quaternion Quaternion::operator-(const Quaternion& q) const {
    return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
}

Quaternion Quaternion::operator*(const Quaternion& q) const {
    Vector v1(x, y, z);
    Vector v2(q.x, q.y, q.z);

    float nw = w * q.w - v1.dotProduct(v2);
    Vector nv = (v2 * w) + (v1 * q.w) + v1.cross(v2);

    return Quaternion(nw, nv.x, nv.y, nv.z);
}

Quaternion Quaternion::operator*(float s) const {
    return Quaternion(w * s, x * s, y * s, z * s);
}

Quaternion Quaternion::operator/(float s) const {
    if (s == 0.0f) throw std::runtime_error("Division by zero in Quaternion");
    return Quaternion(w / s, x / s, y / s, z / s);
}

Quaternion Quaternion::operator/(const Quaternion& b) const {
    return (*this) * b.inverse();
}


Quaternion& Quaternion::operator+=(const Quaternion& q) {
    w += q.w; x += q.x; y += q.y; z += q.z;
    return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& q) {
    w -= q.w; x -= q.x; y -= q.y; z -= q.z;
    return *this;
}

Quaternion& Quaternion::operator*=(const Quaternion& q) {
    *this = (*this) * q;
    return *this;
}

Quaternion& Quaternion::operator*=(float s) {
    w *= s; x *= s; y *= s; z *= s;
    return *this;
}

Quaternion& Quaternion::operator/=(float s) {
    if (s == 0.0f) throw std::runtime_error("Division by zero in Quaternion");
    w /= s; x /= s; y /= s; z /= s;
    return *this;
}

bool Quaternion::operator==(const Quaternion& q) const {
    return (w == q.w && x == q.x && y == q.y && z == q.z);
}

bool Quaternion::operator!=(const Quaternion& q) const {
    return !(*this == q);
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

Quaternion Quaternion::inverse() const {
    float n2 = w * w + x * x + y * y + z * z;
    if (n2 == 0.0f) throw std::runtime_error("Inverse of zero quaternion");
    Quaternion c = conjugate();
    return c / n2;
}

Vector Quaternion::rotate(const Vector& v) const {
    Quaternion qv(0, v.x, v.y, v.z);
    Quaternion res = (*this) * qv * this->inverse();
    return Vector(res.x, res.y, res.z);
}

std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
    os << "(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")";
    return os;
}

