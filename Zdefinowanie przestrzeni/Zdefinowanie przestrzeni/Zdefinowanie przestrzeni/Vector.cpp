#include "Vector.h"
#include <stdexcept>
#include <cmath>
#include <iostream> 

Vector::Vector() : x(0), y(0), z(0) {}

Vector::Vector(float x, float y, float z): x(x), y(y), z(z) {}

Vector::Vector(Vector p1, Vector p2) {
	x = p2.x - p1.x;
	y = p2.y - p1.y;
	z = p2.z - p1.z;
}

Vector::Vector(const Vector& v) {
	x = v.x;
	y = v.y;
	z = v.z;
}

Vector::~Vector() {}

void Vector::operator+=(const Vector& v) {
	x += v.x;
	y += v.y;
	z += v.z;
}

Vector Vector::operator+(const Vector& v) const
{
	Vector pom(x + v.x, y + v.y, z + v.z);
	return pom;
}

void Vector::operator-=(const Vector& v) {
	x -= v.x;
	y -= v.y;
	z -= v.z;
}

Vector Vector::operator-(const Vector& v) {
	x -= v.x;
	y -= v.y;
	z -= v.z;

	return Vector(x, y, z);
}

void Vector::operator/=(float f) {
	if (f != 0) {
		this->x = x / f;
		this->y = y / f;
		this->z = z / f;
	}
	else {
		throw std::runtime_error("Division by zero in Vector");
	}
}

void Vector::operator*=(float f) {
	this->x = x * f;
	this->y = y * f;
	this->z = z * f;
}

Vector Vector::operator*(float f) const
{
	Vector pom;
	pom.x = x * f;
	pom.y = y * f;
	pom.z = z * f;

	return pom;
}

float Vector::length() const {
	return (float) std::sqrt(x * x + y * y + z * z);
}

void Vector::normalize() {
	float n = length();
	try {
		*this /= n;
	}
	catch (std::runtime_error e) {
		throw e;
	}
}

Vector Vector::dot(Vector v) const {
	Vector result;
	result.x = this->x * v.x;
	result.y = this->y * v.y;
	result.z = this->z * v.z;

	return result;
}

float Vector::dotProduct(Vector v) const {
	Vector result = dot(v);
	return result.x + result.y + result.z;
}

Vector Vector::cross(Vector v) const {
	Vector result;
	result.x = this->y * v.z - this->z * v.y;
	result.y = this->z * v.x - this->x * v.z;
	result.z = this->x * v.y - this->y * v.x;

	return result;
}

std::ostream& operator<<(std::ostream& os, const Vector& v) {
	os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
	return os;
}