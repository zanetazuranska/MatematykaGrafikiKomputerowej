#pragma once
#include <iosfwd>
#include "Vector.h"

class Matrix4x4
{
public:
	// Row-Major (row*4+col)
	float entries[16];

	Matrix4x4(bool identity = false);
	Matrix4x4(
		float e0, float e1, float e2, float e3,
		float e4, float e5, float e6, float e7,
		float e8, float e9, float e10, float e11,
		float e12, float e13, float e14, float e15
	);
	Matrix4x4(const float* f);
	Matrix4x4(const Matrix4x4& mat);
	~Matrix4x4();

	Matrix4x4 operator+(const Matrix4x4& mat) const;
	Matrix4x4 operator*(const float f) const;
	Matrix4x4 operator*(const Matrix4x4& mat) const;
	bool operator==(const Matrix4x4& other) const noexcept;
	bool operator!=(const Matrix4x4& other) const noexcept;

	Matrix4x4 getInversed() const;
	void setInversed();

	Matrix4x4 getTransposed() const;
	void setTransposed();

	// reset to identity and set translation
	void setTranslation(const Vector& translation);
	// do not reset, set translation components only
	void setTranslationOnly(const Vector& translation);
	// reset to identity and set scale
	void setScale(const Vector& scale);
	// do not reset, set scale components only
	void setScaleOnly(const Vector& scale);
	// reset to identity and set uniform scale
	void setUniformScale(const float scale);
	// do not reset, set uniform scale components only
	void setUniformScaleOnly(const float scale);

	// reset to identity and set rotation (angles in radians)
	void setRotation(const Vector& rotation);
	// do not reset, set rotation block only
	void setRotationOnly(const Vector& rotation);

	// apply and chain
	Matrix4x4& translate(const Vector& translation);
	Matrix4x4& scale(const Vector& scale);
	Matrix4x4& uniformScale(const float scale);

	// apply and chain
	Matrix4x4& rotate(const Vector& rotation);
	Matrix4x4& rotateX(float angleRad);
	Matrix4x4& rotateY(float angleRad);
	Matrix4x4& rotateZ(float angleRad);

	// single-axis rotation matrices
	static Matrix4x4 rotationX(float angleRad);
	static Matrix4x4 rotationY(float angleRad);
	static Matrix4x4 rotationZ(float angleRad);

private:
	double minorDeterminant(int exclRow, int exclCol) const;
	double cofactor(int row, int col) const;
};

std::ostream& operator<<(std::ostream& os, const Matrix4x4& m);

inline Vector transformPoint(const Matrix4x4& m, const Vector& v)
{
	float x = m.entries[0] * v.x + m.entries[1] * v.y + m.entries[2] * v.z + m.entries[3] * 1.0f;
	float y = m.entries[4] * v.x + m.entries[5] * v.y + m.entries[6] * v.z + m.entries[7] * 1.0f;
	float z = m.entries[8] * v.x + m.entries[9] * v.y + m.entries[10] * v.z + m.entries[11] * 1.0f;
	float w = m.entries[12] * v.x + m.entries[13] * v.y + m.entries[14] * v.z + m.entries[15] * 1.0f;
	if (w != 0.0f && w != 1.0f) { x /= w; y /= w; z /= w; }
	return Vector(x, y, z);
}

inline Vector transformDirection(const Matrix4x4& m, const Vector& v)
{
	float x = m.entries[0] * v.x + m.entries[1] * v.y + m.entries[2] * v.z;
	float y = m.entries[4] * v.x + m.entries[5] * v.y + m.entries[6] * v.z;
	float z = m.entries[8] * v.x + m.entries[9] * v.y + m.entries[10] * v.z;
	return Vector(x, y, z);
}

inline Vector operator*(const Matrix4x4& m, const Vector& v)
{
	return transformPoint(m, v);
}