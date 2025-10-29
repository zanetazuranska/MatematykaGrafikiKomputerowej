#include "Matrix4x4.h"
#include <cstring>
#include <cmath>
#include <iomanip>
#include <ostream>

Matrix4x4::Matrix4x4(bool identity)
	: entries{ 0.0f }
{
	if (identity)
	{
		entries[0] = 1.0f;
		entries[5] = 1.0f;
		entries[10] = 1.0f;
		entries[15] = 1.0f;
	}
}

Matrix4x4::Matrix4x4(
	float e0, float e1, float e2, float e3,
	float e4, float e5, float e6, float e7,
	float e8, float e9, float e10, float e11,
	float e12, float e13, float e14, float e15
)
{
	entries[0] = e0;
	entries[1] = e1;
	entries[2] = e2;
	entries[3] = e3;
	entries[4] = e4;
	entries[5] = e5;
	entries[6] = e6;
	entries[7] = e7;
	entries[8] = e8;
	entries[9] = e9;
	entries[10] = e10;
	entries[11] = e11;
	entries[12] = e12;
	entries[13] = e13;
	entries[14] = e14;
	entries[15] = e15;
}

Matrix4x4::Matrix4x4(const Matrix4x4& mat)
{
	std::memcpy(entries, mat.entries, 16 * sizeof(float));
}

Matrix4x4::Matrix4x4(const float* f)
{
	std::memcpy(entries, f, 16 * sizeof(float));
}

Matrix4x4::~Matrix4x4()
{}

Matrix4x4 Matrix4x4::operator+(const Matrix4x4& mat) const
{
	Matrix4x4 result;
	for (int i = 0; i < 16; i++) result.entries[i] = entries[i] + mat.entries[i];
	return result;
}

Matrix4x4 Matrix4x4::operator-(const Matrix4x4& mat) const
{
	Matrix4x4 result;
	for (int i = 0; i < 16; ++i)
		result.entries[i] = entries[i] - mat.entries[i];
	return result;
}

Matrix4x4 Matrix4x4::operator*(const float f) const
{
	Matrix4x4 result;
	for (int i = 0; i < 16; i++) result.entries[i] = entries[i] * f;
	return result;
}

Matrix4x4 Matrix4x4::operator*(const Matrix4x4& mat) const
{
	Matrix4x4 result;
	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 4; col++) {
			float sum = 0.0f;
			for (int k = 0; k < 4; k++) sum += entries[row * 4 + k] * mat.entries[k * 4 + col];
			result.entries[row * 4 + col] = sum;
		}
	}
	return result;
}

bool Matrix4x4::operator==(const Matrix4x4& other) const noexcept
{
	const float EPS = 1e-5f;
	for (int i = 0; i < 16; ++i) if (std::fabs(entries[i] - other.entries[i]) > EPS) return false;
	return true;
}

bool Matrix4x4::operator!=(const Matrix4x4& other) const noexcept
{
	return !(*this == other);
}

Matrix4x4 Matrix4x4::getTransposed() const
{
	Matrix4x4 result;
	for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) result.entries[r * 4 + c] = entries[c * 4 + r];
	return result;
}

void Matrix4x4::setTransposed()
{
	*this = getTransposed();
}

void Matrix4x4::setTranslation(const Vector& translation)
{
	*this = Matrix4x4(true);
	setTranslationOnly(translation);
}

void Matrix4x4::setTranslationOnly(const Vector& translation)
{
	entries[3] = translation.x;
	entries[7] = translation.y;
	entries[11] = translation.z;
}

void Matrix4x4::setScale(const Vector& scale)
{
	*this = Matrix4x4(true);
	setScaleOnly(scale);
}

void Matrix4x4::setScaleOnly(const Vector& scale)
{
	entries[0] = scale.x;
	entries[5] = scale.y;
	entries[10] = scale.z;
}

Matrix4x4& Matrix4x4::translate(const Vector& translation)
{
	float tx = translation.x, ty = translation.y, tz = translation.z;
	for (int r = 0; r < 4; ++r) {
		entries[r * 4 + 3] = entries[r * 4 + 0] * tx
			+ entries[r * 4 + 1] * ty
			+ entries[r * 4 + 2] * tz
			+ entries[r * 4 + 3];
	}
	return *this;
}

Matrix4x4& Matrix4x4::scale(const Vector& scale)
{
	float sx = scale.x, sy = scale.y, sz = scale.z;
	for (int r = 0; r < 4; ++r) {
		entries[r * 4 + 0] *= sx;
		entries[r * 4 + 1] *= sy;
		entries[r * 4 + 2] *= sz;
	}
	return *this;
}

Matrix4x4& Matrix4x4::uniformScale(const float scale)
{
	for (int r = 0; r < 4; ++r) {
		entries[r * 4 + 0] *= scale;
		entries[r * 4 + 1] *= scale;
		entries[r * 4 + 2] *= scale;
	}
	return *this;
}

Matrix4x4 Matrix4x4::getInversed() const
{
	Matrix4x4 result;
	double cof[16];
	for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) cof[r * 4 + c] = cofactor(r, c);

	double det = 0.0;
	for (int j = 0; j < 4; ++j) det += static_cast<double>(entries[j]) * cof[j];

	if (std::fabs(det) < 1e-12) return result;

	for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c)
		result.entries[r * 4 + c] = static_cast<float>(cof[c * 4 + r] * (1.0 / det));

	return result;
}

void Matrix4x4::setInversed()
{
	*this = getInversed();
}

void Matrix4x4::setUniformScale(const float scale)
{
	*this = Matrix4x4(true);
	setUniformScaleOnly(scale);
}

void Matrix4x4::setUniformScaleOnly(const float scale)
{
	entries[0] = entries[5] = entries[10] = scale;
}

Matrix4x4 Matrix4x4::rotationX(float angleRad)
{
	const float c = std::cos(angleRad);
	const float s = std::sin(angleRad);
	return Matrix4x4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, c, -s, 0.0f,
		0.0f, s, c, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

Matrix4x4 Matrix4x4::rotationY(float angleRad)
{
	const float c = std::cos(angleRad);
	const float s = std::sin(angleRad);
	return Matrix4x4(
		c, 0.0f, s, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		-s, 0.0f, c, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

Matrix4x4 Matrix4x4::rotationZ(float angleRad)
{
	const float c = std::cos(angleRad);
	const float s = std::sin(angleRad);
	return Matrix4x4(
		c, -s, 0.0f, 0.0f,
		s, c, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

void Matrix4x4::setRotation(const Vector& rotation)
{
	*this = Matrix4x4(true);
	Matrix4x4 rx = rotationX(rotation.x);
	Matrix4x4 ry = rotationY(rotation.y);
	Matrix4x4 rz = rotationZ(rotation.z);
	Matrix4x4 R = rz * ry * rx;
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) entries[r * 4 + c] = R.entries[r * 4 + c];
}

void Matrix4x4::setRotationOnly(const Vector& rotation)
{
	Matrix4x4 rx = rotationX(rotation.x);
	Matrix4x4 ry = rotationY(rotation.y);
	Matrix4x4 rz = rotationZ(rotation.z);
	Matrix4x4 R = rz * ry * rx;
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) entries[r * 4 + c] = R.entries[r * 4 + c];
}

Matrix4x4& Matrix4x4::rotate(const Vector& rotation)
{
	Matrix4x4 rx = Matrix4x4::rotationX(rotation.x);
	Matrix4x4 ry = Matrix4x4::rotationY(rotation.y);
	Matrix4x4 rz = Matrix4x4::rotationZ(rotation.z);
	Matrix4x4 R = rz * ry * rx;
	*this = (*this) * R;
	return *this;
}

Matrix4x4& Matrix4x4::rotateX(float angleRad)
{
	*this = (*this) * Matrix4x4::rotationX(angleRad);
	return *this;
}

Matrix4x4& Matrix4x4::rotateY(float angleRad)
{
	*this = (*this) * Matrix4x4::rotationY(angleRad);
	return *this;
}

Matrix4x4& Matrix4x4::rotateZ(float angleRad)
{
	*this = (*this) * Matrix4x4::rotationZ(angleRad);
	return *this;
}

double Matrix4x4::minorDeterminant(int exclRow, int exclCol) const
{
	double m3[3][3];
	int ri = 0;
	for (int r = 0; r < 4; ++r) {
		if (r == exclRow) continue;
		int ci = 0;
		for (int c = 0; c < 4; ++c) {
			if (c == exclCol) continue;
			m3[ri][ci] = static_cast<double>(entries[r * 4 + c]);
			++ci;
		}
		++ri;
	}

	return
		m3[0][0] * (m3[1][1] * m3[2][2] - m3[1][2] * m3[2][1]) -
		m3[0][1] * (m3[1][0] * m3[2][2] - m3[1][2] * m3[2][0]) +
		m3[0][2] * (m3[1][0] * m3[2][1] - m3[1][1] * m3[2][0]);
}

double Matrix4x4::cofactor(int row, int col) const
{
	double minor = minorDeterminant(row, col);
	return (((row + col) & 1) == 0) ? minor : -minor;
}

std::ostream& operator<<(std::ostream& os, const Matrix4x4& m)
{
	constexpr int width = 10;
	constexpr int prec = 2;
	os << std::fixed << std::setprecision(prec);
	os << "[\n";
	for (int r = 0; r < 4; ++r) {
		os << "  [";
		for (int c = 0; c < 4; ++c) {
			os << std::setw(width) << m.entries[r * 4 + c];
			if (c < 3) os << ", ";
		}
		os << "]";
		if (r < 3) os << ",\n";
	}
	os << "\n]";
	return os;
}