#pragma once
class Matrix3n
{
public:
	float matrixData[9] = { 0.0f };

	Matrix3n();
	Matrix3n(float m0, float m1, float m2, float m3, float m4, float m5, float m6, float m7, float m8);
	Matrix3n(const Matrix3n& value);

	~Matrix3n();
private:
};

