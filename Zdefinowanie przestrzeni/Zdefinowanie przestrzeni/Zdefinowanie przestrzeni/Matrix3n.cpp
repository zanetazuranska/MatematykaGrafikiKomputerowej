#include "Matrix3n.h"

Matrix3n::Matrix3n() {}

Matrix3n::Matrix3n(float m0, float m1, float m2, float m3, float m4, float m5, float m6, float m7, float m8)
{
	matrixData[0] = m0;
	matrixData[1] = m1;
	matrixData[2] = m2;
	matrixData[3] = m3;
	matrixData[4] = m4;
	matrixData[5] = m5;
	matrixData[6] = m6;
	matrixData[7] = m7;
	matrixData[8] = m8;
}

Matrix3n::Matrix3n(const Matrix3n& value)
{
	matrixData[0] = value.matrixData[0];
	matrixData[1] = value.matrixData[1];
	matrixData[2] = value.matrixData[2];
	matrixData[3] = value.matrixData[3];
	matrixData[4] = value.matrixData[4];
	matrixData[5] = value.matrixData[5];
	matrixData[6] = value.matrixData[6];
	matrixData[7] = value.matrixData[7];
	matrixData[8] = value.matrixData[8];
}