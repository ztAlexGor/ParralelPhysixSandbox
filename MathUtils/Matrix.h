#pragma once
#include "Vector.h"
#include <cmath>

struct Matrix
{
    float m00, m01;
    float m10, m11;
    

    Matrix() = default;

    Matrix(float radians);

    Matrix(float a, float b, float c, float d);

    void Set(float radians);

    Matrix Abs() const;

    Vector AxisX() const;

    Vector AxisY() const;

    Matrix Transpose() const;

    const Vector operator*(const Vector& other) const;

    const Matrix operator*(const Matrix& other) const;
};