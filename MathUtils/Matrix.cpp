#include "Matrix.h"

Matrix::Matrix(float radians)
{
    float cos = std::cos(radians);
    float sin = std::sin(radians);

    m00 = cos; m01 = -sin;
    m10 = sin; m11 = cos;
}

Matrix::Matrix(float a, float b, float c, float d) : m00(a), m01(b), m10(c), m11(d) {}


void Matrix::Set(float radians)
{
    float cos = std::cos(radians);
    float sin = std::sin(radians);

    m00 = cos; m01 = -sin;
    m10 = sin; m11 = cos;
}


Matrix Matrix::Abs() const
{
    return Matrix(std::abs(m00), std::abs(m01), std::abs(m10), std::abs(m11));
}


Vector Matrix::AxisX() const
{
    return Vector(m00, m10);
}


Vector Matrix::AxisY() const
{
    return Vector(m01, m11);
}


Matrix Matrix::Transpose() const
{
    return Matrix(m00, m10, m01, m11);
}


const Vector Matrix::operator*(const Vector& other) const
{
    return Vector(m00 * other.x + m01 * other.y, m10 * other.x + m11 * other.y);
}


const Matrix Matrix::operator*(const Matrix& other) const
{
    // [00 01] * [00 01] = [00 * 00 + 01 * 10, 00 * 01 + 01 * 11]
    // [10 11]   [10 11]   [10 * 00 + 11 * 10, 10 * 01 + 11 * 11]

    return Matrix(
        m00 * other.m00 + m01 * other.m10,
        m00 * other.m01 + m01 * other.m11,
        m10 * other.m00 + m11 * other.m10,
        m10 * other.m01 + m11 * other.m11
    );
}
