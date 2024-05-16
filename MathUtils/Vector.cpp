#include "Vector.h"
#include <cmath>

Vector::Vector(const Vector& v)
{
	x = v.x;
	y = v.y;
}

void Vector::Set(float x, float y)
{
	this->x = x;
	this->y = y;
}

void Vector::SetZero()
{
	x = 0.0f;
	y = 0.0f;
}

float Vector::Length() const
{
	return sqrt(x*x + y*y);
}

float Vector::SqLength() const
{
	return x * x + y * y;
}

Vector& Vector::Normalize()
{
	float length = Length();

	if (abs(length) > FLT_EPSILON)
	{
		x /= length;
		y /= length;
	}

	return *this;
}

Vector& Vector::operator=(const Vector& other)
{
	x = other.x;
	y = other.y;
	return *this;
}

bool Vector::operator==(const Vector& other)
{
	return x == other.x && y == other.y;
}

bool Vector::operator!=(const Vector& other)
{
	return !(*this==other);
}

Vector& Vector::operator-=(const Vector& other)
{
	x -= other.x;
	y -= other.y;
	return *this;
}

Vector& Vector::operator+=(const Vector& other)
{
	x += other.x;
	y += other.y;
	return *this;
}

Vector& Vector::operator*=(const float val)
{
	x *= val;
	y *= val;
	return *this;
}

Vector& Vector::operator/=(const float val)
{
	x /= val;
	y /= val;
	return *this;
}

Vector Vector::operator-(const Vector& other) const
{
	Vector copy = *this;
	return copy -= other;
}

Vector Vector::operator+(const Vector& other) const
{
	Vector copy = *this;
	return copy += other;
}

Vector Vector::operator*(const float val) const
{
	Vector copy = *this;
	return copy *= val;
}

Vector Vector::operator/(const float val) const
{
	Vector copy = *this;
	return copy /= val;
}

Vector Vector::operator-() const
{
	return Vector(-x, -y);
}

float Vector::DotProduct(const Vector& a, const Vector& b)
{
	return a.x * b.x + a.y * b.y;
}

float Vector::CrossProduct(const Vector& a, const Vector& b)
{
	return a.x * b.y - a.y * b.x;
}

Vector Vector::CrossProduct(float a, const Vector& b)
{
	return Vector(-a * b.y, a * b.x);
}

Vector operator*(const float a, const Vector& v)
{
	return Vector(v.x * a, v.y * a);
}
