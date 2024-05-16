#include "AABB.h"


AABB& AABB::operator=(const AABB& other)
{
	min = other.min;
	max = other.max;
	return *this;
}

AABB AABB::operator+(const Vector& vector) const
{
	AABB copy = *this;
	return copy += vector;
}

AABB& AABB::operator+=(const Vector& vector)
{
	max += vector;
	min += vector;
	return *this;
}

bool AABB::IsIntersect(const AABB& a, const AABB& b)
{
	if (a.max.x < b.min.x || a.max.y < b.min.y ||
		b.max.x < a.min.x || b.max.y < a.min.y) return false;
	return true;
}

