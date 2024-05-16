#pragma once
#include "../MathUtils/Vector.h"

struct AABB
{
	Vector min;
	Vector max;

	AABB(const Vector& leftBottom, const Vector& rightTop): min(leftBottom), max(rightTop) {}

	AABB(): AABB(Vector(0.f, 0.f), Vector(0.f, 0.f)) {}

	AABB(const AABB& other) : min(other.min), max(other.max) {}

	AABB& operator=(const AABB& other);

	AABB operator+(const Vector& vector) const;

	AABB& operator+=(const Vector& vec);

	static bool IsIntersect(const AABB& box1, const AABB& box2);
};

