#pragma once
#include "../Optimisation/AABB.h"
#include "../MathUtils/Matrix.h"


struct MassInfo
{
	float mass;
	float invMass;
	float inertia;
	float invInertia;
};


class Shape
{
public:
	enum class EType
	{
		circle,
		polygon,
		size
	};

protected:
	AABB aabb;
	EType type;
	Matrix matrix;

public:
	Shape(EType type);

	AABB GetAABB() const;

	const AABB& GetAABB();

	EType GetType() const;

	Matrix GetMatrix() const;

	virtual MassInfo ComputeMass(float density) const = 0;

	virtual Shape* Clone() const = 0;

	virtual void Rotate(float angle) = 0;

	//virtual ~Shape() = 0;

	virtual bool IsPointInShape(const Vector point) const = 0;

protected:
	virtual void InitAABB() = 0;
};

