#include "Circle.h"
#include <numbers>

Circle::Circle(float radius) : Shape(EType::circle), m_radius(radius)
{
	InitAABB();
}


float Circle::GetRadius() const
{
	return m_radius;
}


void Circle::SetRadius(float radius)
{
	m_radius = radius;
	ComputeAABB();
}


void Circle::InitAABB()
{
	aabb = AABB(Vector(-m_radius, -m_radius), Vector(m_radius, m_radius));
}

void Circle::ComputeAABB()
{
	aabb.min.Set(-m_radius, -m_radius);
	aabb.max.Set( m_radius,  m_radius);
}


MassInfo Circle::ComputeMass(float density) const
{
	float mass = std::numbers::pi * m_radius * m_radius * density;
	float inertia = mass * m_radius * m_radius / 2.f;

	MassInfo massInfo{ .mass{mass}, .invMass{1.f / mass}, .inertia{inertia}, .invInertia{1.f / inertia} };
	return massInfo;
}


Shape* Circle::Clone() const
{
	return new Circle(*this);
}

bool Circle::IsPointInShape(const Vector point) const
{
	return point.SqLength() < m_radius * m_radius;
}