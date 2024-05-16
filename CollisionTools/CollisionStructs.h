#pragma once
#include "Collision.h"


class CollisionPair
{
	struct Properties
	{
		float restitution;
		float df;
		float sf;
	};

	Body& bodyA;
	Body& bodyB;
	CollisionManifold manifold;
	Properties props;
	
public:
	static bool bIsFrictionEnable;

public:
	CollisionPair(Body& a, Body& b, const CollisionManifold& manifold);

	Body& GetBodyA();

	Body& GetBodyB();

	void InitProperties(float time, const Vector& gravity);

	void FixCollision();

	void PositionalCorrection();
};
