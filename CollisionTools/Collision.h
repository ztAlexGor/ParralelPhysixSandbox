#pragma once
#include <memory>

#include "../Shapes/Circle.h"
#include "../Shapes/Polygon.h"
#include "../Engine/Body.h"


struct CollisionManifold
{
	static constexpr size_t MAX_SIZE = 2;

	float depth;
	Vector normal;
	Vector crossPoint[MAX_SIZE];
	int crossPointsNumber;

	CollisionManifold();
};



class Collision
{
	static CollisionManifold(* JumpTable[static_cast<int>(Shape::EType::size)][static_cast<int>(Shape::EType::size)])(const Body& bodyA, const Body& bodyB);

public:
	static CollisionManifold CheckCollision(const Body& bodyA, const Body& bodyB);

	static bool BroadPhase(const Body& bodyA, const Body& bodyB);

private:
	static CollisionManifold CircleWithCircle(const Body& bodyA, const Body& bodyB);

	static CollisionManifold CircleWithPolygon(const Body& bodyA, const Body& bodyB);

	static CollisionManifold PolygonWithCircle(const Body& bodyA, const Body& bodyB);

	static CollisionManifold PolygonWithPolygon(const Body& bodyA, const Body& bodyB);

	static float FindAxisLeastPenetration(int* faceIndex, const Polygon* polygonA, const Polygon* polygonB, const Body* bodyA, const Body* bodyB);

	static void FindIncidentFace(Vector* v, const Polygon* RefPoly, const Polygon* IncPoly, const Body* Ab, const Body* Bb, Vector referenceNormal);

	static int Clip(const Vector& n, float c, Vector* face);
};

