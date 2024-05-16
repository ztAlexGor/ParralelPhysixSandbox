#pragma once
#include "Body.h"
#include "../CollisionTools/CollisionStructs.h"
#include <vector>
#include <memory>


class World
{
	Vector m_gravity;
	std::vector<Body>m_bodies;
	std::vector<Vector> m_forces;
	bool bIsFrictionEnable;

	std::vector<std::shared_ptr<CollisionPair>> collisions;

	static const int minIterNum = 1;
	static const int maxIterNum = 64;
	static const int threadCount = 2;
public:
	World();

	void SetGravity(const Vector& gravity);

	void SetFrictionEnable(bool arg);

	void AddBody(Body& body);

	void DeleteAllBodies();

	void AddForce(const Vector& force);

	void SetForces(std::vector<Vector> forces);

	void ClearForces();

	Vector GetGravity() const;

	bool IsFrictionEnable() const;

	std::vector<Body>& GetBodies();

	const std::vector<Body>& GetBodies() const;

	std::vector<Vector>& GetForces();

	const std::vector<Vector>& GetForces() const;


	// main engine function
	void Step(float time, size_t iterNum);

private:
	void FindCollisions();

	void ParallelFindCollisions();

	std::vector<std::pair<Body&, Body&>>* GetAllCollisionPairs();

	void FixCollisions(float time, size_t iterNum);
};

