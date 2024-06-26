#include "World.h"
#include <thread>

World::World()
{
    bIsFrictionEnable = true;
}

void World::SetGravity(const Vector& gravity)
{
	m_gravity = gravity;
}

void World::SetFrictionEnable(bool arg)
{
    bIsFrictionEnable = arg;
    CollisionPair::bIsFrictionEnable = arg;
}

void World::AddBody(Body& body)
{
	m_bodies.emplace_back(body);
}

void World::DeleteAllBodies()
{
	m_bodies.clear();
}

void World::AddForce(const Vector& force)
{
	m_forces.emplace_back(force);
}

void World::SetForces(std::vector<Vector> forces)
{
    m_forces = forces;
    //for (Vector& force : forces)
    //{
    //    m_forces.emplace_back(force);
    //}
}

void World::ClearForces()
{
	m_forces.clear();
}

Vector World::GetGravity() const
{
	return m_gravity;
}

bool World::IsFrictionEnable() const
{
    return bIsFrictionEnable;
}

std::vector<Body>& World::GetBodies()
{
    return m_bodies;
}

const std::vector<Body>& World::GetBodies() const
{
    return m_bodies;
}

std::vector<Vector>& World::GetForces()
{
    return m_forces;
}

const std::vector<Vector>& World::GetForces() const
{
    return m_forces;
}


void World::Step(float time, size_t iterNum)
{
    iterNum = iterNum < minIterNum ? minIterNum : iterNum;
    iterNum = iterNum > maxIterNum ? maxIterNum : iterNum;

    time /= (float)iterNum;

    for (int iteration = 0; iteration < iterNum; ++iteration) {
        //����������� �� �������� ���� (���������, ���� ������) �� ��� = time / 2.f
        for (Body& body : m_bodies)
        {
            body.ApplyForces(time / 2.f, m_gravity);
        }

        // find all collision pairs
        FindCollisions();
        //ParallelFindCollisions();
        // fix collisions
        FixCollisions(time, iterNum);

        //��������� ��������
        for (std::shared_ptr<CollisionPair>& collision : collisions)
        {
            collision->PositionalCorrection();
        }

        //��������� ������� ��� �� ������ ������� ����
        for (Body& body : m_bodies)
        {
            body.CalculatePosition(time);
        }

        // ������ ����������� �� ���� �� ��� = time / 2.f
        for (Body& body : m_bodies)
        {
            body.ApplyForces(time / 2.f, m_gravity);
        }

        /////////////////////////////////////////////step///////////////////////////////////////////////////////////
        //for (Body& body : m_bodies)
        //{
        //    if (body.GetAdittionalData() == "key" || body.GetAdittionalData() == "keyCollision")
        //        body.SetAdittionalData("key");
        //    else if (body.GetAdittionalData() == "" || body.GetAdittionalData() == "Collision") {
        //        body.SetAdittionalData("");
        //    }
        //    else {
        //        int r = 0;
        //    }
        //}
        //
        //for (std::shared_ptr<CollisionPair>& collision : collisions)
        //{
        //    collision->GetBodyA().SetAdittionalData((collision->GetBodyA().GetAdittionalData() == "key" || collision->GetBodyA().GetAdittionalData() == "keyCollision") ? "keyCollision" : "Collision");
        //    collision->GetBodyB().SetAdittionalData((collision->GetBodyB().GetAdittionalData() == "key" || collision->GetBodyB().GetAdittionalData() == "keyCollision") ? "keyCollision" : "Collision");
        //}
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //������� ������ �����
        collisions.clear();

        for (Body& body : m_bodies)
        {
            body.ClearForces();
            body.SetTorque(0.f);
        }
    }
}


void World::FindCollisions()
{
    for (size_t i = 0; i < m_bodies.size() - 1; ++i)
    {
        Body& a = m_bodies[i];

        for (size_t j = i + 1; j < m_bodies.size(); ++j)
        {
            Body& b = m_bodies[j];

            // �������� ��� �� ��������� ���� �� �����
            if (a.IsStatic() && b.IsStatic())continue;

            // ������ ���� ��������
            if (!Collision::BroadPhase(a, b))continue;

            // ���������� �� ��������� ���糿 ��� ���� (a, b) �� ��������
            // ���������� ��� �� �� ��'���� CollisionManifold
            CollisionManifold manifold = Collision::CheckCollision(a, b);

            // ���� ������� ����� ��������, ������� ����� ��������
            if (manifold.crossPointsNumber)
            {
                collisions.push_back(std::make_shared<CollisionPair>(CollisionPair(a, b, manifold)));
            }
        }
    }

    //���������� �������� ������� �����
    //numOfColl += collisions.size();
}

void World::FixCollisions(float time, size_t iterNum)
{
    // ���������� ���������� ����� �� ����������
    for (std::shared_ptr<CollisionPair>& collision : collisions)
    {
        collision->InitProperties(time, m_gravity);
    }

    //������� ����� ������ ������� ��������
    //���������� �� iterNum ������� ����
    for (size_t j = 0; j < 1; ++j)
    {
        for (std::shared_ptr<CollisionPair>& collision : collisions)
        {
            collision->FixCollision();
        }
    }
}


void World::ParallelFindCollisions() {
    //��������� ����� ������
    std::vector<std::thread*> threadPool;

    //��������� ����� ��� ������� ������
    std::vector<std::vector<std::shared_ptr<CollisionPair>>> threadBuffers;
    for (int i = 0; i < threadCount; i++) {
        threadBuffers.push_back(std::vector<std::shared_ptr<CollisionPair>>());
    }

    //�������� ������ ��� ��������� �����
    std::vector<std::pair<Body&, Body&>>* allPairs = GetAllCollisionPairs();

    int size = allPairs->size();
    int startPoint = 0;

    //����������� ������ �� ��������
    for (int i = 0; i < threadCount; i++) {
        int endPoint = startPoint + size / threadCount + ((size % threadCount / (i + 1)) ? 1 : 0);

        //��������� ������
        threadPool.push_back(new std::thread([&](int sp, int ep, std::vector<std::shared_ptr<CollisionPair>>& buf) {

            //����� �������� ���� ��� �������� �����
            for (int c = sp; c < ep; c++) {

                Body& a = (*allPairs)[c].first;
                Body& b = (*allPairs)[c].second;

                //��������� �������� ������ ���糿
                CollisionManifold manifold = Collision::CheckCollision(a, b);

                // ���� ������� ����� ��������, ������� ����� ��������
                if (manifold.crossPointsNumber)
                {
                    //collisions.push_back();
                    buf.push_back(std::make_shared<CollisionPair>(CollisionPair(a, b, manifold)));
                }
            }
            //������������� std::ref() ��� �������� ��������� �� ����������
            }, startPoint, endPoint, std::ref(threadBuffers[i])));
        startPoint = endPoint;
    }

    //���곺�� ���������� ������ ������� ������
    for (std::thread* t : threadPool) {
        t->join();
    }

    //�� �������� ���糿 ������ �� ������ �����
    for (auto b : threadBuffers) {
        for (auto c : b) {
            if (c)collisions.push_back(c);
        }
    }

    //���������� �������� ������� �����
    //numOfColl += collisions.size();

    //������� ���'���
    delete allPairs;
    threadBuffers.clear();
    threadPool.clear();
}


std::vector<std::pair<Body&, Body&>>* World::GetAllCollisionPairs() {

    //��������� ������ ��� ���������� ��� �������� ���
    std::vector<std::pair<Body&, Body&>>* allPairs = new std::vector< std::pair<Body&, Body&>>;

    //���������� �� �������
    for (size_t i = 0; i < m_bodies.size() - 1; ++i)
    {
        Body& a = m_bodies[i];

        for (size_t j = i + 1; j < m_bodies.size(); ++j)
        {
            Body& b = m_bodies[j];

            // �������� ��� �� ��������� ���� �� �����
            if (a.IsStatic() && b.IsStatic())continue;

            // ������ ���� ��������
            if (!Collision::BroadPhase(a, b))continue;

            // ���������� �� ��������� ���糿 ��� ���� (a, b) �� ��������
            // ���������� ��� �� �� ��'���� CollisionManifold
            allPairs->push_back(std::make_pair(std::ref(a), std::ref(b)));
        }
    }
    return allPairs;
}