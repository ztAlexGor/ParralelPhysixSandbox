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
        //застосовуємо усі додаткові сили (наприклад, силу тяжіння) за час = time / 2.f
        for (Body& body : m_bodies)
        {
            body.ApplyForces(time / 2.f, m_gravity);
        }

        // find all collision pairs
        FindCollisions();
        //ParallelFindCollisions();
        // fix collisions
        FixCollisions(time, iterNum);

        //позиційна корекція
        for (std::shared_ptr<CollisionPair>& collision : collisions)
        {
            collision->PositionalCorrection();
        }

        //оновлюємо позицію тіла за певний проміжок часу
        for (Body& body : m_bodies)
        {
            body.CalculatePosition(time);
        }

        // вдруге застосовуємо усі сили за час = time / 2.f
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

        //очищуємо список колізій
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

            // статичні тіла не взаємодіють один із одним
            if (a.IsStatic() && b.IsStatic())continue;

            // широка фаза перевірки
            if (!Collision::BroadPhase(a, b))continue;

            // перевіряємо на наявність колізії для пари (a, b) та зберігаємо
            // інформацію про це до об'єкту CollisionManifold
            CollisionManifold manifold = Collision::CheckCollision(a, b);

            // якщо інсують точки перетину, значить колізія відбулась
            if (manifold.crossPointsNumber)
            {
                collisions.push_back(std::make_shared<CollisionPair>(CollisionPair(a, b, manifold)));
            }
        }
    }

    //підраховуємо загальну кількість колізій
    //numOfColl += collisions.size();
}

void World::FixCollisions(float time, size_t iterNum)
{
    // обраховуємо коєфіцієнти тертя та відновлення
    for (std::shared_ptr<CollisionPair>& collision : collisions)
    {
        collision->InitProperties(time, m_gravity);
    }

    //вирішуємо колізію шляхом надання імпульсу
    //повторюємо дію iterNum кількість разів
    for (size_t j = 0; j < 1; ++j)
    {
        for (std::shared_ptr<CollisionPair>& collision : collisions)
        {
            collision->FixCollision();
        }
    }
}


void World::ParallelFindCollisions() {
    //створюємо масив потоків
    std::vector<std::thread*> threadPool;

    //створюємо буфер для кожного потоку
    std::vector<std::vector<std::shared_ptr<CollisionPair>>> threadBuffers;
    for (int i = 0; i < threadCount; i++) {
        threadBuffers.push_back(std::vector<std::shared_ptr<CollisionPair>>());
    }

    //отримуємо список усіх мошжливих колізій
    std::vector<std::pair<Body&, Body&>>* allPairs = GetAllCollisionPairs();

    int size = allPairs->size();
    int startPoint = 0;

    //розподіляємо задачі між потоками
    for (int i = 0; i < threadCount; i++) {
        int endPoint = startPoint + size / threadCount + ((size % threadCount / (i + 1)) ? 1 : 0);

        //запускаємо потоки
        threadPool.push_back(new std::thread([&](int sp, int ep, std::vector<std::shared_ptr<CollisionPair>>& buf) {

            //поток оброблює лише свій інтервал задач
            for (int c = sp; c < ep; c++) {

                Body& a = (*allPairs)[c].first;
                Body& b = (*allPairs)[c].second;

                //викликаємо алгоритм пошуку колізії
                CollisionManifold manifold = Collision::CheckCollision(a, b);

                // якщо інсують точки перетину, значить колізія відбулась
                if (manifold.crossPointsNumber)
                {
                    //collisions.push_back();
                    buf.push_back(std::make_shared<CollisionPair>(CollisionPair(a, b, manifold)));
                }
            }
            //використовуємо std::ref() для передачі параметра за посиланням
            }, startPoint, endPoint, std::ref(threadBuffers[i])));
        startPoint = endPoint;
    }

    //очікіємо завершення роботи кожного потоку
    for (std::thread* t : threadPool) {
        t->join();
    }

    //усі знайдені колізії додаємо до списку колізій
    for (auto b : threadBuffers) {
        for (auto c : b) {
            if (c)collisions.push_back(c);
        }
    }

    //підраховуємо загальну кількість колізій
    //numOfColl += collisions.size();

    //очищуємо пам'ять
    delete allPairs;
    threadBuffers.clear();
    threadPool.clear();
}


std::vector<std::pair<Body&, Body&>>* World::GetAllCollisionPairs() {

    //створюємо вектор для збереження усіх можливих пар
    std::vector<std::pair<Body&, Body&>>* allPairs = new std::vector< std::pair<Body&, Body&>>;

    //перебираєто усі варіанти
    for (size_t i = 0; i < m_bodies.size() - 1; ++i)
    {
        Body& a = m_bodies[i];

        for (size_t j = i + 1; j < m_bodies.size(); ++j)
        {
            Body& b = m_bodies[j];

            // статичні тіла не взаємодіють один із одним
            if (a.IsStatic() && b.IsStatic())continue;

            // широка фаза перевірки
            if (!Collision::BroadPhase(a, b))continue;

            // перевіряємо на наявність колізії для пари (a, b) та зберігаємо
            // інформацію про це до об'єкту CollisionManifold
            allPairs->push_back(std::make_pair(std::ref(a), std::ref(b)));
        }
    }
    return allPairs;
}