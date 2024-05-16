#include "CollisionStructs.h"

bool CollisionPair::bIsFrictionEnable = true;

CollisionPair::CollisionPair(Body& a, Body& b, const CollisionManifold& manifold) :
	bodyA(a), bodyB(b), manifold(manifold)
{
	props.restitution = 0.f;
	props.df = 0.f;
	props.sf = 0.f;
}

Body& CollisionPair::GetBodyA()
{
    return bodyA;
}

Body& CollisionPair::GetBodyB()
{
    return bodyB;
}


void CollisionPair::InitProperties(float time, const Vector& gravity)
{
    props.restitution = std::min(bodyA.GetMaterial().restitution, bodyB.GetMaterial().restitution);

    //обчислюємо коєф. тертя спокою та ковзання
    props.sf = sqrt(bodyA.GetMaterial().staticFriction * bodyB.GetMaterial().staticFriction);
    props.df = sqrt(bodyA.GetMaterial().dynamicFriction * bodyB.GetMaterial().dynamicFriction);

    for (int i = 0; i < manifold.crossPointsNumber; ++i) {
        Vector ra = manifold.crossPoint[i] - bodyA.GetPosition();
        Vector rb = manifold.crossPoint[i] - bodyB.GetPosition();

        Vector rv = bodyB.GetVelocity() + Vector::CrossProduct(bodyB.GetAngularVelocity(), rb) -
                    bodyA.GetVelocity() - Vector::CrossProduct(bodyA.GetAngularVelocity(), ra);

        //якщо на тіла не діє жодна сила окрім гравітації,
        //то не потрібно використовувати коєфіцієнт відшкодування
        if (rv.SqLength() < (gravity * time).SqLength() + 0.0001f)
            props.restitution = 0.f;

        // TODO
        //if (bodyA.GetGravityScale() != 1.f || bodyB.GetGravityScale() != 1.f)throw "gravity scale doesn't considered";
    }
}



void CollisionPair::FixCollision()
{
    //якщо обидва тіла статичні, то взаємодія не відбувається
    if (bodyA.IsStatic() && bodyB.IsStatic()) {
        bodyA.SetVelocity(Vector(0.f, 0.f));
        bodyB.SetVelocity(Vector(0.f, 0.f));
        return;
    }

    const Vector velA = bodyA.GetVelocity();
    const Vector velB = bodyB.GetVelocity();

    const float angVelA = bodyA.GetAngularVelocity();
    const float angVelB = bodyB.GetAngularVelocity();

    //для кожної точки перетину
    for (int i = 0; i < manifold.crossPointsNumber; ++i) {
        Vector ra = manifold.crossPoint[i] - bodyA.GetPosition();
        Vector rb = manifold.crossPoint[i] - bodyB.GetPosition();

        //шукаємо швидкість B відносно А
        Vector rv = velB + Vector::CrossProduct(angVelB, rb) -
                    velA - Vector::CrossProduct(angVelA, ra);


        //швидкість B відносно А вздовж нормалі зіткнення
        float contactVel = Vector::DotProduct(rv, manifold.normal);

        //якщо тіла рухаються у різних напрямках, то не потрібно нічого змінювати
        if (contactVel > 0)
        {
            return;
        }
        //else if (contactVel == 0)
        //{
        //    contactVel = manifold.depth;
        //}

        float raCrossN = Vector::CrossProduct(ra, manifold.normal);
        float rbCrossN = Vector::CrossProduct(rb, manifold.normal);

        float invMassSum = bodyA.GetMassInfo().invMass + raCrossN * raCrossN * bodyA.GetMassInfo().invInertia
                         + bodyB.GetMassInfo().invMass + rbCrossN * rbCrossN * bodyB.GetMassInfo().invInertia;

        //обчислюємо величину імпульсу
        float j = -(1.0f + props.restitution) * contactVel;
        j /= invMassSum;
        j /= (float)manifold.crossPointsNumber;

        //застосовуємо імпульс до тіла A та B
        Vector impulse = manifold.normal * j;
        bodyA.ApplyImpulse(-impulse, ra);
        bodyB.ApplyImpulse( impulse, rb);

        //враховуємо силу тертя
        if (CollisionPair::bIsFrictionEnable) {
            rv = velB + Vector::CrossProduct(angVelB, rb) -
                 velA - Vector::CrossProduct(angVelA, ra);

            //напрямок тангенціального імпульсу
            Vector t = rv - (manifold.normal * Vector::DotProduct(rv, manifold.normal));
            t.Normalize();

            //величина тангенціального імпульсу
            float jt = -Vector::DotProduct(rv, t);
            jt /= invMassSum;
            jt /= (float)manifold.crossPointsNumber;

            //якщо величина імпульсу дуже мала, то просто ігноруємо її
            if (abs(jt) < 0.0001f)continue;

            //в залежності від типу тертя імпульс буде різним
            Vector tangentImpulse;
            if (abs(jt) < j * props.sf)
                tangentImpulse = t * jt;
            else
                tangentImpulse = t * (-j) * props.df;

            //застосовуємо імпульс тертя
            bodyA.ApplyImpulse(-tangentImpulse, ra);
            bodyB.ApplyImpulse( tangentImpulse, rb);
        }
    }
}

void CollisionPair::PositionalCorrection()
{
    //const float k_slop = 0.03f; //допуст проникнення (0.01; 0.1)
    //const float percent = 0.3f; //відсоток проникнення, для корекції (0.2; 0.8)

    //float t = manifold.depth - k_slop;
    //if (t > 0) {
        //Vector correction = (t / (bodyA.GetMassInfo().invMass + bodyB.GetMassInfo().invMass)) * manifold.normal * percent;

        //if (!bodyA.IsStatic())bodyA.SetPosition(Vector(bodyA.GetPosition()) - correction * bodyA.GetMassInfo().invMass);
        //if (!bodyB.IsStatic())bodyB.SetPosition(Vector(bodyB.GetPosition()) + correction * bodyB.GetMassInfo().invMass);

        //if (!bodyA.IsStatic() && Vector::DotProduct(manifold.normal, bodyA.GetVelocity()) > 0) {
        //    float penetrationMultiplier = 1 + t / bodyA.GetMassInfo().invMass;
        //    Vector impulse = manifold.normal * penetrationMultiplier * percent;
        //    bodyA.ApplyImpulse(-2 * impulse);
        //}
        //if (!bodyB.IsStatic() && Vector::DotProduct(manifold.normal, bodyB.GetVelocity()) < 0) {
        //    float penetrationMultiplier = 1 + t / bodyB.GetMassInfo().invMass;
        //    Vector impulse = manifold.normal * penetrationMultiplier * percent;
        //    bodyB.ApplyImpulse(2*impulse);
        //}

        if (bodyA.IsStatic())
        {
            Vector correction = manifold.depth * manifold.normal;
            bodyB.SetPosition(bodyB.GetPosition() + correction);
        }
        else if (bodyB.IsStatic())
        {
            Vector correction = manifold.depth * manifold.normal;
            bodyA.SetPosition(bodyA.GetPosition() - correction);
        }
        else {
            Vector correction = (manifold.depth / (bodyA.GetMassInfo().invMass + bodyB.GetMassInfo().invMass)) * manifold.normal;
            bodyA.SetPosition(bodyA.GetPosition() - correction * bodyA.GetMassInfo().invMass);
            bodyB.SetPosition(bodyB.GetPosition() + correction * bodyB.GetMassInfo().invMass);
        }
    //}
}

