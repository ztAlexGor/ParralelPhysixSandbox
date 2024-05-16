#pragma once
#include "../Shapes/Shape.h"
#include <vector>
#include <string>

struct Material
{
	float restitution;
	float staticFriction;
	float dynamicFriction;

	static constexpr Material wood() { return Material(0.2f, 0.5f, 0.1f); }
	static constexpr Material stone() { return Material(0.1f, 0.5f, 0.2f); }
	static constexpr Material elastic() { return Material(1.f, 0.f, 0.f); }
	static constexpr Material plastic() { return Material(0.3f, 0.4f, 0.2f); }

	constexpr Material()
		: restitution(0.f), staticFriction(0.f), dynamicFriction(0.f)
	{}

	constexpr Material(float restitution, float staticFriction, float dynamicFriction)
		: restitution(restitution), staticFriction(staticFriction), dynamicFriction(dynamicFriction)
	{}
};


struct BodyInit
{
	Vector position;
	Vector velocity;
	float angle;
	float torque;
	float angularVelocity;
	bool bIsStatic = false;
	float gravityScale = 1.f;
	std::vector<Vector> forces;
	std::string adittionalData;

	//BodyInit()
	//{
	//	position = Vector(0.f, 0.f);
	//	velocity = Vector(0.f, 0.f);
	//	angle = 0.f;
	//	torque = 0.f;
	//	angularVelocity = 0.f;
	//	bIsStatic = false;
	//	float gravityScale = 1.f;
	//}
};


class Body
{
	Shape* m_shape;
	MassInfo m_massInfo;
	Material m_material;
	
	Vector m_position;
	Vector m_velocity;
	float m_angle;
	float m_torque;
	float m_angularVelocity;
	bool bIsStatic;
	float m_gravityScale;
	std::vector<Vector> m_forces;
	std::string m_adittionalData;

	Vector m_resultForce;
	bool m_bIsAwake;
	static float awakeEpsilon;

public:
	Body(const BodyInit& init);
	
	// getters
	const Shape* GetShape() const;

	const MassInfo& GetMassInfo() const;

	const Material& GetMaterial() const;

	Vector GetPosition() const;

	Vector GetVelocity() const;

	float GetAngleR() const;

	float GetAngleD() const;

	float GetTorque() const;

	float GetAngularVelocity() const;

	bool IsStatic() const;

	float GetGravityScale() const;

	const std::vector<Vector>& GetForces() const;

	std::string GetAdittionalData() const;

	Shape::EType GetShapeType() const;

	// setters
	void SetShape(Shape* shape, const Material& material, float density);

	void SetMaterial(const Material& material);

	void SetPosition(const Vector& position);

	void SetVelocity(const Vector& velocity);

	void SetAngleR(float angle);

	void SetAngleD(float angle);

	void SetTorque(float torque);

	void SetAngularVelocity(float angularVelocity);

	void SetStatic(bool arg);

	void SetGravityScale(float gravityScale);

	void SetForces(std::vector<Vector>& forces);

	void SetAdittionalData(const std::string& data);

	// main functions
	void ClearForces();

	void AddForce(const Vector& force);

	void ApplyForces(float time, const Vector& gravity);

	void CalculatePosition(float time);

	void ApplyImpulse(const Vector& impulse, const Vector& contactVector);

	void ApplyImpulse(const Vector& impulse);

	bool IsPointInShape(const Vector& point)const;
};

