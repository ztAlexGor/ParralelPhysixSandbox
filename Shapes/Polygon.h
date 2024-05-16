#pragma once
#include "Shape.h"
#include <vector>

class Polygon : public Shape
{
	std::vector<Vector> m_vertices;
	std::vector<Vector> m_edges;
	std::vector<Vector> m_normals;
	// Vector centroid;
	float m_area;

public:
	Polygon(std::vector<Vector> vertices);

	Polygon(float width, float height);

	Polygon(Polygon& other);

	const std::vector<Vector>& GetVertices() const;

	const std::vector<Vector>& GetEdges() const;

	const std::vector<Vector>& GetNormals() const;

	virtual MassInfo ComputeMass(float density) const override;

	virtual Shape* Clone() const override;

	virtual void Rotate(float angle) override;

	virtual bool IsPointInShape(const Vector point) const override;

	Vector GetSupportPoint(const Vector& normal) const;
	
	Vector GetCentroid() const;

protected:
	virtual void InitAABB() override;

private:
	void InitializeArea();

	void InitEdgesAndNormals();

	void ComputeAABB();
	// void InitializeCentroid();
};

