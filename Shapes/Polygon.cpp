#include "Polygon.h"
#include <stdexcept>
#include <numbers>
#include <algorithm>
#include <iterator>


Polygon::Polygon(std::vector<Vector> vertices) : Shape(EType::polygon), m_vertices(std::move(vertices))/*, centroid(Vector(0.f, 0.f))*/
{
    if (m_vertices.size() < 3)throw std::logic_error("List of polygon vertices is incorrect");

    InitEdgesAndNormals();

    // calculate area and mass center
    InitializeArea();
    // InitializeCentroid();

    // aling with centroid
    Vector centroid = GetCentroid();

    for (Vector& vertex : m_vertices) {
        vertex -= centroid;
    }

    InitAABB();
}


Polygon::Polygon(float width, float height) : Shape(EType::polygon)
{
    if (width <= 0 || height <= 0)throw std::logic_error("Incorrect rectangle size");

    float halfW = width / 2.f;
    float halfH = height / 2.f;


    // calculate vertices
    m_vertices.push_back(Vector(-halfW, -halfH));
    m_vertices.push_back(Vector( halfW, -halfH));
    m_vertices.push_back(Vector( halfW,  halfH));
    m_vertices.push_back(Vector(-halfW,  halfH));

    InitEdgesAndNormals();
    
    // calculate area and AABB
    m_area = width * height;

    aabb = AABB(Vector(-halfW, -halfH), Vector(halfW, halfH));
}


Polygon::Polygon(Polygon& other): Polygon(other.m_vertices) {}


const std::vector<Vector>& Polygon::GetVertices() const
{
    return m_vertices;
}


const std::vector<Vector>& Polygon::GetEdges() const
{
    return m_edges;
}


const std::vector<Vector>& Polygon::GetNormals() const
{
    return m_normals;
}


MassInfo Polygon::ComputeMass(float density) const
{
    float inertiaCoef = 0.0f;

    for (int i = 0; i < m_vertices.size(); ++i)
    {
        const Vector& p1 = m_vertices[i];
        const Vector& p2 = m_vertices[i + 1 < m_vertices.size() ? i + 1 : 0];

        float intx2 = p1.x * p1.x + p2.x * p1.x + p2.x * p2.x;
        float inty2 = p1.y * p1.y + p2.y * p1.y + p2.y * p2.y;
        inertiaCoef += (Vector::CrossProduct(p1, p2)) / 12.0f * (intx2 + inty2);
    }

    float mass = m_area * density;
    float inertia = inertiaCoef * density;
    
    MassInfo massInfo{ .mass{mass}, .invMass{1.f / mass}, .inertia{inertia}, .invInertia{1.f / inertia} };
    return massInfo;
}


Shape* Polygon::Clone() const
{
    return new Polygon(std::vector<Vector>(m_vertices));
}


void Polygon::Rotate(float angle)
{
    matrix.Set(angle);
    ComputeAABB();
}


bool Polygon::IsPointInShape(const Vector point) const
{
    for (int i = 0; i < m_vertices.size(); ++i)
    {
        if (Vector::DotProduct(m_normals[i], point - m_vertices[i]) > 0)
        {
            return false;
        }
    }
    return true;
}


Vector Polygon::GetSupportPoint(const Vector& normal) const
{
    float bestProjection = -INFINITY;
    Vector bestVertex;

    //для кожної вершини многокутника обислюємо її відстань у заданому напрямку
    for (const Vector& vertex : m_vertices)
    {
        float projection = Vector::DotProduct(vertex, normal);

        //запам'ятовуємо найвіддаленішу вершину
        if (projection > bestProjection)
        {
            bestProjection = projection;
            bestVertex = vertex;
        }
    }
    return bestVertex;
}


Vector Polygon::GetCentroid() const
{
    Vector centroid(0.f, 0.f);
    for (int i = 0; i < m_vertices.size(); ++i)
    {
        int v1 = i;
        int v2 = i + 1 < m_vertices.size() ? i + 1 : 0;

        centroid += (m_vertices[v1] + m_vertices[v2]) * Vector::CrossProduct(m_vertices[v1], m_vertices[v2]);
    }

    return centroid /= 6 * m_area;
}


//void Polygon::InitializeCentroid()
//{
//    for (int i = 0; i < vertices.size() - 1; ++i)
//    {
//        centroid.x += (vertices[i].x + vertices[i + 1].x) * (vertices[i].x * vertices[i + 1].y - vertices[i - 1].x * vertices[i].y);
//        centroid.y += (vertices[i].y + vertices[i + 1].y) * (vertices[i].x * vertices[i + 1].y - vertices[i - 1].x * vertices[i].y);
//    }
//
//    centroid /= 6 * area;
//}


void Polygon::InitAABB()
{
    Vector min( INFINITY,  INFINITY);
    Vector max(-INFINITY, -INFINITY);

    for (const Vector& dot : m_vertices) {
        if (min.x > dot.x)min.x = dot.x;
        if (min.y > dot.y)min.y = dot.y;
        if (max.x < dot.x)max.x = dot.x;
        if (max.y < dot.y)max.y = dot.y;
    }
    aabb = AABB(min, max);
}


void Polygon::InitializeArea()
{
    m_area = 0.0f;

    // Calculate value of shoelace formula
    size_t j = m_vertices.size() - 1;
    for (size_t i = 0; i < m_vertices.size(); ++i)
    {
        m_area += (m_vertices[j].x + m_vertices[i].x) * (m_vertices[j].y - m_vertices[i].y);

        j = i;
    }
    m_area /= -2;
}

void Polygon::InitEdgesAndNormals()
{
    // calculate edges
    for (int i = 0; i < m_vertices.size() - 1; ++i)
    {
        m_edges.emplace_back(m_vertices[i + 1] - m_vertices[i]);
    }
    m_edges.emplace_back(m_vertices[0] - m_vertices[m_vertices.size() - 1]);

    // calculate normals
    std::transform(m_edges.begin(), m_edges.end(), back_inserter(m_normals),
        [](const Vector& edge) {return Vector(edge.y, -edge.x).Normalize(); });
}

void Polygon::ComputeAABB()
{
    Vector min( INFINITY,  INFINITY);
    Vector max(-INFINITY, -INFINITY);

    for (const Vector& vertex : m_vertices) {
        Vector dot = matrix * vertex;

        if (min.x > dot.x)min.x = dot.x;
        if (min.y > dot.y)min.y = dot.y;
        if (max.x < dot.x)max.x = dot.x;
        if (max.y < dot.y)max.y = dot.y;
    }
    aabb = AABB(min, max);
}
