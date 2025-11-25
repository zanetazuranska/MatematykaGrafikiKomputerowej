#pragma once
#include "Vector.h"
#include <optional>
#include <vector>

// Line (parametric form): P(t) = origin + direction * t
class Line {
public:
    Vector origin;
    Vector direction; // Normalized

    Line();
    Line(const Vector& origin, const Vector& direction);
    Line(const Vector& point1, const Vector& point2, bool fromTwoPoints);
    
    static Line fromTwoPoints(const Vector& point1, const Vector& point2);
    
    Vector getPoint(float t) const;
    
    bool contains(const Vector& point, float epsilon = 1e-5f) const;
};

// Ray - Line with (t >= 0)
class Ray {
public:
    Vector origin;
    Vector direction; // Normalized

    Ray();
    Ray(const Vector& origin, const Vector& direction);
    Ray(const Vector& point1, const Vector& point2, bool fromTwoPoints);
    
    static Ray fromTwoPoints(const Vector& point1, const Vector& point2);
    
    Vector getPoint(float t) const;
    
    bool contains(const Vector& point, float epsilon = 1e-5f) const;
};

// LineSegment - line segment between two points (t in [0, 1])
class LineSegment {
public:
    Vector start;
    Vector end;

    LineSegment();
    LineSegment(const Vector& start, const Vector& end);
    
    // Get point at parameter t (t in [0, 1], where 0=start, 1=end)
    Vector getPoint(float t) const;
    // Get the direction vector (not normalized)
    Vector getDirection() const;
    float length() const;
    
    bool contains(const Vector& point, float epsilon = 1e-5f) const;
};

// Plane - ax + by + cz + d = 0
// Plane - normal * (P - point) = 0
class Plane {
public:
    // (a, b, c) normalized
    Vector normal;
    // Distance from origin along normal
    float d;

    Plane();
    Plane(const Vector& normal, float d);
    Plane(const Vector& normal, const Vector& point);
    Plane(const Vector& p1, const Vector& p2, const Vector& p3); // 3 points
    
    // Signed distance from point to plane
    float distanceToPoint(const Vector& point) const;
    
    bool contains(const Vector& point, float epsilon = 1e-5f) const;
};

// Sphere - center and radius
class Sphere {
public:
    Vector center;
    float radius;

    Sphere();
    Sphere(const Vector& center, float radius);
    
    bool contains(const Vector& point, float epsilon = 1e-5f) const;
    
    // Get distance from point to sphere surface (negative if inside)
    float distanceToPoint(const Vector& point) const;
};

struct LineLineIntersection {
    bool intersects;
    Vector point;          // Intersection point (if exists)
    float t1, t2;          // Parameters on line1 and line2
    bool areParallel;
    bool areCoincident;
    float distance;        // Closest distance between skew lines
    Vector closestPoint1;  // Closest point on line1 (for skew lines)
    Vector closestPoint2;  // Closest point on line2 (for skew lines)
};

struct LinePlaneIntersection {
    bool intersects;
    Vector point;         // Intersection point
    float t;              // Parameter on line
    bool lineInPlane;     // Line lies entirely in plane
    bool isParallel;      // Line parallel to plane but not in it
};

struct LineSphereIntersection {
    bool intersects;
    int numIntersections; // 0, 1, or 2
    Vector point1;        // First intersection point
    Vector point2;        // Second intersection point
    float t1, t2;         // Parameters on line
};

struct LineCircleIntersection {
    bool intersects;
    int numIntersections; // 0, 1, or 2
    Vector point1;        // First intersection point
    Vector point2;        // Second intersection point
    float t1, t2;         // Parameters on line
};

struct PlanePlaneIntersection {
    bool intersects;      // True if planes intersect
    Line line;            // Line of intersection
    bool areParallel;     // True if planes are parallel
    bool areCoincident;   // True if planes are the same
    float distance;       // Distance between parallel planes
};

struct LineSegmentIntersection {
    bool intersects;      // True if segments intersect
    Vector point;         // Intersection point
    float t1, t2;         // Parameters on segment1 and segment2 (in [0,1])
    bool areParallel;     // True if segments are parallel
    bool areCollinear;    // True if segments are on the same line
    bool overlap;         // True if collinear segments overlap
    Vector overlapStart;  // Start of overlap (if overlap=true)
    Vector overlapEnd;    // End of overlap (if overlap=true)
};

std::ostream& operator<<(std::ostream& os, const LineLineIntersection& result);
std::ostream& operator<<(std::ostream& os, const LinePlaneIntersection& result);
std::ostream& operator<<(std::ostream& os, const LineSphereIntersection& result);
std::ostream& operator<<(std::ostream& os, const LineCircleIntersection& result);
std::ostream& operator<<(std::ostream& os, const PlanePlaneIntersection& result);
std::ostream& operator<<(std::ostream& os, const LineSegmentIntersection& result);
std::ostream& operator<<(std::ostream& os, const Line& line);
std::ostream& operator<<(std::ostream& os, const LineSegment& segment);
std::ostream& operator<<(std::ostream& os, const Plane& plane);
std::ostream& operator<<(std::ostream& os, const Sphere& sphere);

class Geometry {
public:
    static LineLineIntersection intersect(const Line& line1, const Line& line2, float epsilon = 1e-5f);
    static LineLineIntersection intersect(const Ray& ray1, const Ray& ray2, float epsilon = 1e-5f);
    
    static LinePlaneIntersection intersect(const Line& line, const Plane& plane, float epsilon = 1e-5f);
    static LinePlaneIntersection intersect(const Ray& ray, const Plane& plane, float epsilon = 1e-5f);
    
    static LineSphereIntersection intersect(const Line& line, const Sphere& sphere, float epsilon = 1e-5f);
    static LineSphereIntersection intersect(const Ray& ray, const Sphere& sphere, float epsilon = 1e-5f);
    
    static PlanePlaneIntersection intersect(const Plane& plane1, const Plane& plane2, float epsilon = 1e-5f);
    
    static LineSegmentIntersection intersect(const LineSegment& seg1, const LineSegment& seg2, float epsilon = 1e-5f);
    
    // Angle in radians [0, PI]
    static float angleBetween(const Line& line1, const Line& line2);
    static float angleBetween(const Ray& ray1, const Ray& ray2);
    static float angleBetween(const Line& line, const Plane& plane);
    static float angleBetween(const Ray& ray, const Plane& plane);
    static float angleBetween(const Plane& plane1, const Plane& plane2);
    
    // Angle in degrees [0, 180]
    static float angleBetweenDegrees(const Line& line1, const Line& line2);
    static float angleBetweenDegrees(const Ray& ray1, const Ray& ray2);
    static float angleBetweenDegrees(const Line& line, const Plane& plane);
    static float angleBetweenDegrees(const Ray& ray, const Plane& plane);
    static float angleBetweenDegrees(const Plane& plane1, const Plane& plane2);
    
private:
    static constexpr float PI = 3.14159265358979323846f;
    static float radToDeg(float rad) { return rad * 180.0f / PI; }
    static float degToRad(float deg) { return deg * PI / 180.0f; }
};
