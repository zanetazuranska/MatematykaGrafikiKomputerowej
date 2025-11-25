#include "Geometry.h"
#include <cmath>
#include <algorithm>
#include <iostream>

// Line Implementation
Line::Line() : origin(0, 0, 0), direction(1, 0, 0) {}

Line::Line(const Vector& origin, const Vector& direction) 
    : origin(origin), direction(direction) {
    this->direction.normalize();
}

Line::Line(const Vector& point1, const Vector& point2, bool fromTwoPoints) 
    : origin(point1) {
    direction = Vector(point1, point2);
    this->direction.normalize();
}

Line Line::fromTwoPoints(const Vector& point1, const Vector& point2) {
    return Line(point1, point2, true);
}

Vector Line::getPoint(float t) const {
    return origin + direction * t;
}

bool Line::contains(const Vector& point, float epsilon) const {
    Vector toPoint = Vector(origin, point);
    Vector cross = direction.cross(toPoint);
    return cross.length() < epsilon;
}

// Ray Implementation
Ray::Ray() : origin(0, 0, 0), direction(1, 0, 0) {}

Ray::Ray(const Vector& origin, const Vector& direction) 
    : origin(origin), direction(direction) {
    this->direction.normalize();
}

Ray::Ray(const Vector& point1, const Vector& point2, bool fromTwoPoints) 
    : origin(point1) {
    direction = Vector(point1, point2);
    this->direction.normalize();
}

Ray Ray::fromTwoPoints(const Vector& point1, const Vector& point2) {
    return Ray(point1, point2, true);
}

Vector Ray::getPoint(float t) const {
    if (t < 0) t = 0;
    return origin + direction * t;
}

bool Ray::contains(const Vector& point, float epsilon) const {
    Vector toPoint = Vector(origin, point);
    float t = direction.dotProduct(toPoint);
    if (t < -epsilon) return false; // t must be >= 0
    
    Vector cross = direction.cross(toPoint);
    return cross.length() < epsilon;
}

// LineSegment Implementation
LineSegment::LineSegment() : start(0, 0, 0), end(1, 0, 0) {}

LineSegment::LineSegment(const Vector& start, const Vector& end) 
    : start(start), end(end) {}

Vector LineSegment::getPoint(float t) const {
    return Vector(
        start.x + t * (end.x - start.x),
        start.y + t * (end.y - start.y),
        start.z + t * (end.z - start.z)
    );
}

Vector LineSegment::getDirection() const {
    return Vector(start, end);
}

float LineSegment::length() const {
    return getDirection().length();
}

bool LineSegment::contains(const Vector& point, float epsilon) const {
    Vector dir = getDirection();
    float len = dir.length();
    if (len < epsilon) return false;
    
    dir.normalize();
    Vector toPoint = Vector(start, point);
    
    // Check if point is on the line
    Vector cross = dir.cross(toPoint);
    if (cross.length() > epsilon) return false;
    
    // Check if point is within segment bounds
    float t = dir.dotProduct(toPoint);
    return t >= -epsilon && t <= len + epsilon;
}

// Plane Implementation
Plane::Plane() : normal(0, 1, 0), d(0) {}

Plane::Plane(const Vector& normal, float d) : normal(normal), d(d) {
    float length = this->normal.length();
    this->normal.normalize();
    this->d = d / length; // Scale d by length
}

Plane::Plane(const Vector& normal, const Vector& point) : normal(normal) {
    this->normal.normalize();
    // d = -normal * point
    d = -this->normal.dotProduct(point);
}

Plane::Plane(const Vector& p1, const Vector& p2, const Vector& p3) {
    // Create two vectors in the plane
    Vector v1(p1, p2);
    Vector v2(p1, p3);
    
    // Normal is cross product
    normal = v1.cross(v2);
    normal.normalize();
    
    // Calculate d using point p1
    d = -normal.dotProduct(p1);
}

float Plane::distanceToPoint(const Vector& point) const {
    return normal.dotProduct(point) + d;
}

bool Plane::contains(const Vector& point, float epsilon) const {
    return std::abs(distanceToPoint(point)) < epsilon;
}

// Sphere Implementation
Sphere::Sphere() : center(0, 0, 0), radius(1.0f) {}

Sphere::Sphere(const Vector& center, float radius) 
    : center(center), radius(radius) {}

bool Sphere::contains(const Vector& point, float epsilon) const {
    Vector toPoint(center, point);
    float dist = toPoint.length();
    return dist <= radius + epsilon;
}

float Sphere::distanceToPoint(const Vector& point) const {
    Vector toPoint(center, point);
    return toPoint.length() - radius;
}

// Geometry - Line-Line Intersection
LineLineIntersection Geometry::intersect(const Line& line1, const Line& line2, float epsilon) {
    LineLineIntersection result;
    result.intersects = false;
    result.areParallel = false;
    result.areCoincident = false;
    result.distance = 0.0f;
    
    Vector d1 = line1.direction;
    Vector d2 = line2.direction;
    Vector w = Vector(line2.origin, line1.origin); // w = P1 - P2
    
    // Check if directions are parallel
    Vector cross_d1_d2 = d1.cross(d2);
    float cross_length = cross_d1_d2.length();
    
    if (cross_length < epsilon) {
        // Lines are parallel
        result.areParallel = true;
        
        // Check if they are coincident
        Vector cross_w_d1 = w.cross(d1);
        if (cross_w_d1.length() < epsilon) {
            result.areCoincident = true;
            result.intersects = true;
            result.point = line1.origin;
            result.t1 = 0.0f;
            result.t2 = d1.dotProduct(w);
        } else {
            // Parallel but not coincident - calculate distance
            result.distance = cross_w_d1.length() / d1.length();
        }
        return result;
    }
    
    // Lines are not parallel - check if they intersect or are skew
    // Using parametric equations: P1 + t1 * d1 = P2 + t2 * d2 
    // t1 * d1 - t2 * d2 = P2 - P1 = -w
    // t1 = [(P2 - P1) x d2] * (d1 x d2) / |d1 x d2|^2
    // t2 = [(P2 - P1) x d1] * (d1 x d2) / |d1 x d2|^2
    
    Vector negW(line1.origin, line2.origin); // P2 - P1
    Vector cross_negW_d2 = negW.cross(d2);
    float numerator = cross_negW_d2.dotProduct(cross_d1_d2);
    float denominator = cross_length * cross_length;
    
    result.t1 = numerator / denominator;
    
    // t2
    Vector cross_negW_d1 = negW.cross(d1);
    numerator = cross_negW_d1.dotProduct(cross_d1_d2);
    result.t2 = numerator / denominator;
    
    // Get points on both lines
    Vector point1 = line1.getPoint(result.t1);
    Vector point2 = line2.getPoint(result.t2);
    
    // Check if points are the same (lines intersect)
    Vector diff(point1, point2);
    result.distance = diff.length();
    
    if (result.distance < epsilon) {
        // Lines intersect
        result.intersects = true;
        result.point = point1;
    } else {
        // Lines are skew
        result.closestPoint1 = point1;
        result.closestPoint2 = point2;
    }
    
    return result;
}

LineLineIntersection Geometry::intersect(const Ray& ray1, const Ray& ray2, float epsilon) {
    // Convert rays to lines and find intersection
    Line line1(ray1.origin, ray1.direction);
    Line line2(ray2.origin, ray2.direction);
    
    LineLineIntersection result = intersect(line1, line2, epsilon);
    
    // Check if intersection point is valid for rays (t >= 0 for both)
    if (result.intersects && !result.areCoincident) {
        if (result.t1 < -epsilon || result.t2 < -epsilon) {
            result.intersects = false;
        }
    }
    
    return result;
}

// Geometry - Line-Plane Intersection
LinePlaneIntersection Geometry::intersect(const Line& line, const Plane& plane, float epsilon) {
    LinePlaneIntersection result;
    result.intersects = false;
    result.lineInPlane = false;
    result.isParallel = false;
    
    // Denominator = normal * direction
    float denom = plane.normal.dotProduct(line.direction);
    
    if (std::abs(denom) < epsilon) {
        // Line is parallel to plane
        result.isParallel = true;
        
        // Check if line lies in plane
        float dist = plane.distanceToPoint(line.origin);
        if (std::abs(dist) < epsilon) {
            result.lineInPlane = true;
            result.intersects = true;
            result.point = line.origin;
            result.t = 0.0f;
        }
        return result;
    }
    
    // t = -(normal * origin + d) / (normal * direction)
    float numerator = -(plane.normal.dotProduct(line.origin) + plane.d);
    result.t = numerator / denom;
    
    // Calculate intersection point
    result.point = line.getPoint(result.t);
    result.intersects = true;
    
    return result;
}

LinePlaneIntersection Geometry::intersect(const Ray& ray, const Plane& plane, float epsilon) {
    Line line(ray.origin, ray.direction);
    LinePlaneIntersection result = intersect(line, plane, epsilon);
    
    // Check if intersection is valid for ray (t >= 0)
    if (result.intersects && !result.lineInPlane) {
        if (result.t < -epsilon) {
            result.intersects = false;
        }
    }
    
    return result;
}

// Geometry - Line-Sphere Intersection
LineSphereIntersection Geometry::intersect(const Line& line, const Sphere& sphere, float epsilon) {
    LineSphereIntersection result;
    result.intersects = false;
    result.numIntersections = 0;
    
    // Vector from sphere center to line origin
    Vector oc(sphere.center, line.origin);
    
    // at^2 + bt + c = 0
    // Ray equation: P(t) = origin + direction * t
    float a = line.direction.dotProduct(line.direction);
    float b = 2.0f * line.direction.dotProduct(oc);
    float c = oc.dotProduct(oc) - sphere.radius * sphere.radius;
    
    float discriminant = b * b - 4 * a * c;
    
    if (discriminant < -epsilon) {
        // No intersection
        return result;
    }
    
    if (discriminant < epsilon) {
        // One intersection (tangent)
        result.numIntersections = 1;
        result.intersects = true;
        result.t1 = -b / (2.0f * a);
        result.t2 = result.t1;
        result.point1 = line.getPoint(result.t1);
        result.point2 = result.point1;
    } else {
        // Two intersections
        result.numIntersections = 2;
        result.intersects = true;
        float sqrtDisc = std::sqrt(discriminant);
        result.t1 = (-b - sqrtDisc) / (2.0f * a);
        result.t2 = (-b + sqrtDisc) / (2.0f * a);
        result.point1 = line.getPoint(result.t1);
        result.point2 = line.getPoint(result.t2);
    }
    
    return result;
}

LineSphereIntersection Geometry::intersect(const Ray& ray, const Sphere& sphere, float epsilon) {
    Line line(ray.origin, ray.direction);
    LineSphereIntersection result = intersect(line, sphere, epsilon);
    
    if (result.intersects) {
        // If t <= 0 discard intersections
        if (result.t1 < -epsilon && result.t2 < -epsilon) {
            // both t <= 0
            result.intersects = false;
            result.numIntersections = 0;
        } else if (result.t1 < -epsilon) {
            // 1st t <= 0
            result.t1 = result.t2;
            result.point1 = result.point2;
            result.numIntersections = 1;
        } else if (result.t2 < -epsilon) {
            // 2nd t <= 0
            result.t2 = result.t1;
            result.point2 = result.point1;
            result.numIntersections = 1;
        }
    }
    
    return result;
}

PlanePlaneIntersection Geometry::intersect(const Plane &plane1, const Plane &plane2, float epsilon)
{
    PlanePlaneIntersection result;
    result.intersects = false;
    result.areParallel = false;
    result.areCoincident = false;
    result.distance = 0.0f;
    
    // Check if normals are parallel
    Vector cross = plane1.normal.cross(plane2.normal);
    float crossLen = cross.length();
    
    // Cross near zero = planes are parallel or coincident
    if (crossLen < epsilon) {
        result.areParallel = true;
        
        float dotProduct = plane1.normal.dotProduct(plane2.normal);
        
        // Planes coincident = d1 +- d2 ~ 0 (depending on normal directions)
        if (dotProduct > 0) {
            // Normals point in same directions
            if (std::abs(plane1.d - plane2.d) < epsilon) {
                result.areCoincident = true;
                result.distance = 0.0f;
            } else {
                result.distance = std::abs(plane1.d - plane2.d);
            }
        } else {
            // Normals point in opposite directions
            if (std::abs(plane1.d + plane2.d) < epsilon) {
                result.areCoincident = true;
                result.distance = 0.0f;
            } else {
                result.distance = std::abs(plane1.d + plane2.d);
            }
        }
        
        return result;
    }
    
    // Planes intersect in a line
    result.intersects = true;
    
    Vector direction = cross;
    direction.normalize();
    
    // Find a point on the intersection line
    // plane1: n1 * P + d1 = 0
    // plane2: n2 * P + d2 = 0
    
    float absX = std::abs(direction.x);
    float absY = std::abs(direction.y);
    float absZ = std::abs(direction.z);
    
    Vector origin;
    
    if (absZ >= absX && absZ >= absY) {
        // n1.x * x + n1.y * y = -d1
        // n2.x * x + n2.y * y = -d2
        float det = plane1.normal.x * plane2.normal.y - plane1.normal.y * plane2.normal.x;
        if (std::abs(det) > epsilon) {
            origin.x = (-plane1.d * plane2.normal.y + plane2.d * plane1.normal.y) / det;
            origin.y = (-plane2.d * plane1.normal.x + plane1.d * plane2.normal.x) / det;
            origin.z = 0;
        }
    } else if (absY >= absX && absY >= absZ) {
        // n1.x * x + n1.z * z = -d1
        // n2.x * x + n2.z * z = -d2
        float det = plane1.normal.x * plane2.normal.z - plane1.normal.z * plane2.normal.x;
        if (std::abs(det) > epsilon) {
            origin.x = (-plane1.d * plane2.normal.z + plane2.d * plane1.normal.z) / det;
            origin.y = 0;
            origin.z = (-plane2.d * plane1.normal.x + plane1.d * plane2.normal.x) / det;
        }
    } else {
        // n1.y * y + n1.z * z = -d1
        // n2.y * y + n2.z * z = -d2
        float det = plane1.normal.y * plane2.normal.z - plane1.normal.z * plane2.normal.y;
        if (std::abs(det) > epsilon) {
            origin.x = 0;
            origin.y = (-plane1.d * plane2.normal.z + plane2.d * plane1.normal.z) / det;
            origin.z = (-plane2.d * plane1.normal.y + plane1.d * plane2.normal.y) / det;
        }
    }
    
    result.line = Line(origin, direction);
    
    return result;
}

// Geometry - Line Segment-Segment Intersection
LineSegmentIntersection Geometry::intersect(const LineSegment& seg1, const LineSegment& seg2, float epsilon) {
    LineSegmentIntersection result;
    result.intersects = false;
    result.areParallel = false;
    result.areCollinear = false;
    result.overlap = false;
    result.t1 = 0.0f;
    result.t2 = 0.0f;
    
    Vector d1 = seg1.getDirection();  // seg1.end - seg1.start
    Vector d2 = seg2.getDirection();  // seg2.end - seg2.start
    Vector w = Vector(seg2.start, seg1.start);  // seg1.start - seg2.start
    
    float len1 = d1.length();
    float len2 = d2.length();
    
    if (len1 < epsilon || len2 < epsilon) {
        return result;
    }
    
    Vector d1_norm = d1;
    d1_norm.normalize();
    Vector d2_norm = d2;
    d2_norm.normalize();
    
    // Check if directions are parallel
    Vector cross_d1_d2 = d1_norm.cross(d2_norm);
    float cross_length = cross_d1_d2.length();
    
    if (cross_length < epsilon) {
        // Segments are parallel
        result.areParallel = true;
        
        // Check if they are collinear
        Vector cross_w_d1 = w.cross(d1_norm);
        if (cross_w_d1.length() < epsilon * std::max(len1, len2)) {
            result.areCollinear = true;
            
            // Calculate projections of seg2 endpoints onto seg1
            float t_start = d1_norm.dotProduct(w);  // seg2.start projected onto seg1
            float t_end = d1_norm.dotProduct(Vector(seg2.end, seg1.start));  // seg2.end projected onto seg1
            
            if (t_start > t_end) {
                std::swap(t_start, t_end);
            }
            
            // Check: [0, len1] intersects [t_start, t_end]
            float overlap_start = std::max(0.0f, t_start);
            float overlap_end = std::min(len1, t_end);
            
            if (overlap_start <= overlap_end + epsilon) {
                result.overlap = true;
                result.intersects = true;
                
                // World coordinates
                result.overlapStart = Vector(
                    seg1.start.x + d1_norm.x * overlap_start,
                    seg1.start.y + d1_norm.y * overlap_start,
                    seg1.start.z + d1_norm.z * overlap_start
                );
                result.overlapEnd = Vector(
                    seg1.start.x + d1_norm.x * overlap_end,
                    seg1.start.y + d1_norm.y * overlap_end,
                    seg1.start.z + d1_norm.z * overlap_end
                );
                
                // Set point to midpoint of overlap
                result.point = Vector(
                    (result.overlapStart.x + result.overlapEnd.x) * 0.5f,
                    (result.overlapStart.y + result.overlapEnd.y) * 0.5f,
                    (result.overlapStart.z + result.overlapEnd.z) * 0.5f
                );
            }
        }
        return result;
    }
    
    // Not parallel - find intersection point
    // P1(s) = seg1.start + s * d1, s in [0, 1]
    // P2(t) = seg2.start + t * d2, t in [0, 1]
    // d1 = seg1.end - seg1.start, d2 = seg2.end - seg2.start
    // seg1.start + s * d1 = seg2.start + t * d2
    // s * d1 - t * d2 = seg2.start - seg1.start
    
    Vector cross_d1_d2_full = d1.cross(d2);
    float denominator = cross_d1_d2_full.dotProduct(cross_d1_d2_full);
    
    if (denominator < epsilon * epsilon) {
        return result;
    }
    
    Vector w0 = Vector(seg1.start, seg2.start);  // seg2.start - seg1.start
    Vector cross_w0_d2 = w0.cross(d2);
    float s = cross_w0_d2.dotProduct(cross_d1_d2_full) / denominator;
    
    Vector cross_w0_d1 = w0.cross(d1);
    float t = cross_w0_d1.dotProduct(cross_d1_d2_full) / denominator;
    
    result.t1 = s;
    result.t2 = t;
    
    // Check if intersection is within both segments [0, 1]
    if (s >= -epsilon && s <= 1.0f + epsilon &&
        t >= -epsilon && t <= 1.0f + epsilon) {
        
        result.intersects = true;
        result.point = seg1.getPoint(s);
    }
    
    return result;
}

// Geometry - Angle Calculations
float Geometry::angleBetween(const Line& line1, const Line& line2) {
    // Angle between line directions
    float dot = line1.direction.dotProduct(line2.direction);
    dot = std::max(-1.0f, std::min(1.0f, dot)); // Clamp to [-1, 1]
    float angle = std::acos(std::abs(dot));
    return angle;
}

float Geometry::angleBetween(const Ray& ray1, const Ray& ray2) {
    float dot = ray1.direction.dotProduct(ray2.direction);
    dot = std::max(-1.0f, std::min(1.0f, dot));
    float angle = std::acos(std::abs(dot));
    return angle;
}

float Geometry::angleBetween(const Line& line, const Plane& plane) {
    // 90 deg - angle between line and plane normal
    float dot = line.direction.dotProduct(plane.normal);
    dot = std::max(-1.0f, std::min(1.0f, dot));
    float angleWithNormal = std::acos(std::abs(dot));
    return PI / 2.0f - angleWithNormal;
}

float Geometry::angleBetween(const Ray& ray, const Plane& plane) {
    float dot = ray.direction.dotProduct(plane.normal);
    dot = std::max(-1.0f, std::min(1.0f, dot));
    float angleWithNormal = std::acos(std::abs(dot));
    return PI / 2.0f - angleWithNormal;
}

float Geometry::angleBetween(const Plane &plane1, const Plane &plane2)
{
    // Angle between normals
    float dot = plane1.normal.dotProduct(plane2.normal);
    dot = std::max(-1.0f, std::min(1.0f, dot));
    float angle = std::acos(std::abs(dot));
    return angle;
}

// Degree versions
float Geometry::angleBetweenDegrees(const Line& line1, const Line& line2) {
    return radToDeg(angleBetween(line1, line2));
}

float Geometry::angleBetweenDegrees(const Ray& ray1, const Ray& ray2) {
    return radToDeg(angleBetween(ray1, ray2));
}

float Geometry::angleBetweenDegrees(const Line& line, const Plane& plane) {
    return radToDeg(angleBetween(line, plane));
}

float Geometry::angleBetweenDegrees(const Ray& ray, const Plane& plane) {
    return radToDeg(angleBetween(ray, plane));
}

float Geometry::angleBetweenDegrees(const Plane &plane1, const Plane &plane2)
{
    return radToDeg(angleBetween(plane1, plane2));
}

// Output Stream Overloads
std::ostream& operator<<(std::ostream& os, const LineLineIntersection& result) {
    os << "LineLineIntersection:\n";
    if (result.areCoincident) {
        os << "  Status: Lines are coincident (same line)\n";
        os << "  Point on line: " << result.point << "\n";
    } else if (result.areParallel) {
        os << "  Status: Lines are parallel\n";
        os << "  Distance: " << result.distance << "\n";
    } else if (result.intersects) {
        os << "  Status: Lines intersect\n";
        os << "  Intersection point: " << result.point << "\n";
        os << "  Parameters: t1=" << result.t1 << ", t2=" << result.t2 << "\n";
    } else {
        os << "  Status: Lines are skew (non-parallel, non-intersecting)\n";
        os << "  Closest distance: " << result.distance << "\n";
        os << "  Closest point on line1: " << result.closestPoint1 << "\n";
        os << "  Closest point on line2: " << result.closestPoint2 << "\n";
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const LinePlaneIntersection& result) {
    os << "LinePlaneIntersection:\n";
    if (result.lineInPlane) {
        os << "  Status: Line lies entirely in the plane\n";
        os << "  Point on line: " << result.point << "\n";
    } else if (result.isParallel) {
        os << "  Status: Line is parallel to plane (no intersection)\n";
    } else if (result.intersects) {
        os << "  Status: Line intersects plane\n";
        os << "  Intersection point: " << result.point << "\n";
        os << "  Parameter: t=" << result.t << "\n";
    } else {
        os << "  Status: No intersection\n";
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const LineSphereIntersection& result) {
    os << "LineSphereIntersection:\n";
    if (result.intersects) {
        os << "  Status: Intersects\n";
        os << "  Number of intersections: " << result.numIntersections << "\n";
        os << "  First intersection: " << result.point1 << " at t=" << result.t1 << "\n";
        if (result.numIntersections == 2) {
            os << "  Second intersection: " << result.point2 << " at t=" << result.t2 << "\n";
        }
    } else {
        os << "  Status: No intersection\n";
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const LineCircleIntersection& result) {
    os << "LineCircleIntersection:\n";
    if (result.intersects) {
        os << "  Status: Intersects\n";
        os << "  Number of intersections: " << result.numIntersections << "\n";
        os << "  First intersection: " << result.point1 << " at t=" << result.t1 << "\n";
        if (result.numIntersections == 2) {
            os << "  Second intersection: " << result.point2 << " at t=" << result.t2 << "\n";
        }
    } else {
        os << "  Status: No intersection\n";
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const PlanePlaneIntersection& result) {
    os << "PlanePlaneIntersection:\n";
    if (result.areCoincident) {
        os << "  Status: Planes are coincident (same plane)\n";
    } else if (result.areParallel) {
        os << "  Status: Planes are parallel\n";
        os << "  Distance between planes: " << result.distance << "\n";
    } else if (result.intersects) {
        os << "  Status: Planes intersect\n";
        os << "  Intersection line:\n";
        os << "    Origin: " << result.line.origin << "\n";
        os << "    Direction: " << result.line.direction << "\n";
    } else {
        os << "  Status: No intersection\n";
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const LineSegmentIntersection& result) {
    os << "LineSegmentIntersection:\n";
    if (result.areCollinear && result.overlap) {
        os << "  Status: Segments are collinear and overlap\n";
        os << "  Overlap start: " << result.overlapStart << "\n";
        os << "  Overlap end: " << result.overlapEnd << "\n";
    } else if (result.areCollinear) {
        os << "  Status: Segments are collinear but don't overlap\n";
    } else if (result.areParallel) {
        os << "  Status: Segments are parallel but not collinear\n";
    } else if (result.intersects) {
        os << "  Status: Segments intersect\n";
        os << "  Intersection point: " << result.point << "\n";
        os << "  Parameters: t1=" << result.t1 << ", t2=" << result.t2 << "\n";
    } else {
        os << "  Status: No intersection\n";
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const Line& line) {
    os << "Line:\n";
    os << "  Origin: " << line.origin << "\n";
    os << "  Direction: " << line.direction << "\n";
    return os;
}

std::ostream& operator<<(std::ostream& os, const LineSegment& segment) {
    os << "LineSegment:\n";
    os << "  Start: " << segment.start << "\n";
    os << "  End: " << segment.end << "\n";
    os << "  Length: " << segment.length() << "\n";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Plane& plane) {
    os << "Plane:\n";
    os << "  Normal: " << plane.normal << "\n";
    os << "  d: " << plane.d << "\n";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Sphere& sphere) {
    os << "Sphere:\n";
    os << "  Center: " << sphere.center << "\n";
    os << "  Radius: " << sphere.radius << "\n";
    return os;
}
