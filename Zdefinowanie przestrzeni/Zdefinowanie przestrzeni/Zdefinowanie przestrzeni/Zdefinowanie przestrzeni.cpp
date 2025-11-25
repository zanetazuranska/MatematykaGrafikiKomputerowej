#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <string>
#include "Matrix4x4.h"
#include "Vector.h"
#include "Quaternion.h"
#include "Geometry.h"

int main() {
    std::cout << std::fixed;
    std::cout.precision(4);

    // 1. Punkt przeciecia dwoch prostych A i B
    // A = (2, -4, 0) + t * (3, 1, 5)
    Vector originA(2.0f, -4.0f, 0.0f);
    Vector directionA(3.0f, 1.0f, 5.0f);
    Line lineA(originA, directionA);
    
    // B = (2, -4, 0) + s * (1, -5, 3)
    Vector originB(2.0f, -4.0f, 0.0f);
    Vector directionB(1.0f, -5.0f, 3.0f);
    Line lineB(originB, directionB);
    
    LineLineIntersection result = Geometry::intersect(lineA, lineB);

    std::cout << "Zadanie 1. Punkt przeciecia dwoch prostych A i B" << std::endl;
    std::cout << "Prosta A:\n" << lineA << std::endl;
    std::cout << "Prosta B:\n" << lineB << std::endl;
    std::cout << "Punkt przeciecia:\n" << result << '\n';
    
    // 2. Kąt pomiędzy prostymi A i B
    std::cout << "Zadanie 2. Kat pomiedzy prostymi A i B" << std::endl;
    float angleDeg = Geometry::angleBetweenDegrees(lineA, lineB);
    std::cout << "Kat (stopnie): " << angleDeg << "\n\n";
    
    // 3. Punkt przecięcia prostej C z płaszczyzną P
    // C = (-2, 2, -1) + t * (3, -1, 2)
    Vector originC(-2.0f, 2.0f, -1.0f);
    Vector directionC(3.0f, -1.0f, 2.0f);
    Line lineC(originC, directionC);

    // P: 2x + 3y + 3z - 8 = 0
    Vector normalP1(2.0f, 3.0f, 3.0f);
    float dP1 = -8.0f;
    Plane planeP1(normalP1, dP1);

    LinePlaneIntersection result2 = Geometry::intersect(lineC, planeP1);
    std::cout << "Zadanie 3. Punkt przeciecia prostej C z plaszczyzna P1" << std::endl;
    std::cout << "Prosta C:\n" << lineC << std::endl;
    std::cout << "Plaszczyzna P1:\n" << planeP1 << std::endl;
    std::cout << "Punkt przeciecia:\n" << result2 << '\n';
    
    // 4. Kąt pomiędzy prostą C a płaszczyzną P1
    std::cout << "Zadanie 4. Kat pomiedzy prosta C a plaszczyzna P1" << std::endl;
    float angleDeg2 = Geometry::angleBetweenDegrees(lineC, planeP1);
    std::cout << "Kat (stopnie): " << angleDeg2 << "\n\n";
    
    // 5 Prosta przecięcia płaszczyzn P2 i P3
    // P2: 2x - y + z - 8 = 0
    Vector normalP2(2.0f, -1.0f, 1.0f);
    float dP2 = -8.0f;
    Plane planeP2(normalP2, dP2);

    // P3: 4x + 3y + z + 14 = 0
    Vector normalP3(4.0f, 3.0f, 1.0f);
    float dP3 = 14.0f;
    Plane planeP3(normalP3, dP3);

    PlanePlaneIntersection result3 = Geometry::intersect(planeP2, planeP3);
    std::cout << "Zadanie 5. Prosta przeciecia plaszczyzn P2 i P3" << std::endl;
    std::cout << "Plaszczyzna P2:\n" << planeP2 << std::endl;
    std::cout << "Plaszczyzna P3:\n" << planeP3 << std::endl;
    std::cout << "Prosta przeciecia:\n" << result3 << '\n';
    
    // 6. Kąt pomiędzy płaszczyznami P2 i P3
    std::cout << "Zadanie 6. Kat pomiedzy plaszczyznami P2 i P3" << std::endl;
    float angleDeg3 = Geometry::angleBetweenDegrees(planeP2, planeP3);
    std::cout << "Kat (stopnie): " << angleDeg3 << "\n\n";
    
    // 7. Punkt przecięcia odcinków D i E
    // D = (5, 5, 4); D' (10, 10, 6)
    Vector pointD1(5.0f, 5.0f, 4.0f);
    Vector pointD2(10.0f, 10.0f, 6.0f);
    LineSegment segmentD(pointD1, pointD2);

    // E = (5, 5, 5); E' = (10, 10, 3)
    Vector pointE1(5.0f, 5.0f, 5.0f);
    Vector pointE2(10.0f, 10.0f, 3.0f);
    LineSegment segmentE(pointE1, pointE2);

    LineSegmentIntersection result4 = Geometry::intersect(segmentD, segmentE);
    std::cout << "Zadanie 7. Punkt przeciecia odcinkow D i E" << std::endl;
    std::cout << "Odcinek D:\n" << segmentD << std::endl;
    std::cout << "Odcinek E:\n" << segmentE << std::endl;
    std::cout << "Punkt przeciecia:\n" << result4 << '\n';
    
    // 8. Punkt przecięcia sfery S z prostą F
    // S: Środek (0, 0, 0), promień sqrt(26)
    Sphere sphereS(Vector(0.0f, 0.0f, 0.0f), std::sqrt(26.0f));

    // F = (3, -1, -2); F' = (5, 3, -4)
    Line lineF(Vector(3.0f, -1.0f, -2.0f), Vector(5.0f, 3.0f, -4.0f), true);
    LineSphereIntersection result5 = Geometry::intersect(lineF, sphereS);
    std::cout << "Zadanie 8. Punkt przeciecia sfery S z prosta F" << std::endl;
    std::cout << "Sfera S:\n" << sphereS << std::endl;
    std::cout << "Prosta F:\n" << lineF << std::endl;
    std::cout << "Punkt(y) przeciecia:\n" << result5 << '\n';

    return 0;
}