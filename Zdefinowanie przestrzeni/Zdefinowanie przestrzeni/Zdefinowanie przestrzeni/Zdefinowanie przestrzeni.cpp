#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <string>
#include "Matrix4x4.h"
#include "Vector.h"
#include "Quaternion.h"

int main() {
    std::cout << std::fixed;
    std::cout.precision(4);

    // ===== 1. Sprawdzenie działania operatorów =====
    Quaternion q1(1, 2, 3, 4);
    Quaternion q2(0.5f, -1, 2, -0.5f);

    std::cout << "q1 = " << q1 << std::endl;
    std::cout << "q2 = " << q2 << std::endl;

    std::cout << "\nDodawanie: q1 + q2 = " << (q1 + q2) << std::endl;
    std::cout << "Odejmowanie: q1 - q2 = " << (q1 - q2) << std::endl;
    std::cout << "Mnozenie: q1 * q2 = " << (q1 * q2) << std::endl;
    std::cout << "Dzielenie: q1 / q2 = " << (q1 / q2) << std::endl;

    std::cout << "\nSprzezenie q1*: " << q1.conjugate() << std::endl;
    std::cout << "Odwrotnosc q1^-1: " << q1.inverse() << std::endl;

    // ===== 2. Obrót punktu [-1, -1, -1] wokół osi X o 270° =====
    Vector point(-1, -1, -1);
    Vector axisX(1, 0, 0);
    float angleDeg = 270.0f;
    float angleRad = angleDeg * M_PI / 180.0f;
    Quaternion q = Quaternion::fromAxisAngle(axisX, angleRad);

    std::cout << "\nKwaternion rotacji wokol osi X o 270: " << q << std::endl;
    Vector rotated = q.rotate(point);

    std::cout << "\nPunkt przed obrotem: ("
        << point.x << ", " << point.y << ", " << point.z << ")\n";
    std::cout << "Punkt po obrocie: ("
        << rotated.x << ", " << rotated.y << ", " << rotated.z << ")\n";

    // ===== 3. Brak przemienności mnożenia =====
    Quaternion qx = Quaternion::fromAxisAngle(Vector(1, 0, 0), M_PI / 2.0f); // 90° wokół X
    Quaternion qy = Quaternion::fromAxisAngle(Vector(0, 1, 0), M_PI / 2.0f); // 90° wokół Y

    Quaternion qxqy = qx * qy;
    Quaternion qyqx = qy * qx;

    std::cout << "\nqx * qy = " << qxqy << std::endl;
    std::cout << "qy * qx = " << qyqx << std::endl;

    return 0;
}