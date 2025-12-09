#define _USE_MATH_DEFINES
#define NOMINMAX

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>   
#include <conio.h>           
#include <windows.h>   

#include "Matrix4x4.h"
#include "Vector.h"
#include "Quaternion.h"
#include "Geometry.h"

bool intersectCube(const Ray& ray)
{
    const float minB = -1.0f;
    const float maxB = 1.0f;

    float tMin = -1e9f;
    float tMax = 1e9f;

    auto checkAxis = [&](float origin, float dir)
        {
            float t1 = (minB - origin) / dir;
            float t2 = (maxB - origin) / dir;
            if (t1 > t2) std::swap(t1, t2);
            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);
        };

    if (std::fabs(ray.direction.x) < 1e-6f) {
        if (ray.origin.x < minB || ray.origin.x > maxB) return false;
    }
    else checkAxis(ray.origin.x, ray.direction.x);

    if (std::fabs(ray.direction.y) < 1e-6f) {
        if (ray.origin.y < minB || ray.origin.y > maxB) return false;
    }
    else checkAxis(ray.origin.y, ray.direction.y);

    if (std::fabs(ray.direction.z) < 1e-6f) {
        if (ray.origin.z < minB || ray.origin.z > maxB) return false;
    }
    else checkAxis(ray.origin.z, ray.direction.z);

    return tMax >= tMin && tMax >= 0.0f;
}

int main()
{
    const int W = 60;
    const int H = 60;

    float camYawDeg = 0.0f;     // (prawo/lewo)
    float camPitchDeg = 0.0f;   // (góra/dół)
    float camRollDeg = 0.0f;    // przechylenie
    float camDist = 5.0f;       

    const float degToRad = M_PI / 180.0f;

    std::vector<std::string> screen(H, std::string(W, '.'));

    while (true)
    {
        if (_kbhit())
        {
            char key = _getch();
            float ang = 5.0f;

            if (key == 'a') camYawDeg -= ang;
            if (key == 'd') camYawDeg += ang;

            if (key == 'w') camPitchDeg += ang;
            if (key == 's') camPitchDeg -= ang;

            if (key == 'q') camRollDeg += ang;
            if (key == 'e') camRollDeg -= ang;

            if (key == 'z') camDist -= 0.2f;
            if (key == 'c') camDist += 0.2f;

            if (key == 'x') break;
        }

        float camYaw = camYawDeg * degToRad;
        float camPitch = camPitchDeg * degToRad;
        float camRoll = camRollDeg * degToRad;

        Vector camPos;
        camPos.x = camDist * std::cos(camPitch) * std::sin(camYaw);
        camPos.y = camDist * std::sin(camPitch);
        camPos.z = camDist * std::cos(camPitch) * std::cos(camYaw);

        Vector center(0.0f, 0.0f, 0.0f);

        Vector forward = (center - camPos);
        forward.normalize();

        Vector worldUp(0.0f, 1.0f, 0.0f);

        Vector right = forward.cross(worldUp);
        if (right.length() < 1e-6f)
        {
            worldUp = Vector(0.0f, 0.0f, 1.0f);
            right = forward.cross(worldUp);
        }
        right.normalize();

        Vector up = right.cross(forward);
        up.normalize();

        if (std::fabs(camRoll) > 1e-6f)
        {
            Quaternion qRoll = Quaternion::fromAxisAngle(forward, camRoll);
            right = qRoll.rotate(right);
            up = qRoll.rotate(up);
        }

        float fov = 60.0f * degToRad;
        float halfTan = std::tan(fov * 0.5f);
        float aspect = float(W) / float(H);

        for (int y = 0; y < H; y++)
        {
            for (int x = 0; x < W; x++)
            {
                float sx = (2.0f * x / (W - 1) - 1.0f);
                float sy = (2.0f * y / (H - 1) - 1.0f);

                float px = sx * aspect * halfTan;
                float py = -sy * halfTan;

                Vector dir = right * px + up * py + forward;
                dir.normalize();

                Ray ray(camPos, dir);
                bool hit = intersectCube(ray);

                screen[y][x] = hit ? '0' : '.';
            }
        }

        system("cls");

        for (int y = 0; y < H; y++)
            std::cout << screen[y] << "\n";

        Sleep(10);
    }

    return 0;
}

