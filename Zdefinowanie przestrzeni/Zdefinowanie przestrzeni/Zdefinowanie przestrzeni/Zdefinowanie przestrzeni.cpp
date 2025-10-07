
#include <iostream>
#include "Vector.h"

int main()
{
    Vector v1(3.f, 2.f, 1.0);
    Vector v2(5.f, 2.f, 1.0);
    v1 /= 0;
    std::cout << v1.x;
}