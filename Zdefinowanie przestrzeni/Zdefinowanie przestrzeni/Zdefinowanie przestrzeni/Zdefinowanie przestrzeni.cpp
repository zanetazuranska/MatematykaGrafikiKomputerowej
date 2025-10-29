#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <string>
#include "Matrix4x4.h"
#include "Vector.h"

static constexpr float EPS = 1e-5f;

bool approxEqual(float a, float b, float eps = EPS) {
    return std::fabs(a - b) <= eps;
}

bool approxEqual(const Vector& a, const Vector& b, float eps = EPS) {
    return approxEqual(a.x, b.x, eps) && approxEqual(a.y, b.y, eps) && approxEqual(a.z, b.z, eps);
}

bool approxEqual(const Matrix4x4& a, const Matrix4x4& b, float eps = EPS) {
    for (int i = 0; i < 16; i++) {
        if (!approxEqual(a.entries[i], b.entries[i], eps)) return false;
    }
    return true;
}

void reportFailure(const std::string& testName, const std::string& msg) {
    std::cerr << "[FAIL] " << testName << ": " << msg << std::endl;
}

void reportSuccess(const std::string& testName) {
    std::cout << "[ OK ] " << testName << std::endl;
}

// Tests

bool test_identity_and_constructors() {
    const std::string name = "identity_and_constructors";

    Matrix4x4 I(true);
    Matrix4x4 Z(false);
    Matrix4x4 fromList(
        1, 2, 3, 4,
        5, 6, 7, 8,
        9, 10, 11, 12,
        13, 14, 15, 16
    );
    if (!approxEqual(I.entries[0], 1.0f) || !approxEqual(I.entries[15], 1.0f)) {
        reportFailure(name, "identity elements incorrect");
        return false;
    }
    if (!approxEqual(Z.entries[0], 0.0f) || !approxEqual(Z.entries[15], 0.0f)) {
        reportFailure(name, "zero constructor incorrect");
        return false;
    }
    if (!approxEqual(fromList.entries[10], 11.0f)) {
        reportFailure(name, "list constructor incorrect");
        return false;
    }

    reportSuccess(name);
    return true;
}

bool test_add_and_scalar_mult() {
    const std::string name = "add_and_scalar_mult";
    Matrix4x4 A(
        1, 1, 1, 1,
        2, 2, 2, 2,
        3, 3, 3, 3,
        4, 4, 4, 4
    );
    Matrix4x4 B = A + A;
    for (int i = 0; i < 16; ++i) {
        if (!approxEqual(B.entries[i], A.entries[i] * 2.0f)) {
            reportFailure(name, "addition not matching expected");
            return false;
        }
    }
    Matrix4x4 C = A * 2.5f;
    for (int i = 0; i < 16; ++i) {
        if (!approxEqual(C.entries[i], A.entries[i] * 2.5f)) {
            reportFailure(name, "scalar multiplication not matching expected");
            return false;
        }
    }
    reportSuccess(name);
    return true;
}

bool test_matrix_multiplication_and_noncommutativity() {
    const std::string name = "matrix_multiplication_and_noncommutativity";
    // A: scale by 2 on X, 3 on Y, 4 on Z
    Matrix4x4 A(true);
    A.setScaleOnly(Vector{ 2,3,4 });
    // B: rotate 90 deg around Y
    float angle = 90.0f * (M_PI / 180.0f);
    Matrix4x4 B = Matrix4x4::rotationY(angle);

    Matrix4x4 AB = A * B;
    Matrix4x4 BA = B * A;
    // In general AB != BA
    if (AB == BA) {
        reportFailure(name, "expected AB != BA but got equality");
        return false;
    }

    // Check multiplication correctness with a known vector:
    Vector p{ 1,0,0 };
    Vector pAB = AB * p; // (A*B) * p
    Vector pBA = BA * p;
    if (approxEqual(pAB, pBA)) {
        reportFailure(name, "AB*p should not equal BA*p for these A,B");
        return false;
    }

    reportSuccess(name);
    return true;
}

bool test_transform_point_and_direction() {
    const std::string name = "transform_point_and_direction";
    // rotate 90 deg about Y: (1,0,0) -> (0,0,-1)
    float angle = 90.0f * (M_PI / 180.0f);
    Matrix4x4 R = Matrix4x4::rotationY(angle);
    Vector p{ 1,0,0 };
    Vector p2 = R * p;
    Vector expected{ 0.0f, 0.0f, -1.0f };
    if (!approxEqual(p2, expected, 1e-4f)) {
        reportFailure(name, "rotationY 90deg result incorrect: got (" +
            std::to_string(p2.x) + "," + std::to_string(p2.y) + "," + std::to_string(p2.z) + ")");
        return false;
    }

    // direction: translation should be ignored
    Matrix4x4 T(true);
    T.setTranslationOnly(Vector{ 5,6,7 });
    Vector dir{ 0,1,0 };
    Vector dirTransformed = transformDirection(T, dir);
    if (!approxEqual(dirTransformed, dir)) {
        reportFailure(name, "transformDirection should ignore translation");
        return false;
    }

    reportSuccess(name);
    return true;
}

bool test_translation_set_and_apply() {
    const std::string name = "translation_set_and_apply";
    Matrix4x4 M;
    M.setTranslation(Vector{ 1,2,3 });
    Vector origin{ 0,0,0 };
    Vector moved = M * origin;
    if (!approxEqual(moved, Vector{ 1,2,3 })) {
        reportFailure(name, "setTranslation did not move origin correctly");
        return false;
    }

    Matrix4x4 N(true);
    N.setTranslationOnly(Vector{ 4,5,6 });
    Vector moved2 = N * origin;
    if (!approxEqual(moved2, Vector{ 4,5,6 })) {
        reportFailure(name, "setTranslationOnly on identity failed");
        return false;
    }

    Matrix4x4 P(true);
    P.translate(Vector{ 7,8,9 });
    Vector moved3 = P * origin;
    if (!approxEqual(moved3, Vector{ 7,8,9 })) {
        reportFailure(name, "translate (in-place) failed");
        return false;
    }

    reportSuccess(name);
    return true;
}

bool test_scale_set_and_apply() {
    const std::string name = "scale_set_and_apply";
    Matrix4x4 S;
    S.setScale(Vector{ 2,3,4 });
    Vector p{ 1,1,1 };
    Vector sP = S * p;
    if (!approxEqual(sP, Vector{ 2,3,4 })) {
        reportFailure(name, "setScale did not produce expected scaled point");
        return false;
    }

    Matrix4x4 S2(true);
    S2.setScaleOnly(Vector{ 5,6,7 });
    Vector sP2 = S2 * p;
    if (!approxEqual(sP2, Vector{ 5,6,7 })) {
        reportFailure(name, "setScaleOnly on identity failed");
        return false;
    }

    Matrix4x4 S3(true);
    S3.scale(Vector{ 2,2,2 });
    Vector sP3 = S3 * Vector{ 1,1,1 };
    if (!approxEqual(sP3, Vector{ 2,2,2 })) {
        reportFailure(name, "scale (in-place) failed");
        return false;
    }

    reportSuccess(name);
    return true;
}

bool test_rotation_set_and_apply_variants() {
    const std::string name = "rotation_set_and_apply_variants";
    float angle = 45.0f * (3.14159265358979323846f / 180.0f);
    Matrix4x4 R; R.setRotation(Vector{ angle, 0, 0 }); // resets to identity then sets rotation
    // Check that the upper-left 3x3 equals rotationX
    Matrix4x4 Rx = Matrix4x4::rotationX(angle);
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
        if (!approxEqual(R.entries[r * 4 + c], Rx.entries[r * 4 + c])) {
            reportFailure(name, "setRotation did not set the rotation block correctly");
            return false;
        }

    Matrix4x4 M(true);
    M.setRotationOnly(Vector{ 0, angle, 0 }); // overwrite only 3x3
    Matrix4x4 Ry = Matrix4x4::rotationY(angle);
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
        if (!approxEqual(M.entries[r * 4 + c], Ry.entries[r * 4 + c])) {
            reportFailure(name, "setRotationOnly did not set rotation block correctly");
            return false;
        }

    // In-place rotate chaining
    Matrix4x4 C(true);
    C.rotate(Vector{ angle, 0, 0 }).rotateY(angle); // should be valid chaining
    // Basic sanity: C must not be identity
    if (C == Matrix4x4(true)) {
        reportFailure(name, "rotate chaining produced identity unexpectedly");
        return false;
    }

    reportSuccess(name);
    return true;
}

bool test_transpose_and_inverse() {
    const std::string name = "transpose_and_inverse";
    Matrix4x4 A(
        1, 2, 3, 4,
        5, 6, 7, 8,
        9, 10, 11, 12,
        0, 0, 0, 1
    );

    Matrix4x4 At = A.getTransposed();
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c)
        if (!approxEqual(At.entries[r * 4 + c], A.entries[c * 4 + r])) {
            reportFailure(name, "getTransposed mismatch");
            return false;
        }

    // Build an invertible affine transform: translate * rotateY * scale
    Matrix4x4 S(true); S.setScaleOnly(Vector{ 2,3,4 });
    Matrix4x4 R = Matrix4x4::rotationY(30.0f * (3.14159265358979323846f / 180.0f));
    Matrix4x4 T(true); T.setTranslationOnly(Vector{ 1,2,3 });
    Matrix4x4 M = T * R * S;
    Matrix4x4 invM = M.getInversed();
    Matrix4x4 I = M * invM;

    if (!(I == Matrix4x4(true))) {
        // Use approxEqual because operator== has epsilon but still be explicit
        if (!approxEqual(I, Matrix4x4(true), 1e-4f)) {
            reportFailure(name, "M * invM not identity (within tolerance)");
            return false;
        }
    }

    // Double transpose equals original
    if (!(A.getTransposed().getTransposed() == A)) {
        reportFailure(name, "double transpose != original");
        return false;
    }

    reportSuccess(name);
    return true;
}

int main() {
    int total = 0;
    int passed = 0;

    auto run = [&](bool(*fn)(), const std::string& name) {
        ++total;
        bool ok = false;
        try {
            ok = fn();
        }
        catch (const std::exception& e) {
            reportFailure(name, std::string("threw exception: ") + e.what());
            ok = false;
        }
        catch (...) {
            reportFailure(name, "threw unknown exception");
            ok = false;
        }
        if (ok) ++passed;
        };

    run(test_identity_and_constructors, "identity_and_constructors");
    run(test_add_and_scalar_mult, "add_and_scalar_mult");
    run(test_matrix_multiplication_and_noncommutativity, "matrix_multiplication_and_noncommutativity");
    run(test_transform_point_and_direction, "transform_point_and_direction");
    run(test_translation_set_and_apply, "translation_set_and_apply");
    run(test_scale_set_and_apply, "scale_set_and_apply");
    run(test_rotation_set_and_apply_variants, "rotation_set_and_apply_variants");
    run(test_transpose_and_inverse, "transpose_and_inverse");

    std::cout << "Passed " << passed << " / " << total << " tests." << std::endl;

    Matrix4x4 A(
        1, 5, 3, 4,
        5, 8, 7, 8,
        9, 2, 11, 12,
        0, 0, 1, 1
    );

    Matrix4x4 B(
        1, 2, 1, 1,
        2, 1, 2, 2,
        3, 3, 1, 2,
        4, 4, 4, 1
    );

    int skalar = 3;

    std::cout << "\n Dane: \n";
    std::cout<<"Macierz A: " << A << "\n";
    std::cout<<"Macierz B: " << B << "\n";
    std::cout << "Skalar: " << skalar << "\n";

    std::cout << "\nWynik dodawania macierzy: " << A + B << "\n";
    std::cout << "\nWynik odejmowania macierzy: " << A - B << "\n";
    std::cout << "\nWynik mnozenia przez skalar macierzy A: " << A * 3 << "\n";
    std::cout << "\nWynik mnozenia macierzy A*B: " << A * B << "\n";

    Matrix4x4 i(true);
    std::cout << "\nTworzenie macierzy jednostkowej: " << i << "\n";

    std::cout << "\nMacierz odwrotna do macierzy A: " << A.getInversed() << "\n";
    std::cout << "\nTranspozycja macierzy A: " << A.getTransposed() << "\n";

    Vector transform(2.0f, 5.0f, 3.0f);
    i.setTranslation(transform);
    std::cout << "\nTransformacje o wektor: (" << transform.x << ", " << transform.y << ", " << transform.z;
    std::cout<<") przedstawiona za pomoca macierzy wyglada nastepujaco:\n"<<i<<"\n";

    Vector scale(1.0f, 2.0f, 3.0f);
    i.setScale(scale);
    std::cout << "\nSkalowanie o wektor: (" << scale.x << ", " << scale.y << ", " << scale.z;
    std::cout << ") przedstawiona za pomoca macierzy wyglada nastepujaco:\n" << i << "\n";

    float angle = 90.0f * (M_PI / 180.0f);
    std::cout << "\nObrot na osi X o 90 stopni: " << Matrix4x4::rotationX(angle)<<"\n";
    std::cout << "\nObrot na osi Y o 90 stopni: " << Matrix4x4::rotationY(angle)<<"\n";
    std::cout << "\nObrot na osi Z o 90 stopni: " << Matrix4x4::rotationZ(angle)<<"\n";

    float angleX = 90.0f * (M_PI / 180.0f);
    float angleY = 45.0f * (M_PI / 180.0f);
    float angleZ = 90.0f * (M_PI / 180.0f);
    Vector rotate(angleX, angleY, angleZ);
    i.setRotation(rotate);
    std::cout << "\nObrot o 90 stopni na osi X, 45 na Y i 90 na Z: " << i << "\n";

    i = Matrix4x4(true);
    std::cout << "\nSkladanie przeksztalcen np transformacja i skala: \n" << i.translate(transform).scale(scale);

    Vector v1 = transformPoint(Matrix4x4::rotationY(angle), Vector(1.0f, 0.0f, 0.0f));
    std::cout << "\n\nObrocenie wektora na osi Y o 90 stopni: (" << v1.x << ", " << v1.y << ", " << v1.z << ") ";

    std::cout << "\n\nBrak przemiennosci mnozenia macierzy:\n";
    std::cout << "Mnozenie A*B: \n" << A * B;
    std::cout << "\nMnozenie B*A: \n" << B*A;



    return (passed == total) ? 0 : 1;
}