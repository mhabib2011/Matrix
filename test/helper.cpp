#include "test_macros.hpp"
#include <matrix/helper_functions.hpp>

using namespace matrix;

int main()
{
    TEST(fabs(wrap(4.0, 0.0, 10.0) - 4.0) < 1e-5);
    TEST(fabs(wrap(4.0, 0.0, 1.0)) < 1e-5);
    TEST(fabs(wrap(-4.0, 0.0, 10.0) - 6.0) < 1e-5);
    TEST(fabs(wrap(-18.0, 0.0, 10.0) - 2.0) < 1e-5);
    TEST(fabs(wrap(-1.0, 30.0, 40.0) - 39.0) < 1e-5);
    TEST(fabs(wrap(-8000.0, -555.0, 1.0) - (-216.0)) < 1e-5);

    TEST(fabs(wrap_pi(4.0) - (4.0 - M_TWOPI)) < 1e-5);
    TEST(fabs(wrap_pi(-4.0) - (-4.0 + M_TWOPI)) < 1e-5);
    TEST(fabs(wrap_pi(3.0) - (3.0)) < 1e-3);
    TEST(fabs(wrap_pi(1000.0f) - (1000.0f - 318 * M_PI)) < 1e-3);
    TEST(fabs(wrap_pi(-1000.0f) - (-1000.0f + 318 * M_PI)) < 1e-3);
    TEST(fabs(wrap_pi(-1001.0f) - (-1001.0f + 318 * M_PI)) < 1e-3);
    TEST(!is_finite(wrap_pi(NAN)));

    TEST(fabs(wrap_2pi(-4.0) - (-4.0 + 2*M_PI)) < 1e-5);
    TEST(fabs(wrap_2pi(3.0) - (3.0)) < 1e-3);
    TEST(fabs(wrap_2pi(2000.0f) - (2000.0f - 318 * M_TWOPI)) < 1e-3);
    TEST(fabs(wrap_2pi(-2001.0f) - (-2001.0f + 319 * M_TWOPI)) < 1e-3);
    TEST(!is_finite(wrap_2pi(NAN)));

    Vector3f a(1, 2, 3);
    Vector3f b(4, 5, 6);
    a.T().print();
    TEST(!isEqual(a, b));
    TEST(isEqual(a, a));

    TEST(isEqualF(1.0f, 1.0f));
    TEST(!isEqualF(1.0f, 2.0f));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
