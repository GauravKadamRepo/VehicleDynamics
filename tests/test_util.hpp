#pragma once

#include <cmath>
#include <iostream>

namespace testutil {

inline int g_failures = 0;

}  // namespace testutil

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::cerr << "FAIL: " << #cond << " at " << __FILE__ << ":"    \
                       << __LINE__ << "\n";                                \
            ++::testutil::g_failures;                                      \
        }                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                               \
    do {                                                                    \
        double checkNear_a = (a);                                           \
        double checkNear_b = (b);                                           \
        double checkNear_tol = (tol);                                       \
        if (std::fabs(checkNear_a - checkNear_b) > checkNear_tol) {         \
            std::cerr << "FAIL: " << #a << " (" << checkNear_a << ") ~= "   \
                       << #b << " (" << checkNear_b << ") within "          \
                       << checkNear_tol << " at " << __FILE__ << ":"        \
                       << __LINE__ << "\n";                                 \
            ++::testutil::g_failures;                                       \
        }                                                                   \
    } while (0)
