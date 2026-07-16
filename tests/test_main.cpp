#include <iostream>

#include "test_util.hpp"

void runMatrixTests();
void runKalmanFilterTests();

int main() {
    runMatrixTests();
    runKalmanFilterTests();

    if (testutil::g_failures == 0) {
        std::cout << "All tests passed.\n";
        return 0;
    }
    std::cerr << testutil::g_failures << " test check(s) failed.\n";
    return 1;
}
