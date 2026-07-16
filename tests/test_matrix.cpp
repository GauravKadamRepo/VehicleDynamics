#include "vehicledynamics/matrix.hpp"

#include "test_util.hpp"

using vehicledynamics::Matrix;

void runMatrixTests() {
    // Addition / subtraction.
    Matrix a(2, 2);
    a(0, 0) = 1; a(0, 1) = 2;
    a(1, 0) = 3; a(1, 1) = 4;

    Matrix b(2, 2);
    b(0, 0) = 5; b(0, 1) = 6;
    b(1, 0) = 7; b(1, 1) = 8;

    Matrix sum = a + b;
    CHECK_NEAR(sum(0, 0), 6.0, 1e-9);
    CHECK_NEAR(sum(1, 1), 12.0, 1e-9);

    Matrix diff = b - a;
    CHECK_NEAR(diff(0, 0), 4.0, 1e-9);
    CHECK_NEAR(diff(1, 1), 4.0, 1e-9);

    // Multiplication.
    Matrix product = a * b;
    // [1 2] [5 6]   [1*5+2*7  1*6+2*8]   [19 22]
    // [3 4] [7 8] = [3*5+4*7  3*6+4*8] = [43 50]
    CHECK_NEAR(product(0, 0), 19.0, 1e-9);
    CHECK_NEAR(product(0, 1), 22.0, 1e-9);
    CHECK_NEAR(product(1, 0), 43.0, 1e-9);
    CHECK_NEAR(product(1, 1), 50.0, 1e-9);

    // Scalar multiplication.
    Matrix scaled = a * 2.0;
    CHECK_NEAR(scaled(0, 0), 2.0, 1e-9);
    CHECK_NEAR(scaled(1, 1), 8.0, 1e-9);

    // Transpose.
    Matrix at = a.transpose();
    CHECK_NEAR(at(0, 1), 3.0, 1e-9);
    CHECK_NEAR(at(1, 0), 2.0, 1e-9);

    // Identity.
    Matrix id = Matrix::identity(3);
    CHECK_NEAR(id(0, 0), 1.0, 1e-9);
    CHECK_NEAR(id(1, 2), 0.0, 1e-9);

    // Inverse: A * A^-1 == I.
    Matrix inv = a.inverse();
    Matrix shouldBeIdentity = a * inv;
    CHECK_NEAR(shouldBeIdentity(0, 0), 1.0, 1e-9);
    CHECK_NEAR(shouldBeIdentity(0, 1), 0.0, 1e-9);
    CHECK_NEAR(shouldBeIdentity(1, 0), 0.0, 1e-9);
    CHECK_NEAR(shouldBeIdentity(1, 1), 1.0, 1e-9);

    // Known 2x2 inverse: [[4,7],[2,6]]^-1 = (1/10)*[[6,-7],[-2,4]]
    Matrix c(2, 2);
    c(0, 0) = 4; c(0, 1) = 7;
    c(1, 0) = 2; c(1, 1) = 6;
    Matrix cInv = c.inverse();
    CHECK_NEAR(cInv(0, 0), 0.6, 1e-9);
    CHECK_NEAR(cInv(0, 1), -0.7, 1e-9);
    CHECK_NEAR(cInv(1, 0), -0.2, 1e-9);
    CHECK_NEAR(cInv(1, 1), 0.4, 1e-9);
}
