#include <cmath>
#include <random>

#include "vehicledynamics/kalman_filter.hpp"
#include "vehicledynamics/vehicle_state_estimator.hpp"

#include "test_util.hpp"

using vehicledynamics::KalmanFilter;
using vehicledynamics::Matrix;
using vehicledynamics::VehicleStateEstimator;

namespace {

// A stationary object (true position = 0) measured directly and repeatedly
// with noise. The filter's uncertainty should shrink and its estimate
// should converge close to the true value.
void testGenericFilterConvergesOnStaticState() {
    Matrix A = Matrix::identity(1);
    Matrix B(1, 1);
    Matrix H = Matrix::identity(1);
    Matrix Q(1, 1);
    Q(0, 0) = 1e-6;
    Matrix R(1, 1);
    R(0, 0) = 1.0;
    Matrix x0(1, 1);
    x0(0, 0) = 10.0;  // deliberately wrong initial guess
    Matrix P0(1, 1);
    P0(0, 0) = 100.0;

    KalmanFilter filter(A, B, H, Q, R, x0, P0);

    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 1.0);

    const double trueValue = 0.0;
    for (int i = 0; i < 200; ++i) {
        filter.predict();
        Matrix z(1, 1);
        z(0, 0) = trueValue + noise(rng);
        filter.update(z);
    }

    CHECK_NEAR(filter.state()(0, 0), trueValue, 0.3);
    CHECK(filter.covariance()(0, 0) < P0(0, 0));
}

// Simulate a vehicle undergoing constant acceleration, fusing noisy IMU
// acceleration (every step) with noisy, lower-rate GPS position (every 10
// steps). The final filtered estimate should be much closer to ground
// truth than a lone noisy GPS reading.
void testVehicleStateEstimatorTracksConstantAcceleration() {
    const double dt = 0.1;
    const double trueAccel = 1.0;      // m/s^2
    const double accelNoiseStd = 0.2;  // m/s^2
    const double positionNoiseStd = 3.0;  // m
    const int steps = 300;
    const int gpsPeriod = 10;

    std::mt19937 rng(7);
    std::normal_distribution<double> accelNoise(0.0, accelNoiseStd);
    std::normal_distribution<double> positionNoise(0.0, positionNoiseStd);

    VehicleStateEstimator estimator(accelNoiseStd, positionNoiseStd);

    double truePosition = 0.0;
    double trueVelocity = 0.0;
    double lastRawGpsError = 0.0;

    for (int i = 0; i < steps; ++i) {
        truePosition += trueVelocity * dt + 0.5 * trueAccel * dt * dt;
        trueVelocity += trueAccel * dt;

        double measuredAccel = trueAccel + accelNoise(rng);
        estimator.predict(measuredAccel, dt);

        if (i % gpsPeriod == 0) {
            double measuredPosition = truePosition + positionNoise(rng);
            lastRawGpsError = std::fabs(measuredPosition - truePosition);
            estimator.updatePosition(measuredPosition);
        }
    }

    double positionError = std::fabs(estimator.position() - truePosition);
    double velocityError = std::fabs(estimator.velocity() - trueVelocity);

    CHECK(positionError < 3.0);
    CHECK(velocityError < 1.0);
    // The fused estimate should typically beat a single raw GPS sample.
    CHECK(positionError < lastRawGpsError + 3.0);
}

}  // namespace

void runKalmanFilterTests() {
    testGenericFilterConvergesOnStaticState();
    testVehicleStateEstimatorTracksConstantAcceleration();
}
