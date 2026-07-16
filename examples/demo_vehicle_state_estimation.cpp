// Demo: fuse a noisy accelerometer with noisy, lower-rate GPS position
// measurements to estimate a vehicle's longitudinal position and velocity.
//
// The vehicle accelerates for the first half of the run and cruises at
// constant velocity for the second half. Prints a table comparing the
// true trajectory, the raw noisy GPS samples, and the Kalman filter's
// fused estimate.

#include <cstdio>
#include <random>

#include "vehicledynamics/vehicle_state_estimator.hpp"

using vehicledynamics::VehicleStateEstimator;

int main() {
    const double dt = 0.1;            // simulation / IMU step [s]
    const int steps = 200;            // 20 seconds total
    const int gpsPeriod = 10;         // GPS update every 1 s
    const double trueAccel = 2.0;     // m/s^2 during the acceleration phase
    const double accelNoiseStd = 0.3; // IMU noise [m/s^2]
    const double positionNoiseStd = 5.0; // GPS noise [m]

    std::mt19937 rng(1);
    std::normal_distribution<double> accelNoise(0.0, accelNoiseStd);
    std::normal_distribution<double> positionNoise(0.0, positionNoiseStd);

    VehicleStateEstimator estimator(accelNoiseStd, positionNoiseStd);

    double truePosition = 0.0;
    double trueVelocity = 0.0;

    std::printf("%8s %10s %10s %14s %10s %10s\n", "time_s", "true_pos", "true_vel",
                "gps_pos", "est_pos", "est_vel");

    for (int i = 0; i < steps; ++i) {
        double accel = (i < steps / 2) ? trueAccel : 0.0;

        truePosition += trueVelocity * dt + 0.5 * accel * dt * dt;
        trueVelocity += accel * dt;

        double measuredAccel = accel + accelNoise(rng);
        estimator.predict(measuredAccel, dt);

        bool hasGps = (i % gpsPeriod == 0);
        double gpsPosition = 0.0;
        if (hasGps) {
            gpsPosition = truePosition + positionNoise(rng);
            estimator.updatePosition(gpsPosition);
        }

        double time = (i + 1) * dt;
        if (hasGps) {
            std::printf("%8.2f %10.3f %10.3f %14.3f %10.3f %10.3f\n", time, truePosition,
                        trueVelocity, gpsPosition, estimator.position(), estimator.velocity());
        } else {
            std::printf("%8.2f %10.3f %10.3f %14s %10.3f %10.3f\n", time, truePosition,
                        trueVelocity, "-", estimator.position(), estimator.velocity());
        }
    }

    return 0;
}
