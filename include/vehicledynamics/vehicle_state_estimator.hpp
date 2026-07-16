#pragma once

#include "vehicledynamics/kalman_filter.hpp"

namespace vehicledynamics {

// Estimates longitudinal position and velocity by fusing a noisy
// accelerometer (used as the control input in the predict step) with
// noisy, lower-rate position measurements (e.g. GPS).
//
// State: x = [position, velocity]^T
// Process model (constant-velocity kinematics with acceleration input):
//   A = [[1, dt], [0, 1]]
//   B = [[0.5*dt^2], [dt]]
// Measurement model (position only):
//   H = [1, 0]
class VehicleStateEstimator {
public:
    // accelStdDev: standard deviation of accelerometer noise (process noise driver)
    // positionStdDev: standard deviation of position measurement noise
    // initialPosition, initialVelocity: initial state estimate
    // initialPositionVariance, initialVelocityVariance: initial state uncertainty
    VehicleStateEstimator(double accelStdDev, double positionStdDev,
                           double initialPosition = 0.0, double initialVelocity = 0.0,
                           double initialPositionVariance = 1.0,
                           double initialVelocityVariance = 1.0);

    // Predict the state forward by dt seconds using measured acceleration
    // (e.g. from an IMU) as the control input.
    void predict(double accel, double dt);

    // Correct the state estimate using a noisy position measurement (e.g. GPS).
    void updatePosition(double measuredPosition);

    double position() const;
    double velocity() const;

private:
    double accelStdDev_;
    KalmanFilter filter_;
};

}  // namespace vehicledynamics
