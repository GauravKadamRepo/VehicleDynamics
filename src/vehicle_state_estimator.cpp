#include "vehicledynamics/vehicle_state_estimator.hpp"

namespace vehicledynamics {

namespace {

Matrix makeTransition(double dt) {
    Matrix A(2, 2);
    A(0, 0) = 1.0;
    A(0, 1) = dt;
    A(1, 0) = 0.0;
    A(1, 1) = 1.0;
    return A;
}

Matrix makeControlModel(double dt) {
    Matrix B(2, 1);
    B(0, 0) = 0.5 * dt * dt;
    B(1, 0) = dt;
    return B;
}

Matrix makeMeasurementModel() {
    Matrix H(1, 2);
    H(0, 0) = 1.0;
    H(0, 1) = 0.0;
    return H;
}

}  // namespace

VehicleStateEstimator::VehicleStateEstimator(double accelStdDev, double positionStdDev,
                                              double initialPosition, double initialVelocity,
                                              double initialPositionVariance,
                                              double initialVelocityVariance)
    : accelStdDev_(accelStdDev) {
    Matrix x0(2, 1);
    x0(0, 0) = initialPosition;
    x0(1, 0) = initialVelocity;

    Matrix P0(2, 2);
    P0(0, 0) = initialPositionVariance;
    P0(1, 1) = initialVelocityVariance;

    Matrix R(1, 1);
    R(0, 0) = positionStdDev * positionStdDev;

    // A, B and Q depend on dt and are recomputed on each predict() call.
    filter_ = KalmanFilter(makeTransition(0.0), makeControlModel(0.0), makeMeasurementModel(),
                            Matrix(2, 2), R, x0, P0);
}

void VehicleStateEstimator::predict(double accel, double dt) {
    Matrix B = makeControlModel(dt);

    // Process noise from uncertain acceleration input, propagated through B:
    // Q = B * B^T * sigma_a^2.
    double accelVariance = accelStdDev_ * accelStdDev_;
    Matrix Q = B * B.transpose() * accelVariance;

    filter_.setTransition(makeTransition(dt));
    filter_.setControlModel(B);
    filter_.setProcessNoise(Q);

    Matrix u(1, 1);
    u(0, 0) = accel;
    filter_.predict(u);
}

void VehicleStateEstimator::updatePosition(double measuredPosition) {
    Matrix z(1, 1);
    z(0, 0) = measuredPosition;
    filter_.update(z);
}

double VehicleStateEstimator::position() const { return filter_.state()(0, 0); }

double VehicleStateEstimator::velocity() const { return filter_.state()(1, 0); }

}  // namespace vehicledynamics
