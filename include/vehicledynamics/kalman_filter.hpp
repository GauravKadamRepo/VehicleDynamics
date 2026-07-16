#pragma once

#include "vehicledynamics/matrix.hpp"

namespace vehicledynamics {

// Generic discrete-time linear Kalman filter:
//   predict: x = A*x + B*u,      P = A*P*A^T + Q
//   update:  y = z - H*x
//            S = H*P*H^T + R
//            K = P*H^T*S^-1
//            x = x + K*y
//            P = (I - K*H)*P
class KalmanFilter {
public:
    // A: state transition (n x n)
    // B: control input model (n x m), may be n x 0 if there is no control input
    // H: measurement model (k x n)
    // Q: process noise covariance (n x n)
    // R: measurement noise covariance (k x k)
    // x0: initial state estimate (n x 1)
    // P0: initial state covariance (n x n)
    KalmanFilter() = default;
    KalmanFilter(Matrix A, Matrix B, Matrix H, Matrix Q, Matrix R, Matrix x0, Matrix P0);

    // Advance the state estimate using the process model and control input u.
    void predict(const Matrix& u);

    // Advance the state estimate with no control input.
    void predict();

    // Incorporate a measurement z, correcting the state estimate.
    void update(const Matrix& z);

    void setTransition(const Matrix& A) { A_ = A; }
    void setControlModel(const Matrix& B) { B_ = B; }
    void setProcessNoise(const Matrix& Q) { Q_ = Q; }
    void setMeasurementNoise(const Matrix& R) { R_ = R; }

    const Matrix& state() const { return x_; }
    const Matrix& covariance() const { return P_; }

private:
    Matrix A_;
    Matrix B_;
    Matrix H_;
    Matrix Q_;
    Matrix R_;
    Matrix x_;
    Matrix P_;
};

}  // namespace vehicledynamics
