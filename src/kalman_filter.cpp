#include "vehicledynamics/kalman_filter.hpp"

namespace vehicledynamics {

KalmanFilter::KalmanFilter(Matrix A, Matrix B, Matrix H, Matrix Q, Matrix R, Matrix x0, Matrix P0)
    : A_(std::move(A)),
      B_(std::move(B)),
      H_(std::move(H)),
      Q_(std::move(Q)),
      R_(std::move(R)),
      x_(std::move(x0)),
      P_(std::move(P0)) {}

void KalmanFilter::predict(const Matrix& u) {
    x_ = A_ * x_ + B_ * u;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::predict() {
    x_ = A_ * x_;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::update(const Matrix& z) {
    Matrix Ht = H_.transpose();
    Matrix y = z - H_ * x_;
    Matrix S = H_ * P_ * Ht + R_;
    Matrix K = P_ * Ht * S.inverse();

    x_ = x_ + K * y;
    Matrix I = Matrix::identity(P_.rows());
    P_ = (I - K * H_) * P_;
}

}  // namespace vehicledynamics
