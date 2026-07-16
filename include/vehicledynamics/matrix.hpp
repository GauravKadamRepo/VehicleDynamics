#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace vehicledynamics {

// Small dense matrix type used by the Kalman filter. Not optimized for
// large problems -- intended for the low state dimensions typical of a
// basic vehicle state estimator.
class Matrix {
public:
    Matrix() : rows_(0), cols_(0) {}

    Matrix(std::size_t rows, std::size_t cols, double fill = 0.0)
        : rows_(rows), cols_(cols), data_(rows * cols, fill) {}

    static Matrix identity(std::size_t n) {
        Matrix m(n, n);
        for (std::size_t i = 0; i < n; ++i) {
            m(i, i) = 1.0;
        }
        return m;
    }

    std::size_t rows() const { return rows_; }
    std::size_t cols() const { return cols_; }

    double& operator()(std::size_t r, std::size_t c) {
        return data_[r * cols_ + c];
    }

    double operator()(std::size_t r, std::size_t c) const {
        return data_[r * cols_ + c];
    }

    Matrix operator+(const Matrix& other) const {
        checkSameShape(other);
        Matrix result(rows_, cols_);
        for (std::size_t i = 0; i < data_.size(); ++i) {
            result.data_[i] = data_[i] + other.data_[i];
        }
        return result;
    }

    Matrix operator-(const Matrix& other) const {
        checkSameShape(other);
        Matrix result(rows_, cols_);
        for (std::size_t i = 0; i < data_.size(); ++i) {
            result.data_[i] = data_[i] - other.data_[i];
        }
        return result;
    }

    Matrix operator*(const Matrix& other) const {
        if (cols_ != other.rows_) {
            throw std::invalid_argument("Matrix::operator*: dimension mismatch");
        }
        Matrix result(rows_, other.cols_);
        for (std::size_t i = 0; i < rows_; ++i) {
            for (std::size_t k = 0; k < cols_; ++k) {
                double a_ik = (*this)(i, k);
                if (a_ik == 0.0) continue;
                for (std::size_t j = 0; j < other.cols_; ++j) {
                    result(i, j) += a_ik * other(k, j);
                }
            }
        }
        return result;
    }

    Matrix operator*(double scalar) const {
        Matrix result(rows_, cols_);
        for (std::size_t i = 0; i < data_.size(); ++i) {
            result.data_[i] = data_[i] * scalar;
        }
        return result;
    }

    Matrix transpose() const {
        Matrix result(cols_, rows_);
        for (std::size_t i = 0; i < rows_; ++i) {
            for (std::size_t j = 0; j < cols_; ++j) {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }

    // Gauss-Jordan elimination with partial pivoting.
    Matrix inverse() const {
        if (rows_ != cols_) {
            throw std::invalid_argument("Matrix::inverse: matrix must be square");
        }
        const std::size_t n = rows_;
        Matrix a(*this);
        Matrix inv = Matrix::identity(n);

        for (std::size_t col = 0; col < n; ++col) {
            std::size_t pivotRow = col;
            double pivotVal = std::fabs(a(col, col));
            for (std::size_t r = col + 1; r < n; ++r) {
                double v = std::fabs(a(r, col));
                if (v > pivotVal) {
                    pivotVal = v;
                    pivotRow = r;
                }
            }
            if (pivotVal < 1e-12) {
                throw std::runtime_error("Matrix::inverse: matrix is singular");
            }
            if (pivotRow != col) {
                a.swapRows(col, pivotRow);
                inv.swapRows(col, pivotRow);
            }

            double pivot = a(col, col);
            for (std::size_t j = 0; j < n; ++j) {
                a(col, j) /= pivot;
                inv(col, j) /= pivot;
            }

            for (std::size_t r = 0; r < n; ++r) {
                if (r == col) continue;
                double factor = a(r, col);
                if (factor == 0.0) continue;
                for (std::size_t j = 0; j < n; ++j) {
                    a(r, j) -= factor * a(col, j);
                    inv(r, j) -= factor * inv(col, j);
                }
            }
        }
        return inv;
    }

private:
    void checkSameShape(const Matrix& other) const {
        if (rows_ != other.rows_ || cols_ != other.cols_) {
            throw std::invalid_argument("Matrix: shape mismatch");
        }
    }

    void swapRows(std::size_t r1, std::size_t r2) {
        for (std::size_t j = 0; j < cols_; ++j) {
            std::swap((*this)(r1, j), (*this)(r2, j));
        }
    }

    std::size_t rows_;
    std::size_t cols_;
    std::vector<double> data_;
};

}  // namespace vehicledynamics
