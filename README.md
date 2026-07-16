# VehicleDynamics

A basic Kalman filter for vehicle state estimation, written in C++17 with no
external dependencies.

## What's here

- `include/vehicledynamics/matrix.hpp` — small dense matrix type (add,
  subtract, multiply, transpose, inverse) used by the filter.
- `include/vehicledynamics/kalman_filter.hpp` / `src/kalman_filter.cpp` —
  a generic discrete-time linear Kalman filter (`predict` / `update`).
- `include/vehicledynamics/vehicle_state_estimator.hpp` /
  `src/vehicle_state_estimator.cpp` — specializes the generic filter to
  estimate longitudinal **position** and **velocity** by fusing:
  - a noisy accelerometer (IMU), used as the control input in the predict
    step, and
  - noisy, lower-rate position measurements (e.g. GPS), used in the update
    step.
- `examples/demo_vehicle_state_estimation.cpp` — simulates a vehicle that
  accelerates and then cruises, and prints true vs. raw-GPS vs.
  filter-estimated position/velocity over time.
- `tests/` — unit tests for the matrix operations and for filter
  convergence, using a small self-contained assert-based test runner (no
  external test framework required).

## Filter design

State: `x = [position, velocity]ᵀ`

Predict (constant-velocity kinematics with measured acceleration as the
control input `u`):

```
A = [[1, dt], [0, 1]]
B = [[0.5*dt^2], [dt]]
x = A*x + B*u
P = A*P*Aᵀ + Q
```

`Q` is derived from the accelerometer noise variance propagated through
`B` (`Q = B*Bᵀ*σₐ²`), so it's automatically recomputed for the current
`dt`.

Update (direct position measurement):

```
H = [1, 0]
y = z - H*x
S = H*P*Hᵀ + R
K = P*Hᵀ*S⁻¹
x = x + K*y
P = (I - K*H)*P
```

## Build & run

```sh
cmake -B build
cmake --build build

# Run the unit tests
./build/unit_tests

# Run the demo
./build/demo_vehicle_state_estimation
```
