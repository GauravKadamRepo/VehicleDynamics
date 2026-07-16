cmake_minimum_required(VERSION 3.14)
project(VehicleDynamics CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

add_library(vehicledynamics_estimation
    src/kalman_filter.cpp
    src/vehicle_state_estimator.cpp
)
target_include_directories(vehicledynamics_estimation PUBLIC include)

add_executable(demo_vehicle_state_estimation
    examples/demo_vehicle_state_estimation.cpp
)
target_link_libraries(demo_vehicle_state_estimation PRIVATE vehicledynamics_estimation)

enable_testing()

add_executable(unit_tests
    tests/test_matrix.cpp
    tests/test_kalman_filter.cpp
    tests/test_main.cpp
)
target_link_libraries(unit_tests PRIVATE vehicledynamics_estimation)
target_include_directories(unit_tests PRIVATE tests)

add_test(NAME unit_tests COMMAND unit_tests)
