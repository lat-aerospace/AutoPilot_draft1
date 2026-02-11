#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace Ap {

// --- Scalar aliases ---
using F64 = double;
using F32 = float;

// --- Vector / Matrix typedefs ---
using Vec3d  = Eigen::Vector3d;
using Vec4d  = Eigen::Vector4d;
using Mat3d  = Eigen::Matrix3d;
using Quatd  = Eigen::Quaterniond;

// EKF state and covariance
using Vec15d = Eigen::Matrix<double, 15, 1>;
using Mat15d = Eigen::Matrix<double, 15, 15>;

// --- Constants ---
constexpr F64 PI       = 3.14159265358979323846;
constexpr F64 DEG2RAD  = PI / 180.0;
constexpr F64 RAD2DEG  = 180.0 / PI;
constexpr F64 GRAVITY  = 9.80665;   // m/s^2

}  // namespace Ap
