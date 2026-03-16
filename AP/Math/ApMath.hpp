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

// --- Reference datum (origin for NED frame) ---
constexpr F64 REF_LAT_DEG = 35.0;
constexpr F64 REF_LON_DEG = -106.0;
constexpr F64 REF_ALT_MSL = 1600.0;
constexpr F64 REF_LAT_RAD = REF_LAT_DEG * DEG2RAD;
constexpr F64 REF_LON_RAD = REF_LON_DEG * DEG2RAD;
constexpr F64 R_EARTH     = 6378137.0;

}  // namespace Ap
