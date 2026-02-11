#pragma once

#include "ApMath.hpp"

namespace Ap {

// --- DCM (Direction Cosine Matrix) from Euler angles (3-2-1 / ZYX) ---
// phi=roll, theta=pitch, psi=yaw  (radians)
inline Mat3d dcmFromEuler(F64 phi, F64 theta, F64 psi) {
    const F64 cp = std::cos(phi),   sp = std::sin(phi);
    const F64 ct = std::cos(theta), st = std::sin(theta);
    const F64 cy = std::cos(psi),   sy = std::sin(psi);

    Mat3d C;
    C(0,0) = ct*cy;           C(0,1) = ct*sy;           C(0,2) = -st;
    C(1,0) = sp*st*cy - cp*sy; C(1,1) = sp*st*sy + cp*cy; C(1,2) = sp*ct;
    C(2,0) = cp*st*cy + sp*sy; C(2,1) = cp*st*sy - sp*cy; C(2,2) = cp*ct;
    return C;  // body-from-NED
}

// --- Quaternion from Euler angles (3-2-1 / ZYX) ---
inline Quatd quatFromEuler(F64 phi, F64 theta, F64 psi) {
    // Eigen's AngleAxis composes as: yaw * pitch * roll
    return Quatd(
        Eigen::AngleAxisd(psi,   Vec3d::UnitZ()) *
        Eigen::AngleAxisd(theta, Vec3d::UnitY()) *
        Eigen::AngleAxisd(phi,   Vec3d::UnitX())
    );
}

// --- Euler angles from quaternion ---
// Returns (phi, theta, psi) in radians
inline Vec3d eulerFromQuat(const Quatd& q) {
    const Mat3d C = q.toRotationMatrix();
    const F64 theta = std::asin(-C(0,2));
    const F64 phi   = std::atan2(C(1,2), C(2,2));
    const F64 psi   = std::atan2(C(0,1), C(0,0));
    return Vec3d(phi, theta, psi);
}

// --- WGS-84 constants ---
constexpr F64 WGS84_A  = 6378137.0;          // semi-major axis (m)
constexpr F64 WGS84_F  = 1.0 / 298.257223563; // flattening
constexpr F64 WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F; // eccentricity^2

// --- NED offset from a reference LLA ---
// ref and point are (lat_rad, lon_rad, alt_m)
// Returns NED displacement in metres
inline Vec3d nedFromLla(const Vec3d& ref, const Vec3d& point) {
    const F64 dLat = point(0) - ref(0);
    const F64 dLon = point(1) - ref(1);
    const F64 dAlt = point(2) - ref(2);

    const F64 sinLat = std::sin(ref(0));
    const F64 Rn = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sinLat * sinLat);
    const F64 Rm = Rn * (1.0 - WGS84_E2) / (1.0 - WGS84_E2 * sinLat * sinLat);

    const F64 north = dLat * Rm;
    const F64 east  = dLon * Rn * std::cos(ref(0));
    const F64 down  = -dAlt;
    return Vec3d(north, east, down);
}

// --- LLA from NED offset + reference LLA ---
inline Vec3d llaFromNed(const Vec3d& ref, const Vec3d& ned) {
    const F64 sinLat = std::sin(ref(0));
    const F64 Rn = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sinLat * sinLat);
    const F64 Rm = Rn * (1.0 - WGS84_E2) / (1.0 - WGS84_E2 * sinLat * sinLat);

    const F64 lat = ref(0) + ned(0) / Rm;
    const F64 lon = ref(1) + ned(1) / (Rn * std::cos(ref(0)));
    const F64 alt = ref(2) - ned(2);
    return Vec3d(lat, lon, alt);
}

}  // namespace Ap
