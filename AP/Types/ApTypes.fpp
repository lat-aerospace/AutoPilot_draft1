module Ap {

    @ 3D vector (NED position, velocity, etc.)
    struct Vec3 {
        x: F64  @< X / North component
        y: F64  @< Y / East component
        z: F64  @< Z / Down component
    }

    @ Quaternion for attitude representation (Hamilton convention)
    struct Quat {
        w: F64  @< Scalar part
        x: F64  @< Vector-i part
        y: F64  @< Vector-j part
        z: F64  @< Vector-k part
    } default {
        w = 1.0
    }

    @ Inertial measurement unit data
    struct ImuData {
        accel_mps2: Vec3   @< Accelerometer reading (m/s^2, body frame)
        gyro_dps: Vec3     @< Gyroscope reading (deg/s, body frame)
        time_s: F64        @< Timestamp (seconds since sim start)
    }

    @ GPS receiver data
    struct GpsData {
        lat_deg: F64       @< Latitude (degrees)
        lon_deg: F64       @< Longitude (degrees)
        alt_msl_m: F64     @< Altitude above mean sea level (m)
        vel_ned_mps: Vec3  @< Velocity in NED frame (m/s)
        time_s: F64        @< Timestamp (seconds since sim start)
    }

    @ Barometric altimeter data
    struct BaroData {
        pressure_pa: F64   @< Static pressure (Pa)
        altitude_m: F64    @< Pressure altitude (m)
        time_s: F64        @< Timestamp (seconds since sim start)
    }

    @ Magnetometer data
    struct MagData {
        field_gauss: Vec3  @< Magnetic field vector (Gauss, body frame)
        time_s: F64        @< Timestamp (seconds since sim start)
    }

    @ Full aircraft state (truth or estimated)
    struct AircraftState {
        position_ned: Vec3      @< Position in NED frame (m)
        velocity_ned: Vec3      @< Velocity in NED frame (m/s)
        attitude_quat: Quat     @< Attitude quaternion (body-from-NED)
        euler_deg: Vec3         @< Euler angles roll/pitch/yaw (deg)
        angular_rate_dps: Vec3  @< Body angular rates (deg/s)
        airspeed_ms: F64        @< True airspeed (m/s)
        time_s: F64             @< Timestamp (seconds since sim start)
    }

    @ Control surface deflection commands
    struct SurfaceCmd {
        aileron: F64   @< Aileron deflection (-1.0 to +1.0)
        elevator: F64  @< Elevator deflection (-1.0 to +1.0)
        rudder: F64    @< Rudder deflection (-1.0 to +1.0)
        $throttle: F64  @< Throttle setting (0.0 to 1.0)
    }

    @ RC receiver channel values
    struct RcChannels {
        roll: F32      @< Roll stick (-1.0 to +1.0)
        pitch: F32     @< Pitch stick (-1.0 to +1.0)
        yaw: F32       @< Yaw stick (-1.0 to +1.0)
        $throttle: F32  @< Throttle stick (0.0 to 1.0)
    }

    @ Flight mode enumeration
    enum FlightMode {
        FBWB    @< Fly-By-Wire-B (stick-mapped guidance)
        AUTO    @< Autonomous waypoint following
    }

    @ Guidance command — output of Autonomy, input of Controller
    struct GuidanceCmd {
        desired_alt_m: F64          @< Desired altitude MSL (m)
        desired_airspeed_ms: F64    @< Desired true airspeed (m/s)
        desired_heading_deg: F64    @< Desired heading (deg, 0=North, CW)
    }

    @ Mission waypoint for autonomous navigation
    struct MissionWaypoint {
        seq: U16          @< Waypoint sequence number
        lat_deg: F64      @< Latitude (degrees)
        lon_deg: F64      @< Longitude (degrees)
        alt_msl_m: F32    @< Altitude above MSL (m)
        speed_ms: F32     @< Desired speed at waypoint (m/s)
    }

}
