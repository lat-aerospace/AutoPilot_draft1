#pragma once

namespace Ap {

namespace detail {
inline double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
}  // namespace detail

/// Header-only PID controller with anti-windup.
///
/// Anti-windup: integral is clamped to [-iMax, +iMax] and integration is
/// frozen when the output is saturated AND the error pushes further into
/// saturation (same sign as clamped output).
class PidController {
  public:
    PidController() = default;

    PidController(double kp, double ki, double kd,
                  double iMax, double outMin, double outMax)
        : m_kp(kp), m_ki(ki), m_kd(kd),
          m_iMax(iMax), m_outMin(outMin), m_outMax(outMax)
    {}

    /// Compute PID output for one timestep.
    /// @param error  setpoint - measurement (positive = need more output)
    /// @param dt     timestep in seconds (must be > 0)
    /// @return clamped control output
    double update(double error, double dt) {
        // Proportional
        double pTerm = m_kp * error;

        // Integral with anti-windup
        // Only integrate if output is NOT saturated, or if the error
        // would reduce the saturation (error sign opposes output clamp).
        bool saturated = (m_prevOutput >= m_outMax && error > 0.0) ||
                         (m_prevOutput <= m_outMin && error < 0.0);
        if (!saturated) {
            m_integral += error * dt;
            m_integral = detail::clamp(m_integral, -m_iMax, m_iMax);
        }
        double iTerm = m_ki * m_integral;

        // Derivative (on error; filtered by first-order difference)
        double dTerm = 0.0;
        if (dt > 0.0 && m_hasLastError) {
            dTerm = m_kd * (error - m_lastError) / dt;
        }
        m_lastError = error;
        m_hasLastError = true;

        // Sum and clamp
        double output = pTerm + iTerm + dTerm;
        output = detail::clamp(output, m_outMin, m_outMax);
        m_prevOutput = output;
        return output;
    }

    /// Reset integrator and derivative state (call on mode changes).
    void reset() {
        m_integral = 0.0;
        m_lastError = 0.0;
        m_hasLastError = false;
        m_prevOutput = 0.0;
    }

  private:
    double m_kp = 0.0;
    double m_ki = 0.0;
    double m_kd = 0.0;
    double m_iMax = 0.0;
    double m_outMin = -1.0;
    double m_outMax = 1.0;

    double m_integral = 0.0;
    double m_lastError = 0.0;
    bool   m_hasLastError = false;
    double m_prevOutput = 0.0;
};

}  // namespace Ap
