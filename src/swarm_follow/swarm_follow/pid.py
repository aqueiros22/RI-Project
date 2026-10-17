"""Discrete PID with filtered derivative and conditional integration."""

from dataclasses import dataclass


@dataclass
class PID:

    kp: float
    ki: float
    kd: float
    integral_limit: float = 1.0  # Error-seconds.
    derivative_tau: float = 0.10  # Low-pass time constant, seconds.

    def __post_init__(self):
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.derivative = 0.0

    def update(self, error, dt, output_limit, *, is_saturated=None):
        if dt <= 0.0 or output_limit < 0.0:
            raise ValueError(
                'dt must be positive and output_limit nonnegative')

        if self.previous_error is not None:
            raw_derivative = (error - self.previous_error) / dt
            alpha = dt / (self.derivative_tau + dt)
            self.derivative += alpha * (raw_derivative - self.derivative)
        self.previous_error = error

        candidate = 0.0
        if self.ki != 0.0:
            candidate = max(-self.integral_limit, min(
                self.integral_limit, self.integral + error * dt))
        proportional_derivative = (
            self.kp * error + self.kd * self.derivative)
        output = proportional_derivative + self.ki * candidate
        limited = max(-output_limit, min(output_limit, output))
        # Coupled actuators can saturate even below the scalar output limit
        # (e.g. wheel commands combining forward speed and angular speed).
        actuator_windup = (
            is_saturated is not None and is_saturated(output)
            and output * self.ki * error > 0.0)
        # Accept integration only if it does not push further into saturation.
        if ((output - limited) * (self.ki * error) <= 0.0
                and not actuator_windup):
            self.integral = candidate
        output = proportional_derivative + self.ki * self.integral
        return max(-output_limit, min(output_limit, output))
