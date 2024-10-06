class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def update(self, current_value, dt):
        # Calculate error
        error = self.setpoint - current_value

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.previous_error) / dt
        D = self.Kd * derivative

        # Calculate output
        output = P + I + D

        # Update previous error
        self.previous_error = error

        return output

# Example usage of two-loop PID controller
if __name__ == "__main__":
    import time

    # Outer loop (Position PID) parameters
    Kp_pos = 1.0
    Ki_pos = 0.1
    Kd_pos = 0.05
    position_setpoint = 10.0  # Desired position

    # Inner loop (Speed PID) parameters
    Kp_speed = 1.0
    Ki_speed = 0.1
    Kd_speed = 0.05

    # Create PID controllers
    position_pid = PIDController(Kp_pos, Ki_pos, Kd_pos, position_setpoint)
    speed_pid = PIDController(Kp_speed, Ki_speed, Kd_speed, 0.0)  # Speed setpoint will be updated

    # Simulate a process
    current_position = 0.0
    current_speed = 0.0
    dt = 0.1  # Time step

    for _ in range(100):
        # Outer loop: Position control
        speed_setpoint = position_pid.update(current_position, dt)

        # Inner loop: Speed control
        speed_pid.setpoint = speed_setpoint
        control = speed_pid.update(current_speed, dt)

        # Simulate process response (for example purposes)
        current_speed += control * dt
        current_position += current_speed * dt

        # Print current values
        print(f"Position: {current_position:.2f}, Speed: {current_speed:.2f}")

        # Wait for next time step
        time.sleep(dt)