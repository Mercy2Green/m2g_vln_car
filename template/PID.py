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

# Example usage
if __name__ == "__main__":
    import time

    # PID controller parameters
    Kp = 1.0
    Ki = 0.1
    Kd = 0.05
    setpoint = 10.0  # Desired setpoint

    # Create PID controller
    pid = PIDController(Kp, Ki, Kd, setpoint)

    # Simulate a process
    current_value = 0.0
    dt = 0.1  # Time step

    for _ in range(100):
        # Update PID controller
        control = pid.update(current_value, dt)

        # Simulate process response (for example purposes)
        current_value += control * dt

        # Print current value
        print(f"Current Value: {current_value:.2f}")

        # Wait for next time step
        time.sleep(dt)