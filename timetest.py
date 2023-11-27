import time
import numpy as np

class DiscreteSystem:
    def __init__(self, Ts):
        self.Ts = Ts  # Sampling time
        self.value = 0

    def update(self, control_input):
        # Simulate the discrete system response to the control input
        self.value += control_input * self.Ts  # A simple accumulation for demonstration purposes
        return self.value

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Target value
        self.error = 0  # Error term
        self.integral = 0  # Integral term
        self.derivative = 0  # Derivative term
        self.last_error = 0  # Last error term

    def update(self, measured_value):
        self.error = self.setpoint - measured_value

        # Proportional term
        P = self.Kp * self.error

        # Integral term
        self.integral += self.error
        I = self.Ki * self.integral

        # Derivative term
        self.derivative = self.error - self.last_error
        D = self.Kd * self.derivative

        # Calculate the output
        output = P + I + D

        # Update last error
        self.last_error = self.error

        return output

# Example usage
Ts = 0.1  # Sampling time
G = 7 / (Ts ** 2)  # Continuous transfer function: 7 / s^2

# Using Tustin method to approximate the discrete transfer function
numerator = [G * Ts ** 2]
denominator = [1, -2, 1]  # Represents the discrete equivalent of s^2: z^2 - 2z + 1

# Example usage
pid = PIDController(Kp=8, Ki=0.1, Kd=0.2, setpoint=15)
system = DiscreteSystem(Ts)

# Simulating the system controlled by the PID controller
for i in range(100):
    # Simulating a measured value (replace this with your real measurement)
    measured_value = system.update(0)  # Simulating a constant measured value of 0

    # Get control output
    control_output = pid.update(measured_value)

    # Apply the control output to the system
    system_value = system.update(control_output)

    print(f"Iteration {i+1}: Measured value: {measured_value}, Control output: {control_output}, System value: {system_value}")
    time.sleep(0.1)
      # Simulate some time passing between updates

prev_time=time.time()
time = time.time()
dt = time-prev_time
prev_time=time