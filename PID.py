import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error

        return output

class SimpleSystem:
    def __init__(self, dt):
        self.dt = dt
        self.w = 0
        self.w_history = []

    def simulate(self, controller, setpoint, timespan):
        self.w_history = []  # Clear history before each simulation
        self.w = 0
        for t in timespan:
            error = setpoint - self.w
            u = controller.update(error, self.dt)
            
            # Simple system dynamics (you can replace this with your own system model)
            self.w += u * self.dt

            self.w_history.append(self.w)

    def plot_response(self, timespan):
        plt.figure()
        plt.plot(timespan, self.w_history, label='System Response')
        plt.title('System Response to PID Control')
        plt.xlabel('Time')
        plt.ylabel('Output')
        plt.axhline(1, color='r', linestyle='--', label='Setpoint')
        plt.legend()
        plt.show()

   
            

          