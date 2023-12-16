import numpy as np
import matplotlib.pyplot as plt
from DCMotor import DCMotor
from PID import PIDController
from PID import SimpleSystem
R = 1.0
L = 0.5
K = 0.01
J = 0.01
B = 0.1
dt = 0.01
u=1
motor = DCMotor(R, L, K, J, B, dt)
timespan = np.arange(0, 10, dt)
motor.simulate(u, timespan)
motor.plot_step_response(timespan)
dt = 0.1
pid_controller = PIDController(Kp=1, Ki=0.5, Kd=0.01)
system = SimpleSystem(dt=dt)

timespan = np.arange(0, 10, dt)
setpoint = 1.0

system.simulate(pid_controller, setpoint, timespan)
system.plot_response(timespan)


    



