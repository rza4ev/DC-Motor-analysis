# DC-Motor-analysis
Analytical model and PID controller implementation for a DC motor system. This repository provides a systematic approach to understanding DC motor dynamics, modeling, and tuning PID controllers for precise control applications.

Reporting
DCMOTOR.py-file
import numpy as np
import matplotlib.pyplot as plt
NumPy (import numpy as np):
NumPy is a powerful numerical computing library in Python.
It provides support for large, multi-dimensional arrays and matrices, along with mathematical 
functions to operate on these arrays.
Matplotlib (import matplotlib.pyplot as plt):
Matplotlib is a 2D plotting library for creating static, animated, and interactive visualizations in 
Python.
The pyplot module is a collection of functions that provide a simple interface for creating 
various types of plots and charts.
class DCMotor:
def __init__(self, R,L, K, J, B,dt,):
self.R = R
self.L = L
self.K = K
self.J = J
self.B = B
self.dt=dt
self.w0=0
self.i0=0
self.w_history=[]
Class Definition:
class DCMotor:: Defines a class named DCMotor.
Constructor (__init__ method):
Initializes the attributes of the DCMotor class with default values:
R: Resistance of the motor.
L: Inductance of the motor.
K: Torque constant.
J: Inertia of the motor.
B: Damping constant.
dt: Time step for simulation.
w0: Initial angular velocity.
i0: Initial current.
w_history: A list to store the history of angular velocities during simulation.
Attributes:
self.R, self.L, self.K, self.J, self.B: Motor parameters.
self.dt: Time step for simulation.
self.w0: Initial angular velocity.
self.i0: Initial current.
self.w_history: List to store the history of angular velocities during simulation.
This class is intended to represent a DC motor with the specified parameters, and 
it provides a convenient way to initialize and store the motor state.
def Model(self,u,ddtheta,i,):
b=self.B
J=self.J
K=self.K
R=self.R
L=self.L
ddtheta=(-b/J)*ddtheta+(K/J)*i
di=(-K/L)*ddtheta-(R/L)*i +u/L
return ddtheta,di
The Model method in your code represents the mathematical model of a DC 
motor.
Explanation:
u: Input voltage to the motor.
ddtheta: Angular acceleration.
i: Current.
The equations are based on the electrical and mechanical dynamics of the DC 
motor. Here's a breakdown of the equations:
Angular Acceleration (ddtheta) Equation:
ddtheta = (-b / J) * ddtheta + (K / J) * i
The left side represents the change in angular acceleration over time.
The right side includes terms related to damping (b), inertia (J), and torque 
constant (K).
Current (i) Equation:
di = (-K / L) * ddtheta - (R / L) * i + u / L
The left side represents the change in current over time.
The right side includes terms related to torque constant (K), inductance (L), 
resistance (R), and the input voltage (u).
 These equations model the dynamics of a DC motor. The method takes the 
current values of u, ddtheta, and i, and it returns the updated values of 
angular acceleration (ddtheta) and current (i) based on the specified model. 
This model is typically used in simulations to predict the behavior of a DC 
motor over time given certain inputs and initial conditions.
 def plot_step_response(self, timespan):
 plt.figure()
 plt.plot(timespan,self.w_history)
 plt.title('Angular velocity')
 plt.xlabel('t')
 plt.ylabel('w')
 plt.axhline(1)
 plt.show()
The plot_step_response method is responsible for plotting the step response of 
the DC motor. Here's an explanation of the code:
plt.figure(): Creates a new figure for the plot.
plt.plot(timespan, self.w_history): Plots the angular velocity (w_history) against 
time (timespan). This line connects the points, creating a continuous curve.
plt.title('Angular velocity'): Sets the title of the plot to "Angular velocity."
plt.xlabel('t'): Labels the x-axis as "t" (time).
plt.ylabel('w'): Labels the y-axis as "w" (angular velocity).
plt.axhline(1): Adds a horizontal line at y = 1 for reference. This line can help 
visualize when the angular velocity reaches or stabilizes around 1.
plt.show(): Displays the plot.
This method is useful for visualizing how the angular velocity of the DC motor 
changes over time in response to a step input. The horizontal line at y = 1 can be 
helpful to identify the time it takes for the system to reach a steady-state or a 
desired value.
 
PID.py-file
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
The provided code defines a simple PID (Proportional-Integral-Derivative) 
controller in Python.
Explanation:
Initialization (__init__ method):
Initializes the PID controller with the specified proportional (Kp), integral (Ki), and 
derivative (Kd) gains.
prev_error: Stores the previous error for computing the derivative term.
integral: Accumulates the integral of the error for the integral term.
update method:
Takes the current error (error) and the time step (dt) as input.
Updates the integral term by accumulating the error over time (self.integral += 
error * dt).
Calculates the derivative term using the difference in errors (derivative = (error -
self.prev_error) / dt).
Computes the PID output using the proportional, integral, and derivative terms.
Updates the prev_error for the next iteration.
Returns the calculated PID output.
This PID controller can be used in control systems to adjust a control signal based 
on the error between a desired setpoint and the actual process variable. The 
proportional, integral, and derivative terms contribute to the overall control 
signal, allowing the controller to respond to different aspects of the system's 
behavior.
You can create an instance of this PIDController class and use the update method 
in a loop to iteratively compute the control signal and adjust your system 
accordingly.
class SimpleSystem:
def __init__(self, dt):
self.dt = dt
self.w = 0
self.w_history = []
def simulate(self, controller, setpoint, timespan):
self.w_history = [] # Clear history before each simulation
self.w = 0
for t in timespan:
error = setpoint - self.w
u = controller.update(error, self.dt)
# Simple system dynamics (you can replace this with your own system 
model)
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
The provided code defines a simple system (SimpleSystem) class with methods for 
simulation and plotting the response to PID control.
 Explanation:
 Initialization (__init__ method):
 Initializes the SimpleSystem class with a time step (dt), initial angular 
velocity (w), and an empty list to store the history of angular velocities 
(w_history).
 simulate method:
 Takes a PID controller (controller), a setpoint (setpoint), and a time span 
(timespan) as input.
 Clears the history before each simulation.
 Iterates over the time span and computes the error between the setpoint 
and the current system output (self.w).
 Uses the PID controller to compute the control input (u).
 Updates the system dynamics (in this case, a simple integration of the 
control input).
 Appends the current system output to the history.
 plot_response method:
 Plots the system response over time, comparing it to a setpoint.
 The x-axis represents time, and the y-axis represents the system output.
 Adds a horizontal dashed line at y = 1 to represent the setpoint.
 The plot is displayed with labels and a legend.
 To use this system and visualize its response to PID control, you would need 
to create instances of both the SimpleSystem and PIDController classes, 
define a time span, and call the simulate method. Finally, you can visualize 
the response using the plot_response method.
 
Test.py file
# Motor parameters
R = 1.0
L = 0.5
K = 0.01
J = 0.01
B = 0.1
dt = 0.01
u = 1
# Create an instance of DCMotor
motor = DCMotor(R, L, K, J, B, dt)
# Simulation time span
timespan = np.arange(0, 10, dt)
# Simulate the motor response to a step input
motor.simulate(u, timespan)
# Plot the motor step response
motor.plot_step_response(timespan)
# Parameters for the PID controller and system simulation
dt = 0.1
pid_controller = PIDController(Kp=1, Ki=0.5, Kd=0.01)
system = SimpleSystem(dt=dt)
# Time span and setpoint for system simulation
timespan = np.arange(0, 10, dt)
setpoint = 1.0
# Simulate the system response to the PID controller
system.simulate(pid_controller, setpoint, timespan)
# Plot the system response
system.plot_response(timespan)
Result: 
Both cases, the plots show the system's response to the given inputs. The 
first plot demonstrates the behavior of the DC motor to a step input, while 
the second plot illustrates the system's response to PID control with a 
specified setpoint.
For the PID-controlled system, you expect the system output to converge to the 
setpoint value due to the corrective actions taken by the PID controller.
