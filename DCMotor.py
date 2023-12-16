import numpy as np
import control as ctrl
import matplotlib.pyplot as plt



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
    def Model(self,u,ddtheta,i,):
        b=self.B
        J=self.J
        K=self.K
        R=self.R
        L=self.L
        ddtheta=(-b/J)*ddtheta+(K/J)*i
        di=(-K/L)*ddtheta-(R/L)*i +u/L
        return ddtheta,di
    def simulate(self,u,timespan):
        w_t=self.w0
        i_t=self.i0
        for t in timespan:
            ddtheta,di=self.Model(u,w_t,i_t)
            self.w_history.append(w_t)
            w_t_plus_1=w_t+ddtheta*self.dt
            i_t_plus1=i_t+di*self.dt
            w_t=w_t_plus_1
            i_t=i_t_plus1
    

    def plot_step_response(self, timespan):
      plt.figure()
      plt.plot(timespan,self.w_history)
      plt.title('Angular velocity')
      plt.xlabel('t')
      plt.ylabel('w')
      plt.axhline(1)
      plt.show()

    






