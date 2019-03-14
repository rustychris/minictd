"""
Simulate vertical motion of drifter and effect of controller
"""

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import numpy as np
from stompy import utils

##

class Drifter(object):
    z_init=0.0 # positive-up initial position
    z_set=0.0 # target position

    # simulation timestep, and ~ sample period of the imu and pressure sensor
    dt=0.05 # (s)
    g=9.8
    
    z_eta=0.0
    z_bed=-4.0 # relative to freesurface at 0.0
    u_star=0.05
    Cd=1.0 # for cylinder, roughly
    
    # Drifter dimensions
    base_volume=0.001 # 1 L, with plunger out (maximum drifter volume)
    proj_area=np.pi*0.11 # without drogue
    mass=1.000 - 0.003 # i.e. 3g positive

    # Control parameters

    # 10-32 rod, 25rpm, 6mm / ml syring
    # 25/60. rev/s * 0.8 mm/rev => 0.33 mm/s
    # 0.33 mm/s /(6mm/ml) => 0.055ml/s
    
    plunger_rate=0.055 * 1e-6 # cc/s * m3/cc ~ m3/s
    plunger_max_volume=5 * 1e-6 # 5ml
    plunger_volume=0.0 # fully out
    deadband=0.5 # no action if  |PID|<deadband.  ~ m

    t_sec=0.0 # simulation time
    w=0.0  # vertical velocity of the drifter

    T_w_est=10.0 # time scale for low pass of w_est
    
    w_est=0.0 # estimated velocity from "pressure"
    z_last=0.0 # for w_est
    
    T_deriv=30.0 # s.  Time constant for derivative term
    
    state_dtype=[('t_sec',np.float64),
                 ('z',np.float64),
                 ('plunger_volume',np.float64),
                 ('w',np.float64),
                 ('w_est',np.float64)]
    
    def __init__(self,**kw):
        utils.set_keywords(self,kw)
        self.z=self.z_init
        self.reset_state()
        
    @property
    def volume(self):
        return self.base_volume-self.plunger_volume

    def reset_state(self):
        self.state=np.zeros(0,self.state_dtype)
        self.save_state()
    def save_state(self):
        s=np.zeros((),self.state_dtype)

        for name,dtype in self.state_dtype:
            s[name]=getattr(self,name)

        self.state=utils.array_append(self.state,s)

    def integrate(self,t_stop):
        while self.t_sec<t_stop:
            self.integrate_step()
            self.save_state()
    def integrate_step(self):
        self.t_sec+=self.dt

        self.update_control() # sets plunger_cmd
        
        self.update_plunger() # applies control 
        self.update_z_turb()
        self.update_z_buoyancy_drag()

    def update_control(self):
        """
        Compare z and setpoint, decide whether to move plunger
        """
        # update estimate of vertical velocity
        w_instant=(self.z-self.z_last)/self.dt
        alpha=(self.dt/self.T_w_est)
        self.w_est=alpha*w_instant + (1-alpha)*self.w_est
        self.z_last=self.z
        
        if 0: # proportional, only
            if self.z>self.z_set: # above set point
                self.plunger_cmd=1.0  # increase plunger volume
            elif self.z<self.z_set:
                self.plunger_cmd=-1.0 # decrease plunger volume
        else: # prop/deriv
            prop=self.z-self.z_set
            deriv=self.w_est
            PD=prop + deriv*self.T_deriv
            
            if PD>self.deadband:
                self.plunger_cmd=1.0  # increase plunger volume
            elif PD<-self.deadband:
                self.plunger_cmd=-1.0 # decrease plunger volume
            else:
                self.plunger_cmd=0.0
                
    def update_plunger(self):
        self.plunger_volume+=self.dt*self.plunger_cmd*self.plunger_rate
        self.plunger_volume=np.clip(self.plunger_volume,0,self.plunger_max_volume)
        
    def update_z_turb(self):
        """ Update z with effect of turbulence 
        """
        K=0.4 *self.u_star * (z-self.z_bed)*(self.z_eta-z)
        dKdz=0.4 *self.u_star * (self.z_bed+self.z_eta - 2*z)

        assert K>=0.0,"what?!"

        R=np.random.random()*2-1.0
        r=1./3
        dz=dKdz*self.dt + R*np.sqrt(2.0/r * K*self.dt)
        
        self.z+=dz
        # bounce
        while 1:
            if self.z > self.z_eta:
                self.z -= 2*(self.z-self.z_eta)
            elif self.z < self.z_bed:
                self.z += 2*(self.z_bed-self.z)
            else:
                break

    def update_z_buoyancy_drag(self):
        """ 
        Update w with the effect of buoyancy, drag,
        update z with w
        """

        rho_water=1000 # kg/m3
        # dw/dt = g_net - rho_water * w |w| Cd A /  mass

        # positive up, net gravitational accel and buoyancy
        g_net=self.g * (rho_water-self.mass/self.volume)
        
        # w[n+1]=w[n] + self.dt*g_net - self.dt*rho_water*w[n+1]*np.abs(w[n])*self.Cd * self.proj_area / self.mass
        # linearize the drag term and make it implicit for stability
        R=rho_water*np.abs(self.w)*self.Cd*self.proj_area/self.mass

        # w[n+1]=w[n] + self.dt*g_net - self.dt* R * w[n+1]
        # w[n+1] = (w[n] + self.dt*g_net) / (1+self.dt*R)
        new_w=(self.w+self.dt*g_net)/(1+self.dt*R)
        self.w=new_w

        self.z+=self.dt*self.w
        if self.z<self.z_bed:
            self.z=self.z_bed
        elif self.z>self.z_eta:
            self.z=self.z_eta
        
    def plot(self):
        gs = gridspec.GridSpec(2, 4)
        fig=plt.figure(1)
        fig.clf()
        
        ax_z=fig.add_subplot(gs[0,:-1])
        ax_z.plot(self.state['t_sec'],
                  self.state['z'],label='z')
        ax_z.axis(ymin=self.z_bed,ymax=self.z_eta)

        ax_hist=fig.add_subplot(gs[0,-1])
        ax_hist.hist(self.state['z'],np.linspace(self.z_bed,self.z_eta,100),
                     orientation='horizontal')

        ax_ctrl=fig.add_subplot(gs[1,:-1])
        ax_ctrl.plot(self.state['t_sec'],
                     1e6*self.state['plunger_volume'],
                     label='plunge V (mL)')

        ax_ctrl.legend(fontsize=8)

#sim=Drifter(z_set=-2.0,Cd=2.0,T_deriv=10,u_star=0.005,T_w_est=1.0)
sim=Drifter(z_set=-2.0,Cd=2.0,T_deriv=60,u_star=0.01,T_w_est=1.0)
sim.integrate(1000)

sim.plot()

# That comes out pretty close to uniform.  Good enough.

# With only proportional control it has a lot of overshoot,
# oscillates surface to bed.

# Observe w from z(t):
#   E-folding time scale of 1.0 s, for u_star=0.005 is good.
#   
