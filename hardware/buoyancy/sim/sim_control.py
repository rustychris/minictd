"""
Simulate vertical motion of drifter and effect of controller
"""

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import numpy as np
from stompy import utils
from scipy.integrate import RK45,BDF

##

class Drifter(object):
    z_init=0.0 # positive-up initial position
    z_set=0.0 # target position

    # simulation timestep, and ~ sample period of the imu and pressure sensor
    dt=0.01 # (s)
    dt_control=0.1 # (s)
    g=9.8

    t_last_control=0.0 # time of the last control step

    z_eta=0.0
    z_bed=-4.0 # relative to freesurface at 0.0
    u_star=0.05
    Cd=1.0 # for cylinder, roughly
    
    # Drifter dimensions
    base_volume=0.001 # 1 L, with plunger out (maximum drifter volume)
    proj_area=np.pi*0.11 # without drogue
    mass=1.000 - 0.003 # i.e. 3g positive
    drifter_L=0.20 # used in determining buoyancy transition at the surface

    # Control parameters

    # 10-32 rod, 25rpm, 6mm / ml syring
    # 25/60. rev/s * 0.8 mm/rev => 0.33 mm/s
    # 0.33 mm/s /(6mm/ml) => 0.055ml/s

    plunger_cmd=0.0
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
        # Sense and control are on a slower time step
        if self.t_last_control+self.dt_control<=self.t_sec:
            self.update_control() # sets plunger_cmd
            self.t_last_control=self.t_sec
            
        # But the plunger is still acting on the last control message
        self.update_plunger() # applies control
        
        self.update_z_turb()
        self.update_z_buoyancy_drag()
        self.t_sec+=self.dt

    def update_control(self):
        """
        Compare z and setpoint, decide whether to move plunger
        """
        # update estimate of vertical velocity
        dt_control=self.t_sec-self.t_last_control
        
        w_instant=(self.z-self.z_last)/dt_control
        alpha=(dt_control/self.T_w_est)
        self.w_est=alpha*w_instant + (1-alpha)*self.w_est
        self.z_last=self.z
        
        # prop/deriv
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
        z=np.clip(self.z,self.z_bed,self.z_eta)
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
        rho_air=1.225  # kg/m3
        
        # dw/dt = g_net - rho_water * w |w| Cd A /  mass

        def deriv(t,wz):
            w,z=wz
            
            # positive up, net gravitational accel and buoyancy
            # if the drifter is at the surface, a small amount may be in air,
            # altering the buoyancy
            
            # This ends up being a bit much, and hard to integrate
            # make it ramp up -- goofy quadratic. total kludge, will
            # only kind of make sense when z is small
            air_fraction=np.clip(z/self.drifter_L,0,1) # 0: all in water, 1: all in air
            rho_effective=(1.0-air_fraction)*rho_water+air_fraction*rho_air
            # this is wrong
            rho_drifter=self.mass/self.volume
            g_net=self.g * (rho_effective/rho_drifter-1.0) 

            # drag
            R=-rho_water*w*np.abs(w)*self.Cd*self.proj_area/self.mass

            return [g_net+R,w]
        
        # w[n+1]=w[n] + self.dt*g_net - self.dt*rho_water*w[n+1]*np.abs(w[n])*self.Cd * self.proj_area / self.mass
        # linearize the drag term and make it implicit for stability
        # assume most of the drag is always wet.

        tN=self.t_sec+self.dt
        integrator=RK45(deriv,self.t_sec,[self.w,self.z],tN)
        while integrator.status=='running' and integrator.t<tN:
            msg=integrator.step()
        assert integrator.status!='failed',msg
        
        self.w=integrator.y[0]
        self.z=integrator.y[1]
        
        if self.z<self.z_bed:
            self.z=self.z_bed
            self.w=0.0 # inelastic collision with bed.
        
    def plot(self,num=1):
        gs = gridspec.GridSpec(2, 4)
        fig=plt.figure(num)
        fig.clf()
        
        ax_z=fig.add_subplot(gs[0,:-1])
        ax_z.plot(self.state['t_sec'],
                  self.state['z'],label='z')
        ax_z.axis(ymin=self.z_bed,ymax=self.z_eta+0.02)

        ax_hist=fig.add_subplot(gs[0,-1])
        ax_hist.hist(self.state['z'],np.linspace(self.z_bed,self.z_eta,100),
                     orientation='horizontal')

        ax_ctrl=fig.add_subplot(gs[1,:-1],sharex=ax_z)
        ax_ctrl.plot(self.state['t_sec'],
                     1e6*self.state['plunger_volume'],
                     label='plunge V (mL)')

        ax_ctrl.legend(fontsize=8)
        ax_ctrl.set_xlabel('seconds')
        ax_ctrl.set_ylabel('Plunger V (ml)')
        ax_z.set_ylabel('Depth (m)')
        fig.subplots_adjust(left=0.12,wspace=0.55)

#sim=Drifter(z_set=-2.0,Cd=2.0,T_deriv=10,u_star=0.005,T_w_est=1.0)
#sim=Drifter(z_set=-2.0,Cd=2.0,T_deriv=60,u_star=0.01,T_w_est=1.0)

sim=Drifter(z_set=-0.10,Cd=0.05,T_deriv=10,u_star=0.001,T_w_est=1.0,
            z_bed=-0.20,dt=0.1,
            deadband=0.03)
sim.integrate(300)
sim.plot(2)

# shorter T_w_est lead to tighter depth-holding, but in reality
# we have to be able to have long T_w_est in order to deal with
# turbulent motions.


# HERE:  consider options:
#  an acceleration term
#  two-stage PID, with an outer loop with a control variable of w,
#    and the inner loop with control variable plunger_cmd.
#  add motion planning -- max acceleration, max velocity, which
#    then output target velocity, and inner loop which adjust the plunger
#    to meet target velocity.
#
