"""
Simulate vertical motion of drifter and effect of controller
 - sim_control_direct.py: control variable is piston position
"""

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import numpy as np
from stompy import utils
from scipy.integrate import RK45,BDF

from scipy.signal import butter

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
    
    # control_mode='direct': plunger_cmd is the plunger position in ml
    # otherwise the percent power to apply to the motorn
    plunger_cmd=0.0
    
    plunger_rate=0.055 * 1e-6 # cc/s * m3/cc ~ m3/s
    plunger_max_volume=5 * 1e-6 # 5ml
    plunger_volume=0.0 # fully out
    deadband=0.5 # no action if  |PID|<deadband.  ~ m

    t_sec=0.0 # simulation time
    w=0.0  # vertical velocity of the drifter

    T_w_est=10.0 # time scale for low pass of w_est
    # T_a_est=20.0 # 

    a_est=0.0 # estimated acceleration
    w_est=0.0 # estimated velocity from "pressure"
    z_est=0.0 # estimate vertical position
    err_int=0.0 # integration of error
    
    z_last=0.0 # for w_est
    w_last=0.0 # for a_est

    # Terms in the PID[A] controller
    prop=0.0
    deriv=0.0
    integ=0.0
    accel=0.0
    
    rate_int=50.0 # [1/s], 0 to disable
    T_deriv=60.0 # s.  Time constant for derivative term
    T_accel=0.0 # s2
    P=1.0 # scale for proportional term

    # when the depth is above this fraction of the way between the surface
    # and set point, go full negative
    transit_fraction=0.3
    
    state_dtype=[('t_sec',np.float64),
                 ('z',np.float64),
                 ('z_set',np.float64),
                 ('plunger_volume',np.float64),
                 ('prop', np.float64),
                 ('deriv',np.float64),
                 ('accel',np.float64),
                 ('integ',np.float64),
                 ('w',np.float64),
                 ('w_est',np.float64),
                 ('a_est',np.float64),
                 ('z_est',np.float64)]
    
    def __init__(self,**kw):
        utils.set_keywords(self,kw)
        self.z=self.z_init
        self.reset_state()
        
    @property
    def volume(self):
        return self.base_volume-self.plunger_volume

    def init_filter(self):
        f_samp_Hz=(1./self.dt_control)
        f_nyq_Hz=f_samp_Hz/2.0
        f_cut_Hz=1./self.T_w_est
        
        B,A = butter(2, # second order
                     f_cut_Hz/f_nyq_Hz, # normalized [0,1] cutoff, relative to nyquist
        )
        self.lp_A=A
        self.lp_B=B
        self.z_x=np.zeros(len(A))
        self.z_y=np.zeros(len(B))

    def update_w_est(self):
        # butterworth lowpass
        # B: numerator
        # A: denominator
        self.z_x[2]=self.z_x[1]
        self.z_x[1]=self.z_x[0]
        self.z_x[0]=self.z
        self.z_y[2]=self.z_y[1]
        self.z_y[1]=self.z_y[0]

        self.z_y[0]=1./self.lp_A[0]*(self.lp_B[0]*self.z_x[0] + self.lp_B[1]*self.z_x[1] + self.lp_B[2]*self.z_x[2]
                                    -self.lp_A[1]*self.z_y[1] - self.lp_A[2]*self.z_y[2])
        self.w_est=(self.z_y[0]-self.z_y[1])/self.dt_control
        self.a_est=(self.z_y[0]-2*self.z_y[1]+self.z_y[2])/self.dt_control**2
        self.z_est=self.z_y[0]
            
    def reset_state(self):
        self.state=np.zeros(0,self.state_dtype)
        self.init_filter()
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

    control_mode='direct_limited' # direct, binary, proportional
    
    def update_control(self):
        """
        Compare z and setpoint, decide whether to move plunger
        """
        self.update_w_est()
        
        # should the prop term use z_est, or z?
        err=self.z_est-self.z_set
            
        self.prop=self.P*err
        self.deriv=self.T_deriv*self.w_est
        self.accel=self.T_accel*self.a_est
        self.integ+=err*self.dt_control*self.rate_int

        ctrl=self.prop + self.deriv + self.accel + self.integ

        if self.control_mode=='binary':
            if ctrl>self.deadband:
                self.plunger_cmd=1.0  # increase plunger volume
            elif ctrl<-self.deadband:
                self.plunger_cmd=-1.0 # decrease plunger volume
            else:
                self.plunger_cmd=0.0
        if self.control_mode=='proportional':
            self.plunger_cmd=np.interp(ctrl,
                                       [-self.deadband,self.deadband],
                                       [-1,1])
        if self.control_mode in ['direct','direct_limited']:
            in_transit=self.z_est/self.z_set < self.transit_fraction
            if in_transit:
                # use the integral term to remember the plunger volume needed to
                # get off the surface
                # it seems like deriv should be subtracted -- not sure why
                # but it appears to create less of an initial overshot if deriv is
                # added in (it would be negative here, so really it's decreasing
                # integ).
                self.integ=self.plunger_volume*1e6 - self.prop + self.deriv 
                self.plunger_cmd=100. # max neg.
            else:            
                if np.abs(ctrl-self.plunger_cmd)>self.deadband:
                    self.plunger_cmd=ctrl

    def update_plunger(self):
        if self.control_mode in ['binary','proportional']:
            self.plunger_volume+=self.dt*self.plunger_cmd*self.plunger_rate
        if self.control_mode=='direct':
            self.plunger_volume=self.plunger_cmd/1e6
        if self.control_mode=='direct_limited':
            req_change=self.plunger_cmd/1e6-self.plunger_volume
            real_change=np.clip(req_change,
                                -self.dt*self.plunger_rate,
                                self.dt*self.plunger_rate)
            self.plunger_volume+=real_change
            
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
        ax_z.plot(self.state['t_sec']-self.T_w_est/4.,
                  self.state['z_est'],label='z est lag')
        ax_z.plot(self.state['t_sec'],
                  self.state['z_set'],label='z set',lw=1.5,zorder=-2,color='0.7')
        ax_z.axis(ymin=self.z_bed,ymax=self.z_eta+0.02)
        ax_z.legend(fontsize=9)

        ax_hist=fig.add_subplot(gs[0,-1],sharey=ax_z)
        ax_hist.hist(self.state['z'],np.linspace(self.z_bed,self.z_eta,100),
                     orientation='horizontal')
        plt.setp(ax_hist.get_yticklabels(),visible=0)

        ax_ctrl=fig.add_subplot(gs[1,:-1],sharex=ax_z)
        ax_ctrl.plot(self.state['t_sec'],
                     1e6*self.state['plunger_volume'],
                     label='plunge V (mL)')
        ax_ctrl.plot(self.state['t_sec'],self.state['prop'],label='P')
        ax_ctrl.plot(self.state['t_sec'],self.state['integ'],label='I')
        ax_ctrl.plot(self.state['t_sec'],self.state['deriv'],label='D')
        ax_ctrl.plot(self.state['t_sec'],self.state['accel'],label='A')

        ax_ctrl.legend(fontsize=8,loc='upper left',bbox_to_anchor=[1.05,1.])
        ax_ctrl.set_xlabel('seconds')
        ax_ctrl.set_ylabel('Plunger V (ml)')
        plt.setp(ax_z.get_xticklabels(),visible=0)
        
        ax_z.set_ylabel('Depth (m)')

        fig.subplots_adjust(left=0.12,wspace=0.25,hspace=0.1)
        return fig
#sim=Drifter(z_set=-2.0,Cd=2.0,T_deriv=10,u_star=0.005,T_w_est=1.0)
#sim=Drifter(z_set=-2.0,Cd=2.0,T_deriv=60,u_star=0.01,T_w_est=1.0)

# T_deriv=10, T_accel=100, T_w_est=20. is not bad, still small
# oscillations.

# with transit_fraction=0.1
sim=Drifter(z_set=-0.10,Cd=0.15,
            P=3.2,T_deriv=19,T_accel=2.0,rate_int=0.12,
            control_mode='direct_limited',transit_fraction=0.1,
            u_star=0.0,T_w_est=10.0,
            z_bed=-0.20,dt=0.1,
            deadband=0.00)
sim.integrate(500)
sim.plot(3)


## 
# with transit_fraction=0.0, 200-280s to first reach depth.
# rate limited
sim=Drifter(z_set=-0.10,Cd=0.15,
            P=3.2,T_deriv=19,T_accel=2.0,rate_int=0.12,
            control_mode='direct_limited',transit_fraction=-100.0,
            u_star=0.01,T_w_est=10.0,
            z_bed=-0.20,dt=0.1,
            deadband=0.00)
sim.integrate(3000)
sim.plot(4)

## 
# using z_est for all control, no turbulence:
sim=Drifter(z_set=-0.10,Cd=0.15,
            P=9,T_deriv=30,T_accel=10.0,rate_int=0.3,
            control_mode='direct',
            u_star=0.00,T_w_est=10.0,
            z_bed=-0.20,dt=0.1,
            deadband=0.03)
sim.integrate(600)
sim.plot(2)

# butter at 20.0 second cutoff introduces about 5 second lag.



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


# What would it look like to add an acceleration term to the PID controller?
# c(t) = P*e(t) + I * int(e(t)) + D * de(t)/dt + A*d2e(t)/dt2
# A has units of time^2
# D has units of time -- with the interpretation that we'll get back to 0 error
#  in D time.
# say we're 0.1m high, w=0, and d2e/dt2=-0.01 m/s2
#      = P*0.1m + ...  + A*-0.01 m/s2
# depending on the physical details, maybe that's a good place to be, so
# 


