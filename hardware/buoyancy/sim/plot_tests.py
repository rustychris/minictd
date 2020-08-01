import numpy as np
from stompy import utils
import matplotlib.pyplot as plt

utils.path("../../../software")

import read_bin

## 

fig_num=3
# duck=read_bin.DuckFile('tests/DATA0040.BIN')
duck=read_bin.DuckFile('../../../software/DATA0001.BIN')

ds=duck.read_to_dataset()
valid=(ds.time>np.datetime64("2019-04-10"))
ds=ds.isel(frame=valid)

from matplotlib import gridspec

gs=gridspec.GridSpec(4,1)

fig=plt.figure(fig_num)
fig.clf()

ax_pressure=fig.add_subplot(gs[0,0])
atm_press=ds.pressure_dPa.values[0]
ax_pressure.plot(ds.time,-(ds.pressure_dPa-atm_press)/1000.0,label='Press. (dBar)')
ax_pressure.plot(ds.time,ds.buoy_z_m,label='Depth (m)')

ax_motor=fig.add_subplot(gs[1,0],sharex=ax_pressure)
ax_motor.plot(ds.time,ds.motor_position.isel(dim2=0),label='Motor position')
PISTON_RATE=0.055
ax_motor.plot(ds.time,ds.buoy_ctrl*1000./PISTON_RATE,label='Ctrl')

ax_ctrl=fig.add_subplot(gs[2,0],sharex=ax_pressure)
ax_ctrl.plot(ds.time,ds.buoy_ctrl,label='ctrl')
ax_ctrl.plot(ds.time,ds.buoy_ctrl_prop,label='ctrl_prop')
ax_ctrl.plot(ds.time,ds.buoy_ctrl_deriv,label='ctrl_deriv')
ax_ctrl.plot(ds.time,ds.buoy_ctrl_integ,label='ctrl_integ')

ax_ctrl.plot(ds.time,ds.motor_position.isel(dim2=0)*PISTON_RATE/1000,label='piston ml')


ax_mot_status=fig.add_subplot(gs[3,0],sharex=ax_pressure)
ax_mot_status.plot(ds.time,ds.motor_status.values[:,0],label='status_a')
ax_mot_status.set_yticks([0,1,2,3,4,5,6])
ax_mot_status.set_yticklabels(["Off",
                               "FWD",
                               "REV",
                               "FWD|REV",
                               "LIM",
                               "FWD|LIM",
                               "REV|LIM"])
    
ax_pressure.legend()
ax_motor.legend()
ax_ctrl.legend()
ax_mot_status.legend()

fig.set_size_inches([12,8],forward=True)
fig.tight_layout()
fig.savefig( "control-" + os.path.join(os.path.basename(duck.fn)) + ".png" )

##

# Other details
gs=gridspec.GridSpec(5,1)

fig=plt.figure(fig_num+1)
fig.clf()

ax_pressure=fig.add_subplot(gs[0,0])
atm_press=ds.pressure_dPa.values[0]
ax_pressure.plot(ds.time,-(ds.pressure_dPa-atm_press)/1000.0,label='Press. (dBar)')
# ax_pressure.plot(ds.time,ds.buoy_z_m,label='Depth (m)')

ax_motor=fig.add_subplot(gs[1,0],sharex=ax_pressure)
ax_motor.plot(ds.time,ds.motor_sense.isel(dim2=0),label='Motor sense')

ax_gyro=fig.add_subplot(gs[2,0],sharex=ax_pressure)
for comp in range(3):
    ax_gyro.plot(ds.time,ds.imu_euler_deg.isel(dim3=comp).values,label='gyro%d'%comp)

ax_acc=fig.add_subplot(gs[3,0],sharex=ax_pressure)
for comp in range(3):
    ax_acc.plot(ds.time,ds.imu_accel_m_s2.isel(dim3=comp).values,label='acc%d'%comp)

ax_mot_status=fig.add_subplot(gs[4,0],sharex=ax_pressure)
ax_mot_status.plot(ds.time,ds.motor_status.values[:,0],label='status_a')

ax_mot_status.set_yticks([0,1,2,3,4,5,6])
ax_mot_status.set_yticklabels(["Off",
                               "FWD",
                               "REV",
                               "FWD|REV",
                               "LIM",
                               "FWD|LIM",
                               "REV|LIM"])

# 
ax_pressure.legend()
ax_motor.legend()
ax_gyro.legend()
ax_acc.legend()
ax_mot_status.legend()

for ax in fig.axes[:-1]:
    plt.setp(ax.get_xticklabels(),visible=0)

fig.tight_layout()
fig.subplots_adjust(hspace=0.01)    


fig.set_size_inches([12,8],forward=True)
fig.tight_layout()
fig.savefig( "imu-" + os.path.join(os.path.basename(duck.fn)) + ".png" )


##

# there are some periods with the motor off, and some possible signal in
# acceleration.
period=[737174.9338505806, 737174.9348740082]
#   that period has a clear difference between the x spectrum and the
#   others, consistent with anisotropy
# period=[737174.9313865333, 737174.932124397]
#   that period has a slightly depressed x spectrum, but they are quite 
#   close to each other.
dnums=utils.to_dnum(ds.time)

sel=(dnums>=period[0])&(dnums<=period[1])

plt.figure(6).clf()
fig,(ax_spec,ax_t)=plt.subplots(2,1,num=6)


# x,y somewhat arbitrary here.a
ax_spec.psd(ds.imu_accel_m_s2.isel(dim3=0,frame=sel),Fs=10,NFFT=512,noverlap=400,label="Acc-z")
ax_spec.psd(ds.imu_accel_m_s2.isel(dim3=1,frame=sel),Fs=10,NFFT=512,noverlap=400,label="Acc-x")
ax_spec.psd(ds.imu_accel_m_s2.isel(dim3=2,frame=sel),Fs=10,NFFT=512,noverlap=400,label="Acc-y")
ax_spec.legend()
ax_spec.set_xlabel("Frequency (Hz)")

t_secs=ds.dec_seconds.isel(dec_seconds=sel).values
t_secs = t_secs-t_secs[0]
ax_t.plot(t_secs,ds.imu_accel_m_s2.isel(dim3=0,frame=sel),label="Acc-z")
ax_t.plot(t_secs,ds.imu_accel_m_s2.isel(dim3=1,frame=sel),label="Acc-x")
ax_t.plot(t_secs,ds.imu_accel_m_s2.isel(dim3=2,frame=sel),label="Acc-y")
ax_t.set_xlabel('Seconds')

# 1, 2 are similar, while 0 has a steeper rolloff.
# these are recorded as accel.x,y,z
# but how are those oriented on the board?
#   0 is x, which is oriented just to the right of the pin 1
#     marker when looking down on the top of the chip.  i.e.
#     if the '701' is oriented, then x is
#     x
#     |
# y--701  and z out of the page.

# this is consistent with x being vertical and having a different rolloff
# compared to y,z.
# that could mean that the large scales sampled by the vehicle are anisotropic
# due to bed/surface constraints.
# would be interesting to see if there is any signal of this as the vehicle
# is closer to the bed/surface.
ax_spec.set_xscale('log')

fig.set_size_inches([10,8],forward=True)
fig.tight_layout()

fig.savefig( "spec-" + os.path.join(os.path.basename(duck.fn)) + ".png" )
