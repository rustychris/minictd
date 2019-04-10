from stompy import utils
import matplotlib.pyplot as plt

utils.path("../../../software")

import read_bin

## 

fig_num=3
# duck=read_bin.DuckFile('tests/DATA0040.BIN')
duck=read_bin.DuckFile('../../../software/DATA0044.BIN')

ds=duck.read_to_dataset()

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
    
ax_pressure.legend()
ax_motor.legend()
ax_ctrl.legend()
ax_mot_status.legend()

fig.tight_layout()
