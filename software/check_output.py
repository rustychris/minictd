import six
import numpy as np
import read_bin
six.moves.reload_module(read_bin)
## 

# duck_file2=read_bin.DuckFile("/media/rusty/6637-3133/DATA0028.BIN")
duck_file2=read_bin.DuckFile("/media/rusty/3437-3133/DATA0010.BIN")

#data1=duck_file1.read_to_dataset()
data2=duck_file2.read_to_dataset()

## 

# 2019-03-08: 9772 frames, and only 27 fixes.
# should be fixed now.

ds=data2
import matplotlib.pyplot as plt
plt.figure(1).clf()
plt.plot(ds.lon,ds.lat,'b-o')

##

ds[ ['lat','lon','gps_time'] ].to_dataframe().to_csv("gps-track.csv")

##

plt.figure(2).clf()
plt.plot(data2.gps_time,data2.fix_frame,'r-o')

##

from stompy.spatial import proj_utils

xy=proj_utils.mapper('WGS84','EPSG:26910')( np.c_[data2.lon,data2.lat])

plt.figure(2).clf()
plt.plot(xy[:,0],xy[:,1],'g-o')
plt.axis('equal')

##

# And what about IMU output?
# seems fine.

# move it around...
plt.figure(2).clf()
fig,axs=plt.subplots(2,1,num=2,sharex=True)

for dim in range(3):
    axs[0].plot(data2.frame,data2.imu_euler_deg.isel(dim3=dim))
    axs[1].plot(data2.frame,data2.imu_accel_m_s2.isel(dim3=dim))
    
