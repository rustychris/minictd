import time
import numpy as np
import pandas as pd
import xarray as xr
from StringIO import StringIO

import device

##

sdc=device.SeaDuckComm()


##

sdc.connect()
sdc.enter_command_mode(timeout=10)

print sdc.send('scan_period',timeout=2)
sdc.disconnect()

##

# Some interface to run a scan across a wide range of frequencies
# Generate nyquist plot and/or bode plot.
# start with a nicer bundling of a single scan
def read_bin(response):
    """
    response is a list of strings, one line of text per string.
    scans for a datatype definition starting and ending with
    brackets, followed by base16 encoded binary data, followed
    by the string 'STOP'
    """
    # response[0] is the echo of the scan command
    # read until we get a data definition
    idx=0
    for idx in range(len(response)):
        line=response[idx]
        if line.strip().startswith('['):
            break
    else:
        # didn't find a data definition
        return None

    fmt_lines=[]
    while 1:    
        fmt_lines.append(line)
        if line.endswith(']'):
            break
        idx+=1
        line=response[idx]

    fmt="".join(fmt_lines)
    raw_lines=[]
    for line in response[idx+1:]:
        if line.startswith('STOP'):
            break
        raw_lines.append(line)
    raw="".join(raw_lines)

    fmt_dtype=np.dtype(eval(fmt)) # data=np.zeros((),eval(fmt))
    import base64

    raw_bin=base64.b16decode(raw)
    # print "Raw: %d bytes  dtype: %d bytes"%(len(raw_bin),fmt_dtype.itemsize)
    if len(raw_bin) == fmt_dtype.itemsize:
        decoded=np.fromstring(raw_bin,fmt_dtype)
    else:
        decoded=None
    return decoded

class Scan(object):
    # Ostensibly it's running at 96MHz (based on overclocking
    # setting in Teensyduino, so maybe the pdb has either a prescaler, or always runs at half
    # the system clock.
    pdb_f0=44100*1087 # 48 MHz
    pdb_period_min=100 # only works if adc is in very high speed
    dac_int_min=4 # interval of <4 skips dac advance.
    base_wave_n_samples=256
    Rsense=470
    Vgain=10.0 # gain on amp on the voltage sense
    max_loops=64
    max_time=5.0 # seconds
    auto_range=True # if signal appears to clip, rescan
    n_discard=0
    dac_shift=7
    dac_mid=2048
    debug=False
    
    def __init__(self,sdc,freq_hz,**kwargs):
        self.sdc=sdc
        self.freq_hz=float(freq_hz)

        self.__dict__.update(kwargs)
        
        self.n_loops=min(self.max_loops,
                         max(1,int(self.max_time*self.freq_hz)))
        
        if sdc is None:
            self.debug=True

    def setup_scan(self):
        if self.n_discard>self.n_loops-1:
            self.n_discard=self.n_loops-1
            
        stride=1
        dac_oversample=1
        while 1:
            period=int(self.pdb_f0/self.freq_hz * dac_oversample*stride/self.base_wave_n_samples)

            # round to multiple of dac_oversample
            period = (period//dac_oversample) * dac_oversample
            # This seems like cheating, but appears to be correct. even though it seems
            # like this leads to uneven intervals between dac sample writes, so far
            # the testing shows that it is correct (though still waiting to see if
            # oscope verifies).
            period += (dac_oversample-1)

            if period<self.pdb_period_min:
                # adc period too small, first try bumping up oversampling
                dac_oversample*=2
                continue

            if dac_oversample*stride>64:
                # assumes original buffer is 256 samples.  So if the adc is seeing less than
                # 4 samples per waveform, then we give up.
                raise Exception("Requested period is too small, frequency too great")

            # period is long enough, and adc gets enough samples, but can
            # dac handle the oversampling?
            if period < self.dac_int_min*dac_oversample:
                # nope - dac interval would be too small, so shift to using a smaller
                # waveform
                dac_oversample = dac_oversample/2
                stride*=2
                continue

            # adc period is okay, oversample interval is okay
            break

        self.dac_oversample=dac_oversample
        self.dac_stride=stride
        self.pdb_period=period
        self.real_freq_hz = self.pdb_f0/float(self.pdb_period) * self.dac_oversample*self.dac_stride/float(self.base_wave_n_samples)

    def do_scan(self):
        """ The work directly associated with talking to the logger board.
        sets self.scan_response to list of lines which came back, first line
        has the command which was sent
        """
        if self.debug:
            print "Fake scan"
            with open("../sketches/cond_spec/data1.csv") as fp:
                self.scan_response = ["scan"] + fp.readlines()
            self.real_freq_hz=self.freq_hz
        else:
            temp_connect=not self.sdc.connected
            if temp_connect:
                self.sdc.connect()
                self.sdc.enter_command_mode(timeout=10)
            try:
                self.sdc.send('scan_period=%d'%self.pdb_period,20)
                print self.sdc.send('stride=%d'%self.dac_stride,20)
                print self.sdc.send('dac_oversample=%d'%self.dac_oversample,20)
                self.sdc.send('n_loops=%d'%self.n_loops,20)
                self.sdc.send('dac_shift=%d'%self.dac_shift)
                self.sdc.send('dac_mid=%d'%self.dac_mid)
                self.sdc.send('n_discard=%d'%self.n_discard)

                print "Scanning..."
                self.scan_response=self.sdc.send('scan',timeout=10)
            finally:
                if temp_connect:
                    self.sdc.disconnect()

    def parse_scan(self):
        """ Parse lines of scan_response to result, a pandas dataframe
        """
        # read_bin returns an array, for now we only deal with a single item
        self.result=read_bin(self.scan_response[1:])[0]
            
    def process_scan(self):
        self.ds=ds=xr.Dataset()

        ds['adc']=( ('adc',), ['cell','shunt'])
        ds['sample']=( ('sample',), np.arange(len(self.result['cell_mean'])) )
        ds['value']= ( ('adc','sample'), [ self.result['cell_mean'].astype('f4'),
                                           self.result['shunt_mean'].astype('f4') ] )
        ds['variance']= ( ('adc','sample'), [ self.result['cell_var'],
                                              self.result['shunt_var']] )
        ds['std'] = np.sqrt( ds.variance )
        ds['dc_offset'] = ds.value.mean(dim='sample')
        ds['value'] -= ds.dc_offset

        # add in some data from known settings
        ds['dac_stride'] = self.dac_stride
        ds['dac_oversample'] = self.dac_oversample
        ds['dac_mid'] = self.dac_mid
        ds['n_loops'] = self.n_loops
        ds['n_discard']=self.n_discard
        ds['pdb_period']=self.pdb_period

        # not included
        # ds['dac']=np.tile(self.df_result.dac[::self.dac_oversample],self.dac_oversample)
        # ds['dac_oversampled'] = ( ('dacsample',), self.df_result.dac )

        self.process_derived()
        
    def process_derived(self):
        ds=self.ds
        N=len(ds.sample)
        cos_part=np.cos( self.dac_oversample * np.arange(N) * 2*np.pi / N )
        sin_part=np.sin( self.dac_oversample * np.arange(N) * 2*np.pi / N )
        # new code doesn't return the dac signal, so we just assume it's sinusoidal
        dac_tf_real = 1.0 # np.mean(cos_part * ds.dac )
        dac_tf_imag = 0.0 # np.mean(sin_part * ds.dac )

        Nadc=len(ds.adc)
        tfunc = np.zeros( (Nadc,2),'f8') 
        ds['mag']=( ('adc',), np.zeros(Nadc,'f8'))
        ds['angle_deg']=( ('adc',), np.zeros(Nadc,'f8'))
        ds['angle_samp']=( ('adc',), np.zeros(Nadc,'f8'))

        for adc_i,adc in enumerate(ds.adc):
            # Get the value of the transfer function here
            # evaluate the dac and adc parts independently, just in case?
            adc_mean=ds.value.isel(adc=adc_i)
            adc_tf_real = np.mean(cos_part * adc_mean )
            adc_tf_imag = np.mean(sin_part * adc_mean )

            response=(adc_tf_real + 1j*adc_tf_imag) / (dac_tf_real + 1j*dac_tf_imag)
            tfunc[adc_i,0]=np.real(response)
            tfunc[adc_i,1]=np.imag(response)
            angle_rad=-np.angle(response)
            ds['mag'].values[adc_i]        = np.abs(response)
            ds['angle_deg'].values[adc_i]  = angle_rad * 180 / np.pi
            ds['angle_samp'].values[adc_i] = angle_rad * len(ds.sample) /(2*np.pi)

        ds['transfer']=( ('adc','complex'), tfunc )
        # Z= V/i
        V=(tfunc[0,0] + 1j*tfunc[0,1]) / self.Vgain
        I=(tfunc[1,0] + 1j*tfunc[1,1]) / self.Rsense
        Z=V / I
        ds['Z'] = (('complex',), [np.real(Z), np.imag(Z)])

        ds['freq_request']=self.freq_hz
        ds['freq_actual'] =self.real_freq_hz

    def is_clipped(self):
        res=[]
        for adc_i in [0,1]:
            adc=self.ds.isel(adc=adc_i)
            if ( (adc.value.min() + adc.dc_offset < -15000)
                 or (adc.value.max() + adc.dc_offset > 15000) ):
                res.append(adc_i)
        return res
    
    def run(self):
        while 1:
            self.setup_scan()
            self.do_scan()
            self.parse_scan()
            self.process_scan()
            if self.auto_range:
                if len(self.is_clipped()):
                    if self.dac_shift>=15:
                        print "CLIPPED - but can't range down any more"
                        break
                    self.dac_shift+=1
                    print "CLIPPED - will rescan"
                    continue
            break
        
    def figure_waveform(self,num=None,dac_scale=1.0,reps=2,
                        delta=False):
        fig=plt.figure(num=num)
        fig.clf()
        fig,ax=plt.subplots(num=num)

        ds=self.ds

        colors=['b','r']
        lines=[]

        sample=ds.sample.values
        if reps==2:
            sample=np.concatenate( [sample,sample+len(sample)] )
        
        for adc_i,adc in enumerate(ds.adc.values):
            color=colors[adc_i]

            adc_mean=np.tile( ds.value.isel(adc=adc_i), reps)
            adc_std =np.tile( ds['std'].isel(adc=adc_i), reps)
            
            ax.plot(sample,adc_mean,color=color,label=adc)
            ax.fill_between(sample,
                            adc_mean - adc_std,
                            adc_mean + adc_std,
                            zorder=-2,alpha=0.4,color=color)
            ax.fill_between(sample, -adc_std, adc_std,
                            zorder=-2,alpha=0.4,color=color)
            lines+=["ADC %d"%adc_i,
                    r"response= %.3f$\angle$%.2f$^\circ$"%( float(ds.mag.isel(adc=adc_i)),
                                                            float(ds.angle_deg.isel(adc=adc_i))),
                    ""]

        Z=(ds.Z.values[0] + 1j* ds.Z.values[1])
        lines.append( r"Z= %.3f$\angle$%.2f$^\circ$ @ f=%.2f Hz"%( np.abs(Z), np.angle(Z) * 180/np.pi,
                                                                   self.real_freq_hz) )

        ax.text(0.05,0.05,"\n".join(lines),
                transform=ax.transAxes,va='bottom')

        ax.legend()



##         
if 1:
    sc=Scan(sdc,2000,dac_shift=4,dac_mid=2048,auto_range=True)
    sc.n_loops=20
    sc.n_discard=3
    sc.run()

    # 50kHz => actual 47181, pdb_period=127, dac_oversample=32
## 

sc.figure_waveform(3,reps=1,delta=False)

##
# Get all the settings in place:
sc=Scan(sdc,2000,dac_shift=4,dac_mid=2048,auto_range=True)
sc.n_loops=10
sc.n_discard=3
sc.run()

# Attempt at fast update scans:

plt.figure(1).clf()
fig,ax=plt.subplots(1,num=1)
cell_line=ax.plot(sc.result['cell_mean'],'b')[0]
shunt_line=ax.plot(sc.result['shunt_mean'],'r')[0]

sc.sdc.connect()
try:
    for i in range(10):
        t=time.time()
        scan_response=sc.sdc.send('scan',timeout=10)
        scan_time=time.time() - t
        result=read_bin(scan_response[1:])[0]
        t=time.time()
        cell_line.set_ydata(result['cell_mean'])
        shunt_line.set_ydata(result['shunt_mean'])
        plt.pause(0.001)
        draw_time=time.time() - t
finally:
    sc.sdc.disconnect()

# This is dominated by the scan time, at 0.21s, vs. 0.03 draw time.
# Next up:
#   Test this in water, see if I still get the crazy noise
##

class Sweep(object):
    def __init__(self,sdc,fmin_hz,fmax_hz,n_steps):
        self.sdc=sdc
        self.fmin_hz=fmin_hz
        self.fmax_hz=fmax_hz
        self.n_steps=n_steps
        self.log_steps=True

    def run(self):
        self.do_sweep()
        self.process_sweep()
        
    def do_sweep(self):
        unique=True
        if self.fmin_hz==self.fmax_hz:
            freqs=self.fmin_hz * np.ones(self.n_steps)
            unique=False
        elif self.log_steps:
            freqs=np.exp( np.linspace(np.log(self.fmin_hz),
                                      np.log(self.fmax_hz),
                                      self.n_steps) )
        else:
            freqs=np.linspace(self.fmin_hz,self.fmax_hz,self.n_steps)
        
        self.scans=[]

        last_freq=None
        last_shift=4
        
        for f in freqs:
            scan= Scan(sdc=sdc, freq_hz=f, dac_shift=last_shift )
            scan.setup_scan()
            if (not unique) or (last_freq is None) or (scan.real_freq_hz>last_freq):
                self.scans.append(scan)
                scan.n_discard=int(scan.n_loops//2)
                print "f=%g Hz"%(scan.real_freq_hz)
                scan.run()
                last_freq=scan.real_freq_hz
                last_shift=scan.dac_shift # in case of auto ranging.
            else:
                print "[skip]"

    def tile_samples(self,ds,target=256,dim='dacsample'):
        fac=target / ds.dims[dim]
        if fac==1:
            return ds
        else:
            new_ds=xr.concat([ds]*fac,dim=dim,data_vars='minimal')
            # to allow concatetation, dimension must be equal in size and
            # value
            # new_ds[dim][:]=np.arange(target)
            new_ds[dim]=( (dim,), np.arange(target))
            return new_ds
            
    def process_sweep(self):
        scan=self.scans[0].ds
        safe_vars=[v
                   for v in scan.data_vars
                   if 'sample' not in scan[v].dims ]
        all_ds = [self.tile_samples(scan.ds[ safe_vars]) for scan in self.scans]        

        self.ds=ds=xr.concat(all_ds,dim='scan')
        
        ds['freq_hz']= ds.freq_actual
        # ds['complex'] = ( ('complex',), [0,1])
        # ds['Z']= ( ('scan','complex'), [s.ds.Z for s in self.scans] )

    def figure_bode(self,num=None):
        fig=plt.figure(num=num)
        fig.clf()
        fig,(ax_amp,ax_phase)=plt.subplots(2,1,sharex=True,num=num)

        ds=self.ds

        # logf=np.log10(ds.freq_hz)
        Z=self.ds.Z.isel(complex=0) + 1j*self.ds.Z.isel(complex=1)
        ax_amp.loglog(ds.freq_hz, np.abs(Z) )
        ax_phase.semilogx(ds.freq_hz,np.angle(Z) * 180/np.pi )

    def figure_nyquist(self,num=None):
        fig=plt.figure(num=num)
        fig.clf()
        fig,ax=plt.subplots(num=num)

        ds=self.ds

        zr=self.ds.Z.isel(complex=0)
        zi=self.ds.Z.isel(complex=1)

        ax.plot(zr,zi,'k-')
        coll=ax.scatter(zr,zi,50,np.log10(ds.freq_hz),lw=0 )
        ax.axis('equal')
        plt.colorbar(coll,label="$\log_{10}(Hz)$",ax=ax)

#print "Sleeping"
#time.sleep(3.0)

sweep=Sweep(sdc,500,108000,60)
sweep.run()
sweep.figure_bode(1)
sweep.figure_nyquist(2)
