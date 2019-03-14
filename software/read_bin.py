"""
Encapsulate parsing an output file from the minictd
"""

import numpy as np
import xarray as xr
from stompy.io import nmea
from stompy import utils

class DuckFile(object):
    def __init__(self,fn,header=None):
        self.fn=fn
        self.header=header

    def read_to_dataset(self):
        raw=self.read_to_numpy()

        string_cleaner=np.vectorize(self.clean_string)
        
        ds=xr.Dataset()
        for field in raw.dtype.names:
            if np.issubdtype(raw[field].dtype,np.dtype('S')):
                raw[field]=string_cleaner(raw[field])
            extra_dims=raw[field].shape[1:]
            dim_names=['frame'] + ["dim%d"%d for d in extra_dims]
            ds[field]=tuple(dim_names), raw[field]

        if 'gps_nmea' in ds:
            gps_ds=self.parse_gps(ds)
            ds.update(gps_ds,inplace=True)
            
        return ds
    
    def read_to_numpy(self):
        frames=[]
        
        # Assumes that the header does not change
        # Allows for STOP to be interspersed.
        with open(self.fn,'rb') as fp:
            header=fp.readline().strip()
            if self.header is None:
                self.header=header
            else:
                print("Replacing file's header:")
                print(header)
                print("With override:")
                print(self.header)
                header=self.header
                
            rec_dtype=self.header_to_dtype(header)
            hex_chars_per_frame=2*rec_dtype.itemsize

            # gather bytes
            hex_chars=[]
            count=0 # number of characters in hex_chars
            for line in fp:
                line=line.strip()
                if line==b'STOP':
                    if count:
                        print("%d of %d extra characters at STOP"%(count,hex_chars_per_frame))
                        import pdb
                        pdb.set_trace()
                    hex_chars=[]
                    continue
                hex_chars.append(line)
                count+=len(line)
                if count>=hex_chars_per_frame:
                    hex_buff=b"".join(hex_chars)
                    while len(hex_buff)>=hex_chars_per_frame:
                        frame_chars=hex_buff[:hex_chars_per_frame]
                        hex_buff=hex_buff[hex_chars_per_frame:]
                        # the replace(...) is because some versions of the
                        # firmware replace the last character for each sensor
                        # with a null character
                        framechar_str=frame_chars.decode('ascii').replace("\0","0")
                        raw_buff=bytes.fromhex(framechar_str)
                        frame=np.frombuffer(raw_buff,rec_dtype)
                        frames.append(frame)
                    hex_chars=[hex_buff]
                    count=len(hex_buff)
        frames=np.concatenate(frames)

        return frames
    def header_to_dtype(self,header):
        return np.dtype(eval(header))

    def clean_strings(self,data,field='gps_nmea'):
        for rec in range(len(data)):
            for sent,full in enumerate(data[rec][field]):
                end=full.find(b"\0")
                if end >=0:
                    full=full[:end]
                full=full.strip()
                data[rec][field][sent]=full

    def clean_string(self,s):
        end=s.find(b"\0")
        if end>=0:
            s=s[:end]
        return s.strip()

    def parse_gps(self,ds,field='gps_nmea'):
        """
        ds: dataset with decoded data, including NMEA sentences in the given
        field
        Pull a timeseries of lat/lon
        returns a new dataset with each fix.
        currently only processes $GPRMC sentences
        """
        fixes=[]

        all_sentences=ds[field].values

        all_indexes=np.ndindex(*ds[field].shape)
        
        for idx in utils.progress(all_indexes):
            sent=all_sentences[idx]
            if not sent.startswith(b'$GPRMC'): continue

            # latin is more forgiving than ascii
            try:
                parsed=nmea.parse_sentence(sent.decode('latin'))
            except nmea.ParseError as exc:
                continue
            if parsed['sentence']=='$GPRMC':
                frame=idx[0]
                fix=[frame,parsed['time'],parsed['lat'],parsed['lon']]
                fixes.append(fix)
        gps_ds=xr.Dataset()

        if len(fixes):
            idx,gps_time,lat,lon = zip(*fixes)

            gps_ds['gps_time']=('fix',),np.array(gps_time)
            gps_ds['fix_frame']=('fix',),np.array(idx)
            gps_ds['lat']=('fix',), np.array(lat)
            gps_ds['lon']=('fix',), np.array(lon)
        return gps_ds