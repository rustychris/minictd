import serial
import os
import sys
import glob
import logging
import serial.tools.list_ports
import time
import datetime
import weakref
import threading
import numpy as np
import collections

# Even with threads, this code sometimes drops data
# while the serial console does not.


class TextRing(collections.deque):
    def __init__(self,maxlen):
        super(TextRing,self).__init__(maxlen=maxlen)
        self.cond=threading.Condition()
        
    def write(self,buff):
        with self.cond:
            self.extend(buff)
            self.cond.notify()

    def available(self):
        return len(self)>0

    def popleft_or_wait(self,deadline=None):
        """ deadline is when to give up on the pop
        """
        with self.cond:
            while not self.available():
                if deadline is not None:
                    timeout=deadline-time.time()
                    if timeout>=0:
                        self.cond.wait(timeout)
                    else:
                        # never got anything.
                        return None
                else:
                    self.cond.wait()
            # success
            return self.popleft()

    def clear(self):
        """ Remove any text sitting in the ring
        """
        with self.cond:
            # self.read(timeout=0)
            super(TextRing,self).clear()

    def read(self,n=None,timeout=None):
        """ Collect up to n characters, in up to timeout seconds,
        and return as a string.

        if n==None implies n=inf,
        timeout==None implies timeout=inf.

        so you have to specify at least one.
        """
        if n is None and timeout is None:
            raise Exception("Can't wait forever to read infinite characters")

        buff=[]
        if timeout==0:
            # simple case - return whatever is in the buffer, up to length limit 
            with self.cond:
                while self.available() and ((n is None) or (len(buff)<n)):
                    buff.append(self.popleft())
        elif timeout is None and n is not None:
            with self.cond:
                while len(buff)<n:
                    while not self.available():
                        # possible that it's getting stuck here..
                        # had been no timeout - debugging with 10s
                        self.cond.wait(10) 
                    buff.append(self.popleft())
        else:
            # stop either when length or time limit is reached
            t_stop=time.time() + timeout
            with self.cond:
                while len(buff)<n:
                    while not self.available():
                        timeout=t_stop-time.time()
                        if timeout>0:
                            self.cond.wait(timeout)
                        else:
                            break
                    if self.available():
                        buff.append(self.popleft())
                    else:
                        break
        return ''.join(buff)

class SeaDuckComm(object):
    """ A class for interacting with an attached logger board
    """
    TEENSY31_USBID='16c0:483'
    prompt='seaduck>'
    timeout=10
    
    def __init__(self):
        self.serial=None
        self.last_port=None 
        logging.basicConfig(level=logging.INFO)
        self.log=logging.getLogger() # can change this to something more appropriate later
        self.log.level=logging.INFO
        self.consumers=[]
        self.serial_buffer=self.add_listener()

    def __del__(self):
        self.disconnect()

    def available_serial_ports(self):
        """ 
        returns a list: [ [port_path, port type, info, score], ... ]

        where score is a rough determination of how likely it is to be a freebird.
        
        """
        ports=[]
        for port_path,port_type,port_info in serial.tools.list_ports.comports():
            score=0
            if port_type=='USB Serial':
                score+=5
            if port_path.find('usbmodem')>=0:
                score+=5
            if port_info.find(self.TEENSY31_USBID)>=0:
                score+=10
            if port_path.find('HC-0')>=0:
                # on Mac, the BT2S adapter shows up as HC-06-DevB
                # by default.
                score+=7
            if port_path.find('freebird')>=0:
                # if the BT2S has been reprogrammed, should have freebird
                # in the name
                score+=15
            ports.append( [port_path,port_type,port_info,score] )
        ports.sort(key=lambda elt: -elt[3])
        return ports
            
    def connect(self,ports=None,min_score=15):
        """ Attempt to connect. 
        timeout: stop retrying after this many seconds.  0 for one shot, -1
          to try forever
        port: a list of paths to specific ports, otherwise, cycle through all ports
          above min_score, in order of decreasing score.
        min_score: vague heuristic for choosing which serial ports are good candidates.
           for starters, 15 means the USB ID matches a teensy 3.1
        return True if successful.
        """
        timeout=self.timeout
        
        t_start=time.time()
        while 1:
            self.log.debug("Trying to connect...")
            if ports:
                ports_to_try = [(p,"n/a","n/a",min_score+1) for p in ports]
            elif self.last_port:
                ports_to_try = [(self.last_port,"n/a","n/a",min_score+1)]
            else:
                ports_to_try = self.available_serial_ports()

            self.log.debug('available ports: %s'%str(ports_to_try))

            for port,ptype,pinfo,pscore in ports_to_try:
                if pscore<=min_score:
                    continue
                if self.open_serial_port(port):
                    self.log.debug("Opened port %s"%port)
                    self.last_port=port
                    self.io_thread=threading.Thread(target=self.listen)
                    self.io_thread.daemon=False # was True, experimenting..
                    self.io_thread.start()
                    time.sleep(0.05) # give that a chance to start up?
                    return True
            
            if timeout>=0 and time.time() > t_start+timeout:
                break
            time.sleep(1.0)
        return False
    
    def disconnect(self):
        if self.serial is not None:
            # in this order so that the reader thread will set
            # self.serial is None, before it sees that the port is
            # closed.
            if not self.io_thread.isAlive():
                self.log.warning("About to disconnect, but io thread already dead")
                
            port=self.serial
            self.serial=None
            port.close()
            self.log.debug("Joining IO thread")
            self.io_thread.join(10.0)
            if self.io_thread.isAlive():
                self.log.warning("Failed to join after waiting")
            self.io_thread=None

    @property
    def connected(self):
        return self.serial is not None

    def open_serial_port(self,port):
        self.disconnect()
        try:
            self.serial = serial.Serial(port=port,baudrate=115200,timeout=0)
        except Exception as exc:
            self.log.warning("Failed to open serial port %s"%port)
            self.log.warning("Exception: %s"%str(exc))
            return False
        return True
        
    def enter_command_mode(self,timeout=None):
        self.log.debug('entering command mode')

        timeout=timeout or self.timeout
        t_stop = time.time() + timeout

        # could have some stuff waiting around
        self.serial_buffer.clear()
        
        while time.time() < t_stop:
            self.write("!\r")
            while 1:
                line=self.readline(timeout=0.1)
                if line.strip()==self.prompt:
                    self.log.debug("enter_command_mode: found prompt - good")
                    return True
                elif len(line)==0:
                    break

        self.log.warning("readline timed out trying for command mode")
        return False

    # "low-level" I/O methods - multiplex access to the serial stream
    def add_listener(self):
        tring=TextRing(100*1024)
        self.consumers.append(tring)
        return tring
        
    def listen(self):
        """ should be started in separate thread
        """
        self.log.debug("Beginning listener loop")
        while self.serial:
            # try leaving the timeout at the original 0.1s
            #self.serial.timeout=1.0
            try:
                # the inWaiiting() part is important - this lets it always
                # empty the buffer as efficiently as possible
                buff=self.serial.read(self.serial.inWaiting())
                # serial can disappear at any moment, which can cause
                # a range of exceptions:
            except (serial.SerialException,AttributeError,IOError) as exc:
                if not self.serial:
                    # probably the main thread closed the port
                    pass
                else:
                    print "Serial exception ",exc
                    self.serial.close()
                    break
                buff=""
                
            if len(buff)>0:
                # sys.stdout.write(buff)
                # sys.stdout.flush()
                
                for tring in self.consumers:
                    tring.write(buff)
        self.log.debug("Leaving listener loop")

    def read(self,n=None,timeout='default'):
        """ return bytes ready from the serial port -
        callable in the main thread.
        if n is None or 0, return all available data.

        if timeout is None, wait forever.  otherwise, a decimal
        number of seconds to wait.
        if 'default', use self.timeout
        """
        if timeout is 'default':
            timeout=self.timeout
        return self.serial_buffer.read(n=n,timeout=timeout)
        
    def write(self,buff):
        self.serial.write(buff)
        self.serial.flush() # don't use flushOutput!  it deletes buffer contents!
        # maybe this is unnecessary?
        #time.sleep(0.05) # give that a chance to turn around?
        self.log.debug("Wrote %r"%buff)
        
    def readline(self,timeout='default'):
        """ Read data from the io_consumer until an end of line
        (either \r or \n) is seen.  If any single read takes longer
        than timeout, return whatever data has been read so far.

        returned lines include the EOL character, unless it timed out
        """
        buff=[]
        while 1:
            char=self.read(n=1,timeout=timeout)
            if char is None:
                break
            buff.append(char)
            if char in "\r\n":
                break
        return "".join(buff)

    # "high-level" methods for interacting with device
    def interact(self,msg,timeout=None):
        """ call/response interaction as a generator

        return values are single lines of output returned from device,
          with whitespace stripped from ends
          
        first line is msg as it was echoed back.
        blank lines are skipped
        assumes that response will end with the prompt - so this cannot
        be used to switch modes.

        if the read times out, with no prompt seen, returns None, and
        stops iteration.

        if a prompt is seen, it is not returned and iteration stops
        """
        # assumes that no input is coming down the line, but clear
        # the buffer in case the last command left some extra stuff
        # in there.
        self.serial_buffer.clear()

        self.write(msg+"\r")

        while 1:
            line=self.readline(timeout=timeout)
            # ton of output - but it never hung.
            # self.log.info("interact: readline=%r"%line)
            if len(line)==0:
                self.log.warning("Failed to get enough input for msg=%s"%msg)
                import pdb
                pdb.set_trace()
                yield None
                break
            line=line.strip()
            if line.find(self.prompt)>=0:
                # print "Found the prompt"
                break
            else:
                if len(line)>0:
                    yield line
            
    def send(self,cmd,timeout=None):
        """ Send a command, return a list of response lines
        """
        output=[]
        for line in self.interact(cmd,timeout=timeout):
            output.append(line)
        return output


