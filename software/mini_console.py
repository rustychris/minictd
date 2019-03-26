import serial
import subprocess
import sys
import serial.tools.miniterm as mod_miniterm
from serial.tools.miniterm import key_description

# this does figure out when zmodem is trying to start,
# fine for feeding characters to rz.
# but how to feed characters back?  not really a filter then.

class ZModem(mod_miniterm.Transform):
    """Detect and start zmodem downloads"""
    parent=None
    def __init__(self):
        super(ZModem,self).__init__()
        self.recent=["x","x"]
        
    def rx(self, text):
        for t in text:
            self.recent.pop(0)
            self.recent.append(t)
            if self.parent and self.parent.rz is None:
                if (self.recent[-2]=='*'):
                    if (self.recent[-1]=="\x18"):
                        if self.parent:
                            sys.stderr.write("<GOT ZMODEM starting rz>")
                            self.parent.start_rz()
                        else:
                            sys.stderr.write("<GOT ZMODEM no parent>")
                        sys.stderr.flush()
        return text

    def tx(self, text):
        return text

mod_miniterm.TRANSFORMATIONS['zmodem']=ZModem

class zMiniterm(mod_miniterm.Miniterm):
    def __init__(self,*a,**k):
        super(zMiniterm,self).__init__(*a,**k)
        self.rz=None # active rz process
        print("zMiniterm!")
    def update_transformations(self):
        super(zMiniterm,self).update_transformations()
        for xf in self.rx_transformations:
            xf.parent=self
        for xf in self.tx_transformations:
            xf.parent=self
    def start_rz(self):
        if self.rz is not None:
            sys.stderr("rz already running")
        else:
            self.rz=subprocess.Popen(["/usr/bin/rz","-b","-E","-vvv","-X"],
                                     stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE,
                                     stderr=sys.stderr)
    def reader(self):
        """loop and copy serial->console"""
        try:
            while self.alive and self._reader_alive:
                # read all that is there or wait for one byte
                data = self.serial.read(self.serial.in_waiting or 1)
                if data:
                    if self.raw:
                        self.console.write_bytes(data)
                    else:
                        text = self.rx_decoder.decode(data)
                        for transformation in self.rx_transformations:
                            text = transformation.rx(text)
                        self.console.write(text)
                    if self.rz is not None:
                        if self.rz.returncode is not None:
                            self.rz.wait()
                            self.rz=None
                            self.console.write("[rz quit]")
                        else:
                            self.rz.stdin.write(data)
                        
        except serial.SerialException:
            self.alive = False
            self.console.cancel()
            raise       # XXX handle instead of re-raise?

    def writer(self):
        """\
        Loop and copy console->serial until self.exit_character character is
        found. When self.menu_character is found, interpret the next key
        locally.
        """
        menu_active = False
        try:
            while self.alive:
                if self.rz is not None:
                    if self.rz.returncode is not None:
                        self.rz.wait()
                        self.rz=None
                        self.console.write("[rz quit]")
                        continue
                    c=self.rz.stdout.read(1)
                    self.serial.write(c)
                    text=self.rx_decoder.decode(c)
                    self.console.write(text) # show to user, too.
                    continue
                else:
                    try:
                        c = self.console.getkey()
                    except KeyboardInterrupt:
                        c = '\x03'
                if not self.alive:
                    break
                if menu_active:
                    self.handle_menu_key(c)
                    menu_active = False
                elif c == self.menu_character:
                    menu_active = True      # next char will be for menu
                elif c == self.exit_character:
                    self.stop()             # exit app
                    break
                else:
                    #~ if self.raw:
                    text = c
                    for transformation in self.tx_transformations:
                        text = transformation.tx(text)
                    self.serial.write(self.tx_encoder.encode(text))
                    if self.echo:
                        echo_text = c
                        for transformation in self.tx_transformations:
                            echo_text = transformation.echo(echo_text)
                        self.console.write(echo_text)
        except:
            self.alive = False
            raise
        
# hard to monkey patch main, so copy it. :-(

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# default args can be used to override when calling main() from an other script
# e.g to create a miniterm-my-device.py
def main(default_port=None, default_baudrate=9600, default_rts=None, default_dtr=None):
    """Command line tool, entry point"""

    import argparse

    parser = argparse.ArgumentParser(
        description="Miniterm - A simple terminal program for the serial port.")

    parser.add_argument(
        "port",
        nargs='?',
        help="serial port name ('-' to show port list)",
        default=default_port)

    parser.add_argument(
        "baudrate",
        nargs='?',
        type=int,
        help="set baud rate, default: %(default)s",
        default=default_baudrate)

    group = parser.add_argument_group("port settings")

    group.add_argument(
        "--parity",
        choices=['N', 'E', 'O', 'S', 'M'],
        type=lambda c: c.upper(),
        help="set parity, one of {N E O S M}, default: N",
        default='N')

    group.add_argument(
        "--rtscts",
        action="store_true",
        help="enable RTS/CTS flow control (default off)",
        default=False)

    group.add_argument(
        "--xonxoff",
        action="store_true",
        help="enable software flow control (default off)",
        default=False)

    group.add_argument(
        "--rts",
        type=int,
        help="set initial RTS line state (possible values: 0, 1)",
        default=default_rts)

    group.add_argument(
        "--dtr",
        type=int,
        help="set initial DTR line state (possible values: 0, 1)",
        default=default_dtr)

    group.add_argument(
        "--ask",
        action="store_true",
        help="ask again for port when open fails",
        default=False)

    group = parser.add_argument_group("data handling")

    group.add_argument(
        "-e", "--echo",
        action="store_true",
        help="enable local echo (default off)",
        default=False)

    group.add_argument(
        "--encoding",
        dest="serial_port_encoding",
        metavar="CODEC",
        help="set the encoding for the serial port (e.g. hexlify, Latin1, UTF-8), default: %(default)s",
        default='UTF-8')

    group.add_argument(
        "-f", "--filter",
        action="append",
        metavar="NAME",
        help="add text transformation",
        default=[])

    group.add_argument(
        "--eol",
        choices=['CR', 'LF', 'CRLF'],
        type=lambda c: c.upper(),
        help="end of line mode",
        default='CRLF')

    group.add_argument(
        "--raw",
        action="store_true",
        help="Do no apply any encodings/transformations",
        default=False)

    group = parser.add_argument_group("hotkeys")

    group.add_argument(
        "--exit-char",
        type=int,
        metavar='NUM',
        help="Unicode of special character that is used to exit the application, default: %(default)s",
        default=0x1d)  # GS/CTRL+]

    group.add_argument(
        "--menu-char",
        type=int,
        metavar='NUM',
        help="Unicode code of special character that is used to control miniterm (menu), default: %(default)s",
        default=0x14)  # Menu: CTRL+T

    group = parser.add_argument_group("diagnostics")

    group.add_argument(
        "-q", "--quiet",
        action="store_true",
        help="suppress non-error messages",
        default=False)

    group.add_argument(
        "--develop",
        action="store_true",
        help="show Python traceback on error",
        default=False)

    args = parser.parse_args()

    if args.menu_char == args.exit_char:
        parser.error('--exit-char can not be the same as --menu-char')

    if args.filter:
        if 'help' in args.filter:
            sys.stderr.write('Available filters:\n')
            sys.stderr.write('\n'.join(
                '{:<10} = {.__doc__}'.format(k, v)
                for k, v in sorted(TRANSFORMATIONS.items())))
            sys.stderr.write('\n')
            sys.exit(1)
        filters = args.filter
    else:
        filters = ['zmodem']

    while True:
        # no port given on command line -> ask user now
        if args.port is None or args.port == '-':
            try:
                args.port = mod_miniterm.ask_for_port()
            except KeyboardInterrupt:
                sys.stderr.write('\n')
                parser.error('user aborted and port is not given')
            else:
                if not args.port:
                    parser.error('port is not given')
        try:
            serial_instance = serial.serial_for_url(
                args.port,
                args.baudrate,
                parity=args.parity,
                rtscts=args.rtscts,
                xonxoff=args.xonxoff,
                do_not_open=True)

            if not hasattr(serial_instance, 'cancel_read'):
                # enable timeout for alive flag polling if cancel_read is not available
                serial_instance.timeout = 1

            if args.dtr is not None:
                if not args.quiet:
                    sys.stderr.write('--- forcing DTR {}\n'.format('active' if args.dtr else 'inactive'))
                serial_instance.dtr = args.dtr
            if args.rts is not None:
                if not args.quiet:
                    sys.stderr.write('--- forcing RTS {}\n'.format('active' if args.rts else 'inactive'))
                serial_instance.rts = args.rts

            serial_instance.open()
        except serial.SerialException as e:
            sys.stderr.write('could not open port {}: {}\n'.format(repr(args.port), e))
            if args.develop:
                raise
            if not args.ask:
                sys.exit(1)
            else:
                args.port = '-'
        else:
            break

    miniterm = zMiniterm(
        serial_instance,
        echo=args.echo,
        eol=args.eol.lower(),
        filters=filters)
    miniterm.exit_character = mod_miniterm.unichr(args.exit_char)
    miniterm.menu_character = mod_miniterm.unichr(args.menu_char)
    miniterm.raw = args.raw
    miniterm.set_rx_encoding(args.serial_port_encoding)
    miniterm.set_tx_encoding(args.serial_port_encoding)

    if not args.quiet:
        sys.stderr.write('--- Miniterm on {p.name}  {p.baudrate},{p.bytesize},{p.parity},{p.stopbits} ---\n'.format(
            p=miniterm.serial))
        sys.stderr.write('--- Quit: {} | Menu: {} | Help: {} followed by {} ---\n'.format(
            key_description(miniterm.exit_character),
            key_description(miniterm.menu_character),
            key_description(miniterm.menu_character),
            key_description('\x08')))

    miniterm.start()
    try:
        miniterm.join(True)
    except KeyboardInterrupt:
        pass
    if not args.quiet:
        sys.stderr.write("\n--- exit ---\n")
    miniterm.join()
    miniterm.close()

        

if __name__=='__main__':
    main()
    
##

# this is how it responds to sz command:
# [RX:u'*'] [RX:u'\x18A\x04\x00\x00\x00\x00'] [RX:u'\ufffd\x06DATA0'] [RX:u'008.B'] [RX:u'IN\x0076'] [RX:u'80'] 80 [RX:u' ']
# [RX:u'0 0 0 '] [RX:u'1 '] [RX:u'7'] [RX:u'680'] [RX:u'\x00\x18'] [RX:u'k'] [RX:u'k\ufffd'] [RX:u'\x11']
# TX stuff
# [RX:u'*'] [RX:u'*\x18B0800000000022d\r\ufffd'] 

# so it's
# * \x18 A \x04 \x00 \x00 \x00 \x00 \ufffd \x06
# DATA0008.BIN
# 76

# somewhere in there ought to be a ZRQINIT frame.
# '*' is the character that begins frames.
# Seems that a frame should be  ZPAD ('*') ZDLE (030) ZBIN ('A')
# I'm getting \x18 instead of 030.  But is 030 octal?
#  yes, so those are the same.
# and I'm getting type \x04.
# which is ZFILE -- it's telling me the filename.
# but maybe here we can just detect '*' \x18, and then fire up rz?

# rz gets patched into the reader and writer, I assume.
# 
