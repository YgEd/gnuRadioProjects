#!/usr/bin/env python3
from gnuradio import gr, digital, blocks
from mavGNUBlock import mav_packet_source, mav_packet_reader
import threading
from pymavlink.dialects.v20 import common as mavlink2

class my_flowgraph(gr.top_block):
    def __init__(self):
        gr.top_block.__init__(self)
        
        # Blocks
        self.source = mav_packet_source()
        self.destination = mav_packet_reader()
        self.gfsk_mod = digital.gfsk_mod(
            samples_per_symbol=2,
            sensitivity=1.0,
            bt=0.35,
            verbose=False,
            log=False,
            do_unpack=False)
        self.samp_rate = samp_rate = 64e3
        self.throttle = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate, False)


        self.gfsk_demod = digital.gfsk_demod(
            samples_per_symbol=2,
            sensitivity=1.0,
            gain_mu=0.3,
            mu=0.5,
            omega_relative_limit=0.005,
            freq_error=0.0,
            verbose=False,
            log=False)
        # ... sink block (SDR, file, etc.)
        
        # Connections
        self.connect(self.source, self.gfsk_mod)
        self.connect(self.gfsk_mod, self.throttle)
        self.connect(self.throttle, self.gfsk_demod)
        self.connect(self.gfsk_demod, self.destination)
        # self.connect(self.gfsk_mod, self.sink)



def cli_thread(packet_source):
    mav = mavlink2.MAVLink(None)
    mav.srcSystem = 255
    mav.srcComponent = 1
    
    while True:
        cmd = input("Enter command: ")
        if cmd == 'arm':
            msg = mav.command_long_encode(1,1,400,0,1,0,0,0,0,0,0)
            packet_source.send_message(msg.pack(mav), True)
        elif cmd == 'guided':
            msg = mav.command_long_encode(1,1,176,0,1,4,0,0,0,0,0)
            packet_source.send_message(msg.pack(mav), True)
        elif cmd == 'quit':
            break

if __name__ == '__main__':
    tb = my_flowgraph()
    tb.start()
    
    # Run CLI in main thread (or separate thread)
    cli_thread(tb.source)
    
    tb.stop()
    tb.wait()