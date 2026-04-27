from gnuradio import gr
import pmt
from pymavlink import mavutil
import csv
import numpy as np
from channel_coding import bits_to_bytes
from datetime import datetime




class blankSource(gr.sync_block):
    def __init__(self):
        gr.sync_block.__init__(
            self,
            name='blank source block',
            in_sig=None,
            out_sig=[np.byte]
        )

    
    def work(self, input_items, output_items):
        out = output_items[0]
        out[:] = 0xAA  # 10101010 — distinct from 0x55 (01010101)
        return len(out)