from gnuradio import gr
import pmt
from pymavlink import mavutil
import csv
import numpy as np
from channel_coding import bits_to_bytes
from datetime import datetime


class bitDebug(gr.sync_block):
    def __init__(self):
        gr.sync_block.__init__(
            self,
            name='bit debug block',
            in_sig=[np.byte],
            out_sig=None
        )

        self.time = str(datetime.now().minute) + ":" + str(datetime.now().second)

        self.all_bits = []
        self.found_sync = False
        self.count = 0

def work(self, input_items, output_items):
    data = input_items[0]
    for bit in data:
        val = int(bit) & 0xFF
        # if len(self.all_bits) >= 24:
        #     if self.all_bits[-24:] == sync:
        with open(f'bits_out-{self.time}.txt', 'a+') as f:
            f.write(f"{val}")
            self.count+=1
            if self.count == 8:
                f.write('\n')
                self.count = 0
            # dump next 200 bits after sync
            # self.found_sync = True
            # self.post_sync_count = 0
        # if self.found_sync and self.post_sync_count < 200:
        #     # write to file
        #     self.post_sync_count += 1
    return len(data)