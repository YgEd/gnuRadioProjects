from gnuradio import gr
import pmt
from pymavlink import mavutil
import csv
import numpy as np
from channel_coding import bits_to_bytes
import time



class debugBlock(gr.sync_block):
    def __init__(self):
        gr.sync_block.__init__(
            self,
            name="File write debug Block",
            in_sig=[np.byte],
            out_sig=None
        )

        self.time = time.time()
        self.HEARTBEAT_LEN = 67
        self.count = 0

    def work(self, input_items, output_items):
        data = input_items[0]
        with open(f'packed_output-{self.time}.txt', 'a+') as f:
            for byte in data:
                val = byte
                self.count+=1
                f.write(f"{val & 0xFF} ")
                if self.count == self.HEARTBEAT_LEN:
                    f.write("\n")
                    self.count = 0
        return len(data)
        # count = 0
        # byte = []
        # case1 = np.array([1,0,1,0,1,0,1,0], dtype=np.int32)
        # case2 = np.array([0,1,0,1,0,1,0,1], dtype=np.int32)
        # one_count = 0
        # heartbeat_count = 0
        # good_bytes = []
        # for bit in data:
        #     if count < 8:
        #             byte.append(bit)
        #             count+=1
        #     else:
        #         byte = np.array(byte, dtype=np.int32)
        #         if (not np.array_equal(case1, byte)) and (not np.array_equal(case2, byte)):
        #             f = open(f'pack_output-{self.time}.txt', 'a+')
        #             con_bytes = bits_to_bytes(byte)
        #             f.write(f"full byte: {con_bytes[0]}, in hex: {hex(con_bytes[0])} ")
        #             f.write(f"{one_count} idle bytes before this message\n")
        #             one_count = 0
        #             good_bytes.append(hex(con_bytes[0]))
        #             heartbeat_count += 1
        #             if heartbeat_count == self.HEARTBEAT_LEN:
        #                 f.write(f"full hearbeat packet length seen: {good_bytes}")
        #                 heartbeat_count = 0
        #                 good_bytes = []
        #         else:
        #             one_count += 1
        #         byte = []
        #         count = 0
        
        return len(input_items)