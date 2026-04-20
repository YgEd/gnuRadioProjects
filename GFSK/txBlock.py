import numpy as np
from gnuradio import gr
import pmt
from pymavlink import mavutil
import datetime
import csv
import os
import sitl_manager as sitl
from pymavlink.dialects.v20 import common as mavlink2
import time
from channel_coding import (
    ConvolutionalEncoder, BlockInterleaver,
    compute_coded_length, encode_length_field,
    INTERLEAVER_ROWS, CODED_LEN_FIELD_BITS
)

# construct logging file name
now = datetime.datetime.now().isoformat()
log_name = f"packet-log-{now}.csv"
sync_word = np.unpackbits(np.array([0x02, 0xb8, 0xdb], dtype=np.uint8)).tolist()




# whitening scrambles data before modulation so never long runs of identical bits which leads to problems on reconstruction
# This is because the demodulator needs to know exactly when each bit starts and ends
# During a identical stretch clock recovery algorithm is guessing where bit boundaries are based on its last restimate so slight drifts add up causing bit shifts
def whiten(data, seed=0x1FF):
    lfsr = seed
    out = []
    for byte in data:
        whitened = 0
        for bit in range(8):
            # XOR data bit with LFSR output
            lfsr_bit = (lfsr >> 0) & 1
            data_bit = (byte >> bit) & 1
            whitened |= ((data_bit ^ lfsr_bit) << bit)
            # Advance LFSR (x^9 + x^5 + 1)
            feedback = ((lfsr >> 0) ^ (lfsr >> 4)) & 1
            lfsr = (lfsr >> 1) | (feedback << 8)
        out.append(whitened)
    return bytearray(out)


def crc8(data, poly=0x07, init=0x00):
    crc = init
    for byte in data:
        crc ^= int(byte)
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc= (crc << 1) & 0xFF
    return crc

def crc16(data, poly =0x8005, init=0xFFFF):
    crc = init
    for byte in data:
        crc ^= int(byte) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc





class mav_packet_source(gr.sync_block):
    def __init__(self, freq, set_bladerf_gain=None, bladerf=None, metrics_logger=None):
        gr.sync_block.__init__(
            self,
            name="MavLink Packet Source",
            in_sig=None,
            out_sig = [np.byte]
        )


        # potentially self.preamble = [0x55] * 6          # 6 bytes timing recovery self.sync_word = [0xD3, 0x91]       # 2 byte sync word
        # Define preamble: at least 4 bytes / 32 bits
        self.preamble = np.unpackbits(np.array([37,85,85,85,85,85], dtype=np.uint8)).tolist()
        # Sync word: something that looks NOTHING like the preamble
        # 0xD391 is a common choice, or 0x2DD4 (Barker-like)
        self.sync_word = sync_word
        # Define postamble: at least 4 bytes / 32 bits
        self.freq = freq
        self.packet_queue = []

        # mavlink stuff
        self._mav = mavlink2.MAVLink(None)
        self._mav.srcSystem = 255
        self._mav.srcComponent = 1
        self._pending_bits = []

        # logger
        self.metrics_logger = metrics_logger

        # reference to sdr to vary gain
        self.bladerf = bladerf


        # attempt to connect to sitl
        self.sitl = sitl.SITLManager()
        print("[TX] Attempting to start and connect to sitl")
        self.sitl.start()
        self._time_since_telem = int(time.time())
        self._time_since_gps = int(time.time())
        self._time_since_gain_change = int(time.time())

        self._gain_index = 0
        self.set_bladerf_gain = set_bladerf_gain

        # FEC + interleaving for low-SNR robustness
        self._fec_encoder = ConvolutionalEncoder()
        self._interleaver = BlockInterleaver()


    
    def string_to_bits(self, text):
        # Convert string to list of bits MSB first
        # First convert text to ascii byte values, ord(char) returns unicode value
        byte_array = np.array([ord(c) for c in text], dtype=np.uint8)
        print(f"payload in byte format: {byte_array}")
        # Then convert the byte array to bit array that represents it
        bit_array = np.unpackbits(byte_array)
        return bit_array
    
    
    # expects bytes (whitened MAVLink payload)
    def build_packet(self, message, raw=False):

        payload_bits = np.unpackbits(message)
        payload_bytes = list(np.packbits(np.array(payload_bits, dtype=np.uint8)))

        # CRC-16 over the whitened payload bytes
        payload_crc = crc16(payload_bytes)
        payload_crc_bits = np.unpackbits(
            np.array([payload_crc >> 8, payload_crc & 0xFF], dtype=np.uint8)
        )

        # ---- FEC: protect payload + CRC together ----
        # Concatenate payload bits and CRC bits into one data stream
        data_bits = np.concatenate([payload_bits, payload_crc_bits])

        # Convolutional encode (rate 1/2) + flush tail
        self._fec_encoder.reset()
        coded = self._fec_encoder.encode(data_bits)
        tail = self._fec_encoder.flush()
        coded = np.concatenate([coded, tail])

        # Interleave (spreads burst errors across codeword)
        payload_len = int(np.ceil(len(payload_bits) / 8))
        interleaved_len, cols, _ = compute_coded_length(payload_len)
        interleaved = self._interleaver.interleave(coded, cols)
        # ---- end FEC ----

        # Length field: FEC-encoded (16-bit len + 8-bit CRC-8 = 60 coded bits)
        # Replaces old 3x majority voting (56 bits) — much stronger correction
        coded_len_field = encode_length_field(payload_len, crc8)

        # Frame: [preamble | sync | FEC-coded length(60) | FEC-coded payload]
        # Both length and payload are now inside FEC-protected regions
        packet = np.concatenate([
            self.preamble,
            self.sync_word,
            coded_len_field,
            interleaved              # FEC-coded payload+CRC
        ])

        packet_for_log = np.concatenate([
            self.sync_word,
            coded_len_field,
            interleaved
        ])

        len_crc = crc8([payload_len >> 8, payload_len & 0xFF])
        crc_for_log = np.concatenate([
            np.unpackbits(np.array([len_crc], dtype=np.uint8)),
            payload_crc_bits
        ])

        return (packet, packet_for_log, crc_for_log)

    def send_message(self, message, raw=False):
        if raw:
            print(f"mavlink packet in byte form: {[int(b) for b in message]}")
            pass
      
        whitened_msg = whiten(message)
        # print(f"whitened message {whitened_msg}, whiten function ran again: {whiten(whitened_msg)}")
        packet, packet_for_log, crc_for_log = self.build_packet(whitened_msg, raw)
        
        # Compute CRC bytes directly instead of slicing the bit array
        payload_len_bytes = [len(message) >> 8, len(message) & 0xFF]
        len_crc_byte = crc8(payload_len_bytes)
        
        payload_bytes_list = list(np.packbits(np.unpackbits(whitened_msg)))
        payload_crc_val = crc16(payload_bytes_list)

        try:
            msg = self._mav.parse_char(message) 
        except Exception as e:
            print(f"[GNUTXBlock] Error when converting payload bytes to mavlink string: {e}")


        packet_info = {
            'raw_payload_bytes':bytearray(message),
            'whitened_payload_bytes':whitened_msg,
            'payload_len':len(message),
            'payload_len_crc':bytearray([len_crc_byte]),
            'payload_crc':bytearray([payload_crc_val >> 8, payload_crc_val & 0xFF]),
            'raw_packet_bytes':bytearray(np.packbits(packet_for_log).tolist()),
            'message':msg
        }

        if self.metrics_logger is not None:
            self.metrics_logger.log_packet_outcome('TX', self.freq, packet_info, success=True, ber='N/A')
        
        self.packet_queue.extend(packet)


    def sendGuard(self, msg, curr_time):
        # message should be mavlink object

        # if msg is hearbeat send it right away
        if msg.get_type() == 'HEARTBEAT':
            print(f"[mavGNUTX] Sending {msg.get_type()} Message")
            self.send_message(msg.pack(self._mav))

        # if msg is status send right away
        if msg.get_type() == 'STATUSTEXT':
            print(f"[mavGNUTX] Sending {msg.get_type()} Message")
            self.send_message(msg.pack(self._mav))

        # if msg is GPS only send if 2 seconds have elapsed since the last time you send GPS
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            if self._time_since_gps + 2 < curr_time:
                print(f"[mavGNUTX] Sending {msg.get_type()} Message")
                self.send_message(msg.pack(self._mav))
                self._time_since_gps = int(time.time())
               
        
        # only send any other telem message if its been 5 seconds since last telem
        if (not msg.get_type() == 'HEARTBEAT') and (not msg.get_type() == 'GLOBAL_POSITION_INT'):
            if self._time_since_telem + 5 < curr_time:
                print(f"[mavGNUTX] Sending {msg.get_type()} Message")
                self.send_message(msg.pack(self._mav))
                self._time_since_telem = int(time.time())
    
        
    
    def setGain(self, curr_time):
        gain_set = None
        gains = [30.0, 20.0, 15.0, 10.0, 5.0, 3.0, 2.0, 1.0]
        if curr_time >= self._time_since_gain_change + 120:
            self._gain_index = (self._gain_index + 1) % len(gains)
            gain_set = self.bladerf.set_gain(gains[self._gain_index],0)
            self._time_since_gain_change = int(time.time())

            # update parents gain value so it is logged correctly
            self.set_bladerf_gain(gain_set)
        else:
            gain_set = None
        return gain_set

        


    
    def work(self, input_items, output_items):
        out = output_items[0]
        n_requested = len(out)
        

        if self.sitl is not None:
            msg = self.sitl.get_mavlink_msg()
            if msg is not None:
                
                # update gain amount
                gain_set = self.setGain(int(time.time()))
                if gain_set is not None:
                    print(f'[mavGNUTX] Gain updated to {gain_set}')
                # only send messages at set frequencies based on their type
                self.sendGuard(msg, int(time.time()))
                



        if len(self.packet_queue) == 0:
            # output zeros (idle when no data)
            # maybe return no data when there's no messages?
            out[:] = 0
            return n_requested
        
        # When a message is present output as many bits as we can
        n = min(n_requested, len(self.packet_queue))
        # output items becomes every packet in the queue up to n inclusive
        out[:n] = self.packet_queue[:n]
        # update the queue to now be everything we didn't output, aka everything past n exclusive
        self.packet_queue = self.packet_queue[n:]

        # fill remainig with zeros
        if n < n_requested:
            out[n:] = 0

        return n_requested
    
    def stop(self):
        self.sitl.stop()