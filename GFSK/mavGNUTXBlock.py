import numpy as np
from gnuradio import gr
import pmt
from pymavlink import mavutil
import datetime
import csv
import os
import sitl_manager as sitl
from pymavlink.dialects.v20 import common as mavlink2

# construct logging file name
now = datetime.datetime.now().isoformat()
log_name = f"packet-log-{now}.csv"

def log_packet(filename, direction, **fields):

    log_dir = "packet-logs"

    # Create directory if it doesn't exist
    os.makedirs(log_dir, exist_ok=True)

    # Build full file path inside packet-logs
    filepath = os.path.join(log_dir, filename)

    file_exists = os.path.exists(filepath)

    file_exists = os.path.exists(filepath)
    
    with open(filepath, 'a', newline='') as f:
        writer = csv.writer(f)

        if not file_exists:
            writer.writerow([
                'timestamp', 'direction', 'payload_len', 'payload_len_crc', 'payload_crc',
                'raw_payload_bytes', 'whitened_payload_bytes', 'packet_bytes', 'message'
            ])
        
        def fmt(data):
            if isinstance(data, (bytes, bytearray)):
                return data.hex(' ')
            elif isinstance(data, (list, np.ndarray)):
                return ''.join(str(b) for b in data)
            return str(data)
        
        writer.writerow([
            datetime.datetime.now().isoformat(),
            direction,
            fields.get('payload_len', ''),
            fmt(fields.get('payload_len_crc','')),
            fmt(fields.get('payload_crc','')),
            fmt(fields.get('raw_payload_bytes', '')),
            fmt(fields.get('whitened_payload_bytes', '')),
            fmt(fields.get('packet_bytes', '')),
            fmt(fields.get('message', ''))
        ])




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
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc= (crc << 1) & 0xFF
    return crc

def crc16(data, poly =0x8005, init=0xFFFF):
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc



sync_word = np.unpackbits(np.array([0x02, 0xb8, 0xdb], dtype=np.uint8)).tolist()

class mav_packet_source(gr.sync_block):
    def __init__(self):
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

        self.packet_queue = []

        # mavlink stuff
        self._mav = mavlink2.MAVLink(None)
        self._mav.srcSystem = 255
        self._mav.srcComponent = 1
        self._pending_bits = []


        # attempt to connect to sitl
        self.sitl = sitl.SITLManager()
        print("[TX] Attempting to start and connect to sitl")
        self.sitl.start()



    
    def string_to_bits(self, text):
        # Convert string to list of bits MSB first
        # First convert text to ascii byte values, ord(char) returns unicode value
        byte_array = np.array([ord(c) for c in text], dtype=np.uint8)
        print(f"payload in byte format: {byte_array}")
        # Then convert the byte array to bit array that represents it
        bit_array = np.unpackbits(byte_array)
        return bit_array
    
    
    # expects bytes
    def build_packet(self, message, raw=False):
       
        payload_bits = np.unpackbits(message, dytpe=np.uint8)
        # payload_bits = np.unpackbits(np.array(message, dytpe=np.uint8))
        
        payload_bytes = list(np.packbits(np.array(payload_bits, dtype=np.uint8)))
        payload_crc = crc16(payload_bytes)
        payload_crc_bits = np.unpackbits(
            np.array([payload_crc >> 8, payload_crc & 0xFF], dtype=np.uint8)
        ).tolist()

        payload_len = np.ceil(len(payload_bits) / 8).astype(int)
        payload_len_bytes = np.array([payload_len >> 8, payload_len & 0xFF], dtype=np.uint8)
    
        # CRC-8 over the 2 length bytes
        len_crc = crc8(payload_len_bytes)

        len_crc_bits = np.unpackbits(
            np.array([len_crc], dtype=np.uint8)
        ).tolist()

        payload_len_bits = np.unpackbits(
            np.array(payload_len_bytes, dtype=np.uint8)
        ).tolist()

        # Repeat length field 3 times for majority voting
        payload_len_bits_voted = payload_len_bits * 3

        packet = np.concatenate([
            self.preamble,
            self.sync_word,
            payload_len_bits_voted,  # 48 bits instead of 16
            len_crc_bits,
            payload_bits,
            payload_crc_bits
        ])

        packet_for_log = np.concatenate([
            self.sync_word,
            payload_len_bits_voted,  # 48 bits instead of 16
            payload_bits
        ])

        crc_for_log = np.concatenate([
            len_crc_bits,
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


        log_packet(log_name, 'TX',
            raw_payload_bytes=bytearray(message),
            whitened_payload_bytes=whitened_msg,
            payload_len=len(message),
            payload_len_crc=bytearray([len_crc_byte]),
            payload_crc=bytearray([payload_crc_val >> 8, payload_crc_val & 0xFF]),
            packet_bytes=bytearray(np.packbits(packet_for_log).tolist()),
            message='Success'
        )
        self.packet_queue.extend(packet)

    
    def work(self, input_items, output_items):
        out = output_items[0]
        n_requested = len(out)

        if self.sitl is not None:
            hb_fields = self.sitl.get_heartbeat()
            if hb_fields is not None:
                # Reconstruct MAVLink hearbeat message
                hb_msg = self._mav.heartbeat_encode(
                    type= hb_fields['type'],
                    autopilot=       hb_fields['autopilot'],
                    base_mode=       hb_fields['base_mode'],
                    custom_mode=     hb_fields['custom_mode'],
                    system_status=   hb_fields['system_status'],
                    mavlink_version= hb_fields['mavlink_version'],
                )
                print("[TX] Sending Hearbeat")
                self.send_message(hb_msg.pack(self._mav))

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
    
