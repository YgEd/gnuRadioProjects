"""
genTXBlock.py

Modified TX packet source that works with BOTH gfsk_mod and generic_mod.
 
Key changes from the GFSK-only version:
1. Idle output sends alternating 0/1 instead of all zeros
   (keeps PSK/QAM timing recovery alive between packets)
2. Adds a postamble after each packet (guard bits so coded
   data isn't at the very end when loops might drift)
3. Everything else — FEC, CRC, whitening, framing — is identical.

The flowgraph handles bit packing:
  txBlock (unpacked bits) -> unpacked_to_packed -> generic_mod

For GFSK, connect directly:
  txBlock (unpacked bits) -> gfsk_mod (do_unpack=False)
"""

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

# Pad the 60-bit FEC-coded length field to 64 bits (8 bytes).
# Without this, the unpacked_to_packed_bb block in PSK/QAM mode
# grabs 4 bits from the interleaved payload to complete a byte,
# shifting everything downstream by 4 bits.
#
# Frame alignment check:
#   Preamble:     256 bits  (32 bytes)  ✓
#   Sync word:     24 bits  ( 3 bytes)  ✓
#   Length field:  64 bits  ( 8 bytes)  ✓  (was 60 — padded with 4 zeros)
#   Interleaved:  16*cols   (always multiple of 16, hence of 8)  ✓
#   Postamble:     64 bits  ( 8 bytes)  ✓
#
# RX must also read 64 bits for the length field and strip the 4 pad bits.
CODED_LEN_FIELD_PADDED = 64
LEN_FIELD_PAD_BITS = CODED_LEN_FIELD_PADDED - CODED_LEN_FIELD_BITS  # = 4

now = datetime.datetime.now().isoformat()
log_name = f"packet-log-{now}.csv"
sync_word = np.unpackbits(np.array([0x02, 0xb8, 0xdb], dtype=np.uint8)).tolist()


def whiten(data, seed=0x1FF):
    lfsr = seed
    out = []
    for byte in data:
        whitened = 0
        for bit in range(8):
            lfsr_bit = (lfsr >> 0) & 1
            data_bit = (byte >> bit) & 1
            whitened |= ((data_bit ^ lfsr_bit) << bit)
            feedback = ((lfsr >> 0) ^ (lfsr >> 4)) & 1
            lfsr = (lfsr >> 1) | (feedback << 8)
        out.append(whitened)
    return bytearray(out)


def crc8(data, poly=0x07, init=0x00):
    crc = init
    for byte in data:
        crc ^= int(byte)
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crc16(data, poly=0x8005, init=0xFFFF):
    crc = init
    for byte in data:
        crc ^= int(byte) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# Postamble: keeps timing recovery alive through the tail end of the packet
# so the last coded bits (which include CRC) don't get corrupted by loop drift
POSTAMBLE = [1, 0] * 32  # 64 bits alternating


class mav_packet_source(gr.sync_block):
    def __init__(self, freq, pack, set_bladerf_gain=None, bladerf=None, metrics_logger=None):
        gr.sync_block.__init__(
            self,
            name="MavLink Packet Source",
            in_sig=None,
            out_sig=[np.byte]
        )

        # 256 bits of alternating pattern — long enough for PSK/QAM
        # loop acquisition (Costas + M&M + freq tracking)
        self.preamble = np.unpackbits(
            np.array([0x55] * 32, dtype=np.uint8)
        ).tolist()

        self.sync_word = sync_word
        self.freq = freq
        self.packet_queue = []

        # Idle pattern state: alternating bit counter
        # This ensures continuous transitions even between packets,
        # which keeps the RX timing recovery loop locked.
        self._idle_phase = 0

        self._mav = mavlink2.MAVLink(None)
        self._mav.srcSystem = 255
        self._mav.srcComponent = 1
        self._pending_bits = []

        self.metrics_logger = metrics_logger
        self.bladerf = bladerf

        self.sitl = sitl.SITLManager()
        print("[TX] Attempting to start and connect to sitl")
        self.sitl.start()
        self._time_since_telem = int(time.time())
        self._time_since_gps = int(time.time())
        self._time_since_gain_change = int(time.time())

        self._gain_index = 0
        self.set_bladerf_gain = set_bladerf_gain

        self._fec_encoder = ConvolutionalEncoder()
        self._interleaver = BlockInterleaver()

        # packed flag
        self.pack = pack

    def string_to_bits(self, text):
        byte_array = np.array([ord(c) for c in text], dtype=np.uint8)
        return np.unpackbits(byte_array)

    def build_packet(self, message, raw=False):
        """Build a framed, FEC-coded packet from whitened payload bytes.

        Frame structure (all unpacked bits, every section multiple of 8):
        [preamble 256b | sync 24b | FEC-coded len 64b | FEC-coded payload | postamble 64b]
        """
        payload_bits = np.unpackbits(message)
        payload_bytes = list(np.packbits(np.array(payload_bits, dtype=np.uint8)))

        payload_crc = crc16(payload_bytes)
        payload_crc_bits = np.unpackbits(
            np.array([payload_crc >> 8, payload_crc & 0xFF], dtype=np.uint8)
        )

        # FEC: protect payload + CRC together
        data_bits = np.concatenate([payload_bits, payload_crc_bits])
        self._fec_encoder.reset()
        coded = self._fec_encoder.encode(data_bits)
        tail = self._fec_encoder.flush()
        coded = np.concatenate([coded, tail])

        payload_len = int(np.ceil(len(payload_bits) / 8))
        interleaved_len, cols, _ = compute_coded_length(payload_len)
        interleaved = self._interleaver.interleave(coded, cols)

        # FEC-encode the length field (produces 60 bits)
        coded_len_field = encode_length_field(payload_len, crc8)

        # Pad to 64 bits so the packer's byte boundaries align
        # with packet structure. RX strips these 4 zeros before decoding.
        coded_len_padded = np.concatenate([
            coded_len_field,
            np.zeros(LEN_FIELD_PAD_BITS, dtype=np.uint8)
        ])

        # Frame with postamble
        packet = np.concatenate([
            self.preamble,       # 256 bits (32 bytes)
            self.sync_word,      #  24 bits ( 3 bytes)
            coded_len_padded,    #  64 bits ( 8 bytes) — was 60, now padded
            interleaved,         #  multiple of 16, hence of 8
            POSTAMBLE            #  64 bits ( 8 bytes)
        ])

        # Verify every section is byte-aligned so generic_mod works
        assert len(self.preamble) % 8 == 0, f"Preamble not byte-aligned: {len(self.preamble)}"
        assert len(self.sync_word) % 8 == 0, f"Sync not byte-aligned: {len(self.sync_word)}"
        assert len(coded_len_padded) % 8 == 0, f"Length field not byte-aligned: {len(coded_len_padded)}"
        assert len(interleaved) % 8 == 0, f"Interleaved not byte-aligned: {len(interleaved)}"
        assert len(packet) % 8 == 0, f"Total packet not byte-aligned: {len(packet)}"

        packet_for_log = np.concatenate([
            self.sync_word,
            coded_len_padded,
            interleaved
        ])

        len_crc = crc8([payload_len >> 8, payload_len & 0xFF])
        crc_for_log = np.concatenate([
            np.unpackbits(np.array([len_crc], dtype=np.uint8)),
            payload_crc_bits
        ])

        if self.pack:
            # Pack the final bit array into bytes BEFORE queuing
            packet = np.packbits(packet.astype(np.uint8))
            print(f'[DEBUG] PACKING BEFORE SENDING')


        return (packet, packet_for_log, crc_for_log)

    def send_message(self, message, raw=False):
        if raw:
            print(f"mavlink packet in byte form: {[int(b) for b in message]}")

        whitened_msg = whiten(message)
        packet, packet_for_log, crc_for_log = self.build_packet(whitened_msg, raw)
        print(f"[mavGNUTX] Packet length: {len(packet)} bits, {float(len(packet))/8} bytes")

        payload_len_bytes = [len(message) >> 8, len(message) & 0xFF]
        len_crc_byte = crc8(payload_len_bytes)

        payload_bytes_list = list(np.packbits(np.unpackbits(whitened_msg)))
        payload_crc_val = crc16(payload_bytes_list)

        try:
            msg = self._mav.parse_char(message)
        except Exception as e:
            print(f"[GNUTXBlock] Error converting to mavlink string: {e}")

        packet_info = {
            'raw_payload_bytes': bytearray(message),
            'whitened_payload_bytes': whitened_msg,
            'payload_len': len(message),
            'payload_len_crc': bytearray([len_crc_byte]),
            'payload_crc': bytearray([payload_crc_val >> 8, payload_crc_val & 0xFF]),
            'raw_packet_bytes': bytearray(np.packbits(packet).tolist()),
            'message': msg
        }

        if self.metrics_logger is not None:
            self.metrics_logger.log_packet_outcome(
                'TX', self.freq, packet_info, success=True, ber='N/A'
            )

        self.packet_queue.extend(packet)

    def sendGuard(self, msg, curr_time):
        if msg.get_type() == 'HEARTBEAT':
            print(f"[mavGNUTX] Sending {msg.get_type()} Message")
            self.send_message(msg.pack(self._mav))

        # if msg.get_type() == 'STATUSTEXT':
        #     print(f"[mavGNUTX] Sending {msg.get_type()} Message")
        #     self.send_message(msg.pack(self._mav))

        # if msg.get_type() == 'GLOBAL_POSITION_INT':
        #     if self._time_since_gps + 2 < curr_time:
        #         print(f"[mavGNUTX] Sending {msg.get_type()} Message")
        #         self.send_message(msg.pack(self._mav))
        #         self._time_since_gps = int(time.time())

        # if (not msg.get_type() == 'HEARTBEAT') and (not msg.get_type() == 'GLOBAL_POSITION_INT'):
        #     if self._time_since_telem + 5 < curr_time:
        #         print(f"[mavGNUTX] Sending {msg.get_type()} Message")
        #         self.send_message(msg.pack(self._mav))
        #         self._time_since_telem = int(time.time())

    def setGain(self, curr_time):
        gain_set = None
        gains = [30.0, 20.0, 15.0, 10.0, 5.0, 3.0, 2.0, 1.0]
        if curr_time >= self._time_since_gain_change + 120:
            self._gain_index = (self._gain_index + 1) % len(gains)
            gain_set = self.bladerf.set_gain(gains[self._gain_index], 0)
            self._time_since_gain_change = int(time.time())
            self.set_bladerf_gain(gain_set)
        return gain_set

        
    def work(self, input_items, output_items):
        out = output_items[0]
        n_requested = len(out)

        if self.sitl is not None:
            msg = self.sitl.get_mavlink_msg()
            if msg is not None:
                gain_set = self.setGain(int(time.time()))
                if gain_set is not None:
                    print(f'[mavGNUTX] Gain updated to {gain_set}')
                self.sendGuard(msg, int(time.time()))

        if len(self.packet_queue) == 0:
            print("NO PACKETS IN QUEUE")
            # ---- KEY CHANGE: alternating bits instead of zeros ----
            # Zeros kill timing recovery in PSK/QAM because there are
            # no symbol transitions for the M&M loop to track.
            # Alternating 0/1 gives continuous transitions.
            # For GFSK this is also fine — it just produces a tone.
            out[:] = 0x55
            return n_requested

        n = min(n_requested, len(self.packet_queue))
        out[:n] = self.packet_queue[:n]
        self.packet_queue = self.packet_queue[n:]

        if n < n_requested:
            print("PACKET ISNT LONG ENOUGH")
            # Fill remainder with alternating too
            out[n:] = 0x55
            

        return n_requested

    def stop(self):
        self.sitl.stop()