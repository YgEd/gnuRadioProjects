from gnuradio import gr
import pmt
from pymavlink import mavutil
import csv
import numpy as np
from genTXBlock import sync_word, whiten, crc8, crc16, log_name, CODED_LEN_FIELD_PADDED
from channel_coding import (
    ViterbiDecoder, BlockInterleaver,
    compute_coded_length, decode_length_field,
    CODED_LEN_FIELD_BITS
)
from gcsPublisher import GCSPublisher
import time

# Import your existing helpers from mavGNUBlock
# (whiten, crc8, crc16, log_packet, sync_word, log_name are defined there)
# This file only defines the updated class — import it alongside mavGNUBlock
class Colors:
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    RESET = '\033[0m'


class mav_packet_reader_with_metrics(gr.sync_block):
    """
    Drop-in replacement for mav_packet_reader that adds:
      - BER estimation from CRC outcomes (PER → BER conversion)
      - Calls metrics_logger.log_packet_outcome() after each packet attempt

    Usage in rx.py:
        from rf_metrics import mav_packet_reader_with_metrics
        self.destination = mav_packet_reader_with_metrics(
            metrics_logger=self.metrics_logger
        )
    """

    # MAVLink COMMAND_LONG is 33 bytes = 264 bits
    # Use as N in PER->BER conversion: BER = 1 - (1-PER)^(1/N)
    PACKET_LENGTH_BITS = 264
    THRESHOLD_BW = (2*np.pi)/100

    def __init__(self, freq, setSDRGain=None, metrics_logger=None, publish_to_gcs=False, host='127.0.0.1', port=8080, pfb=None, cl=None, varygain=False):
        gr.sync_block.__init__(
            self,
            name="MavLink Packet Reader (Metrics)",
            in_sig=[np.byte],
            out_sig=None
        )

        # ---- same setup as your original mav_packet_reader ----
        # Import sync_word from mavGNUBlock at runtime to avoid circular import
        self._whiten     = whiten
        self._crc8       = crc8
        self._crc16      = crc16


        self.preamble     = np.unpackbits(np.array([37,85,85,85,85,85], dtype=np.uint8))
        self.sync_word    = sync_word
        self.sync_len     = len(sync_word)
        self.state        = 'SEARCHING'
        self.freq = freq
        self.bit_buffer   = []
        self.payload_len  = 0
        self._stopped = False
        self.sitl = None
        self.publish = publish_to_gcs
        self.addr = (host, port)

        # for gain variation
        self.varygain = varygain
        self.setSDRGain = setSDRGain
        self.time_since_gain_change = time.time()
        self.gain_index = 0

        # set loop bandwidth for polyphase PLL and costas PLL
        self.pfb = pfb
        self.cl = cl
        self.time_since_sync = None

        self.packets_arr = []



        # self.sitl = sitl.SITLManager()
        # print("Attempting to connect to SITL...")
        # self.sitl.start()

        # ---- end original setup ----

        self.metrics_logger = metrics_logger

        # Running PER tracker for BER estimation
        self._packet_attempts = 0
        self._packet_failures = 0
        self._per_window      = 50    # rolling window size

        # Rolling outcome history for windowed PER
        self._outcome_history = []

        # FEC decoder + deinterleaver
        self._viterbi = ViterbiDecoder()
        self._interleaver = BlockInterleaver()

    def stop(self):
        if not self._stopped:
            self._stopped = True
            print("[MavReader] stop() called by GNU radio scheduler")
            if self.sitl is not None:
                self.sitl.stop()
        return True

    def setpfb(self, pfb):
        self.pfb = pfb
        return self.pfb

    def setcl(self, cl):
        self.cl = cl
        return self.cl
    
    def __del__(self):
        try:
            if not self._stopped and self.sitl is not None:
                self.sitl.stop()
        except Exception:
            pass

    def bits_to_bytes(self, bits):
        bit_array = np.array(bits, dtype=np.uint8)
        pad = (8 - len(bit_array) % 8) % 8
        if pad:
            bit_array = np.append(bit_array, np.zeros(pad, dtype=np.uint8))
        return bytearray(np.packbits(bit_array))

    def _estimate_ber(self, success: bool) -> float:
        """
        Maintain a rolling window of packet outcomes and convert
        windowed PER to BER using:  BER = 1 - (1 - PER)^(1/N)
        where N = packet length in bits.
        """
        self._outcome_history.append(0 if success else 1)
        if len(self._outcome_history) > self._per_window:
            self._outcome_history.pop(0)

        per = sum(self._outcome_history) / len(self._outcome_history)

        # Avoid log(0) edge cases
        if per >= 1.0:
            return 1.0
        if per <= 0.0:
            return 0.0

        ber = 1.0 - (1.0 - per) ** (1.0 / self.PACKET_LENGTH_BITS)
        return ber
    
    def gainSetter(self):
        gains = [30.0, 20.0, 15.0, 10.0, 5.0, 3.0, 2.0, 1.0]
        curr_time = time.time()
        
        if  curr_time >= self.time_since_gain_change + 2:
            self.gain_index = (self.gain_index + 1) % len(gains)
            gainset = self.setSDRGain(gains[self.gain_index])
            self.time_since_gain_change = time.time()
            print(f'[GNURXBlock] Updated gain to {gainset}db')
            return gainset
        




    def work(self, input_items, output_items):

        # narrow bandwidth if two syc words are found in a row
        now = time.time()
        
        # haven't seen sync in 2 seconds
        if self.time_since_sync is not None and self.time_since_sync + 5 < now:
            pfb_bw = self.pfb.loop_bandwidth()
            if round(self.THRESHOLD_BW/2 ,3) == round(pfb_bw,3):
                self.pfb.set_loop_bandwidth(pfb_bw*2)
                print(f'[rxBlock] No sync word seen in {now - self.time_since_sync}s increased pfb loop bandwidth to {self.pfb.loop_bandwidth()}')

        if len(self.packets_arr) == 2:
            if all(self.packets_arr):
                pfb_bw = self.pfb.loop_bandwidth()
                if round(self.THRESHOLD_BW,3) == round(pfb_bw,3):
                    # two successful packest in a row
                    self.pfb.set_loop_bandwidth(pfb_bw/2)
                    print(f'[rxBlock] reduced pfb loop bandwidth to {self.pfb.loop_bandwidth()}')
            else:
                if round(self.THRESHOLD_BW / 2,3) == round(self.pfb.loop_bandwidth(),3):
                    pfb_bw = self.pfb.loop_bandwidth()
                    self.pfb.set_loop_bandwidth(pfb_bw*2)
                    print(f'[rxBlock] increased pfb loop bandwidth to {self.pfb.loop_bandwidth()}')
            
            self.packets_arr = []

        if self.varygain:
            self.gainSetter()
        in_data = input_items[0]

        for bit in in_data:
            self.bit_buffer.append(int(bit))

            if self.state == 'SEARCHING':
                if len(self.bit_buffer) >= self.sync_len:
                    tail = self.bit_buffer[-self.sync_len:]
                    if np.array_equal(tail, self.sync_word):
                        print("[GNURXBlock] sync word found!")

                        # reset self.time_since_sync
                        self.time_since_sync = time.time()

                        self.bit_buffer   = []
                        self.state        = 'READ_LENGTH'
                        self.constructed_bits = list(self.sync_word)

            elif self.state == 'READ_LENGTH':
                if len(self.bit_buffer) >= CODED_LEN_FIELD_PADDED:
                    coded_len_bits = np.array(
                        self.bit_buffer[:CODED_LEN_FIELD_PADDED], dtype=np.uint8
                    )

                    # FEC-decode the length field (Viterbi + CRC-8 check)
                    self.payload_len, len_valid = decode_length_field(
                        coded_len_bits, crc8
                    )
                    self.length_bytes = bytearray([
                        self.payload_len >> 8, self.payload_len & 0xFF
                    ])

                    if not len_valid:
                        print(f"{Colors.RED}[GNURXBlock] Length field FEC decode FAILED{Colors.RESET}")
                        ber = self._estimate_ber(success=False)

                        # APPEND False to packets_arr
                        self.packets_arr.append(False)

                        if self.metrics_logger:
                            packet_info = {
                                'payload_len': self.payload_len,
                                'payload_len_crc': bytearray(1),
                                'payload_crc': bytearray(2),
                                'raw_payload_bytes': bytearray(0),
                                'whitened_payload_bytes': bytearray(0),
                                'raw_packet_bytes': bytearray(0),
                                'message': 'Length field FEC decode failed (CRC-8 mismatch after Viterbi)',
                                'failure_level': 'CRC payload length mismatch failure'
                                
                            }
                            self.metrics_logger.log_packet_outcome(
                                'RX', self.freq, packet_info,
                                success=False, ber=ber
                            )

                        self.bit_buffer = []
                        self.constructed_bits = []
                        self.state = 'SEARCHING'
                    else:
                        self.constructed_bits.extend(
                            self.bit_buffer[:CODED_LEN_FIELD_PADDED]
                        )
                        print(f"[GNURXBlock] Payload length: {self.payload_len} bytes (FEC+CRC OK)")
                        self.bit_buffer = []
                        self.state = 'READ_PAYLOAD'

            elif self.state == 'READ_PAYLOAD':
                # Compute how many FEC-coded bits to expect from the
                # original payload length. The TX FEC-encodes
                # (payload + CRC-16) together, then interleaves.
                interleaved_len, cols, _ = compute_coded_length(self.payload_len)

                if len(self.bit_buffer) >= interleaved_len:
                    coded_bits = np.array(
                        self.bit_buffer[:interleaved_len], dtype=np.uint8
                    )

                    # ---- FEC decode ----
                    # Deinterleave
                    deinterleaved = self._interleaver.deinterleave(coded_bits, cols)

                    # Viterbi decode: recover (payload + CRC-16) bits
                    n_data_bits = self.payload_len * 8 + 16
                    try:
                        decoded_bits = self._viterbi.decode(
                            deinterleaved, n_data_bits=n_data_bits
                        )
                    except Exception as e:
                        print(f"{colors.RED}[GNURXBlock] Viterbi decode error: {e}{colors.RESET}")
                        ber = self._estimate_ber(success=False)

                        # APPEND False to packets_arr
                        self.packets_arr.append(False)

                        if self.metrics_logger:
                            packet_info = {
                                'payload_len': self.payload_len,
                                'payload_len_crc': bytearray([crc8(self.length_bytes)]),
                                'payload_crc': bytearray(2),
                                'raw_payload_bytes': bytearray(0),
                                'whitened_payload_bytes': bytearray(0),
                                'raw_packet_bytes': bytearray(0),
                                'message': f'FEC Viterbi decode failed: {e}',
                                'failure_level': 'FEC payload decode failure'
                            }
                            self.metrics_logger.log_packet_outcome(
                                'RX', self.freq, packet_info,
                                success=False, ber=ber
                            )
                        self.bit_buffer = []
                        self.constructed_bits = []
                        self.state = 'SEARCHING'
                        continue

                    # Split decoded bits into payload and CRC
                    payload_bits = decoded_bits[:self.payload_len * 8].tolist()
                    crc_bits = decoded_bits[self.payload_len * 8:
                                            self.payload_len * 8 + 16].tolist()
                    # ---- end FEC decode ----

                    unwhitened_bytes   = self.bits_to_bytes(payload_bits)
                    received_crc_bytes = self.bits_to_bytes(crc_bits)
                    received_crc_val   = (received_crc_bytes[0] << 8) | received_crc_bytes[1]
                    expected_crc_val   = crc16(list(unwhitened_bytes))

                    payload_bytes      = whiten(unwhitened_bytes)

                    self.constructed_bits.extend(payload_bits)
                    packet_bytes = bytearray(np.packbits(
                        np.array(self.constructed_bits, dtype=np.uint8)
                    ).tolist())

                    if received_crc_val != expected_crc_val:
                        print(f"{Colors.RED}[GNURXBlock] Payload CRC FAILED: got {received_crc_val:#x}, expected {expected_crc_val:#x} {Colors.RESET}")
                        ber = self._estimate_ber(success=False)

                        # APPEND False to packets_arr
                        self.packets_arr.append(False)

                        if self.metrics_logger:
                            packet_info = {
                                'raw_payload_bytes': bytearray(payload_bytes),
                                'whitened_payload_bytes': unwhitened_bytes,
                                'payload_len': self.payload_len,
                                'payload_len_crc': bytearray([crc8(self.length_bytes)]),
                                'payload_crc': bytearray(received_crc_bytes),
                                'raw_packet_bytes': packet_bytes,
                                'message': f'Payload CRC Failed: got {received_crc_val:#x} expected {expected_crc_val:#x}',
                                'failure_level': 'CRC payload mismatch failure'
                            }
                            self.metrics_logger.log_packet_outcome(
                                'RX', self.freq, packet_info,
                                success=False, ber=ber
                            )

                        self.bit_buffer = []
                        self.constructed_bits = []
                        self.state = 'SEARCHING'
                        continue

                    # SUCCESS — FEC corrected any errors, CRC confirmed
                    ber = self._estimate_ber(success=True)
                    msg = None
                    print(f"{Colors.GREEN}[GNURXBlock] Payload Found! (FEC decoded){Colors.RESET}")
                    try:
                        mav = mavutil.mavlink.MAVLink(None)
                        msg = mav.parse_char(payload_bytes)
                    except Exception as e:
                        print(f"[GNURXBlock] Error when converting payload bytes to mavlink string: {e}")
                    
                    # APPEND SUCCESS TO self.packets_arr
                    self.packets_arr.append(True)
                    
                    if self.metrics_logger:
                        packet_info = {
                            'raw_payload_bytes': bytearray(payload_bytes),
                            'whitened_payload_bytes': unwhitened_bytes,
                            'payload_len': self.payload_len,
                            'payload_len_crc': bytearray([crc8(self.length_bytes)]),
                            'payload_crc': bytearray(received_crc_bytes),
                            'raw_packet_bytes': packet_bytes,
                            'message': msg,
                            'failure_level': 'N/A'
                        }
                        self.metrics_logger.log_packet_outcome(
                            'RX', self.freq, packet_info,
                            success=True, ber=ber
                        )
                        if self.publish:
                            try:
                                publisher = GCSPublisher(self.addr)
                                publisher.publish(payload_bytes)
                            except Exception as e:
                                print(f"[GNURXBlock] Failed to publish mavlink message to GCS: {e}")

                    self.constructed_bits = []
                    self.bit_buffer = []
                    self.state = 'SEARCHING'

        return len(in_data)