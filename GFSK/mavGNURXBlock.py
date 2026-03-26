
from gnuradio import gr
import pmt
from pymavlink import mavutil
import csv
import numpy as np
from mavGNUTXBlock import sync_word, whiten, crc8, crc16
from gcsPublisher import GCSPublisher

# Import your existing helpers from mavGNUBlock
# (whiten, crc8, crc16, log_packet, sync_word, log_name are defined there)
# This file only defines the updated class — import it alongside mavGNUBlock


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

    def __init__(self, freq, metrics_logger=None, publish_to_gcs=False, host='127.0.0.1', port=8080):
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

    def stop(self):
        if not self._stopped:
            self._stopped = True
            print("[MavReader] stop() called by GNU radio scheduler")
            if self.sitl is not None:
                self.sitl.stop()
        return True
    
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



    def work(self, input_items, output_items):
        from mavGNUTXBlock import log_name, whiten, crc8, crc16

        in_data = input_items[0]

        for bit in in_data:
            self.bit_buffer.append(int(bit))

            if self.state == 'SEARCHING':
                if len(self.bit_buffer) >= self.sync_len:
                    tail = self.bit_buffer[-self.sync_len:]
                    if np.array_equal(tail, self.sync_word):
                        print("[GNURXBlock] sync word found!")
                        self.bit_buffer   = []
                        self.state        = 'READ_LENGTH'
                        self.constructed_bits = list(self.sync_word)

            elif self.state == 'READ_LENGTH':
                if len(self.bit_buffer) >= 56:
                    raw_bits = self.bit_buffer[:48]

                    voted_bits = []
                    for i in range(16):
                        a, b, c = raw_bits[i], raw_bits[i+16], raw_bits[i+32]
                        voted_bits.append(1 if (a+b+c) >= 2 else 0)

                    self.length_bytes = length_bytes = self.bits_to_bytes(voted_bits)
                    received_crc  = self.bits_to_bytes(self.bit_buffer[48:56])[0]
                    expected_crc  = crc8(list(length_bytes))
                    self.payload_len = (length_bytes[0] << 8 | length_bytes[1])

                    if received_crc != expected_crc:
                        print(f"[GNURXBlock] Length CRC FAILED: got {received_crc:#x}, expected {expected_crc:#x}")
                        # Log packet failure
                        ber = self._estimate_ber(success=False)
                        
                        
                        
                        if self.metrics_logger:
                            packet_info =  {
                                'payload_len':bytearray(self.payload_len),
                                'payload_len_crc':bytearray([received_crc]),
                                'payload_crc':(0),
                                'raw_payload_bytes':bytearray(0),
                                'whitened_payload_bytes':bytearray(0),
                                'raw_packet_bytes':bytearray(0),
                                'message':f'Length CRC Failed: got {received_crc:#x} expected {expected_crc:#x}'
                            }
                            self.metrics_logger.log_packet_outcome('RX', self.freq, packet_info, success=False, ber=ber)

                        self.bit_buffer     = []
                        self.constructed_bits = []
                        self.state          = 'SEARCHING'
                    else:
                        self.constructed_bits.extend(self.bit_buffer[:56])
                        print(f"[GNURXBlock] Payload length: {self.payload_len} bytes (CRC OK)")
                        self.bit_buffer = []
                        self.state = 'READ_PAYLOAD'

            elif self.state == 'READ_PAYLOAD':
                total_bits_needed = (self.payload_len * 8) + 16
                if len(self.bit_buffer) >= total_bits_needed:
                    payload_bits      = self.bit_buffer[:self.payload_len * 8]
                    crc_bits          = self.bit_buffer[self.payload_len * 8: total_bits_needed]

                    unwhitened_bytes  = self.bits_to_bytes(payload_bits)
                    received_crc_bytes= self.bits_to_bytes(crc_bits)
                    received_crc_val  = (received_crc_bytes[0] << 8) | received_crc_bytes[1]
                    expected_crc_val  = crc16(list(unwhitened_bytes))

                    payload_bytes     = whiten(unwhitened_bytes)

                    self.constructed_bits.extend(payload_bits)
                    packet_bytes = bytearray(np.packbits(
                        np.array(self.constructed_bits, dtype=np.uint8)
                    ).tolist())

                    if received_crc_val != expected_crc_val:
                        print(f"[GNURXBlock] Payload CRC FAILED: got {received_crc_val:#x}, expected {expected_crc_val:#x}")
                        ber = self._estimate_ber(success=False)

                        
                        if self.metrics_logger:
                            packet_info = {
                                'raw_payload_bytes':bytearray(payload_bytes),
                                'whitened_payload_bytes':unwhitened_bytes,
                                'payload_len':self.payload_len,
                                'payload_len_crc':bytearray([crc8(self.length_bytes)]),
                                'payload_crc':bytearray(received_crc_bytes),
                                'raw_packet_bytes':packet_bytes,
                                'message':f'Payload CRC Failed: got {received_crc_val:#x} expected {expected_crc_val:#x}'
                            }
                            self.metrics_logger.log_packet_outcome('RX', self.freq, packet_info, success=False, ber=ber)

                        self.bit_buffer     = []
                        self.constructed_bits = []
                        self.state = 'SEARCHING'
                        continue

                    # SUCCESS
                    ber = self._estimate_ber(success=True)
                    # turn mavlink message to string to log
                    msg = None
                    print(f"[GNURXBlock] Payload Found!")
                    try:
                        mav = mavutil.mavlink.MAVLink(None)
                        msg = mav.parse_char(payload_bytes) 
                    except Exception as e:
                        print(f"[GNURXBlock] Error when converting payload bytes to mavlink string: {e}")
                        
                    if self.metrics_logger:
                        packet_info = {
                                'raw_payload_bytes':bytearray(payload_bytes),
                               'whitened_payload_bytes':unwhitened_bytes,
                                'payload_len':self.payload_len,
                                'payload_len_crc':bytearray([crc8(self.length_bytes)]),
                                'payload_crc':bytearray(received_crc_bytes),
                                'raw_packet_bytes':packet_bytes,
                                'message':msg
                            }
                        self.metrics_logger.log_packet_outcome('RX', self.freq ,packet_info, success=True, ber=ber)
                        if self.publish:
                            try:
                                # publish mavlink message to port 8080 for gcs
                                publisher = GCSPublisher(self.addr)
                                publisher.publish(payload_bytes)
                            except Exception as e:
                                print(f"[GNURXBlock] Failed to publish mavlink message to GCS: {e}")

                    # try:
                    #     self.sitl.forward_packet(payload_bytes)
                    # except Exception as e:
                    #     print(f"Error forwarding to SITL: {e}")

                    self.constructed_bits = []
                    self.bit_buffer       = []
                    self.state            = 'SEARCHING'

        return len(in_data)