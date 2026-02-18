import numpy as np
from gnuradio import gr
import pmt
from pymavlink import mavutil



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
        self.preamble = np.unpackbits(np.array([37,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85, 85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85, 85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85], dtype=np.uint8)).tolist()
        # Define syncword
        self.sync_word = np.unpackbits(np.array([85,85,85,93], dtype=np.uint8)).tolist()
        # Define postamble: at least 4 bytes / 32 bits

        self.packet_queue = []
    
    def string_to_bits(self, text):
        # Convert string to list of bits MSB first
        # First convert text to ascii byte values, ord(char) returns unicode value
        byte_array = np.array([ord(c) for c in text], dtype=np.uint8)
        print(f"payload in byte format: {byte_array}")
        # Then convert the byte array to bit array that represents it
        bit_array = np.unpackbits(byte_array)
        return bit_array
    
    
    
    def build_packet(self, message, raw=False):
        # Assemble full packet
        if not raw:
            payload_bits = self.string_to_bits(message)
        else:
            # payload_bits = np.unpackbits(np.array(message, dtype=np.uint8)).tolist()
            payload_bits = np.unpackbits(message).tolist()
        # ensure the payload length is denoted as the amount of bytes
        payload_len = np.ceil(len(payload_bits) / 8).astype(int)
        # convert payload_len from bytes to bit array
        # Mavlink2 packets can be from 12-280 bytes so at the maximum length we need two bytes to represent it
        # payload_len >> 8 right bit shift by 8 positions so it "grabs" the higher bit and just leaves what is in lower byte
        # payload_len & 0xFF (0xFF = 11111111) gets just the lower byte and leaves just the higher bytes 
        # so with these two operations you get a number over 255 nicely represented into two bytes
        payload_len_bits = np.unpackbits(np.array([payload_len >> 8, payload_len & 0xFF], dtype=np.uint8)).tolist()
        print(f"payload length in byte array form is: {payload_len}, in bit form: {payload_len_bits}")
        packet = (np.concatenate([
            self.preamble,
            self.sync_word,
            payload_len_bits,
            payload_bits
            # add crc bits 
            ])
        )
        return packet

    def send_message(self, message, raw=False):
        if raw:
            print(f"mavlink packet in byte form: {[int(b) for b in message]}")
      
        whitened_msg = whiten(message)
        print(f"whitened message {whitened_msg}, whiten function ran again: {whiten(whitened_msg)}")
        packet = self.build_packet(whitened_msg, raw)
        self.packet_queue.extend(packet)

    
    def work(self, input_items, output_items):
        out = output_items[0]

        if len(self.packet_queue) == 0:
            # output zeros (idle when no data)
            # maybe return no data when there's no messages?
            out[:] = 0
            return len(out)
        
        # When a message is present output as many bits as we can
        n = min(len(out), len(self.packet_queue))
        # output items becomes every packet in the queue up to n inclusive
        out[:n] = self.packet_queue[:n]
        # update the queue to now be everything we didn't output, aka everything past n exclusive
        self.packet_queue = self.packet_queue[n:]

        # fill remainig with zeros
        if n < len(out):
            out[n:] = 0

        return len(out)
    
class mav_packet_reader(gr.sync_block):
    def __init__(self, sitl_address='udp:127.0.0.1:14550'):
        gr.sync_block.__init__(
            self,
            name="MavLink Packet Reader",
            in_sig=[np.byte],
            out_sig=None
        )
    
        # VARIABLES
        self.preamble = np.unpackbits(np.array([37,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85, 85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85, 85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85], dtype=np.uint8))
        # Define syncword
        self.sync_word = np.unpackbits(np.array([85,85,85,93], dtype=np.uint8))
        self.sync_len = len(self.sync_word)

        # Establish a state machine for easier processing
        self.state = 'SEARCHING'
        self.bit_buffer = []
        self.payload_len = 0

        #SITL connection
        self.master = mavutil.mavlink_connection(sitl_address)
        print("Attempting to connect to SITL...")
        heartbeat_received = self.master.wait_heartbeat(timeout=5)
        if heartbeat_received:
            print("Decoder connected to SITL")
        else:
            print("Decoder not connected to SITL")
    
    # convert bit array back to byte array
    def bits_to_bytes(self, bits):
        bit_array = np.array(bits, dtype=np.uint8)

        # incase bits are not nice groups of 8 pad will hold the amount of bits that must be padded
        pad = (8 - len(bit_array) % 8) % 8
        # this will only trigger if pad != 0 meaning the bit array is not divisible by 8
        if pad:
            bit_array = np.append(bit_array, np.zeros(pad, dtype=np.uint8))
        return bytearray(np.packbits(bit_array))
    
    def work(self, input_items, output_items):
        in_data = input_items[0]
        # Get in bits as them come into in_data
        # Then add each bit the buffer at a time and iterate through the buffer

        for bit in in_data:
            self.bit_buffer.append(int(bit))

            if self.state == 'SEARCHING':
                # check if the end of the buffer matches sync word
                if len(self.bit_buffer) >= self.sync_len:
                    # -self.sync_len: gets the last self.sync_len bits exclusive so if self.sync_len = 2 it would get the last 2 bits in self.bit_buffer
                    tail = self.bit_buffer[-self.sync_len:]
                    if np.array_equal(tail, self.sync_word):
                        print("Sync word found!")
                        self.bit_buffer = []
                        self.state ='READ_LENGTH'
            
            elif self.state == 'READ_LENGTH':
                # Read 2 bytes for payload length
                if len(self.bit_buffer) >= 16:
                    length_bytes = self.bits_to_bytes(self.bit_buffer[:16])
                    
                    # shift byte down 8 leaving higher bits
                    # | concats the lower bits with the higher bits
                    self.payload_len = (length_bytes[0] << 8 | length_bytes[1])
                    print(f"length_bytes = {length_bytes[0]}, {length_bytes[1]}")
                    print(f"Payload length: {self.payload_len} bytes")
                    self.bit_buffer = []
                    self.state = 'READ_PAYLOAD'
            
            elif self.state == 'READ_PAYLOAD':
                # Read payload_len bytes 
                if len(self.bit_buffer) >= self.payload_len * 8:
                    payload_bits = self.bit_buffer[:self.payload_len * 8]
                    unwhitened_bytes = self.bits_to_bytes(payload_bits)
                    # run whiten again because XOR is the compliment to itself
                    payload_bytes = whiten(unwhitened_bytes)

                    print(f"Received payload: {list(payload_bytes)}")

                    # forward raw mavlink bytes to SITL
                    try:
                        self.master.write(payload_bytes)
                        print("Forwarded to SITL")
                    except Exception as e:
                        print(f"Error forwarding to SITL: {e}")
                    
                    self.bit_buffer = []
                    self.state = 'SEARCHING'

        return len(in_data)
            
