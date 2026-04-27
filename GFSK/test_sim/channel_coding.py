"""
channel_coding.py

FEC and interleaving for GFSK drone link.
Rate 1/2, K=7 convolutional code + block interleaver.

Integrates into existing mavGNUTXBlock / mavGNURXBlock framing.
The CRC-16 is placed INSIDE the FEC-protected region so that
error correction covers both payload and its checksum.

TX: whitened_bytes -> CRC-16 -> concat -> FEC encode -> interleave
RX: deinterleave -> Viterbi decode -> split payload+CRC -> verify CRC
"""

import numpy as np
import math

# ============================================================
# Constants
# ============================================================

# Convolutional code: rate 1/2, constraint length 7
# Same code as 802.11a/g and GPS L1
CC_K = 7
CC_RATE = 2
CC_POLYS = [0o171, 0o133]

# Interleaver rows: controls max correctable burst length.
# Bursts up to ROWS bits become isolated errors after deinterleaving.
# Columns are computed dynamically to fit the coded payload.
INTERLEAVER_ROWS = 16

# FEC-coded length field: 24 data bits (16 len + 8 CRC-8)
# -> (24 + 6 tail) * rate 2 = 60 coded bits
# Both TX and RX must agree on this constant.
CODED_LEN_FIELD_BITS = (16 + 8 + CC_K - 1) * CC_RATE  # = 60



def bits_to_bytes(bits):
    bit_array = np.array(bits, dtype=np.uint8)
    pad = (8 - len(bit_array) % 8) % 8
    if pad:
        bit_array = np.append(bit_array, np.zeros(pad, dtype=np.uint8))
    return bytearray(np.packbits(bit_array))

def compute_coded_length(original_byte_len):
    """
    Given original payload length in bytes, compute how many
    interleaved coded bits the TX will produce.

    The FEC-protected region is: payload (N*8 bits) + CRC-16 (16 bits)
    Then: convolutional encode (rate 1/2) + K-1 tail bits
    Then: interleave (zero-padded to fill rows x cols block)

    RX uses this to know how many bits to read after the length field.
    """
    data_bits = original_byte_len * 8 + 16   # payload + CRC-16
    coded_bits = (data_bits + CC_K - 1) * CC_RATE  # FEC + tail
    cols = math.ceil(coded_bits / INTERLEAVER_ROWS)
    interleaved_len = INTERLEAVER_ROWS * cols
    return interleaved_len, cols, coded_bits


def encode_length_field(payload_len, crc8_func):
    """
    FEC-encode the 16-bit length field + 8-bit CRC-8.

    Replaces the old 3x majority voting scheme. Same convolutional code
    as the payload but no interleaver (60 bits is too short to benefit).

    Parameters
    ----------
    payload_len : int
        Original payload byte count (0-65535)
    crc8_func : callable
        The crc8() function from mavGNUTXBlock

    Returns
    -------
    np.ndarray of uint8, length = CODED_LEN_FIELD_BITS (60)
    """
    len_bytes = [payload_len >> 8, payload_len & 0xFF]
    len_crc = crc8_func(len_bytes)

    # 24 data bits: 16 length + 8 CRC
    data_bits = np.concatenate([
        np.unpackbits(np.array(len_bytes, dtype=np.uint8)),
        np.unpackbits(np.array([len_crc], dtype=np.uint8))
    ])

    enc = ConvolutionalEncoder()
    enc.reset()
    coded = enc.encode(data_bits)
    tail = enc.flush()
    return np.concatenate([coded, tail]).astype(np.uint8)




def decode_length_field(coded_bits, crc8_func):
    """
    Viterbi-decode the FEC-coded length field and verify CRC-8.

    Parameters
    ----------
    coded_bits : array of uint8, length = CODED_LEN_FIELD_BITS (60)
    crc8_func : callable
        The crc8() function from mavGNUTXBlock

    Returns
    -------
    (payload_len, valid) : (int, bool)
        payload_len is the decoded byte count, valid is True if CRC-8 passed.
        If decoding fails entirely, returns (0, False).
    """
    coded_bits = np.asarray(coded_bits, dtype=np.uint8)
    if len(coded_bits) < CODED_LEN_FIELD_BITS:
        return 0, False

    dec = ViterbiDecoder()
    decoded = dec.decode(coded_bits[:CODED_LEN_FIELD_BITS], n_data_bits=24)

    # Split: first 16 bits = length, last 8 bits = CRC-8
    len_bits = decoded[:16]
    crc_bits = decoded[16:24]

    len_bytes = np.packbits(len_bits).tolist()  # [MSB, LSB]
    received_crc = int(np.packbits(crc_bits)[0])
    expected_crc = crc8_func(len_bytes)

    payload_len = (len_bytes[0] << 8) | len_bytes[1]
    return payload_len, (received_crc == expected_crc)


# ============================================================
# Convolutional Encoder
# ============================================================

class ConvolutionalEncoder:
    """
    Rate 1/2, K=7 convolutional encoder.
    6-bit shift register, two generator polynomials.
    Each input bit produces 2 output bits.
    """

    def __init__(self, polys=None, k=CC_K):
        self.polys = polys or CC_POLYS
        self.k = k
        self.n_states = 1 << (k - 1)
        self.state = 0

    def encode(self, input_bits):
        """Encode bits. Returns array of 2 * len(input_bits)."""
        output = np.empty(len(input_bits) * len(self.polys), dtype=np.uint8)
        idx = 0
        for bit in input_bits:
            reg = (self.state << 1) | int(bit)
            for poly in self.polys:
                output[idx] = bin(reg & poly).count('1') % 2
                idx += 1
            self.state = reg & (self.n_states - 1)
        return output[:idx]

    def flush(self):
        """Push K-1 zeros to terminate trellis at state 0."""
        tail = self.encode(np.zeros(self.k - 1, dtype=np.uint8))
        self.state = 0
        return tail

    def reset(self):
        self.state = 0


# ============================================================
# Viterbi Decoder (Hard Decision)
# ============================================================

class ViterbiDecoder:
    """
    Hard-decision Viterbi decoder for rate 1/2, K=7.
    64-state trellis, Hamming distance metric.
    """

    def __init__(self, polys=None, k=CC_K):
        self.polys = polys or CC_POLYS
        self.k = k
        self.n_states = 1 << (k - 1)
        self._build_trellis()

    def _build_trellis(self):
        n = self.n_states
        self.next_state = np.zeros((n, 2), dtype=np.int32)
        self.expected_out = np.zeros((n, 2, len(self.polys)), dtype=np.uint8)
        for state in range(n):
            for inp in (0, 1):
                reg = (state << 1) | inp
                self.next_state[state][inp] = reg & (n - 1)
                for i, poly in enumerate(self.polys):
                    self.expected_out[state][inp][i] = (
                        bin(reg & poly).count('1') % 2
                    )

    def decode(self, received_bits, n_data_bits=None):
        """
        Decode received coded bits.
        
        Parameters
        ----------
        received_bits : array of uint8, length must be even
        n_data_bits : int, optional
            If provided, return exactly this many decoded bits
            (before tail). Otherwise return n_pairs - (K-1) bits.
        """
        received_bits = np.asarray(received_bits, dtype=np.uint8)
        n_pairs = len(received_bits) // 2
        n_states = self.n_states
        INF = 0x7FFFFFFF

        pm = np.full(n_states, INF, dtype=np.int32)
        pm[0] = 0

        tb_state = np.zeros((n_pairs, n_states), dtype=np.int32)
        tb_input = np.zeros((n_pairs, n_states), dtype=np.uint8)

        for t in range(n_pairs):
            r = received_bits[2 * t: 2 * t + 2]
            new_pm = np.full(n_states, INF, dtype=np.int32)

            for s in range(n_states):
                if pm[s] >= INF:
                    continue
                for inp in (0, 1):
                    ns = self.next_state[s][inp]
                    dist = int(np.sum(r != self.expected_out[s][inp]))
                    candidate = pm[s] + dist
                    if candidate < new_pm[ns]:
                        new_pm[ns] = candidate
                        tb_state[t][ns] = s
                        tb_input[t][ns] = inp

            pm = new_pm

        # Traceback from state 0 (encoder was flushed)
        state = 0
        decoded = np.empty(n_pairs, dtype=np.uint8)
        for t in range(n_pairs - 1, -1, -1):
            decoded[t] = tb_input[t][state]
            state = tb_state[t][state]

        # Return only the data bits (strip K-1 tail bits)
        if n_data_bits is not None:
            return decoded[:n_data_bits]
        return decoded[:-(self.k - 1)]


# ============================================================
# Block Interleaver
# ============================================================

class BlockInterleaver:
    """
    Write row-wise, read column-wise.
    Spreads burst errors so the convolutional decoder
    sees isolated errors it can correct.
    Columns are sized dynamically to fit the data.
    """

    def __init__(self, rows=INTERLEAVER_ROWS):
        self.rows = rows

    def interleave(self, bits, cols):
        """
        Interleave bits into a (rows x cols) block.
        Zero-pads if len(bits) < rows * cols.
        """
        block_size = self.rows * cols
        padded = np.zeros(block_size, dtype=np.uint8)
        padded[:len(bits)] = bits
        return padded.reshape(self.rows, cols).T.flatten()

    def deinterleave(self, bits, cols):
        """Reverse: column-major write, row-major read."""
        block_size = self.rows * cols
        block = np.zeros(block_size, dtype=np.uint8)
        n = min(len(bits), block_size)
        block[:n] = bits[:n]
        return block.reshape(cols, self.rows).T.flatten()


# ============================================================
# Self-test
# ============================================================

if __name__ == "__main__":
    # Stub crc8 for self-test (same as mavGNUTXBlock.crc8)
    def crc8(data, poly=0x07, init=0x00):
        crc = init
        for byte in data:
            crc ^= int(byte)
            for _ in range(8):
                crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        return crc

    print("=== Channel Coding Self-Test ===")
    print()

    # ---- Test 1: Payload FEC round-trip ----
    original = bytes(range(33))
    crc_val = 0xBEEF
    crc_bytes = np.array([crc_val >> 8, crc_val & 0xFF], dtype=np.uint8)

    data_bits = np.concatenate([
        np.unpackbits(np.frombuffer(original, dtype=np.uint8)),
        np.unpackbits(crc_bytes)
    ])

    enc = ConvolutionalEncoder()
    enc.reset()
    coded = np.concatenate([enc.encode(data_bits), enc.flush()])

    interleaved_len, cols, n_coded = compute_coded_length(len(original))
    ilv = BlockInterleaver()
    interleaved = ilv.interleave(coded, cols)

    noisy = interleaved.copy()
    rng = np.random.default_rng(42)
    errors = rng.random(len(noisy)) < 0.03
    noisy[errors] ^= 1
    n_err = int(errors.sum())
    print(f"Payload FEC: {n_err}/{len(interleaved)} errors ({100*n_err/len(interleaved):.1f}% BER)")

    deinterleaved = ilv.deinterleave(noisy, cols)
    dec = ViterbiDecoder()
    decoded = dec.decode(deinterleaved, n_data_bits=len(original)*8+16)
    recovered = bytes(np.packbits(decoded[:len(original)*8]))
    print(f"Payload match: {recovered == original}")

    # ---- Test 2: Length field FEC ----
    print()
    print(f"Length field: {CODED_LEN_FIELD_BITS} coded bits (was 56 with majority voting)")

    for ber in [0.0, 0.05, 0.10, 0.15]:
        ok = 0
        trials = 200
        for _ in range(trials):
            coded_len = encode_length_field(33, crc8)
            noisy_len = coded_len.copy()
            if ber > 0:
                noisy_len[rng.random(len(noisy_len)) < ber] ^= 1
            plen, valid = decode_length_field(noisy_len, crc8)
            if valid and plen == 33:
                ok += 1
        print(f"  BER={ber:5.0%}: {ok}/{trials} length fields correct ({100*ok/trials:.0f}%)")