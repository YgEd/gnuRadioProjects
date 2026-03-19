"""
RF Metrics Collector for SDR BN
================================
Provides two additions to your existing GNU Radio chain:

1. RFMetricsProbe  — a gr.sync_block that taps IQ samples after your
                     rx_resampler_lowpass and computes:
                       - SNR (dB)
                       - Noise floor (dBm)
                       - Frequency offset (Hz)
                       - Doppler shift (Hz)  [requires baseline measurement]
                       - Jitter (ns RMS)

2. MetricsLogger   — a plain Python class (not a GR block) that
                     mav_packet_reader calls on each packet outcome.
                     Combines the IQ-derived metrics with packet-level
                     BER/CRC results and writes one row per packet to
                     a CSV that feeds directly into your BN DataFrame.

Wiring in rx.py (minimal changes):
------------------------------------
    from rf_metrics import RFMetricsProbe, MetricsLogger

    # In flow_graph.__init__, after existing blocks:
    self.metrics_logger = MetricsLogger()
    self.metrics_probe = RFMetricsProbe(
        samp_rate=self.samp_rate,          # 100e3 — post-resampler rate
        samples_per_symbol=self.samples_per_symbol,
        center_freq=self.center_freq,
        logger=self.metrics_logger
    )

    # Tap off after resampler — parallel to AGC path
    self.connect(self.rx_resampler_lowpass, self.metrics_probe)

    # Pass logger into packet reader (see updated mav_packet_reader below)
    self.destination = mav_packet_reader(metrics_logger=self.metrics_logger)
"""

import numpy as np
from gnuradio import gr
import csv
import os
import datetime
import threading
from scipy.signal import welch
from collections import deque
import sitl_manager as sitl


# ---------------------------------------------------------------------------
# CONSTANTS from your flowgraph — keep in sync with rx.py
# ---------------------------------------------------------------------------
SAMP_RATE          = 100e3    # post-resampler sample rate
SAMPLES_PER_SYMBOL = 4
CENTER_FREQ        = 915e6
SYMBOL_RATE        = SAMP_RATE / SAMPLES_PER_SYMBOL   # 25 kHz


# ---------------------------------------------------------------------------
# METRICS LOGGER
# Shared object between RFMetricsProbe and mav_packet_reader.
# Thread-safe: probe writes IQ metrics, reader writes packet outcome,
# logger merges and flushes one CSV row per packet event.
# ---------------------------------------------------------------------------

class MetricsLogger:
    """
    Collects IQ-derived metrics from RFMetricsProbe and packet-level
    metrics from mav_packet_reader, merges them, and writes to CSV.
    """

    def __init__(self, log_dir='packet-logs'):
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)

        now = datetime.datetime.now().isoformat()
        self.filepath = os.path.join(log_dir, f'rf-metrics-{now}.csv')
        self._lock = threading.Lock()

        # Latest IQ-derived metrics — updated by RFMetricsProbe
        self._iq_metrics = {
            'snr_db':            None,
            'noise_floor_dbm':   None,
            'freq_offset_hz':    None,
            'doppler_hz':        None,
            'jitter_ns':         None,
        }

        # Static baseline freq for Doppler calculation.
        # Set this from a known stationary measurement before flight.
        self._baseline_freq_hz = None

        self._write_header()

    def _write_header(self):
        headers = [
            # Original measurement fields
            'timestamp', 'snr_db', 'noise_floor_dbm', 'freq_offset_hz', 'doppler_hz', 'jitter_ns', 'ber',
            # Discretized / derived fields
            'SNR', 'ChannelNoise', 'FreqOffset', 'Doppler', 'Jitter', 'BER', 'PacketSuccess',
            # Detailed packet info fields
            'MAVLink_message', 'payload_len', 'payload_len_crc', 'payload_crc',
            'raw_payload_bytes', 'whitened_payload_bytes', 'full_packet_bytes'
        ]

        with open(self.filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)

    def update_iq_metrics(self, snr_db, noise_floor_dbm,
                          freq_offset_hz, doppler_hz, jitter_ns):
        """Called by RFMetricsProbe each processing window."""
        with self._lock:
            self._iq_metrics = {
                'snr_db':          snr_db,
                'noise_floor_dbm': noise_floor_dbm,
                'freq_offset_hz':  freq_offset_hz,
                'doppler_hz':      doppler_hz,
                'jitter_ns':       jitter_ns,
            }

    def set_baseline_freq(self, freq_hz):
        """
        Call this once during a static (no motion) calibration period.
        Doppler is then measured relative to this baseline.
        """
        with self._lock:
            self._baseline_freq_hz = freq_hz
        print(f"[MetricsLogger] Baseline freq set: {freq_hz:.2f} Hz offset from center")

    def log_packet_outcome(self, packet_info, success: bool, ber: float | str):
        """
        Called by mav_packet_reader after each packet attempt.
        Merges with latest IQ and packet metrics and writes one CSV row.
        """
        # Pares packet_info values
        def fmt(data):
            if data is None:
                return "N/A"
            elif isinstance(data, (bytes, bytearray)):
                return data.hex(' ')
            elif isinstance(data, (list, np.ndarray)):
                return ''.join(str(b) for b in data)
            return str(data)

        with self._lock:
            m = dict(self._iq_metrics)  # snapshot

        headers = [
            # Original measurement fields
            'timestamp', 'snr_db', 'noise_floor_dbm', 'freq_offset_hz', 'doppler_hz', 'jitter_ns', 'ber',
            # Discretized / derived fields
            'SNR', 'ChannelNoise', 'FreqOffset', 'Doppler', 'Jitter', 'BER', 'PacketSuccess',
            # Detailed packet info fields
            'MAVLink_message', 'payload_len', 'payload_len_crc', 'payload_crc',
            'raw_payload_bytes', 'whitened_payload_bytes', 'full_packet_bytes'
        ]

            # Step 2: initialize row with None for all headers
        row = {key: None for key in headers}

        # Step 3: populate the base fields
        row.update({
            'timestamp': datetime.datetime.now().isoformat(),
            'snr_db': m.get('snr_db'),
            'noise_floor_dbm': m.get('noise_floor_dbm'),
            'freq_offset_hz': m.get('freq_offset_hz'),
            'doppler_hz': m.get('doppler_hz'),
            'jitter_ns': m.get('jitter_ns'),
            'ber': ber,
        })

        # Step 4: populate discretized fields if measurements available
        if all(v is not None for v in m.values()):
            row.update({
                'SNR': bin_snr(m['snr_db']),
                'ChannelNoise': bin_channel_noise(m['noise_floor_dbm']),
                'FreqOffset': bin_freq_offset(m['freq_offset_hz']),
                'Doppler': bin_doppler(m['doppler_hz']),
                'Jitter': bin_jitter(m['jitter_ns']),
                'BER': bin_ber(ber),
                'PacketSuccess': 'success' if success else 'failure'
            })
        else:
            row.update({
                'SNR': 'unknown',
                'ChannelNoise': 'unknown',
                'FreqOffset': 'unknown',
                'Doppler': 'unknown',
                'Jitter': 'unknown',
                'BER': 'unknown',
                'PacketSuccess': 'unknown'
            })

        # Step 5: populate detailed packet info fields
        row.update({
            'MAVLink_message': fmt(packet_info['message']),
            'payload_len': fmt(packet_info['payload_len']),
            'payload_len_crc': fmt(packet_info['payload_len_crc']),
            'payload_crc': fmt(packet_info['payload_crc']),
            'raw_payload_bytes': fmt(packet_info['raw_payload_bytes']),
            'whitened_payload_bytes': fmt(packet_info['whitened_payload_bytes']),
            'full_packet_bytes': fmt(packet_info['packet_bytes'])
        })

        # Step 6: write row to CSV safely
        with open(self.filepath, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            if f.tell() == 0:
                writer.writeheader()
            writer.writerow(row)


# ---------------------------------------------------------------------------
# BIN FUNCTIONS  (same as sdr_bn.py — kept here so rf_metrics is self-contained)
# ---------------------------------------------------------------------------

def bin_snr(snr_db):
    if snr_db < 10:    return 'poor'
    if snr_db <= 18:   return 'marginal'
    return 'good'

def bin_channel_noise(noise_dbm):
    if noise_dbm < -100:  return 'low'
    if noise_dbm <= -85:  return 'moderate'
    return 'high'

def bin_freq_offset(offset_hz):
    offset_hz = abs(offset_hz)
    if offset_hz < 1000:   return 'low'
    if offset_hz <= 5000:  return 'medium'
    return 'high'

def bin_doppler(doppler_hz):
    doppler_hz = abs(doppler_hz)
    if doppler_hz < 100:   return 'negligible'
    if doppler_hz <= 500:  return 'moderate'
    return 'severe'

def bin_jitter(jitter_ns):
    jitter_norm = jitter_ns * 1e-6
    if jitter_norm < 10:   return 'low'
    if jitter_norm <= 50:  return 'medium'
    return 'high'

def bin_ber(ber):
    if ber < 1e-4:   return 'excellent'
    if ber <= 1e-2:  return 'acceptable'
    return 'poor'


# ---------------------------------------------------------------------------
# RF METRICS PROBE — GNU Radio block
# Taps IQ samples after rx_resampler_lowpass (post-decimation, 100 kHz rate)
# ---------------------------------------------------------------------------

class RFMetricsProbe(gr.sync_block):
    """
    Sits in parallel on the output of rx_resampler_lowpass.
    Does NOT modify samples — consumes them for measurement only.

    Connect like:
        self.connect(self.rx_resampler_lowpass, self.metrics_probe)

    The block accumulates WINDOW_SIZE samples then computes all metrics
    and pushes them to MetricsLogger. Window size of 4096 at 100 kHz
    gives ~41ms snapshots — roughly one packet duration.
    """

    WINDOW_SIZE = 4096   # samples per measurement window at 100 kHz

    def __init__(self, samp_rate, samples_per_symbol, center_freq, logger, sensitivity=0):
        gr.sync_block.__init__(
            self,
            name='RFMetricsProbe',
            in_sig=[np.complex64],
            out_sig=None               # sink block — no output
        )
        self.samp_rate          = samp_rate
        self.samples_per_symbol = samples_per_symbol
        self.center_freq        = center_freq
        self.logger             = logger
        self.sensitivity = sensitivity

        self._buffer     = []
        self._noise_ref  = None   # set during known-silence calibration

        # Running list of peak frequencies for Doppler baseline estimation
        self._freq_history = deque(maxlen=50)

    # ------------------------------------------------------------------
    # SNR ESTIMATION
    # Uses Welch Power Spectral Density (PSD) — signal power is in the occupied bandwidth,
    # noise power is in the spectral guard regions.
    # ------------------------------------------------------------------
    def _estimate_snr(self, samples):
        freqs, psd = welch(samples, fs=self.samp_rate,
                           nperseg=512, return_onesided=False)
        
        # output of welch orders frequency bins in unintuitive manner of positive samples then negative
        # for example if you had a bandwidth of 2 MHz with bins of 1MHz surrounding a center frequency of 0 you'd see [0, 1, -1] instead of what you'd expect [-1, 0, 1]
        # to organize this into the intuitive manner you need to call np.fft.fftshift()
        freqs = np.fft.fftshift(freqs)
        psd   = np.fft.fftshift(psd)

        # Occupied bandwidth for GFSK ≈ symbol_rate * 2 * h
        # With sensitivity=0.4, h ≈ sensitivity * samples_per_symbol / pi ≈ 0.51
        # Occupied BW ≈ 25000 * 2 * 0.51 ≈ 25.5 kHz — use 30 kHz to be safe
        h = self.sensitivity * (self.samples_per_symbol / np.pi)
        symbol_rate = self.samp_rate/self.samples_per_symbol
        signal_bw_hz = np.ceil(symbol_rate * 2 * h)
        sig_mask  = np.abs(freqs) <= (signal_bw_hz / 2)

        # Noise region: outside signal BW but inside filter passband
        # Your lowpass cutoff is ~(100e3/4)*(1+0.35) ≈ 33.75 kHz
        # Use 35–50 kHz region as noise reference
        noise_mask = (np.abs(freqs) > 35e3) & (np.abs(freqs) <= 50e3)

        if noise_mask.sum() == 0 or sig_mask.sum() == 0:
            return None, None

        p_signal = np.mean(psd[sig_mask])
        p_noise  = np.mean(psd[noise_mask])

        if p_noise <= 0 or p_signal <= 0:
            return None, None

        snr_db = 10 * np.log10(p_signal / p_noise)

        # Noise floor in dBm — assumes 50 ohm, BladeRF normalized samples
        # This gives relative dBm; for absolute you'd need the gain chain offset
        noise_floor_dbm = 10 * np.log10(p_noise / 1e-3 + 1e-30)

        return snr_db, noise_floor_dbm

    # ------------------------------------------------------------------
    # FREQUENCY OFFSET / DOPPLER
    # FFT peak relative to center. Static offset = freq_offset.
    # Change from baseline = Doppler.
    # ------------------------------------------------------------------
    def _estimate_freq_offset(self, samples):

        # compute exact expected tone frequencies from first principles
        if self.sensitivity != 0:
            f_dev = (self.sensitivity * self.samp_rate) / (2 * np.pi)
            expected_tones = np.array([+f_dev, -f_dev])


        N   = len(samples)

        # use hanning window as it is a moderately narrow window (narrow main lob) while having low enough side lobe amplitude
        window = np.hanning(N)
        windowed_samples = window * samples

        # Add zero padding to increase frequency resolution
        # Padding 4*N interpolates the spectrum giving finer grid of frequencies bins to find peak on
        fft_size = 4*N
        fft = np.fft.fft(windowed_samples, n=fft_size)
        fft = np.fft.fftshift(fft) # fft holds the actual complex numbers representing magnitude and face of each frequency
        freqs = np.fft.fftfreq(fft_size, d=1.0/self.samp_rate) # d is the sample spacing which would be equal to 1/samp rate
        freqs = np.fft.fftshift(freqs) #freqs holds and array of the frequency values

        # fft holds the actual complex number and freqs holds the array of frequencies

        # np.abs caclulates the amplitudes
        magnitudes = np.abs(fft)

        # Don't look at entire spectrum for peaks look at around DC where you predict the signal to be


        # search_bw_hz = 10000.0 what I had for general search not GFSK

        # search within + or - search_bw of where the tone "should" be

        search_bw_hz = f_dev * 0.3
        measured_tones = []
        for expected in expected_tones:
            mask = np.abs(freqs - expected) <= search_bw_hz
            if not np.any(mask):
                continue
            restricted = np.where(mask, magnitudes, 0.0)
            peak_idx = np.argmax(restricted)


            if 1 <= peak_idx <= len(magnitudes) - 2:
                left = magnitudes[peak_idx - 1]
                center = magnitudes[peak_idx]
                right = magnitudes[peak_idx + 1]
                # Parabolic interpolation formula — solves for the offset from peak_idx
                denom = (left - 2*center + right)

                if denom != 0:
                    delta_bin = 0.5 * (left - right)/denom
                else:
                    delta_bin = 0
                bin_spacing_hz = freqs[1] - freqs[0]
                peak_freq = freqs[peak_idx] + delta_bin * bin_spacing_hz
            else:
                peak_freq = freqs[peak_idx]
        
        if len(measured_tones) < 1:
            return 0.0, 0.0

        search_mask = np.abs(freqs) <= search_bw_hz
        # restricted_magnitudes = np.where(search_mask, magnitudes, 0.0)
        # peak_idx   = np.argmax(restricted_magnitudes)
        # peak_freq_coarse = freqs[peak_idx]

        # Parabolic interpolatio of max amplitude around peak
        # true peak almost never falls directly on bin center
        # fitting a parabola through the peak and two neighbors
        # --- Frequency offset = measured tone position minus expected position ---
        # Average the offset across both tones for a more robust estimate
        offsets = [measured - expected for expected, measured, _ in measured_tones]
        freq_offset_hz = np.mean(offsets)

        # --- You can also detect modulation index error ---
        # If both tones are visible, the actual deviation tells you if the
        # modulator is hitting the right depth
        if len(measured_tones) == 2:
            actual_separation = abs(measured_tones[0][1] - measured_tones[1][1])
            expected_separation = 2 * f_dev
            modulation_index_error = (actual_separation - expected_separation) / expected_separation
        else:
            modulation_index_error = None

        # Sanity Check - Reject implausible peaks
        # Implausible peaks mean that the peak does not have a sificiently large amplitude differnece from noise
        # Check your peak to median magnitude of your samples as a rough SNR check
        # Tune the threshold (6x in this case) to link budget 
        # noise_floor_estimate = np.median(magnitudes[search_mask])
        # peak_magnitude = magnitudes[peak_idx]
        
        # if peak_magnitude < 6 * noise_floor_estimate:
        #     # Not confident, reuse last estimate
        #     if len(self._freq_history) > 0:
        #         peak_freq = self._freq_history[-1]
        #     else:
        #         peak_freq = 0



        # freq_offset is peak relative to 0 (center already tuned out by SDR)
        freq_offset_hz = peak_freq
        # History accumlation that uses collections.deque instead of pop
        # pop is O(N) collections.deque is O(1)
        #         # Accumulate history for baseline
        self._freq_history.append(freq_offset_hz)

        # Doppler: deviation from running median (removes static offset)
        if len(self._freq_history) >= 10:
            # use trimmed slice to exclude most recent point from baseline
            baseline = np.median(list(self._freq_history)[:-1])
            doppler_hz = peak_freq - baseline
        else:
            doppler_hz = 0.0

        return freq_offset_hz, doppler_hz

    # ------------------------------------------------------------------
    # JITTER ESTIMATION
    # Measures RMS timing error of zero crossings vs expected symbol grid.
    # Uses real part of the post-filter IQ samples.
    # ------------------------------------------------------------------
    def _estimate_jitter(self, samples):
        real = np.real(samples)

        # Find zero crossings via sign changes
        signs = np.sign(real)
        sign_changes = np.where(np.diff(signs) != 0)[0]

        if len(sign_changes) < 4:
            return None

        # Sub-sample interpolation for each crossing
        # Because crossing don't happen exactly at an index, you assume that the between any idx and idx+1 the signal is a striagh line.
        # so the line is y(t) = real[idx] + t * (real[idx+1] - real[idx])
        # Because in this case we want to find t where the crossing actually happens (at 0) we set the equation = 0 and solve for t
        #  0 = real[idx] + t * (real[idx+1] - real[idx])
        #  frac = t = -real[idx] / (real[idx+1] - real[idx])
        #  so the crossing happens at the time idx+frac
        crossings = []
        for idx in sign_changes:
            if idx + 1 < len(real):
                frac = -real[idx] / (real[idx+1] - real[idx] + 1e-30)
                crossings.append(idx + frac)

        crossings = np.array(crossings)

        # Expected crossings at half-symbol intervals
        # (zero crossings happen at ~0.5 and ~1.0 of each symbol period)
        half_sps = self.samples_per_symbol / 2.0
        # you round where crossing is divided by the half sps to snap it to the the expected position
        expected = np.round(crossings / half_sps) * half_sps

        timing_errors_samples = crossings - expected
        timing_errors_sec     = timing_errors_samples / self.samp_rate
        rms_jitter_ns         = np.std(timing_errors_sec) * 1e9

        return rms_jitter_ns

    # ------------------------------------------------------------------
    # GNU Radio work() — called by the scheduler
    # ------------------------------------------------------------------
    def work(self, input_items, output_items):
        samples = input_items[0]
        self._buffer.extend(samples.tolist())

        while len(self._buffer) >= self.WINDOW_SIZE:
            window = np.array(self._buffer[:self.WINDOW_SIZE], dtype=np.complex64)
            self._buffer = self._buffer[self.WINDOW_SIZE:]

            snr_db, noise_dbm = self._estimate_snr(window)
            freq_offset, doppler = self._estimate_freq_offset(window)
            jitter = self._estimate_jitter(window)

            # Only push if all estimates succeeded
            if all(v is not None for v in [snr_db, noise_dbm, freq_offset, doppler, jitter]):
                self.logger.update_iq_metrics(
                    snr_db=snr_db,
                    noise_floor_dbm=noise_dbm,
                    freq_offset_hz=freq_offset,
                    doppler_hz=doppler,
                    jitter_ns=jitter
                )

                # # JUST FOR SIMULATION
                # self.logger.log_packet_outcome(success=True, ber=0.0)

        return len(samples)





# ---------------------------------------------------------------------------
# LOAD METRICS CSV INTO DATAFRAME FOR BN TRAINING
# Call this after a logging session to get a DataFrame ready for
# update_cpd_from_observations() in sdr_bn.py
# ---------------------------------------------------------------------------

def load_metrics_for_bn(csv_path: str):
    """
    Load a metrics CSV and return a DataFrame with only the
    discrete BN columns. Drops rows where any value is 'unknown'.

    Usage:
        from rf_metrics import load_metrics_for_bn
        from sdr_bn import build_model, update_cpd_from_observations

        df = load_metrics_for_bn('packet-logs/rf-metrics-2026-02-25.csv')
        model = build_model()
        model = update_cpd_from_observations(model, df)
    """
    import pandas as pd

    df = pd.read_csv(csv_path)

    bn_columns = ['SNR', 'ChannelNoise', 'FreqOffset', 'Doppler',
                  'Jitter', 'BER', 'PacketSuccess']

    df_bn = df[bn_columns].copy()
    df_bn = df_bn[df_bn.ne('unknown').all(axis=1)]  # drop incomplete rows

    print(f"Loaded {len(df_bn)} usable observations from {csv_path}")
    print(df_bn.value_counts('PacketSuccess'))

    return df_bn