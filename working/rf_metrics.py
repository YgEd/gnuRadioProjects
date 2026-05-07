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
import libsql
from dotenv import load_dotenv
import itertools
import pandas as pd



# ---------------------------------------------------------------------------
# CONSTANTS from your flowgraph — keep in sync with rx.py
# ---------------------------------------------------------------------------
SAMP_RATE          = 100e3    # post-resampler sample rate
SAMPLES_PER_SYMBOL = 4
CENTER_FREQ        = 915e6
SYMBOL_RATE        = SAMP_RATE / SAMPLES_PER_SYMBOL   # 25 kHz

# load env
load_dotenv()


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

    def __init__(self, getGain, getMod, direction, log_dir='packet-logs'):
        self.log_dir = log_dir
        self.getGain = getGain
        self.getMod = getMod
        self.direction = direction
        os.makedirs(log_dir, exist_ok=True)

        now = datetime.datetime.now().isoformat()
        self.filepath = os.path.join(log_dir, f'rf-metrics-{now}.csv')
        self._lock = threading.Lock()

        # variable to mark when the object is active to know when to cleanup
        self.active = None

        self.sql_table = f'{direction}_metrics'

        # Latest IQ-derived metrics — updated by RFMetricsProbe
        self._iq_metrics = {
            'snr_db':            None,
            'noise_floor_dbm':   None,
            'freq_offset_hz':    None,
            'doppler_hz':        None,
            'jitter_ns':         None,
            'RSSI': None,
        }

        # Static baseline freq for Doppler calculation.
        # Set this from a known stationary measurement before flight.
        self._baseline_freq_hz = None
        self.headers = [
            # Original measurement fields
            'timestamp', 'tx_rx', 'gain', 'frequency', 'modulation', 'RSSI', 'snr_db', 'noise_floor_dbm', 'freq_offset_hz', 'doppler_hz', 'jitter_ns', 'ber',
            # Discretized / derived fields
            'snr_bin', 'channelnoise_bin', 'freq_offset_bin', 'doppler_bin', 'jitter_bin', 'ber_bin', 'failure_level','packet_success',
            # Detailed packet info fields
            'message', 'payload_len', 'payload_len_crc', 'payload_crc',
            'raw_payload_bytes', 'whitened_payload_bytes', 'raw_packet_bytes'
        ]
        self._write_header()

    def _writedb(self):
        try:
            print('[MetricLogger] Attempting to connect to db...')
            conn = libsql.connect(
                "rfdata.db",
                sync_url = os.getenv("TURSO_DB_URL"),
                auth_token = os.getenv("TURSO_DB_TOKEN")
            )
            conn.sync() # force immediate connection to db
        except Exception as e:
            print(f'[MetricLogger] Failed to connect to db: {e}')
            return

        print('[MetricLogger] Successfully connected to db, writing to db...')
        
        conn.execute(f"""
            CREATE TABLE IF NOT EXISTS {self.sql_table} (
                id      INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp   TEXT,
                tx_rx       TEXT,
                gain        INTEGER,
                frequency   REAL,
                modulation  TEXT,
                RSSI        REAL,
                snr_db      REAL,
                noise_floor_dbm     REAL,
                freq_offset_hz      REAL,
                doppler_hz          REAL,
                jitter_ns           REAL,
                ber                 REAL,
                snr_bin             TEXT,
                channelnoise_bin    TEXT,
                freq_offset_bin     TEXT,
                doppler_bin         TEXT,
                jitter_bin          TEXT,
                ber_bin             TEXT,
                failure_level      TEXT,
                packet_success      TEXT,
                message         TEXT,
                payload_len         INTEGER,
                payload_len_crc     INTEGER,
                payload_crc         INTEGER,
                raw_payload_bytes   BLOB,
                whitened_payload_bytes  BLOB,
                raw_packet_bytes    BLOB
                     )
                     """)
        with open(self.filepath) as f:
            total_rows = sum(1 for _ in f) - 1  # subtract 1 for header
        
        batch_size = 200
        sql = (
            f"INSERT INTO {self.sql_table} ({', '.join(self.headers)}) "
            f"VALUES ({', '.join('?' * len(self.headers))})"
        )

        print(f'[MetricLogger] {total_rows} rows to write...')

        with open(self.filepath) as f:
            reader = csv.DictReader(f)
            batch_num = 0
            total_written = 0

            while True:
                batch = [
                    tuple(row[h] for h in self.headers) for row in itertools.islice(reader, batch_size)
                ]

                if not batch:   # islice returns empty list when reader is exhausted
                    break

                batch_num += 1
                print(f'[MetricLogger] Writing batch {batch_num} ({len(batch)} rows)...')

                for i, row in enumerate(batch, 1):
                    conn.execute(sql, row)
                    print(f'\r[MetricLogger]   Row {i}/{total_rows-total_written} Batch {batch_num}', end='', flush=True)

                conn.commit()
                total_written += len(batch)
                print(f'\n[MetricLogger] Batch {batch_num} committed. Total so far: {total_written}')

        conn.sync()
        print(f'[MetricLogger] Done. {total_written} rows written.')

    
    def start(self):
        self.active = True
        print("[MetricLogger] MetricLogger Activated")

    def close(self):
        if self.active is not None:
            print("[MetricLogger] Closing metriclogger and writing logs to db")
            try:
                self._writedb()
            except Exception as e:
                print(f'[MetricLogger] Failed to connect to db: {e}')
            # write to db work
            self.active = None

    def _write_header(self):
    
        with open(self.filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.headers)

    def update_iq_metrics(self, snr_db, noise_floor_dbm,
                          freq_offset_hz, doppler_hz, jitter_ns, rssi):
        """Called by RFMetricsProbe each processing window."""
        with self._lock:
            self._iq_metrics = {
                'snr_db':          snr_db,
                'RSSI': rssi,
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

    def log_packet_outcome(self, TXRX, freq, packet_info, success: bool, ber: float | str):
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

            # Step 2: initialize row with None for all headers
        row = {key: None for key in self.headers}

        # Step 3: populate the base fields
        row.update({
            'timestamp': datetime.datetime.now().isoformat(),
            'tx_rx': TXRX,
            'gain': self.getGain(),
            'frequency': freq,
            'RSSI': m.get('RSSI'),
            'modulation': self.getMod(),
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
                'snr_bin': bin_snr(m['snr_db']),
                'channelnoise_bin': bin_channel_noise(m['noise_floor_dbm']),
                'freq_offset_bin': bin_freq_offset(m['freq_offset_hz']),
                'doppler_bin': bin_doppler(m['doppler_hz']),
                'jitter_bin': bin_jitter(m['jitter_ns']),
                'ber_bin': bin_ber(ber),
                'failure_level': packet_info['failure_level'],
                'packet_success': 'success' if success else 'failure'
            })
        else:
            row.update({
                'snr_bin': 'unknown',
                'channelnoise_bin': 'unknown',
                'freq_offset_bin': 'unknown',
                'doppler_bin': 'unknown',
                'jitter_bin': 'unknown',
                'ber_bin': 'unknown',
                'packet_success': False
            })

        # Step 5: populate detailed packet info fields
        row.update({
            'message': fmt(packet_info['message']),
            'payload_len': fmt(packet_info['payload_len']),
            'payload_len_crc': fmt(packet_info['payload_len_crc']),
            'payload_crc': fmt(packet_info['payload_crc']),
            'raw_payload_bytes': fmt(packet_info['raw_payload_bytes']),
            'whitened_payload_bytes': fmt(packet_info['whitened_payload_bytes']),
            'raw_packet_bytes': fmt(packet_info['raw_packet_bytes'])
        })

        # Step 6: write row to CSV safely
        with open(self.filepath, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.headers)
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
    if ber != 'N/A': #in the case the BER is not applicable (bad length or payload CRC)
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

    def __init__(self, samp_rate, samples_per_symbol, center_freq=0.0, logger=None, excess_bw=0.35, sensitivity=None, cal_offset_dbm=None, total_sdr_gain_db=0.0):
        gr.sync_block.__init__(
            self,
            name='RFMetricsProbe',
            in_sig=[np.complex64],
            out_sig=None               # sink block — no output
        )
        self.samp_rate          = float(samp_rate)
        self.samples_per_symbol = float(samples_per_symbol)
        self.symbol_rate        = self.samp_rate / self.samples_per_symbol
        self.center_freq        = center_freq
        self.logger             = logger
        self.excess_bw          = excess_bw
        self.signal_bw_hz       = self.symbol_rate * (1.0 + excess_bw)
        self.cal_offset_dbm     = cal_offset_dbm
        self.total_sdr_gain_db  = total_sdr_gain_db
        self._freq_history      = deque(maxlen=200)
        self.sensitivity = sensitivity

        self._buffer     = []
        self._noise_ref  = None   # set during known-silence calibration

        # Running list of peak frequencies for Doppler baseline estimation
        self._freq_history = deque(maxlen=50)

    def _estimate_rssi(self, samples):
        """
        Average received power across the analysis bandwidth.
        Returns (rssi, units) where units is 'dBm' if calibrated, else 'dBFS'.
        """
        if len(samples) == 0:
            return None, None

        power_lin = float(np.mean(np.abs(samples)**2))
        if power_lin <= 0:
            return None, None

        rssi_dbfs = 10.0 * np.log10(power_lin + 1e-30)

        if self.cal_offset_dbm is not None:
            rssi_dbm = rssi_dbfs + self.cal_offset_dbm - self.total_sdr_gain_db
            return rssi_dbm, 'dBm'
        return rssi_dbfs, 'dBFS'

    # ------------------------------------------------------------------
    # SNR ESTIMATION
    # Uses Welch Power Spectral Density (PSD) — signal power is in the occupied bandwidth,
    # noise power is in the spectral guard regions.
    # ------------------------------------------------------------------
    def _estimate_snr(self, samples):
        """
        Modulation-agnostic SNR via PSD partitioning.
        Signal band: ±(Rs(1+α))/2 around DC (assumes signal already centered).
        Noise band : spectrum outside (1.3 × signal half-BW), within Nyquist.
        """
        if len(samples) < 512:
            return None, None

        freqs, psd = welch(samples, fs=self.samp_rate,
                        nperseg=min(1024, len(samples)),
                        return_onesided=False)
        freqs = np.fft.fftshift(freqs)
        psd   = np.fft.fftshift(psd)

        sig_half_bw   = self.signal_bw_hz / 2.0
        guard_inner   = sig_half_bw * 1.3        # leave a small skirt
        nyquist       = self.samp_rate / 2.0

        sig_mask   = np.abs(freqs) <= sig_half_bw
        noise_mask = (np.abs(freqs) > guard_inner) & (np.abs(freqs) <= nyquist)

        if sig_mask.sum() == 0 or noise_mask.sum() == 0:
            return None, None

        p_sig_plus_noise = float(np.mean(psd[sig_mask]))      # signal+noise in passband
        p_noise_density  = float(np.mean(psd[noise_mask]))    # noise PSD outside

        if p_noise_density <= 0 or p_sig_plus_noise <= p_noise_density:
            return None, None

        snr_db = 10.0 * np.log10((p_sig_plus_noise - p_noise_density) / p_noise_density)

        noise_floor = 10.0 * np.log10(p_noise_density + 1e-30)
        if self.cal_offset_dbm is not None:
            noise_floor = noise_floor + self.cal_offset_dbm - self.total_sdr_gain_db

        return snr_db, noise_floor

    # ------------------------------------------------------------------
    # FREQUENCY OFFSET / DOPPLER
    # FFT peak relative to center. Static offset = freq_offset.
    # Change from baseline = Doppler.
    # ------------------------------------------------------------------
    def _estimate_freq_offset(self, samples):
        """
        Carrier offset estimate via spectral centroid of the signal band.
        Valid for any spectrum-symmetric modulation; biased only if the spectrum
        is asymmetric (rare in practice — RRC and GFSK are both symmetric).
        """
        if len(samples) < 512:
            return 0.0, 0.0

        freqs, psd = welch(samples, fs=self.samp_rate,
                        nperseg=min(2048, len(samples)),
                        return_onesided=False)
        freqs = np.fft.fftshift(freqs)
        psd   = np.fft.fftshift(psd)

        # Search band: 1.3× the nominal signal half-BW so a real offset
        # doesn't push the signal out of the search window.
        search_half_bw = self.signal_bw_hz / 2.0 * 1.3
        band_mask = np.abs(freqs) <= search_half_bw

        if not band_mask.any():
            return 0.0, 0.0

        # Subtract noise floor so the centroid isn't dragged toward DC by
        # broadband noise. Use median of out-of-band PSD as the noise estimate.
        out_of_band = psd[~band_mask]
        noise_floor = float(np.median(out_of_band)) if out_of_band.size else 0.0

        psd_signal = np.maximum(psd[band_mask] - noise_floor, 0.0)
        weight     = psd_signal.sum()

        if weight <= 0:
            return 0.0, 0.0

        freq_offset_hz = float(np.sum(freqs[band_mask] * psd_signal) / weight)

        self._freq_history.append(freq_offset_hz)

        if len(self._freq_history) >= 10:
            baseline   = np.median(list(self._freq_history)[:-1])
            doppler_hz = freq_offset_hz - baseline
        else:
            doppler_hz = 0.0

        return freq_offset_hz, doppler_hz

    # ------------------------------------------------------------------
    # JITTER ESTIMATION
    # Measures RMS timing error of zero crossings vs expected symbol grid.
    # Uses real part of the post-filter IQ samples.
    # ------------------------------------------------------------------
    def _estimate_jitter(self, samples):
        """
        Modulation-agnostic timing jitter.

        The squared envelope |x|^2 of any RRC-shaped signal carries a spectral
        line at the symbol rate (Gardner 1986 / Oerder–Meyr 1988). The phase
        of the Fourier coefficient at f = R_sym is proportional to the
        symbol-timing offset:

                tau_hat = -(T_sym / 2π) · arg(Σ_k |x[k]|^2 · exp(-j 2π k / sps))

        Jitter is the standard deviation of tau_hat across short windows
        (after detrending to remove residual frequency offset, which appears
        as a linear timing ramp).
        """
        sps = self.samples_per_symbol
        if sps < 2 or len(samples) < 32 * sps:
            return None

        env_sq = np.abs(samples) ** 2

        window_symbols = 32
        window_len     = int(window_symbols * sps)
        n_windows      = len(env_sq) // window_len
        if n_windows < 4:
            return None

        T_sym  = 1.0 / self.symbol_rate
        phases = np.empty(n_windows)

        for i in range(n_windows):
            chunk = env_sq[i * window_len : (i + 1) * window_len]
            k     = np.arange(len(chunk))
            X     = np.sum(chunk * np.exp(-1j * 2 * np.pi * k / sps))
            phases[i] = np.angle(X)

        # Unwrap so a residual frequency offset shows as a linear ramp,
        # which detrending then removes.
        phases_unwrapped = np.unwrap(phases)
        tau_sec          = -T_sym * phases_unwrapped / (2 * np.pi)

        x        = np.arange(len(tau_sec))
        slope, b = np.polyfit(x, tau_sec, 1)
        residual = tau_sec - (slope * x + b)

        rms_jitter_ns = float(np.std(residual)) * 1e9
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
            rssi, _ = self._estimate_rssi(window)

            # Only push if all estimates succeeded
            if all(v is not None for v in [snr_db, noise_dbm, freq_offset, doppler, jitter]):
                self.logger.update_iq_metrics(
                    snr_db=snr_db,
                    noise_floor_dbm=noise_dbm,
                    freq_offset_hz=freq_offset,
                    doppler_hz=doppler,
                    jitter_ns=jitter,
                    rssi=rssi,
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

    df = pd.read_csv(csv_path)

    bn_columns = ['SNR', 'ChannelNoise', 'FreqOffset', 'Doppler',
                  'Jitter', 'BER', 'PacketSuccess']

    df_bn = df[bn_columns].copy()
    df_bn = df_bn[df_bn.ne('unknown').all(axis=1)]  # drop incomplete rows

    print(f"Loaded {len(df_bn)} usable observations from {csv_path}")
    print(df_bn.value_counts('PacketSuccess'))

    return df_bn