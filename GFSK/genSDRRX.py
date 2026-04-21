"""
rx_flowgraph_generic.py

RX flowgraph that supports both GFSK and PSK/QAM demodulation.
The RX packet reader (rxBlock) is identical for both — only the
demodulator and unpacker stage differ.

Signal flow:
  GFSK:     SDR → resamp+LPF → AGC → gfsk_demod ──────────────────→ destination (unpacked bits)
  PSK/QAM:  SDR → resamp+LPF → AGC → generic_demod → unpacker (1:8) → destination (unpacked bits)

The unpacker reverses what the TX packer did: each packed byte
from generic_demod becomes 8 individual bit-per-byte samples
that the RX block's bit-level sync search can process.
"""

from gnuradio import gr, blocks, digital, filter, analog, qtgui
from gnuradio.filter import firdes
import threading
from pymavlink.dialects.v20 import common as mavlink2
import osmosdr
import sys
import sip
import signal
from PyQt5 import Qt
import numpy as np
from rxBlock import mav_packet_reader_with_metrics
from rf_metrics import RFMetricsProbe, MetricsLogger


class flow_graph(gr.top_block, Qt.QWidget):
    def __init__(self, modulation='gfsk'):
        """
        Parameters
        ----------
        modulation : str
            Must match the TX modulation. One of 'gfsk', 'bpsk', 'qpsk', '8psk', '16qam'
        """
        gr.top_block.__init__(self)
        Qt.QWidget.__init__(self)
        self.setWindowTitle(f"RX — {modulation.upper()}")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "osmos_test")
        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)

        ####################################
        # Variables
        ####################################
        self.modulation = modulation
        self.samp_rate = 100e3
        self.sdr_samp_rate = 2e6
        self.center_freq = 915e6
        self.samples_per_symbol = 4
        self.sensitivity = 0.785
        self.bt = 0.5
        self.gain_mu = 0.08
        self.mu = 0.5
        self.omega_relative_limit = 0.02
        self.freq_error = 0.0
        self.rx_decimation = 20
        self.sdr_RF_gain = 30

        self.constellations = {
            'bpsk':  digital.constellation_bpsk().base(),
            'qpsk':  digital.constellation_qpsk().base(),
            '8psk':  digital.constellation_8psk().base(),
            '16qam': digital.constellation_16qam().base(),
        }

        #######################
        # Blocks
        #######################

        # RX resampler + LPF
        # For PSK/QAM the occupied bandwidth is determined by excess_bw (RRC rolloff)
        # rather than BT*symbol_rate. With excess_bw=0.35:
        #   BW = symbol_rate * (1 + excess_bw) = 25000 * 1.35 = 33.75 kHz
        # This is similar to GFSK with h=0.5 and BT=0.5, so same filter works.
        self.rx_resampler_lowpass = filter.freq_xlating_fir_filter_ccf(
            decimation=self.rx_decimation,
            taps=firdes.low_pass(
                gain=1,
                sampling_freq=self.sdr_samp_rate,
                cutoff_freq=(self.samp_rate / self.samples_per_symbol) * (1 + self.bt) * 1.25,
                transition_width=(self.samp_rate / self.samples_per_symbol) * (1 + self.bt) * 1.25 * 0.25
            ),
            center_freq=0,    # adjust if you have a known freq offset
            sampling_freq=self.sdr_samp_rate
        )

        # AGC
        self.agc = analog.agc2_cc(
            attack_rate=1e-2,
            decay_rate=1e-3,
            reference=1.0,
            gain=1.0
        )

        # ---- Demodulator selection ----
        if modulation == 'gfsk':
            self.demod = digital.gfsk_demod(
                samples_per_symbol=self.samples_per_symbol,
                sensitivity=self.sensitivity,
                gain_mu=self.gain_mu,
                mu=self.mu,
                omega_relative_limit=self.omega_relative_limit,
                freq_error=self.freq_error,
                verbose=False,
                log=False
            )
            self.unpacker = None  # gfsk_demod already outputs unpacked bits
        else:
            # PSK/QAM: generic_demod outputs packed bytes
            # We unpack to individual bits so the RX block's
            # bit-level sync search works identically to GFSK.
            self.demod = digital.generic_demod(
                constellation=self.constellations[modulation],
                differential=True,
                samples_per_symbol=self.samples_per_symbol,
                pre_diff_code=True,
                excess_bw=0.35,
                freq_bw=6.28/50,      # wider than default for faster acquisition
                timing_bw=6.28/50,
                phase_bw=6.28/50,
                verbose=False,
                log=False
            )

            # Unpack: each packed byte → 8 individual bit bytes
            # This is the inverse of the TX packer.
            # After this, the data stream looks exactly like gfsk_demod output:
            # one bit per byte, 0x00 or 0x01.
            self.unpacker = blocks.packed_to_unpacked_bb(
                1,               # bits per output byte
                gr.GR_MSB_FIRST  # must match TX packer
            )

        # SDR source
        self.osmosdr_source = osmosdr.source(
            args="numchan=1 bladeRF=0"
        )
        self.osmosdr_source.set_time_unknown_pps(osmosdr.time_spec_t())
        self.osmosdr_source.set_sample_rate(self.sdr_samp_rate)
        self.osmosdr_source.set_center_freq(self.center_freq, 0)
        self.osmosdr_source.set_antenna('RX1', 0)
        self.osmosdr_source.set_freq_corr(0, 0)
        self.osmosdr_source.set_dc_offset_mode(0, 0)
        self.osmosdr_source.set_iq_balance_mode(0, 0)
        self.osmosdr_source.set_gain_mode(False, 0)
        self.osmosdr_source.set_gain(self.sdr_RF_gain, 0)
        self.osmosdr_source.set_if_gain(10, 0)
        self.osmosdr_source.set_bb_gain(16, 0)
        self.osmosdr_source.set_bandwidth(0, 0)

        # FFT display
        self.fft_sink = qtgui.freq_sink_c(
            1024, 5, self.center_freq, self.sdr_samp_rate, "OTA Signal"
        )
        self.fft_sink.set_update_time(0.10)
        self.fft_sink.set_y_axis(-140, 10)
        self.fft_win = sip.wrapinstance(self.fft_sink.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self.fft_win, 0, 0, 1, 1)

        # Metrics
        self.metrics_logger = MetricsLogger(getGain=self.getGain)
        self.metrics_logger.start()
        self.metrics_probe = RFMetricsProbe(
            samp_rate=self.samp_rate,
            samples_per_symbol=self.samples_per_symbol,
            center_freq=self.center_freq,
            logger=self.metrics_logger,
            sensitivity=self.sensitivity
        )

        # Packet reader (identical for all modulations — it sees unpacked bits)
        self.destination = mav_packet_reader_with_metrics(
            self.center_freq,
            setSDRGain=self.setGain,
            metrics_logger=self.metrics_logger,
            publish_to_gcs=True,
            host="192.168.0.255",
            port=8080
        )

        ##########################
        # Connections
        ##########################
        self.connect(self.osmosdr_source, self.fft_sink)
        self.connect(self.osmosdr_source, self.rx_resampler_lowpass)
        self.connect(self.rx_resampler_lowpass, self.agc)
        self.connect(self.agc, self.demod)

        if self.unpacker is not None:
            # PSK/QAM: demod (packed bytes) → unpacker → destination (unpacked bits)
            self.connect(self.demod, self.unpacker)
            self.connect(self.unpacker, self.destination)
        else:
            # GFSK: demod outputs unpacked bits directly
            self.connect(self.demod, self.destination)

        self.connect(self.rx_resampler_lowpass, self.metrics_probe)

    def closeEvent(self, event):
        self._safe_shutdown()
        event.accept()

    def getGain(self):
        return self.sdr_RF_gain

    def setGain(self, gain):
        self.sdr_RF_gain = gain
        return self.osmosdr_source.set_gain(gain, 0)

    def _safe_shutdown(self):
        print("Shutting down BladeRF RX...")
        try:
            self.osmosdr_source.set_gain(0, 0)
            self.osmosdr_source.set_if_gain(0, 0)
            self.osmosdr_source.set_bb_gain(0, 0)
        except Exception as e:
            print(f"Warning: could not zero gains: {e}", file=sys.stderr)

        self.metrics_logger.close()
        self.stop()
        self.wait()
        print("Flow graph stopped.")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--mod', default='gfsk',
                        choices=['gfsk', 'bpsk', 'qpsk', '8psk', '16qam'],
                        help='Modulation scheme (must match TX)')
    args = parser.parse_args()

    app = Qt.QApplication(sys.argv)
    tb = flow_graph(modulation=args.mod)
    tb.show()
    app.processEvents()

    def sig_handler(sig=None, frame=None):
        print("\nCaught SIGINT, shutting down...")
        tb._safe_shutdown()
        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    print(f"[RX] Starting with modulation: {args.mod.upper()}")
    tb.start()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        sig_handler()