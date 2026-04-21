"""
tx_flowgraph_generic.py

TX flowgraph that supports both GFSK and PSK/QAM modulation.
The packet source is identical for both — only the modulator
and the packer stage differ.

Signal flow:
  GFSK:     source (unpacked bits) ──────────────────→ gfsk_mod → gain → resamp → SDR
  PSK/QAM:  source (unpacked bits) → packer (8:1) → generic_mod → gain → resamp → SDR
"""

from gnuradio import gr, blocks, digital, filter, analog, qtgui
from gnuradio.filter import firdes
from PyQt5 import Qt
from genTXBlock import mav_packet_source
from rf_metrics import RFMetricsProbe, MetricsLogger
import threading
from pymavlink.dialects.v20 import common as mavlink2
import osmosdr
import sys
import sip
import signal
import time
import gc


class flow_graph(gr.top_block, Qt.QWidget):
    def __init__(self, modulation='gfsk'):
        """
        Parameters
        ----------
        modulation : str
            One of 'gfsk', 'bpsk', 'qpsk', '8psk', '16qam'
        """
        gr.top_block.__init__(self)
        Qt.QWidget.__init__(self)
        self.setWindowTitle(f"TX — {modulation.upper()}")
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
        self.sensitivity = 0.785   # h=0.5
        self.bt = 0.5
        self.gain_mu = 0.08
        self.mu = 0.5
        self.omega_relative_limit = 0.02
        self.freq_error = 0.0
        self.tx_interpolation = 20
        self.tx_decimation = 1
        self.fractional_bw = 0.49
        self.tx_gain_scalar = 1
        self.sdr_RF_gain = 35

        # Constellation lookup
        self.constellations = {
            'bpsk':  digital.constellation_bpsk().base(),
            'qpsk':  digital.constellation_qpsk().base(),
            '8psk':  digital.constellation_8psk().base(),
            '16qam': digital.constellation_16qam().base(),
        }

        #######################
        # Blocks
        #######################

        # TX resampler: 100 kHz → 2 MHz
        self.tx_resampler = filter.rational_resampler_ccf(
            interpolation=self.tx_interpolation,
            decimation=self.tx_decimation,
            taps=[],
            fractional_bw=self.fractional_bw
        )

        self.tx_gain = blocks.multiply_const_cc(self.tx_gain_scalar)

        # ---- Modulator selection ----
        if modulation == 'gfsk':
            # GFSK: source outputs unpacked bits → mod directly
            self.mod = digital.gfsk_mod(
                samples_per_symbol=self.samples_per_symbol,
                sensitivity=self.sensitivity,
                bt=self.bt,
                verbose=False,
                log=False,
                do_unpack=False   # expects unpacked bits (1 bit per byte)
            )
            self.packer = None  # no packing needed
        else:
            # PSK/QAM: source outputs unpacked bits → pack → mod
            #
            # Why the packer is needed:
            #   Your source outputs one bit per byte: [0x00, 0x01, 0x00, 0x01, ...]
            #   generic_mod expects packed bytes:      [0x55, ...]
            #   The packer collects 8 unpacked bytes into 1 packed byte.
            self.packer = blocks.unpacked_to_packed_bb(
                1,               # bits per input byte (1 = unpacked)
                gr.GR_MSB_FIRST  # bit order matches np.unpackbits
            )

            self.mod = digital.generic_mod(
                constellation=self.constellations[modulation],
                differential=True,       # handles phase ambiguity
                samples_per_symbol=self.samples_per_symbol,
                pre_diff_code=True,
                excess_bw=0.35,          # RRC rolloff
                verbose=False,
                log=False
            )

        # SDR sink
        self.osmosdr_sink = osmosdr.sink(
            args="numchan=1 bladeRF=0"
        )
        self.osmosdr_sink.set_sample_rate(self.sdr_samp_rate)
        self.osmosdr_sink.set_center_freq(self.center_freq, 0)
        self.osmosdr_sink.set_antenna('TX1', 0)
        self.osmosdr_sink.set_freq_corr(0, 0)
        self.osmosdr_sink.set_gain(self.sdr_RF_gain, 0)
        self.osmosdr_sink.set_if_gain(15, 0)
        self.osmosdr_sink.set_bb_gain(15, 0)
        self.osmosdr_sink.set_bandwidth(0, 0)

        # GUI sinks
        self.qt_freq_sink = qtgui.freq_sink_c(
            1024, 5, self.center_freq, self.sdr_samp_rate, "TX Monitor"
        )
        self.qt_freq_sink.set_update_time(0.10)
        self.qt_freq_sink.set_y_axis(-140, 10)

        self._qt_freq_sink_win = sip.wrapinstance(
            self.qt_freq_sink.qwidget(), Qt.QWidget
        )
        self.top_grid_layout.addWidget(self._qt_freq_sink_win, 0, 0, 1, 1)

        # Metrics
        self.metrics_logger = MetricsLogger(getGain=self.getSDRgain)
        self.metrics_logger.start()
        self.metrics_probe = RFMetricsProbe(
            samp_rate=self.samp_rate,
            samples_per_symbol=self.samples_per_symbol,
            center_freq=self.center_freq,
            logger=self.metrics_logger,
            sensitivity=self.sensitivity
        )

        # Packet source (identical for all modulations)
        self.source = mav_packet_source(
            self.center_freq, self.setSDRGain, self.osmosdr_sink,
            metrics_logger=self.metrics_logger
        )

        ##########################
        # Connections
        ##########################
        if self.packer is not None:
            # PSK/QAM path: source → packer → mod
            self.connect(self.source, self.packer)
            self.connect(self.packer, self.mod)
        else:
            # GFSK path: source → mod directly
            self.connect(self.source, self.mod)

        # Common path from modulator onward
        self.connect(self.mod, self.tx_gain)
        self.connect(self.tx_gain, self.tx_resampler)
        self.connect(self.tx_resampler, self.osmosdr_sink)
        self.connect(self.tx_resampler, self.qt_freq_sink)
        self.connect(self.tx_resampler, self.metrics_probe)

    def _safe_shutdown(self):
        print("Shutting down BladeRF TX...")
        try:
            self.osmosdr_sink.set_gain(0, 0)
            self.osmosdr_sink.set_if_gain(0, 0)
            self.osmosdr_sink.set_bb_gain(0, 0)
            self.osmosdr_sink.set_center_freq(2.4e9, 0)
        except Exception as e:
            print(f"Warning: could not zero gains: {e}", file=sys.stderr)

        self.source.stop()
        self.metrics_logger.close()
        self.stop()
        self.wait()

        try:
            del self.osmosdr_sink
            self.osmosdr_sink = None
            gc.collect()
        except Exception as e:
            print(f"Warning: could not release sink: {e}", file=sys.stderr)

        print("Flow graph stopped and device released.")

    def setSDRGain(self, gain):
        self.sdr_RF_gain = gain
        print(f'[mavSDRTX] Gain updated to {self.sdr_RF_gain}')

    def getSDRgain(self):
        return self.sdr_RF_gain


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--mod', default='gfsk',
                        choices=['gfsk', 'bpsk', 'qpsk', '8psk', '16qam'],
                        help='Modulation scheme')
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

    print(f"[TX] Starting with modulation: {args.mod.upper()}")
    tb.start()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        sig_handler()