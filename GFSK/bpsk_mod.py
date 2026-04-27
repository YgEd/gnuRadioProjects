#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Not titled yet
# GNU Radio version: 3.10.9.2

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio import blocks
from gnuradio import digital
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
import sip

sys.path.append('../GFSK')


from genTXBlock import mav_packet_source
from rf_metrics import RFMetricsProbe, MetricsLogger
from rxBlock import mav_packet_reader_with_metrics



class fil_Transfer_skeleton(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Not titled yet", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Not titled yet")
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

        self.settings = Qt.QSettings("GNU Radio", "fil_Transfer_skeleton")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 100e3
        self.sps = sps = 4
        self.symbol_rate = symbol_rate = samp_rate/sps
        self.qpsk_obj = qpsk_obj = digital.constellation_qpsk().base()
        self.bpsk_obj = bpsk_obj = digital.constellation_bpsk().base()
        self.qpsk_obj.set_npwr(1.0)
        # self.modulation = modulation
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

        ##################################################
        # Blocks
        ##################################################

        self.qtgui_time_sink_x_1 = qtgui.time_sink_c(
            256, #size
            symbol_rate, #samp_rate
            "Recovered Symbols", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_time_sink_x_1.set_update_time(0.10)
        self.qtgui_time_sink_x_1.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_1.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_1.enable_tags(True)
        self.qtgui_time_sink_x_1.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_1.enable_autoscale(False)
        self.qtgui_time_sink_x_1.enable_grid(False)
        self.qtgui_time_sink_x_1.enable_axis_labels(True)
        self.qtgui_time_sink_x_1.enable_control_panel(False)
        self.qtgui_time_sink_x_1.enable_stem_plot(False)


        labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(2):
            if len(labels[i]) == 0:
                if (i % 2 == 0):
                    self.qtgui_time_sink_x_1.set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    self.qtgui_time_sink_x_1.set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                self.qtgui_time_sink_x_1.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_1.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_1.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_1.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_1.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_1.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_1_win = sip.wrapinstance(self.qtgui_time_sink_x_1.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_time_sink_x_1_win)
        self.qtgui_time_sink_x_0 = qtgui.time_sink_c(
            1024, #size
            samp_rate, #samp_rate
            "QPSK MOD Output", #name
            1, #number of inputs
            None # parent
        )

        self.demod_gui = qtgui.time_sink_c(
            1024,
            samp_rate,
            "QPSK DEMOD Output",
            1,
            None
        )
        self.qtgui_time_sink_x_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_0.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0.enable_tags(True)
        self.qtgui_time_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_0.enable_autoscale(False)
        self.qtgui_time_sink_x_0.enable_grid(False)
        self.qtgui_time_sink_x_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_0.enable_control_panel(False)
        self.qtgui_time_sink_x_0.enable_stem_plot(False)


        labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(2):
            if len(labels[i]) == 0:
                if (i % 2 == 0):
                    self.qtgui_time_sink_x_0.set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    self.qtgui_time_sink_x_0.set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                self.qtgui_time_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_time_sink_x_0_win)
        self.filter_fft_rrc_filter_0 = filter.fft_filter_ccc(1, firdes.root_raised_cosine(1, samp_rate, symbol_rate, 0.35, (int(11*(samp_rate/symbol_rate)))), 1)
        self.digital_symbol_sync_xx_0 = digital.symbol_sync_cc(
            digital.TED_SIGNAL_TIMES_SLOPE_ML,
            sps,
            0.045,
            1.0,
            1.0,
            1.5,
            1,
            digital.constellation_bpsk().base(),
            digital.IR_MMSE_8TAP,
            128,
            [])
        self.digital_diff_decoder_bb_0 = digital.diff_decoder_bb(
            2, # modulation order 2^(amount of bits per symbol)
        digital.DIFF_DIFFERENTIAL)
        
        self.digital_costas_loop_cc_0 = digital.costas_loop_cc(
            (6.28/100), 
            2, # amount of symbols = 2^(amount of bits per symbol)
            False)
        
        self.digital_constellation_modulator_0 = digital.generic_mod(
            constellation=bpsk_obj,
            differential=True,
            samples_per_symbol=sps,
            pre_diff_code=True,
            excess_bw=0.35,
            verbose=False,
            log=False,
            truncate=False)
        self.digital_constellation_decoder_cb_0 = digital.constellation_decoder_cb(bpsk_obj)
        self.blocks_vector_sink_x_0 = blocks.vector_sink_b(1, 1024)
        self.blocks_throttle2_0 = blocks.throttle( gr.sizeof_gr_complex*1, samp_rate, True, 0 if "auto" == "auto" else max( int(float(0.1) * samp_rate) if "auto" == "time" else int(0.1), 1) )
        self.blocks_tag_gate_0 = blocks.tag_gate(gr.sizeof_gr_complex * 1, False)
        self.blocks_tag_gate_0.set_single_key("")


        ##################################################
        # Metrics Blocks

        self.metrics_logger = MetricsLogger(getGain=self.getSDRgain)
        self.metrics_logger.start()
        self.metrics_probe = RFMetricsProbe(
            samp_rate=self.samp_rate,
            samples_per_symbol=self.samples_per_symbol,
            center_freq=self.center_freq,
            logger=self.metrics_logger,
            sensitivity=self.sensitivity
        )

        ##################################################
        # Mav Source and Destination Blocks
        self.source = mav_packet_source(
            self.center_freq, self.setSDRGain, None,
            metrics_logger=self.metrics_logger,
            packed=True
        )

        self.destination = mav_packet_reader_with_metrics(
            self.center_freq,
            setSDRGain=self.setSDRGain,
            metrics_logger=self.metrics_logger,
            publish_to_gcs=True,
            host="192.168.0.255",
            port=8080
        )

        # unpacker for mod output
        self.unpack = blocks.unpack_k_bits_bb(1) # number is how many bits per symbol

        # constellation graph gui
        self.constellation_plot = qtgui.const_sink_c(
            1024,
            "received constellation",
            1,
            None
        )
        self.constellation_plot.set_update_time(0.10)
        self.constellation_plot.set_y_axis(-2,2)
        self.constellation_plot.set_x_axis(-2,2)

        self._constellation_plot_win = sip.wrapinstance(
            self.constellation_plot.qwidget(), Qt.QWidget
        )
        self.top_layout.addWidget(self._constellation_plot_win)




        ##################################################
        # Connections
        ##################################################
        self.connect(self.source, (self.digital_constellation_modulator_0, 0))
        self.connect((self.digital_constellation_modulator_0, 0), (self.blocks_tag_gate_0, 0))
        self.connect((self.digital_constellation_modulator_0, 0), (self.qtgui_time_sink_x_0, 0))
        self.connect((self.blocks_tag_gate_0), (self.blocks_throttle2_0, 0))
        self.connect((self.blocks_throttle2_0, 0), (self.filter_fft_rrc_filter_0, 0))
        self.connect((self.filter_fft_rrc_filter_0, 0), (self.digital_symbol_sync_xx_0, 0))
        self.connect((self.digital_symbol_sync_xx_0, 0), (self.digital_costas_loop_cc_0, 0))
        self.connect((self.digital_costas_loop_cc_0, 0), (self.digital_constellation_decoder_cb_0, 0))
        self.connect((self.digital_costas_loop_cc_0, 0), (self.qtgui_time_sink_x_1, 0))
        self.connect(self.digital_costas_loop_cc_0, self.constellation_plot)
        self.connect((self.digital_constellation_decoder_cb_0), (self.digital_diff_decoder_bb_0, 0))
        self.connect((self.digital_diff_decoder_bb_0, 0), self.unpack)
        self.connect(self.unpack, self.destination)
 

    def getSDRgain(self):
        return self.sdr_RF_gain

    def setSDRGain(self, gain):
        self.sdr_RF_gain = gain
        return self.sdr_RF_gain


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "fil_Transfer_skeleton")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_symbol_rate(self.samp_rate/4)
        self.blocks_throttle2_0.set_sample_rate(self.samp_rate)
        self.filter_fft_rrc_filter_0.set_taps(firdes.root_raised_cosine(1, self.samp_rate, self.symbol_rate, 0.35, (int(11*(self.samp_rate/self.symbol_rate)))))
        self.qtgui_time_sink_x_0.set_samp_rate(self.samp_rate)

    def get_symbol_rate(self):
        return self.symbol_rate

    def set_symbol_rate(self, symbol_rate):
        self.symbol_rate = symbol_rate
        self.filter_fft_rrc_filter_0.set_taps(firdes.root_raised_cosine(1, self.samp_rate, self.symbol_rate, 0.35, (int(11*(self.samp_rate/self.symbol_rate)))))
        self.qtgui_time_sink_x_1.set_samp_rate(self.symbol_rate)

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.digital_symbol_sync_xx_0.set_sps(self.sps)

    def get_qpsk_obj(self):
        return self.qpsk_obj

    def set_qpsk_obj(self, qpsk_obj):
        self.qpsk_obj = qpsk_obj
        self.digital_constellation_decoder_cb_0.set_constellation(self.qpsk_obj)




def main(top_block_cls=fil_Transfer_skeleton, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
