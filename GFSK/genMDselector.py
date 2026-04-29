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
import numpy as np
import threading

sys.path.append('../GFSK')


from genTXBlock import mav_packet_source
from rf_metrics import RFMetricsProbe, MetricsLogger
from rxBlock import mav_packet_reader_with_metrics



class fil_Transfer_skeleton(gr.top_block, Qt.QWidget):

    def __init__(self, mod='bpsk'):
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

        # modulation scheme selection variables
        self.current_mode = 0
        self.mod_strs = ['BPSK', 'QPSK', '16QAM']
        
        ##################################################
        # Blocks
        ##################################################

        # modulation variables
        self.bpsk_obj = bpsk_obj = digital.constellation_bpsk().base()
        self.qpsk_obj = qpsk_obj = digital.constellation_qpsk().base()
        self.qam16_obj = qam16_obj = digital.constellation_16qam().base()
    

        # modulation selection
        self.mod_obj = bpsk_obj
        self.bits_per_symbol = 1
        self.rotational_sym_order = 2
        self.bps_arr = [1, 2, 4]
        self.rso_arr = [2, 4, 16]

        # # TX selector: 1 input (source), 3 outputs for each modulation type
        # self.tx_selector = blocks.selector(gr.sizeof_char, 0, 0)

        # global rrc_taps block
        nfilts = 32
        rrc_taps = firdes.root_raised_cosine(
            nfilts, #gain (= nfilts for polyphase)
            nfilts,# sampling rate
            1.0 /float(self.sps), # normalized symbol rate
            0.35, #excess BW
            11 * self.sps * nfilts #num of taps
        )

        ##################################################
        # TX Mod chains
        ##################################################

        # TX selector: 1 input (source), 3 outputs for each modulation type
        self.tx_in_selector = blocks.selector(gr.sizeof_char, 0, 0)
        self.tx_out_selector = blocks.selector(gr.sizeof_gr_complex, 0, 0)

        # constellation modulator blocks
        self.cmod_0 = self.digital_constellation_modulator_0 = digital.generic_mod(
            constellation=self.bpsk_obj,
            differential=True,
            samples_per_symbol=sps,
            pre_diff_code=True,
            excess_bw=0.35,
            verbose=False,
            log=False,
            truncate=False)
        
        self.cmod_1 = self.digital_constellation_modulator_0 = digital.generic_mod(
            constellation=self.qpsk_obj,
            differential=True,
            samples_per_symbol=sps,
            pre_diff_code=True,
            excess_bw=0.35,
            verbose=False,
            log=False,
            truncate=False)

        self.cmod_2 = self.digital_constellation_modulator_0 = digital.generic_mod(
            constellation=self.qam16_obj,
            differential=True,
            samples_per_symbol=sps,
            pre_diff_code=True,
            excess_bw=0.35,
            verbose=False,
            log=False,
            truncate=False)

        # put tx blocsk in a mode ordered array
        self.cmods = [self.cmod_0, self.cmod_1, self.cmod_2]

        ##################################################
        # RX Demod chains
        ##################################################

        # RX input selector
        self.rx_in_selector = blocks.selector(gr.sizeof_gr_complex, 0, 0) # 1 input, 3 outputs
        self.rx_out_selector = blocks.selector(gr.sizeof_char, 0, 0)

        # Polyphase Clock Sync blocks
        self.pfb_0 = self.pfb_clock_sync = digital.pfb_clock_sync_ccf(self.sps, 6.28/100, rrc_taps, nfilts, nfilts/2, 1.5, 1)
        self.pfb_1 = self.pfb_clock_sync = digital.pfb_clock_sync_ccf(self.sps, 6.28/100, rrc_taps, nfilts, nfilts/2, 1.5, 1)
        self.pfb_2 = self.pfb_clock_sync = digital.pfb_clock_sync_ccf(self.sps, 6.28/100, rrc_taps, nfilts, nfilts/2, 1.5, 1)

        # Differential Decoder blocks
        self.diffd_0 = digital.diff_decoder_bb(self.rso_arr[0], digital.DIFF_DIFFERENTIAL)
        self.diffd_1 = digital.diff_decoder_bb(self.rso_arr[1], digital.DIFF_DIFFERENTIAL)
        self.diffd_2 = digital.diff_decoder_bb(self.rso_arr[2], digital.DIFF_DIFFERENTIAL)

        # Costas loop blocks
        self.cl_0 = digital.costas_loop_cc((6.28/100), self.rso_arr[0], False)
        self.cl_1 = digital.costas_loop_cc((6.28/100), self.rso_arr[1], False)
        self.cl_2 = digital.costas_loop_cc((6.28/100), self.rso_arr[2], False)

        # Constellation decoder blocks
        self.cdecode_0 = digital.constellation_decoder_cb(self.bpsk_obj)
        self.cdecode_1 = digital.constellation_decoder_cb(self.qpsk_obj)
        self.cdecode_2 = digital.constellation_decoder_cb(self.qam16_obj)

        # unpacker blocks
        self.unpack_0 = blocks.unpack_k_bits_bb(self.bps_arr[0])
        self.unpack_1 = blocks.unpack_k_bits_bb(self.bps_arr[1])
        self.unpack_2 = blocks.unpack_k_bits_bb(self.bps_arr[2])
        
        # put rx chains in mode ordered arrays
        self.pfbs = [self.pfb_0, self.pfb_1, self.pfb_2]
        self.costasls = [self.cl_0, self.cl_1, self.cl_2]
        self.cdecodes = [self.cdecode_0, self.cdecode_1, self.cdecode_2]
        self.diffds = [self.diffd_0, self.diffd_1, self.diffd_2]
        self.unpacks = [self.unpack_0, self.unpack_1, self.unpack_2]

        ##################################################
        # GUI Blocks
        ##################################################

        # self.rx_gui_selector = blocks.selector(gr.sizeof_gr_complex, 0, 0)
        # self.tx_gui_selector = blocks.selector(gr.sizeof_gr_complex, 0, 0)


        # self.qtgui_time_sink_x_1 = qtgui.time_sink_c(
        #     256, #size
        #     symbol_rate, #samp_rate
        #     "RX Recovered Symbols", #name
        #     1, #number of inputs
        #     None # parent
        # )
        # self.qtgui_time_sink_x_1.set_update_time(0.10)
        # self.qtgui_time_sink_x_1.set_y_axis(-1, 1)

        # self.qtgui_time_sink_x_1.set_y_label('Amplitude', "")

        # self.qtgui_time_sink_x_1.enable_tags(True)
        # self.qtgui_time_sink_x_1.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        # self.qtgui_time_sink_x_1.enable_autoscale(False)
        # self.qtgui_time_sink_x_1.enable_grid(False)
        # self.qtgui_time_sink_x_1.enable_axis_labels(True)
        # self.qtgui_time_sink_x_1.enable_control_panel(False)
        # self.qtgui_time_sink_x_1.enable_stem_plot(False)


        # labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
        #     'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        # widths = [1, 1, 1, 1, 1,
        #     1, 1, 1, 1, 1]
        # colors = ['blue', 'red', 'green', 'black', 'cyan',
        #     'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        # alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
        #     1.0, 1.0, 1.0, 1.0, 1.0]
        # styles = [1, 1, 1, 1, 1,
        #     1, 1, 1, 1, 1]
        # markers = [-1, -1, -1, -1, -1,
        #     -1, -1, -1, -1, -1]


        # for i in range(2):
        #     if len(labels[i]) == 0:
        #         if (i % 2 == 0):
        #             self.qtgui_time_sink_x_1.set_line_label(i, "Re{{Data {0}}}".format(i/2))
        #         else:
        #             self.qtgui_time_sink_x_1.set_line_label(i, "Im{{Data {0}}}".format(i/2))
        #     else:
        #         self.qtgui_time_sink_x_1.set_line_label(i, labels[i])
        #     self.qtgui_time_sink_x_1.set_line_width(i, widths[i])
        #     self.qtgui_time_sink_x_1.set_line_color(i, colors[i])
        #     self.qtgui_time_sink_x_1.set_line_style(i, styles[i])
        #     self.qtgui_time_sink_x_1.set_line_marker(i, markers[i])
        #     self.qtgui_time_sink_x_1.set_line_alpha(i, alphas[i])

        # self._qtgui_time_sink_x_1_win = sip.wrapinstance(self.qtgui_time_sink_x_1.qwidget(), Qt.QWidget)
        # self.top_layout.addWidget(self._qtgui_time_sink_x_1_win)
        # self.qtgui_time_sink_x_0 = qtgui.time_sink_c(
        #     1024, #size
        #     samp_rate, #samp_rate
        #     "TX Symbol Output", #name
        #     1, #number of inputs
        #     None # parent
        # )

        # self.qtgui_time_sink_x_0.set_update_time(0.10)
        # self.qtgui_time_sink_x_0.set_y_axis(-1, 1)

        # self.qtgui_time_sink_x_0.set_y_label('Amplitude', "")

        # self.qtgui_time_sink_x_0.enable_tags(True)
        # self.qtgui_time_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        # self.qtgui_time_sink_x_0.enable_autoscale(False)
        # self.qtgui_time_sink_x_0.enable_grid(False)
        # self.qtgui_time_sink_x_0.enable_axis_labels(True)
        # self.qtgui_time_sink_x_0.enable_control_panel(False)
        # self.qtgui_time_sink_x_0.enable_stem_plot(False)


        # labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
        #     'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        # widths = [1, 1, 1, 1, 1,
        #     1, 1, 1, 1, 1]
        # colors = ['blue', 'red', 'green', 'black', 'cyan',
        #     'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        # alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
        #     1.0, 1.0, 1.0, 1.0, 1.0]
        # styles = [1, 1, 1, 1, 1,
        #     1, 1, 1, 1, 1]
        # markers = [-1, -1, -1, -1, -1,
        #     -1, -1, -1, -1, -1]


        # for i in range(2):
        #     if len(labels[i]) == 0:
        #         if (i % 2 == 0):
        #             self.qtgui_time_sink_x_0.set_line_label(i, "Re{{Data {0}}}".format(i/2))
        #         else:
        #             self.qtgui_time_sink_x_0.set_line_label(i, "Im{{Data {0}}}".format(i/2))
        #     else:
        #         self.qtgui_time_sink_x_0.set_line_label(i, labels[i])
        #     self.qtgui_time_sink_x_0.set_line_width(i, widths[i])
        #     self.qtgui_time_sink_x_0.set_line_color(i, colors[i])
        #     self.qtgui_time_sink_x_0.set_line_style(i, styles[i])
        #     self.qtgui_time_sink_x_0.set_line_marker(i, markers[i])
        #     self.qtgui_time_sink_x_0.set_line_alpha(i, alphas[i])

        # self._qtgui_time_sink_x_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0.qwidget(), Qt.QWidget)
        # self.top_layout.addWidget(self._qtgui_time_sink_x_0_win)
        
        
        rrc_taps = firdes.root_raised_cosine(
            nfilts, #gain (= nfilts for polyphase)
            nfilts,# sampling rate
            1.0 /float(self.sps), # normalized symbol rate
            0.35, #excess BW
            11 * self.sps * nfilts #num of taps
        )

        self.pfb_clock_sync = digital.pfb_clock_sync_ccf(
            self.sps,
            6.28/100,
            rrc_taps,
            nfilts,
            nfilts/2,
            1.5,
            1
        )

        self.digital_diff_decoder_bb_0 = digital.diff_decoder_bb(
            self.rotational_sym_order, 
        digital.DIFF_DIFFERENTIAL)
        
        self.digital_costas_loop_cc_0 = digital.costas_loop_cc(
            (6.28/100), 
            self.rotational_sym_order, #
            False)
        
        self.digital_constellation_modulator_0 = digital.generic_mod(
            constellation=self.mod_obj,
            differential=True,
            samples_per_symbol=sps,
            pre_diff_code=True,
            excess_bw=0.35,
            verbose=False,
            log=False,
            truncate=False)
        self.digital_constellation_decoder_cb_0 = digital.constellation_decoder_cb(
            self.mod_obj
        )
        self.blocks_vector_sink_x_0 = blocks.vector_sink_b(1, 1024)
        self.throttle = blocks.throttle( gr.sizeof_gr_complex*1, samp_rate, True, 0 if "auto" == "auto" else max( int(float(0.1) * samp_rate) if "auto" == "time" else int(0.1), 1) )
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
        self.unpack = blocks.unpack_k_bits_bb(self.bits_per_symbol) # number is how many bits per symbol

        # constellation plot for demod
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
        
        # TX Chain
        # modulation selector
        self.connect(self.source, self.cmod_0)
        self.connect(self.cmod_0, self.throttle)

        # RX Chain
        self.connect(self.throttle, self.pfb_0, self.cl_0, self.cdecode_0, self.diffd_0, self.unpack_0)
        
        self.connect(self.unpack_0, self.destination)
        self.connect(self.cl_0, self.constellation_plot)



    def _connect_chain(self, mode):
        print(f'[genMDsim] Connecting {self.mod_strs[mode]} chain...', )
        # TX Blocks
        mod = self.cmods[mode]
        # RX Blocks
        pfb = self.pfbs[mode]
        cl = self.costasls[mode]
        decode = self.cdecodes[mode]
        diffd = self.diffds[mode]
        unpack = self.unpacks[mode]

        # TX Chain
        self.connect(self.source, mod, self.throttle)
        # RX Chain
        self.connect(self.throttle, pfb, cl, decode, diffd, unpack, self.destination)
        self.connect(cl, self.constellation_plot)
        print(f'[genMDsim] {self.mod_strs[mode]} Chain connected!')

    def _disconnection_chain(self, mode):
        print(f'[genMDsim] Disconnecting {self.mod_strs[mode]} chain...')
        # TX Block
        mod = self.cmods[mode]
        # RX Blocks
        pfb = self.pfbs[mode]
        cl = self.costasls[mode]
        decode = self.cdecodes[mode]
        diffd = self.diffds[mode]
        unpack = self.unpacks[mode]

        # TX Chain
        self.disconnect(self.source, mod, self.throttle)
        # RX Chain
        self.disconnect(self.throttle, pfb, cl, decode, diffd, unpack, self.destination)
        self.disconnect(cl, self.constellation_plot)
        print(f'[genMDsim] {self.mod_strs[mode]} Chain disconnected!')

    def switch_modulation(self, mode):
        print(f'[genMDsim] Swithcing modulation scheme from {self.mod_strs[self.current_mode]} to {self.mod_strs[mode]}')
        if mode == self.current_mode:
            return

        self.lock()
        self._disconnection_chain(self.current_mode)
        self._connect_chain(mode)
        self.current_mode=mode
        self.unlock()
        print(f"[genMDsim] switched modulation scheme to {self.mod_strs[mode]}")
 

    def getSDRgain(self):
        return self.sdr_RF_gain

    def setSDRGain(self, gain):
        self.sdr_RF_gain = gain
        return self.sdr_RF_gain


    def closeEvent(self, event=None):
        self.settings = Qt.QSettings("GNU Radio", "fil_Transfer_skeleton")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()


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


def cli_loop(sig_handler, tb):
    modes = {'bpsk': 0, 'qpsk':1, '16qam':2}
    while True:
        try:
            cmd = input("\n[CMD] Enter modulation (bpsk/qpsk/16qam) or 'q' to quit: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == 'q':
            sig_handler()
            return
        elif cmd in modes:
            tb.switch_modulation(modes[cmd])
        else:
            print(f"[CMD] Unknown command: {cmd}")
            print(f"Options: {list(modes.keys())}, q")




def main(top_block_cls=fil_Transfer_skeleton, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()
    print(f'[GNURadio Main Thread] Started Thread')

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.source.perm_stop()
        tb.closeEvent()
        tb.wait()
        Qt.QApplication.quit()
        print(f'[GNURadio Main Thread] Stopped Thread')
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    cli_thread = threading.Thread(target=cli_loop, args=(sig_handler,tb,),daemon=True)
    cli_thread.start()

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
