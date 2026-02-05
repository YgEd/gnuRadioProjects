from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import QtCore
from gnuradio import blocks
from gnuradio import digital
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
import sip
import threading

# mavpacket generator
from mavpacket import mavheartbeat

class GFSK(gr.top_block, Qt.QWidget):
    def guisetup(self):
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

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "GFSK")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)
        self.flowgraph_started = threading.Event()

    def __init__(self):
        gr.top_block.__init__(self, "GFSK Modulation", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("GFSK Modulation")
        self.guisetup()
        

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 32e3
        self.delay = delay = 0
        # mavlink packet in bit array fromat
        self.heartbeat = mavheartbeat()
        # 14 bit sync pattern to line up bit streams from rx to tx
        # self.sync_word = "11111101001110"
        # self.sync_bits = [int(b) for b in self.sync_word]
        # self.heartbeat_with_sync = self.sync_bits + self.heartbeat
        self.heartbeat_with_sync = self.heartbeat
        
        ##################################################
        # Blocks
        ##################################################

        self._delay_range = qtgui.Range(0, 100, 1, 0, 200)
        self._delay_win = qtgui.RangeWidget(self._delay_range, self.set_delay, "Delay", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._delay_win)
        self.qtgui_time_sink_x_0 = qtgui.time_sink_f(
            100, #size
            samp_rate, #samp_rate
            "", #name
            2, #number of inputs
            None # parent
        )
        self.qtgui_time_sink_x_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_0.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0.enable_tags(False)
        self.qtgui_time_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_0.enable_autoscale(True)
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
                self.qtgui_time_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_time_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_time_sink_x_0_win)

        self.digital_gfsk_mod_0 = digital.gfsk_mod(
            samples_per_symbol=2,
            sensitivity=1.0,
            bt=0.35,
            verbose=True,
            log=False,
            do_unpack=True)
        
        self.digital_gfsk_demod_0 = digital.gfsk_demod(
            samples_per_symbol=2,
            sensitivity=1.0,
            gain_mu=0.3,
            mu=0.5,
            omega_relative_limit=0.005,
            freq_error=0.0,
            verbose=True,
            log=False)
        
        # Access correlator for frame sync
        self.digital_correlate_access_code_tag_xx_0 = digital.correlate_access_code_tag_bb(
            self.sync_word,
            0,  # threshold (must match exactly)
            "frame_start"  # tag name when sync found
        )


        # Source bitstream
        # heartbeat bitstream in list format
        self.blocks_vector_source_x_0 = blocks.vector_source_b(tuple(self.heartbeat_with_sync), True, 1, [])
        
        
        # uchar_to_float of input
        self.blocks_uchar_to_float_0_1 = blocks.uchar_to_float()
        # uchar_to_float for demod output
        self.blocks_uchar_to_float_0 = blocks.uchar_to_float()

        # Add a throttle to control flow rate 
        self.blocks_throttle_0 = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate, True)
        self.blocks_stream_to_tagged_stream_0 = blocks.stream_to_tagged_stream(gr.sizeof_char, 1, len(self.heartbeat_with_sync), "packet_len")
        self.blocks_repack_bits_bb_0 = blocks.repack_bits_bb(1, 8, "", False, gr.GR_MSB_FIRST)
        self.blocks_repack_bits_bb_1 = blocks.repack_bits_bb(1, 8, "", False, gr.GR_MSB_FIRST)
        self.blocks_delay_0 = blocks.delay(gr.sizeof_float*1, delay)

        # Debugging blocks to check mavlink bitstream inputs and outputs
        self.blocks_vector_sink_input = blocks.vector_sink_b()
        self.blocks_vector_sink_output = blocks.vector_sink_b()
        self.blocks_tag_debug_0 = blocks.tag_debug(gr.sizeof_char, "Sync Tags")
        self.blocks_tag_debug_0.set_display(False)


        print(f"\n=== Block Output Signatures ===")
        print(f"GFSK Mod output: {self.digital_gfsk_mod_0.output_signature()}")
        print(f"GFSK Demod input: {self.digital_gfsk_demod_0.input_signature()}")
        print(f"GFSK Demod output: {self.digital_gfsk_demod_0.output_signature()}")


        ##################################################
        # Connections
        ##################################################
        # delay -> gui
        self.connect((self.blocks_delay_0, 0), (self.qtgui_time_sink_x_0, 1))

        # repack -> uchar-to-float
        self.connect((self.blocks_repack_bits_bb_0, 0), (self.blocks_uchar_to_float_0, 0))



        #########################
        # TX
        #########################
        # vector-source -> stream-to-tagged
        self.connect((self.blocks_vector_source_x_0, 0), (self.blocks_stream_to_tagged_stream_0, 0))
        # stream-to-tagged -> GFSK-mod
        self.connect((self.blocks_stream_to_tagged_stream_0, 0), (self.digital_gfsk_mod_0, 0))
        
        # stream tagged -> uchar-to-float
        self.connect((self.blocks_stream_to_tagged_stream_0, 0), (self.blocks_uchar_to_float_0_1, 0))
        # uchar-to-float -> delay
        self.connect((self.blocks_uchar_to_float_0_1, 0), (self.blocks_delay_0, 0))
        # uchar-to-float -> gui
        self.connect((self.blocks_uchar_to_float_0, 0), (self.qtgui_time_sink_x_0, 0))

        #########################
        # TX -> RX
        #########################
        # Throttle mod to ensure there is no timining issue
        # GFSK-mod -> throttle
        self.connect((self.digital_gfsk_mod_0, 0), (self.blocks_throttle_0, 0))

        # throttle -> GFSK-demod
        self.connect((self.blocks_throttle_0, 0), (self.digital_gfsk_demod_0, 0))



        #########################
        # RX
        #########################
        # GFSK-demod -> correlator
        self.connect((self.digital_gfsk_demod_0,0), (self.digital_correlate_access_code_tag_xx_0, 0))
        # correlator -> repack
        self.connect((self.digital_correlate_access_code_tag_xx_0, 0), (self.blocks_repack_bits_bb_0, 0))

        # connect debug blocks to check bistream otuput
        self.connect((self.blocks_vector_source_x_0, 0), (self.blocks_vector_sink_input, 0))
        self.connect((self.digital_correlate_access_code_tag_xx_0, 0), (self.blocks_vector_sink_output, 0))
        self.connect((self.digital_correlate_access_code_tag_xx_0, 0), (self.blocks_tag_debug_0, 0))
        
        
        # repacking at input for comparison
        # # stream-to-tagged -> repack
        # self.connect((self.blocks_stream_to_tagged_stream_0, 0), (self.blocks_repack_bits_bb_1, 0))

        # # repack to uchar-to-float
        # self.connect((self.blocks_repack_bits_bb_1, 0), (self.blocks_uchar_to_float_0_1, 0))
