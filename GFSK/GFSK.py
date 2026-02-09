#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: GFSK Modulation
# GNU Radio version: 3.10.11.0

from PyQt5 import Qt
import numpy as np
from gnuradio import qtgui, analog
import pmt
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
from collections import deque
import time

# mavpacket generator
from mavpacket import mavheartbeat, mavreader

class GFSK(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "GFSK Modulation", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("GFSK Modulation")
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

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 64e3
        self.delay = delay = 0
        # mavlink packet in bit array fromat
        self.heartbeat = mavheartbeat()
        # 14 bit sync pattern to line up bit streams from rx to tx
        self.sync_word = "11111101001110"
        self.sync_bits = [int(b) for b in self.sync_word]
        preamble = [0] * 32
        postamble = [0] * 32
        # Need to add pre and post amble as GFSK can chop off bytes from beginning and end
        self.heartbeat_with_sync = preamble + self.sync_bits + self.heartbeat + postamble # add pre and post amble as GFSK cuts off bits

        
        print(f"==============================\nTRANSMITTED DATA\n==============================")
        print(f"Transmitted mavlink heartbeat packet:\n")
        mavreader(np.packbits(self.heartbeat))
        print(f"\nheartbeat bitstream == {self.heartbeat}")
        print(f"preamble + sync word + hearbeart + postabmle bitstream == {self.heartbeat_with_sync}")
        print(f"heartbeat byte stream == {np.packbits(self.heartbeat)}")
        print(f"==============================\n==============================\n")
        ##################################################
        # Blocks
        ##################################################

        self._delay_range = qtgui.Range(0, 100, 1, 0, 200) #2 delay by default
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


        labels = ['TX Signal 1', 'RX Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
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

        # qt sink gui to look at GFSK mod signal
        self.qtgui_sink_x_0 = qtgui.sink_c(
            1024, #fftsize
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            "", #name
            True, #plotfreq
            True, #plotwaterfall
            True, #plottime
            True, #plotconst
            None # parent
        )
        self.qtgui_sink_x_0.set_update_time(1.0/10)
        self._qtgui_sink_x_0_win = sip.wrapinstance(self.qtgui_sink_x_0.qwidget(), Qt.QWidget)

        self.qtgui_sink_x_0.enable_rf_freq(False)

        self.top_layout.addWidget(self._qtgui_sink_x_0_win)

        self.digital_gfsk_mod_0 = digital.gfsk_mod(
            samples_per_symbol=2,
            sensitivity=1.0,
            bt=0.35,
            verbose=False,
            log=False,
            do_unpack=False)
        
        self.digital_gfsk_demod_0 = digital.gfsk_demod(
            samples_per_symbol=2,
            sensitivity=1.0,
            gain_mu=0.3,
            mu=0.5,
            omega_relative_limit=0.005,
            freq_error=0.0,
            verbose=False,
            log=False)
        
        # Automatic Gain Control (AGC) block to mitigate any gain issues
        self.analog_agc_xx_0 = analog.agc_cc(
            1e-4, #attack rate
            1.0, #reference target amplitude
            1.0, #gain initial gain
            65536 #max gain
        )
        
        # Access correlator for frame sync
        self.digital_correlate_access_code_tag_xx_0 = digital.correlate_access_code_tag_bb(
            self.sync_word,
            0,  # threshold (must match exactly)
            "frame_start"  # tag name when sync found
        )

       
        # Source bitstream
        # heartbeat bitstream in list format
        # True False value is conditional decidign whether it should be continuously transmitting or not
        self.blocks_vector_source_x_0 = blocks.vector_source_b(tuple(self.heartbeat_with_sync), True, 1, [])
        
        
        # uchar_to_float of input
        self.blocks_uchar_to_float_0_1 = blocks.uchar_to_float()
        # uchar_to_float for demod output
        self.blocks_uchar_to_float_0 = blocks.uchar_to_float()
       

        # Add a throttle to control flow rate 
        self.blocks_throttle_0 = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate, False)
        self.blocks_stream_to_tagged_stream_0 = blocks.stream_to_tagged_stream(gr.sizeof_char, 1, len(self.heartbeat_with_sync), "packet_len")
        self.blocks_repack_bits_bb_0 = blocks.repack_bits_bb(1, 8, "", False, gr.GR_MSB_FIRST)
        self.blocks_repack_bits_bb_1 = blocks.repack_bits_bb(1, 8, "", False, gr.GR_MSB_FIRST)
        self.blocks_delay_0 = blocks.delay(gr.sizeof_float*1, 0)
        self.blocks_delay_1 = blocks.delay(gr.sizeof_gr_complex*1, delay)

        # Debugging blocks to check mavlink bitstream inputs and outputs
        self.blocks_vector_sink_input = blocks.vector_sink_b()
        self.blocks_vector_sink_output = blocks.vector_sink_b()
        self.blocks_tag_debug_0 = blocks.tag_debug(gr.sizeof_char, "Sync Tags")
        self.blocks_tag_debug_0.set_display(False)
        # repacked bits
        self.blocks_vector_sink_repack = blocks.vector_sink_b()




        ##################################################
        # Connections
        ##################################################


        #########################
        # TX
        #########################
        # vector-source -> stream-to-tagged
        self.connect((self.blocks_vector_source_x_0, 0), (self.blocks_stream_to_tagged_stream_0, 0))
        # stream-to-tagged -> GFSK-mod
        self.connect((self.blocks_stream_to_tagged_stream_0, 0), (self.digital_gfsk_mod_0, 0))
        
        # stream tagged -> uchar-to-float
        self.connect((self.blocks_stream_to_tagged_stream_0, 0), (self.blocks_uchar_to_float_0_1, 0))
        # uchar-to-float -> gui
        self.connect((self.blocks_uchar_to_float_0_1, 0), (self.qtgui_time_sink_x_0, 0))
        

        #########################
        # TX -> RX
        #########################
        # Throttle mod to ensure there is no timining issue
        # GFSK-mod -> throttle
        self.connect((self.digital_gfsk_mod_0, 0), (self.blocks_throttle_0, 0))
        # throttle -> AGC
        self.connect((self.blocks_throttle_0,0),(self.analog_agc_xx_0, 0))
        # AGC -> delay for whole packet to send first
        self.connect((self.analog_agc_xx_0, 0), (self.blocks_delay_1, 0))
        # delay to GFSK-demod
        self.connect((self.blocks_delay_1, 0),(self.digital_gfsk_demod_0, 0))
        # connect qt sink gui to GFSK-mod to visually see signal output
        self.connect((self.digital_gfsk_mod_0, 0),(self.qtgui_sink_x_0, 0))



        #########################
        # RX
        #########################
        # GFSK-demod -> correlator
        self.connect((self.digital_gfsk_demod_0,0), (self.digital_correlate_access_code_tag_xx_0, 0))
        # correlator -> uchar-to-float
        self.connect((self.digital_correlate_access_code_tag_xx_0, 0), (self.blocks_uchar_to_float_0, 0))
        # uchar-to-float -> gui
        self.connect((self.blocks_uchar_to_float_0, 0),(self.qtgui_time_sink_x_0, 1))

        # Get bitstream, repack it and change it into mavlink msg
        # correlator -> repack
        self.connect((self.digital_correlate_access_code_tag_xx_0, 0), (self.blocks_repack_bits_bb_0, 0))
        self.connect((self.blocks_repack_bits_bb_0, 0),(self.blocks_vector_sink_repack, 0))
        
        
        # connect debug blocks to check bistream otuput
        self.connect((self.blocks_vector_source_x_0, 0), (self.blocks_vector_sink_input, 0))
        self.connect((self.digital_correlate_access_code_tag_xx_0, 0), (self.blocks_vector_sink_output, 0))
        self.connect((self.digital_correlate_access_code_tag_xx_0, 0), (self.blocks_tag_debug_0, 0))

        
        
    def get_mav(self):
        rx_data = list(self.blocks_vector_sink_output.data())
        print(f"==============================\nRECEIVED DATA\n==============================")
        print(f"Received raw unsliced bit stream: {rx_data}")

        # tags = self.blocks_vector_sink_output.tags()
        # print(f"all bits from rx_data: {rx_data}")
        # for t in tags:
        #     print(
        #         "offset:", t.offset,
        #         "key:", pmt.symbol_to_string(t.key),
        #         "value:", pmt.to_python(t.value)
        #     )
        # This parses tags that correlator may have appended to stream into vector_sink_output
        # We correlator will have put the amount of bits before the payload as named as "frame_start" based on its
        # access code or 'self.sync_word'
        frame_tag = next(
            (t for t in self.blocks_vector_sink_output.tags()
            if pmt.symbol_to_string(t.key) == "frame_start"),
            None
        )
        if frame_tag is None:
            return

        sync_start = frame_tag.offset
        payload_start = sync_start
        payload_end = payload_start + len(self.heartbeat)

        payload_bits = rx_data[payload_start:payload_end]
        payload = np.packbits(payload_bits)
        print(f"received payload = {payload}")
        print(f"\nReceived bitstream converted to mavlink message:\n")
        mavreader(payload)
        print(f"\n==============================\n==============================\n")
   


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "GFSK")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.qtgui_time_sink_x_0.set_samp_rate(self.samp_rate)

    def get_delay(self):
        return self.delay

    def set_delay(self, delay):
        self.delay = delay
        self.blocks_delay_0.set_dly(int(self.delay))




def main(top_block_cls=GFSK, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    
    tb.start()
    tb.flowgraph_started.set()

    tb.show()
    tb.get_mav()


    timer1 = Qt.QTimer()
    timer1.start(3000)
    # timer1.timeout.connect(tb.check_data)
    timer1.timeout.connect(tb.get_mav)
    
    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)
    # tb.stop() #stop flowgraph after one pass imediately
    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()


# funciton to check input and output bitsreams:
    # def check_data(self):
    #     tx_data = list(self.blocks_vector_sink_input.data())
    #     rx_data = list(self.blocks_vector_sink_output.data())
        
    #     # Get the tags to see if sync word was found
    #     # We need to access tags differently
        
    #     sync_len = len(self.sync_word)
    #     packet_len = len(self.heartbeat)
        
    #     print(f"\n=== Debug Info ===")
    #     print(f"TX total length: {len(tx_data)}")
    #     print(f"RX total length: {len(rx_data)}")
    #     print(f"Sync word: {self.sync_word}")
    #     print(f"Expected packet length: {packet_len}")
        
    #     # Check if RX data starts with sync word (it shouldn't if correlator worked)
    #     if len(rx_data) >= sync_len:
    #         rx_start = ''.join(str(b) for b in rx_data[:sync_len])
    #         print(f"RX first {sync_len} bits: {rx_start}")
    #         print(f"Does RX start with sync? {rx_start == self.sync_word}")
            
    #         # If RX still has sync word, correlator didn't strip it!
    #         if rx_start == self.sync_word:
    #             print("WARNING: Sync word still in RX data - correlator may not be working!")
        
    #     # Try different offsets to find alignment
    #     print(f"\n=== Trying different offsets ===")
    #     tx_payload = tx_data[sync_len:sync_len + packet_len]
        
    #     best_ber = 1.0
    #     best_offset = 0
        
    #     for offset in range(0, min(50, len(rx_data) - packet_len)):
    #         rx_test = rx_data[offset:offset + packet_len]
    #         errors = sum(a != b for a, b in zip(tx_payload, rx_test))
    #         ber = errors / packet_len
            
    #         if ber < best_ber:
    #             best_ber = ber
    #             best_offset = offset
        
    #     print(f"Best BER: {best_ber:.6f} at offset {best_offset}")
        
    #     # Show comparison at best offset
    #     rx_payload = rx_data[best_offset:best_offset + packet_len]
        
    #     print(f"\n=== Comparison at offset {best_offset} ===")
    #     print(f"TX (first 50): {tx_payload[:50]}")
    #     print(f"RX (first 50): {rx_payload[:50]}")
    #     print(f"TX (last 20):  {tx_payload[-20:]}")
    #     print(f"RX (last 20):  {rx_payload[-20:]}")
        
    #     errors = sum(a != b for a, b in zip(tx_payload, rx_payload))
    #     print(f"\nBit errors: {errors}/{packet_len}")
    #     print(f"BER: {best_ber:.6f}")