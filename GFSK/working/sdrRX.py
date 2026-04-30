from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio import blocks
from gnuradio import digital
from gnuradio import filter
from gnuradio import analog
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
import osmosdr

# Created Custom Python blocks
from genTXBlock import mav_packet_source
from rf_metrics import RFMetricsProbe, MetricsLogger
from rxBlock import mav_packet_reader_with_metrics
from guiBlock import qtGUIs



class modSwitcher(gr.top_block, Qt.QWidget):

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

        self.settings = Qt.QSettings("GNU Radio", "modSwitcher")

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
        self.fractional_bw = 0.49
        self.sdr_RF_gain = 35
        self.rx_decimation = 20

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
        # RX Demod chains
        ##################################################

        # filter bandwidths
        self.filter_bw = (2*np.pi)/100

        # Polyphase Clock Sync blocks
        self.pfb_0 = self.pfb_clock_sync = digital.pfb_clock_sync_ccf(self.sps, self.filter_bw, rrc_taps, nfilts, nfilts/2, 1.5, 1)
        self.pfb_1 = self.pfb_clock_sync = digital.pfb_clock_sync_ccf(self.sps, self.filter_bw, rrc_taps, nfilts, nfilts/2, 1.5, 1)
        self.pfb_2 = self.pfb_clock_sync = digital.pfb_clock_sync_ccf(self.sps, self.filter_bw, rrc_taps, nfilts, nfilts/2, 1.5, 1)

        # Differential Decoder blocks
        self.diffd_0 = digital.diff_decoder_bb(self.rso_arr[0], digital.DIFF_DIFFERENTIAL)
        self.diffd_1 = digital.diff_decoder_bb(self.rso_arr[1], digital.DIFF_DIFFERENTIAL)
        self.diffd_2 = digital.diff_decoder_bb(self.rso_arr[2], digital.DIFF_DIFFERENTIAL)

        # Costas loop blocks
        self.cl_0 = digital.costas_loop_cc((self.filter_bw), self.rso_arr[0], False)
        self.cl_1 = digital.costas_loop_cc((self.filter_bw), self.rso_arr[1], False)
        self.cl_2 = digital.costas_loop_cc((self.filter_bw), self.rso_arr[2], False)

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
        # DSP BLocks
        # RX resampler: 2MHz MHz → 100 kHz
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

        self.agc = analog.agc2_cc(
            attack_rate=1e-2,
            decay_rate=1e-3,
            reference=1.0,
            gain=1.0
        )

        # Strips off local oscillator frequency offset
        self.fll = digital.fll_band_edge_cc(
            self.samples_per_symbol,  # sps at FLL input
            0.35,                      # rolloff (must match your TX RRC alpha)
            44,                        # filter size
            (2 * 3.14159 / 100)        # loop BW
        )

        # Blocks DC spike from bladeRF
        self.dc_blocker = filter.dc_blocker_cc(64, True)

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
        self.destination = mav_packet_reader_with_metrics(
            self.center_freq,
            setSDRGain=self.setSDRGain,
            metrics_logger=self.metrics_logger,
            publish_to_gcs=True,
            host="192.168.0.255",
            port=8080
        )

        
        ##################################################
        # osmosdr bladeRF block
        self.osmosdr_source = osmosdr.source(
            args="numchan=1 bladeRF=0"
        )
        self.osmosdr_source.set_time_unknown_pps(osmosdr.time_spec_t())
        self.osmosdr_source.set_sample_rate(self.sdr_samp_rate)
        self.osmosdr_source.set_center_freq(self.center_freq, 0)
        self.osmosdr_source.set_antenna('RX1', 0)
        self.osmosdr_source.set_freq_corr(0, 0)
        self.osmosdr_source.set_dc_offset_mode(0, 0) # automatic DC correction
        self.osmosdr_source.set_iq_balance_mode(0, 0) # automatic
        self.osmosdr_source.set_gain(self.sdr_RF_gain, 0)
        self.osmosdr_source.set_if_gain(15, 0)
        self.osmosdr_source.set_bb_gain(15, 0)
        self.osmosdr_source.set_bandwidth(0, 0)

        ##################################################
        # GUIS
        self.qtguis = qtGUIs(self.top_layout, self.top_grid_layout, self.symbol_rate)
        # self.time_sink = self.qtguis.getTimeSink("TX Output")
        self.freq_sink = self.qtguis.getFreqSink(self.center_freq, self.sdr_samp_rate, "RX Input")
        self.constellation_plot = self.qtguis.getConstellation("RX Recovered Symbols")
        self.filtered_plot = self.qtguis.getConstellation("Filtered Symbols")

        ##################################################
        # Connections
        ##################################################
        
        # RX Chain
  
        # Main rx chain 
        self.connect(self.osmosdr_source, self.rx_resampler_lowpass, self.agc, self.fll, self.pfb_0, self.cl_0, self.cdecode_0, self.diffd_0, self.unpack_0, self.destination)
        self.connect(self.rx_resampler_lowpass, self.metrics_probe)
        self.connect(self.rx_resampler_lowpass, self.freq_sink)
        self.connect(self.rx_resampler_lowpass, self.constellation_plot)
        self.connect(self.cl_0, self.filtered_plot)


        
    def _connect_chain(self, mode):
        print(f'[genMDsim] Connecting {self.mod_strs[mode]} chain...', )
        # RX Blocks
        pfb = self.pfbs[mode]
        cl = self.costasls[mode]
        decode = self.cdecodes[mode]
        diffd = self.diffds[mode]
        unpack = self.unpacks[mode]


        # RX Chain removed
        self.connect(self.osmosdr_source, self.rx_resampler_lowpass, self.agc, self.fll, pfb, cl, decode, diffd, unpack, self.destination)
        self.connect(self.rx_resampler_lowpass, self.metrics_probe)
        self.connect(self.rx_resampler_lowpass, self.freq_sink)
        self.connect(self.rx_resampler_lowpass, self.constellation_plot)
        self.connect(cl, self.filtered_plot)
        print(f'[genMDsim] {self.mod_strs[mode]} Chain connected!')

    def _disconnection_chain(self, mode):
        print(f'[genMDsim] Disconnecting {self.mod_strs[mode]} chain...')
        # RX Blocks
        pfb = self.pfbs[mode]
        cl = self.costasls[mode]
        decode = self.cdecodes[mode]
        diffd = self.diffds[mode]
        unpack = self.unpacks[mode]


        # RX Chain
        self.disconnect(self.osmosdr_source, self.rx_resampler_lowpass, self.agc, self.fll, pfb, cl, decode, diffd, unpack, self.destination)
        self.disconnect(self.rx_resampler_lowpass, self.metrics_probe)
        self.disconnect(self.rx_resampler_lowpass, self.freq_sink)
        self.disconnect(self.rx_resampler_lowpass, self.constellation_plot)
        self.disconnect(cl, self.filtered_plot)
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
        print("Shutting down BladeRF TX...")
        try:
            self.osmosdr_source.set_gain(0, 0)
            self.osmosdr_source.set_if_gain(0, 0)
            self.osmosdr_source.set_bb_gain(0, 0)
            self.osmosdr_source.set_center_freq(2.4e9, 0)
        except Exception as e:
            print(f"Warning: could not zero gains: {e}", file=sys.stderr)
        self.settings = Qt.QSettings("GNU Radio", "modSwitcher")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()


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




def main(top_block_cls=modSwitcher, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()
    print(f'[GNURadio Main Thread] Started Thread')

    tb.show()

    def sig_handler(sig=None, frame=None):
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