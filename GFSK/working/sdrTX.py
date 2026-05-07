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
     
        self.symbol_rate = 50000
        self.samp_rate = 100e3
        self.sdr_samp_rate = 2e6
        self.sps = sps = int(self.samp_rate / self.symbol_rate)
        self.center_freq = 915e6

        self.sensitivity = 0.785   # h=0.5
        self.bt = 0.5
        self.gain_mu = 0.08
        self.mu = 0.5
        self.omega_relative_limit = 0.02
        self.freq_error = 0.0
        self.tx_interpolation = 20
        self.tx_decimation = 1
        self.fractional_bw = 0.49
        self.tx_gain_scalar = 0.5
        self.sdr_RF_gain = 10

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
        # txgain selections based on modulation scheme as peak-to-average (PAPR) increases with modulation scheme do to RRC pulse shaping
        self.gain_scale = [0.5, 0.35, 0.30]
        



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

#       #################################################
        # DSP Blocks
        # global rrc_taps block
    
        self.tx_amplitude = blocks.multiply_const_cc(self.tx_gain_scalar)
        

        ##################################################
        # Metrics Blocks
        # TX resampler: 100 kHz → 2 MHz
        self.tx_resampler = filter.rational_resampler_ccf(
            interpolation=self.tx_interpolation,
            decimation=self.tx_decimation,
            taps=[],
            fractional_bw=self.fractional_bw
        )

        ##################################################
        # Metrics Blocks

        self.metrics_logger = MetricsLogger(getGain=self.getSDRgain, getMod=self.getMod, direction='TX')
        self.metrics_logger.start()
        self.metrics_probe = RFMetricsProbe(
            samp_rate=self.sdr_samp_rate,
            samples_per_symbol=self.sps,
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

        
        ##################################################
        # osmosdr bladeRF block
        self.osmosdr_sink = osmosdr.sink(
            args="numchan=1 bladeRF=0"
        )
        self.osmosdr_sink.set_sample_rate(self.sdr_samp_rate)
        self.osmosdr_sink.set_center_freq(self.center_freq, 0)
        self.osmosdr_sink.set_antenna('TX1', 0)
        self.osmosdr_sink.set_freq_corr(0, 0)
        self.osmosdr_sink.set_gain(self.sdr_RF_gain, 0)
        self.osmosdr_sink.set_if_gain(10, 0)
        self.osmosdr_sink.set_bb_gain(10, 0)
        self.osmosdr_sink.set_bandwidth(0, 0)

        ##################################################
        # GUIS
        self.qtguis = qtGUIs(self.top_layout, self.top_grid_layout, self.symbol_rate)
        # self.time_sink = self.qtguis.getTimeSink("TX Output")
        self.freq_sink = self.qtguis.getFreqSink(self.center_freq, self.sdr_samp_rate, "TX Output")

        ##################################################
        # Connections
        ##################################################
        
        # TX Chain
        # modulation selector
        self.connect(self.source, self.cmod_0)
        self.connect(self.cmod_0, self.tx_resampler)
        self.connect(self.tx_resampler, self.tx_amplitude, self.osmosdr_sink)
        self.connect(self.tx_amplitude, self.metrics_probe)
        self.connect(self.tx_amplitude, self.freq_sink)


        
    def _connect_chain(self, mode):
        print(f'[genMDsim] Connecting {self.mod_strs[mode]} chain...', )
        # TX Blocks
        mod = self.cmods[mode]
        self.tx_amplitude.set_k(self.gain_scale[mode])

        # TX Chain
        self.connect(self.source, mod, self.tx_resampler, self.tx_amplitude, self.osmosdr_sink)
        self.connect(self.tx_amplitude, self.metrics_probe)
        self.connect(self.tx_amplitude, self.freq_sink)
        print(f'[genMDsim] {self.mod_strs[mode]} Chain connected!')

    def _disconnection_chain(self, mode):
        print(f'[genMDsim] Disconnecting {self.mod_strs[mode]} chain...')
        # TX Block
        mod = self.cmods[mode]

        # TX Chain
        self.disconnect(self.source, mod, self.tx_resampler, self.tx_amplitude, self.osmosdr_sink)
        self.disconnect(self.tx_amplitude, self.metrics_probe)
        self.disconnect(self.tx_amplitude, self.freq_sink)
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
 
    def getMod(self):
        return self.mod_strs[self.current_mode]

    def getSDRgain(self):
        return self.sdr_RF_gain

    def setSDRGain(self, gain):
        self.sdr_RF_gain = gain
        return self.sdr_RF_gain

    def closeEvent(self, event=None):
        print("Shutting down BladeRF TX...")
        try:
            self.osmosdr_sink.set_gain(0, 0)
            self.osmosdr_sink.set_if_gain(0, 0)
            self.osmosdr_sink.set_bb_gain(0, 0)
            self.osmosdr_sink.set_center_freq(2.4e9, 0)
        except Exception as e:
            print(f"Warning: could not zero gains: {e}", file=sys.stderr)
        self.settings = Qt.QSettings("GNU Radio", "modSwitcher")
        self.metrics_logger.close()
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