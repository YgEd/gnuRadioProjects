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
from mavGNURXBlock import mav_packet_reader_with_metrics
from rf_metrics import RFMetricsProbe, MetricsLogger


class flow_graph(gr.top_block, Qt.QWidget):
    def __init__(self):
        gr.top_block.__init__(self)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Ouput RX bandpass")
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

        # Need rational resampler to upsample to SDR smaple rate for 100e3 samp rate a 20/1 ratio is good (20 interpolation to 1 decimation)
        # So on TX end need resampler with 20 interp 1 dec 
        # On RX end need resampler with 20 dec 1 interp
        # samp_rate determines DSP chain sampling rate
        self.samp_rate = samp_rate = 100e3
        self.sdr_samp_rate = 2e6

        # symbol rate = samp_rate / samples per symbol
        
        self.center_freq = 915e6 #915 MHz
        self.samples_per_symbol = 4

        # potentially plan for a 10mW output

        # sensitivity is the frequency deviation factor. It controls the how far the carrier frequency shifts when you send a 1 vs 0 symbol
        # sensitivity = (pi x h) /samples_per_symbol
        # h = (2 x delta f) / symbol_rate
        # h is you modulation index
        # sensitivity of 1 means you have a modulation index of 1.27 which is very high
        # Typically you'd want a modulation index of about 0.5 which measn you'd hae a sensitivity of around 0.39
        self.sensitivity = 0.4

        # bt is the bandwidth time product, controls the gaussian pulse-shaping filter that smooths frequency transitions. 
        # Low value like 0.3 induces heavy smoothing, gradual transitions leading to narrowest bandwidth occupancy but introduces more intersymbol interference
        # High value like 1 induces minimal smoothing, sharp transitions leading to wide bandwidth occupancy with little intersymbol interference
        self.bt = 0.35

        # gain_mu is timing recovery loop gain, how aggresively the clock recovery algorithm corrects its estimate. 
        # Higher gain_mu (0.3-0.5) -> faster lock, tracks rapid timing changes, jittery
        # Lower gain_mu (0.05-0.1) -> slower lock, smoother, more stable once locked
        # kind of like PID tuning?
        self.gain_mu = 0.25

        # mu is hte initial fractional symbol timing offset estimate, where within a symbol period the demod starts sampling. Ranges from 0 to 1.
        # 0.5 means you start sampling within the middle of a period
        self.mu = 0.5

        # omega relative limit establishes the allowed clock rate mismatch between TX and RX. This threshold prevents the timing recovery loop from going unstable
        # same reference (simulation or share clock) 0.005 is fine
        # Independent clocks (quality SDRs OTA) 0.01-0.02 is good to tolerate real oscillator drift
        # Cheap SDRs may need as large as 0.05
        self.omega_relative_limit = 0.02

        # used to compensate for a consistent freqeuncy offset from tx and rx. Leave as 0 and adjust as needed or progromatically adjust with other blocks such as (Frequency Xlating FIR Filter)
        self.freq_error = 0.0

        # RX interpolation and decimation ratio for rational resampler
        self.rx_interpolation=1
        self.rx_decimation=20
        self.fractional_bw=0.4
        self.tx_gain_scalar=1

        # SDR RF gain
        self.sdr_RF_gain = 30

    

        #######################
        # Blocks
        #######################


       

        # resampler and lowpass all in one
        self.rx_resampler_lowpass = filter.freq_xlating_fir_filter_ccf(
            decimation=self.rx_decimation,
            taps=firdes.low_pass(
                gain=1,
                sampling_freq=self.sdr_samp_rate,
                # to determine cutoff freq you need to compute occupied bandwidth:
                # occupied_bandwidth ~ symbol_rate * (1 + BT) in this case our with a samp_rate of 100e3 and samples per symbol at 4 and BT at 0.35 we get
                # occupied_bandwidth ~ 100e3/4 * (1 + 0.35)
                # Then to get cutoff freq we calculate
                # cutoff_freq = occupied_bandwidth * 0.75 to 1 (this 0.75 to 1 scalar is a trade off between (next two lines))
                # 0.75 -> tighter, less noise, but sensitive to freq offset
                # 1 -> more open, tolerant to some offset
                # If you have a high modulation index (h value) you may want ot expand your cut_off frequency as there will be a bigger freq gap between 1 and 0 signals
                # You can expand up until samp_rate/2 because that is the nyquist limit so in this case cutoff_freq can be at most 50 Khz
                cutoff_freq=(self.samp_rate/self.samples_per_symbol) * (1 + self.bt) * 1,
                # transition width rule of thumb is
                # transition width ~ cutoff_freq * 0.25
                transition_width=(self.samp_rate/self.samples_per_symbol) * (1 + self.bt) * 1 * 0.25
            ),
            center_freq=0, #leave at 0 by default this is a constant offset if you know consistent tx rx freq offset
            sampling_freq=self.sdr_samp_rate
        )
        

        # agc for rx
        # self.agc = analog.agc_cc(
        #     rate=1e-4,       # adaptation rate — how fast AGC adjusts Higher values (1e-2 to 1e-3) get fast adaption but fluctuates iwth noise. Lower values (1e-4 to 1e-5) slow adaption but stable gain during packet
        #     reference=1.0,   # target output amplitude
        #     gain=1.0         # initial gain estimate
        # )

        # AGC2 is better as it has two different rates for when a strong signal appears you want to clamp down on it quick vs a sustained signal you want to back off on the rate
        self.agc = analog.agc2_cc(
            attack_rate=1e-2,   # how fast gain DECREASES (strong signal arrives)
            decay_rate=1e-4,    # how fast gain INCREASES (signal weakens/disappears)
            reference=1.0,
            gain=1.0
        )

        
        # gfsk demod
        self.gfsk_demod = digital.gfsk_demod(
            samples_per_symbol=self.samples_per_symbol,
            sensitivity=self.sensitivity,
            gain_mu=self.gain_mu,
            mu=self.mu,
            omega_relative_limit=self.omega_relative_limit,
            freq_error=self.freq_error,
            verbose=False,
            log=False)
        
        # osmos bladerf souce block
        self.osmosdr_source = osmosdr.source(
            args="numchan=" + str(1) + " " + "bladeRF=0"
        )
        self.osmosdr_source.set_time_unknown_pps(osmosdr.time_spec_t())
        self.osmosdr_source.set_sample_rate(self.sdr_samp_rate)
        self.osmosdr_source.set_center_freq(self.center_freq, 0)
        self.osmosdr_source.set_antenna('RX1', 0) # set RX antenna earlier to ensure gain is applied to correct port
        self.osmosdr_source.set_freq_corr(0, 0)
        self.osmosdr_source.set_dc_offset_mode(0, 0)
        self.osmosdr_source.set_iq_balance_mode(0, 0)
        self.osmosdr_source.set_gain_mode(False, 0)
        self.osmosdr_source.set_gain(self.sdr_RF_gain, 0)
        self.osmosdr_source.set_if_gain(10, 0)
        self.osmosdr_source.set_bb_gain(16, 0)
        self.osmosdr_source.set_bandwidth(0,0)

        # self.qt_freq_sink = qtgui.freq_sink_c(
        #     1024,                # FFT size
        #     5,  # window type 5 == blackman harris
        #     self.center_freq,    # center freq for display
        #     self.sdr_samp_rate,  # bandwidth (use post-resampler rate)
        #     "RX Monitor"
        # )

        # self.qt_time_sink = qtgui.time_sink_c(
        #     1024,               # number of points
        #     self.samp_rate, # sample rate
        #     "RX Time Domain"
        # )

        # Adding gui sinks to gui

        # Create the FFT sink
        self.fft_sink = qtgui.freq_sink_c(
            1024,              # FFT size
            5,  # Window type
            self.center_freq,       # Center frequency (e.g., 915e6)
            self.sdr_samp_rate,         # Sample rate at this point in the chain
            "OTA Signal"       # Label
        )
        self.fft_sink.set_update_time(0.10)  # Update every 100ms
        self.fft_sink.set_y_axis(-140, 10)  # dB range

        # Get the Qt widget (needed to actually display it)
        self.fft_win = sip.wrapinstance(self.fft_sink.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self.fft_win, 0, 0, 1, 1)
    
        self.metrics_logger = MetricsLogger()
        self.metrics_logger.start()
        self.metrics_probe = RFMetricsProbe(
            samp_rate=self.samp_rate,          # 100e3 — post-resampler rate
            samples_per_symbol=self.samples_per_symbol,
            center_freq=self.center_freq,
            logger=self.metrics_logger,
            sensitivity=self.sensitivity
        )
       
        # Main source block, gcs host specified should be the entire subnet (ending in 255 usually for /24)
        self.destination = mav_packet_reader_with_metrics(self.center_freq, metrics_logger=self.metrics_logger, publish_to_gcs=True, host="192.168.0.255", port=8080)

        ##########################
        # Connections
        #########################

        # gui snks
        # self.connect(self.rx_resampler_lowpass, self.qt_freq_sink)
        # self.connect(self.rx_resampler_lowpass, self.qt_time_sink)
        self.connect(self.osmosdr_source, self.fft_sink)

        self.connect(self.osmosdr_source, self.rx_resampler_lowpass)
        self.connect(self.rx_resampler_lowpass, self.agc)
        self.connect(self.agc, self.gfsk_demod)
        self.connect(self.rx_resampler_lowpass, self.metrics_probe)
        # self.connect(self.gfsk_demod, self.destination)

        self.connect(self.gfsk_demod, self.destination)

    def closeEvent(self, event):
        """Handle window close button — same cleanup as SIGINT."""
        self._safe_shutdown()
        event.accept()

    def _safe_shutdown(self):
        """Zero RF gains and stop the flow graph cleanly."""
        print("Shutting down BladeRF TX...")
        try:
            # Kill RF output before stopping the scheduler
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
    app = Qt.QApplication(sys.argv)

    tb = flow_graph()
    tb.show()
    app.processEvents()

    def sig_handler(sig=None, frame=None):
        print("\nCaught SIGINT, shutting down...")
        tb._safe_shutdown()
        Qt.QApplication.quit()


    def noop():
        pass

    signal.signal(signal.SIGINT, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(noop)

    tb.start()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        sig_handler()
        


