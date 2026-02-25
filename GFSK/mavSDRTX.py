from gnuradio import gr, blocks, digital, filter, analog, qtgui
from gnuradio.filter import firdes
from PyQt5 import Qt
from mavGNUBlock import mav_packet_reader, mav_packet_source
import threading
from pymavlink.dialects.v20 import common as mavlink2
import osmosdr
import sys
import sip
import signal
import time

class flow_graph(gr.top_block,Qt.QWidget):
    def __init__(self):
        gr.top_block.__init__(self)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Ouput TX bassaband")
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
        self.gain_mu = 0.175 

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

        # TX interpolation and decimation amounts for rational resampler ratio
        self.tx_interpolation=20
        self.tx_decimation=1
        
        # RX interpolation and decimation amounts for rational resampler ratio
        self.rx_interpolation=1
        self.rx_decimation=20
        # Fractional bw for TX rational resampler
        self.fractional_bw=0.4
        # TX gain scalar constant
        self.tx_gain_scalar=1

        ##########################
        # BladeRF source variables
        ##########################

        self.sdr_RF_gain = 35       

    

        #######################
        # Blocks
        #######################

        self.source = mav_packet_source()
                        
        # upsample from 100e3 to 2e6 or (2Mhz sampling rate limit for bladerf is 61.44Mhz)
        self.tx_resampler = filter.rational_resampler_ccf(
            interpolation=self.tx_interpolation,
            decimation=self.tx_decimation,
            taps=[],
            fractional_bw=self.fractional_bw
        )
        
        # how much you are scaling power form sdr 0.25 is good for bench testing, 0.5 half power good more realistic testing, 1 is max power may cause clipping
        self.tx_gain = blocks.multiply_const_cc(self.tx_gain_scalar) 

   

        # gfsk mod
        self.gfsk_mod = digital.gfsk_mod(
            samples_per_symbol=self.samples_per_symbol,
            sensitivity=self.sensitivity,
            bt=self.bt,
            verbose=False,
            log=False,
            do_unpack=False)
        
        # osmos sink
        self.osmosdr_sink = osmosdr.sink(
            args="numchan=" + str(1) + " " + "bladeRF=0"
        )
        self.osmosdr_sink.set_sample_rate(self.sdr_samp_rate)
        self.osmosdr_sink.set_center_freq(self.center_freq, 0)
        self.osmosdr_sink.set_antenna('TX1', 0) # set antenna earlier so that the gain settings apply to that antenna
        self.osmosdr_sink.set_freq_corr(0, 0)
        self.osmosdr_sink.set_gain(self.sdr_RF_gain, 0)        # TX gain - start low
        self.osmosdr_sink.set_if_gain(15, 0)
        self.osmosdr_sink.set_bb_gain(15, 0)
        self.osmosdr_sink.set_bandwidth(0, 0)

        # print(f"For TX 2")
        # print(self.osmosdr_sink.get_gain("dsa", 0))
        # print(self.osmosdr_sink.get_gain("system", 0))

        self.qt_freq_sink = qtgui.freq_sink_c(
            1024,                # FFT size
            5,  # window type 5 == blackman harris
            self.center_freq,    # center freq for display
            self.sdr_samp_rate,  # bandwidth (use post-resampler rate)
            "TX Monitor"
        )

        self.qt_freq_sink.set_update_time(0.10)
        self.qt_freq_sink.set_y_axis(-140, 10)

        self.qt_time_sink = qtgui.time_sink_c(
            1024,               # number of points
            self.sdr_samp_rate, # sample rate
            "TX Time Domain"
        )

        # Adding gui sinks to gui
    
        self._qt_freq_sink_win = sip.wrapinstance(self.qt_freq_sink.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qt_freq_sink_win, 0, 0, 1, 1)

        self._qt_time_sink_win = sip.wrapinstance(self.qt_time_sink.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qt_time_sink_win, 1, 0, 1, 1)

        # Add to your __init__ after your existing blocks:

        # === Loopback debug chain ===
        self.gfsk_demod = digital.gfsk_demod(
            samples_per_symbol=self.samples_per_symbol,
            sensitivity=self.sensitivity,
            gain_mu=self.gain_mu,
            mu=self.mu,
            omega_relative_limit=self.omega_relative_limit,
            freq_error=self.freq_error,
            verbose=False,
            log=False
        )

        self.debug_sink = mav_packet_reader()  # your existing RX packet block



        
        ##########################
        # Connections
        #########################

        self.connect(self.source, self.gfsk_mod)
        self.connect(self.gfsk_mod, self.tx_gain)
        self.connect(self.tx_resampler, self.qt_freq_sink)
        self.connect(self.tx_gain, self.qt_time_sink)
        self.connect(self.tx_gain, self.tx_resampler)
        self.connect(self.tx_resampler, self.osmosdr_sink)

        # Connect loopback — tap off after GFSK mod
        # Just for debug perposes
        # self.connect(self.gfsk_mod, self.gfsk_demod)
        # self.connect(self.gfsk_demod, self.debug_sink)

def cli_thread(packet_source):
    mav = mavlink2.MAVLink(None)
    mav.srcSystem = 255
    mav.srcComponent = 1
    
    transmitting = True
    
    def input_listener():
        nonlocal transmitting
        while True:
            try:
                cmd = input("Enter command (start/stop/arm/guided/quit): ")
            except (KeyboardInterrupt, EOFError):
                print("\nProgram Killed")
                Qt.QApplication.quit()
                return
            
            if cmd == 'stop':
                transmitting = False
                print("[CLI] Transmission stopped")
            elif cmd == 'start':
                transmitting = True
                print("[CLI] Transmission started")
            elif cmd == 'arm':
                msg = mav.command_long_encode(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
                packet_source.send_message(msg.pack(mav), True)
                print("[CLI] Arm command sent")
            elif cmd == 'guided':
                msg = mav.command_long_encode(1, 1, 176, 0, 1, 4, 0, 0, 0, 0, 0)
                packet_source.send_message(msg.pack(mav), True)
                print("[CLI] Guided command sent")
            elif cmd == 'quit':
                transmitting = False
                Qt.QApplication.quit()
                return

    listener = threading.Thread(target=input_listener, daemon=True)
    listener.start()

    while True:
        if transmitting:
            msg = mav.command_long_encode(1, 1, 176, 0, 1, 4, 0, 0, 0, 0, 0)
            packet_source.send_message(msg.pack(mav), True)
            print("[TX] Packet sent")
        time.sleep(5)

if __name__ == '__main__':
    app = Qt.QApplication(sys.argv)
    tb = flow_graph()
    tb.show()
    app.processEvents()

    def sig_handler(sig=None, frame=None):
        print("\nCaught SIGINT, shutting down...")
        tb.stop()
        tb.wait()
        Qt.QApplication.quit()

    def noop():
        pass

    signal.signal(signal.SIGINT, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(noop)

    tb.start()
    
    # CLI needs to be in a separate thread now since app.exec_() blocks
    cli = threading.Thread(target=cli_thread, args=(tb.source,), daemon=True)
    cli.start()
    

    try:
        app.exec_()
    except KeyboardInterrupt:
        sig_handler()
    finally:
        tb.stop()
        tb.wait()

        


