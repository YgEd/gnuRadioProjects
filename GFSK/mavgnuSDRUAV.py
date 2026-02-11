from gnuradio import gr, blocks, digital, filter, analog
from gnuradio.filter import firdes

from mavGNUBlock import mav_packet_reader, mav_packet_source
import threading
from pymavlink.dialects.v20 import common as mavlink2



class flow_graph(gr.top_block):
    def __init__(self):
        gr.top_block.__init__(self)

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

        self.tx_interpolation=20
        self.tx_decimation=1
        
        self.rx_interpolation=1
        self.rx_decimation=20
        self.fractional_bw=0.4
        self.tx_gain_scalar=1

    

        #######################
        # Blocks
        #######################

        self.source = mav_packet_source()
        self.destination = mav_packet_reader()
        self.throttle = blocks.throttle(gr.sizeof_gr_complex*1, self.sdr_samp_rate, False)
        
        # upsample from 100e3 to 2e6 or (2Mhz sampling rate limit for bladerf is 61.44Mhz)
        self.tx_resampler = filter.rational_resampler_ccf(
            interpolation=self.tx_interpolation,
            decimation=self.tx_decimation,
            taps=[],
            fractional_bw=self.fractional_bw
        )

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
                # 1 -> more open, tolerant ot some offset
                # If you have a high modulation index (h value) you may want ot expand your cut_off frequency as there will be a bigger freq gap between 1 and 0 signals
                # You can expand up until samp_rate/2 because that is the nyquist limit so in this case cutoff_freq can be at most 50 Khz
                cutoff_freq=max((self.samp_rate/self.samples_per_symbol) * (1 + self.bt) * 1, samp_rate/2 - samp_rate/10),
                # transition width rule of thumb is
                # transition width ~ cutoff_freq * 0.25
                transition_width=(self.samp_rate/self.samples_per_symbol) * (1 + self.bt) * 1 * 0.25
            ),
            center_freq=0, #leave at 0 by default this is a constant offset if you know consistent tx rx freq offset
            sampling_freq=self.sdr_samp_rate
        )
        
        # how much you are scaling power form sdr 0.25 is good for bench testing, 0.5 half power good more realistic testing, 1 is max power may cause clipping
        self.tx_gain = blocks.multiply_const_cc(self.tx_gain_scalar) 

        # agc for rx
        # self.agc = analog.agc_cc(
        #     rate=1e-4,       # adaptation rate — how fast AGC adjusts Higher values (1e-2 to 1e-3) get fast adaption but fluctuates iwth noise. Lower values (1e-4 to 1e-5) slow adaption but stable gain during packet
        #     reference=1.0,   # target output amplitude
        #     gain=1.0         # initial gain estimate
        # )

        # AGC2 is better as it has tow different rates for when a strong signal appears you want to clamp down on it quick vs a sustained signal you want to back off on the rate
        self.agc = analog.agc2_cc(
            attack_rate=1e-1,   # how fast gain DECREASES (strong signal arrives)
            decay_rate=1e-4,    # how fast gain INCREASES (signal weakens/disappears)
            reference=1.0,
            gain=1.0
        )


        # gfsk mod
        self.gfsk_mod = digital.gfsk_mod(
            samples_per_symbol=self.samples_per_symbol,
            sensitivity=self.sensitivity,
            bt=self.bt,
            verbose=False,
            log=False,
            do_unpack=False)
        
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
        
        ##########################
        # Connections
        #########################

        self.connect(self.source, self.gfsk_mod)
        self.connect(self.gfsk_mod, self.tx_gain)
        self.connect(self.tx_gain, self.tx_resampler)
        
        # throttle for simulation sake
        self.connect(self.tx_resampler, self.throttle)
        self.connect(self.throttle, self.rx_resampler_lowpass)
        # self.connect(self.tx_resampler, self.rx_resampler_lowpass)

        self.connect(self.rx_resampler_lowpass, self.agc)
        self.connect(self.agc, self.gfsk_demod)
        self.connect(self.gfsk_demod, self.destination)


def cli_thread(packet_source):
    mav = mavlink2.MAVLink(None)
    mav.srcSystem = 255
    mav.srcComponent = 1
    
    while True:
        cmd = input("Enter command: ")
        if cmd == 'arm':
            msg = mav.command_long_encode(1,1,400,0,1,0,0,0,0,0,0)
            packet_source.send_message(msg.pack(mav), True)
        elif cmd == 'guided':
            msg = mav.command_long_encode(1,1,176,0,1,4,0,0,0,0,0)
            packet_source.send_message(msg.pack(mav), True)
        elif cmd == 'heart':
            msg = mav.command_long_encode(1,1,0,0,0,0,0,0,0,0,0)
            packet_source.send_message(msg.pack(mav), True)
        elif cmd == 'quit':
            break

if __name__ == '__main__':
    tb = flow_graph()
    tb.start()
    
    # Run CLI in main thread (or separate thread)
    cli_thread(tb.source)
    
    tb.stop()
    tb.wait()

        


