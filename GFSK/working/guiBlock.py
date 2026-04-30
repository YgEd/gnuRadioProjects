from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio.fft import window
import sip




class qtGUIs():
    def __init__(
        self,
        top_layout,
        top_grid_layout,
        symbol_rate,
    ):

        self.top_layout = top_layout
        self.top_grid_layout = top_grid_layout
        self.symbol_rate = symbol_rate


    def getFreqSink(self, center_freq, sdr_samp_rate, label):
        fft_sink = qtgui.freq_sink_c(
            1024, 5, center_freq, sdr_samp_rate, label
        )
        fft_sink.set_update_time(0.10)
        fft_sink.set_y_axis(-140, 10)
        fft_win = sip.wrapinstance(fft_sink.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(fft_win, 0, 0, 1, 1)
        return fft_sink

    def getConstellation(self, label):
        constellation_plot = qtgui.const_sink_c(
            1024,
            label,
            1,
            None
        )
        constellation_plot.set_update_time(0.10)
        constellation_plot.set_y_axis(-2,2)
        constellation_plot.set_x_axis(-2,2)

        _constellation_plot_win = sip.wrapinstance(
            constellation_plot.qwidget(), Qt.QWidget
        )
        self.top_layout.addWidget(_constellation_plot_win)
        return constellation_plot

    def getTimeSink(self, label: str):

        time_sink = qtgui.time_sink_c(
            256, #size
            self.symbol_rate, #samp_rate
            label, #name
            1, #number of inputs
            None # parent
        )
        time_sink.set_update_time(0.10)
        time_sink.set_y_axis(-1, 1)

        time_sink.set_y_label('Amplitude', "")

        time_sink.enable_tags(True)
        time_sink.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        time_sink.enable_autoscale(False)
        time_sink.enable_grid(False)
        time_sink.enable_axis_labels(True)
        time_sink.enable_control_panel(False)
        time_sink.enable_stem_plot(False)


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
                    time_sink.set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    time_sink.set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                time_sink.set_line_label(i, labels[i])
            time_sink.set_line_width(i, widths[i])
            time_sink.set_line_color(i, colors[i])
            time_sink.set_line_style(i, styles[i])
            time_sink.set_line_marker(i, markers[i])
            time_sink.set_line_alpha(i, alphas[i])

        _time_sink_win = sip.wrapinstance(time_sink.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(_time_sink_win)

        return time_sink