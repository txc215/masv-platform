import wx
import os
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('WXAgg')
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.figure import Figure

curr_path = os.path.dirname(os.path.abspath(__file__))

__simple_data__ = curr_path + "/../data/sample_inputs/dummy_gnss_imu.csv"
__output_data__ = curr_path + "/../data/sample_outputs/dummy_gnss_imu_ekf.csv"

class ViewerFrame(wx.Frame):
    def __init__(self, parent, title = "Plot data", size=(400, 200)):
        super().__init__(parent, title=title, size=size)

        panel = wx.Panel(self)

        vbox = wx.BoxSizer(wx.VERTICAL)

        self.figure = Figure()
        self.axes = self.figure.add_subplot(111)

        self.canvas = FigureCanvas(panel, -1, self.figure)

        plot_sizer = wx.BoxSizer(wx.VERTICAL)
        plot_sizer.Add(self.canvas, flag=wx.EXPAND | wx.ALL, border=5)
        

        self.plot_btn = wx.Button(panel, label="Load Data")
        # event binding
        self.plot_btn.Bind(wx.EVT_BUTTON, self.on_load_model)
        ctrl_sizer = wx.BoxSizer(wx.HORIZONTAL)
        ctrl_sizer.Add(self.plot_btn, 0, wx.ALL)

        vbox.Add(ctrl_sizer, proportion=0, flag=wx.CENTER)
        # vbox.Add(plot_sizer, 1, border=15)
        vbox.Add(plot_sizer, proportion=1, flag=wx.ALL | wx.EXPAND, border=15)

        panel.SetSizer(vbox)
        self.Centre()
        self.Show()
        self.Layout()

    def mag_to_heading(self, mag_x_uT, mag_y_uT):

        heading_rad = np.arctan2(-mag_y_uT, mag_x_uT)
        
        if heading_rad < 0:
            heading_rad += 2 * np.pi
        
        return heading_rad

    def on_load_model(self, event):

        imu_df = pd.read_csv(__simple_data__, usecols=["timestamp_sec","gyro_z_radps", "mag_x_uT", "mag_y_uT"])
        
        time_sec, gz_radps, mag_heading_rad = [], [], []
        data_len = len(imu_df["timestamp_sec"])

        for i in range(data_len):
            time_sec.append(imu_df["timestamp_sec"][i])
            gz_radps.append(imu_df["gyro_z_radps"][i])
            heading_rad = self.mag_to_heading(imu_df["mag_x_uT"][i], imu_df["mag_y_uT"][i])
            mag_heading_rad.append(heading_rad)

        self.axes.clear()
        self.axes.plot(time_sec, gz_radps, label="gyro (rad/s)", color='red')
        self.axes.plot(time_sec, mag_heading_rad, label="magnetometer heading (rad)", color='green')
        
        self.axes.set_title("GRYO Z and Magnetometer heading data")
        self.axes.legend()
        self.canvas.draw()

