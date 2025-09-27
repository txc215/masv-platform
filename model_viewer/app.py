import os

import wx
from viewer import ViewerFrame

if __name__ == "__main__":
    app = wx.App(False)
    frame = ViewerFrame(None, title="Model Viewer", size=(800, 600))
    frame.Show()
    app.MainLoop()