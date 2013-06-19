#!/usr/bin/env python
import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import matplotlib
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
from matplotlib.figure import Figure

class PlotWindow(QMainWindow):
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle('CrazyFlie Magnetometer Calibration')
        self.create_main_frame()
        self.on_draw()

    def save_plot(self):
        pass

    def on_about(self):
        pass

    def on_pick(self, event):
        pass

    def on_draw(self):
        self.axes2D.clear()        
        self.axes2D.grid(True)
        self.axes3D.clear()        
        self.axes3D.grid(True)
        self.canvas.draw()

    def create_main_frame(self):
        self.main_frame = QWidget()
        self.dpi = 100
        self.fig = Figure((8.0, 4.0), dpi=self.dpi)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self.main_frame)
        self.axes3D = self.fig.add_subplot(121, projection='3d')
        self.axes2D = self.fig.add_subplot(122)
        self.canvas.mpl_connect('pick_event', self.on_pick)
        self.mpl_toolbar = NavigationToolbar(self.canvas, self.main_frame)     
        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        vbox.addWidget(self.mpl_toolbar)
        self.main_frame.setLayout(vbox)
        self.setCentralWidget(self.main_frame)