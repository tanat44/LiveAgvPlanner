from time import time
from PyQt5 import QtGui
from PyQt5.QtWidgets import QHBoxLayout, QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QPixmap
import sys
import cv2
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
import numpy as np
from ManagerUi.VideoThread import VideoThread
from Agv.Agv import Agv

class ManagerUi(QWidget):
    def __init__(self, motionPlanner, onClose = None, USE_RMQ = False):
        super().__init__()
        self.setWindowTitle("Live AGV Planner")
        self.setFixedSize(1000,800)
        self.onClose = onClose

        # OBJECTS
        self.motionPlanner = motionPlanner
        self.targetRoute = []
        self.agv = Agv(USE_RMQ)

        # VIDEO DISPLAY
        self.imageLabel = QLabel(self)
        self.imageLabel.mousePressEvent = self.imageClick
        self.imageLabel.setStyleSheet("border: 1px solid black")
        self.videoThread = VideoThread()
        self.videoThread.change_pixmap_signal.connect(self.update_image)
        self.videoThread.start()

        # CONTROL PANEL
        controlPanelLayout = QVBoxLayout()
        self.controlPanel = QWidget()
        self.controlPanel.setLayout(controlPanelLayout)
        self.controlPanel.setFixedWidth(300)

        # MAIN LAYOUT
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(self.imageLabel)
        mainLayout.addWidget(self.controlPanel)

        self.setLayout(mainLayout)

    def onRabbitMessage(self, msg):
        data = eval(msg)
        self.videoThread.vehiclePos = np.array(data['pos'])

    def closeEvent(self, event):
        event.accept()

    def convert_cv_qt(self, cvimg):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cvimg, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qimg = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        self.imageLabel.setFixedWidth(w)
        self.imageLabel.setFixedHeight(h)
        # p = convert_to_Qt_format.scaled(self.imageLabel.width(), self.imageLabel.height(), Qt.KeepAspectRatio)
        return QPixmap.fromImage(qimg)

    def imageClick(self, event):
        # labelPos = self.imageLabel.mapFrom(self, event.pos())
        pos = np.array([event.pos().x(), event.pos().y()])
        if len(self.targetRoute) < 2:
            self.targetRoute.append(pos)
        
        if len(self.targetRoute) == 2:
            print("execute planner", self.targetRoute)
            start = time()
            path = self.motionPlanner.findPath(self.targetRoute[0], self.targetRoute[1])
            print(time()-start)
            cvimg = self.motionPlanner.drawPathOnMap(path)
            self.videoThread.cvimg = cvimg
            
            # update AGV
            self.agv.target = self.targetRoute[1]
            self.agv.pos = self.targetRoute[0]

            self.targetRoute = []
            

    # SLOT
    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        qt_img = self.convert_cv_qt(cv_img)
        self.imageLabel.setPixmap(qt_img)

    def closeEvent(self, event):
        print("UI closing")
        if self.onClose is not None:
            self.onClose()