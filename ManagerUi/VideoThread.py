import numpy as np
import cv2
import time
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
import time
from MotionPlanner.MotionPlannerBase import MotionPlannerBase

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.running = True
        self.cvimg = None
        self.vehiclePos = np.array([100,50])
        time.sleep(2.0)
        
    def run(self, mapFile = "Assets/map.png"):        
        canvas = cv2.imread(mapFile)
        while self.running:
            if self.cvimg is not None:
                canvas = np.copy(self.cvimg)
            
            # draw vehicle
            # rot_rectangle = ((int(self.vehiclePos[0]), int(self.vehiclePos[1])), (MotionPlannerBase.VEHICLE_SIZE, MotionPlannerBase.VEHICLE_SIZE), 0)
            # box = cv2.boxPoints(rot_rectangle) 
            # box = np.int0(box) 
            # cv2.drawContours(canvas,[box],0,(0,0,255),2)

            self.change_pixmap_signal.emit(canvas)
            time.sleep(0.1)

    def stop(self):
        self.running = False
        self.wait()