import numpy as np
import cv2
import time
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
import time

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.running = True
        self.playing = False
        time.sleep(2.0)
        
    def run(self, mapFile = "Assets/map.png"):        
        while self.running:
            # canvas = np.zeros((500, 500, 3), dtype="uint8")
            canvas = cv2.imread(mapFile)
            self.change_pixmap_signal.emit(canvas)
            while not self.playing:
                time.sleep(0.5)
            time.sleep(0.03)

    def stop(self):
        self.running = False
        self.playing = True
        self.wait()

    def resizeToFit(self, targetWidth, targetHeight, image):
        h, w, c = image.shape
        wRatio = targetWidth / w
        hRatio = targetHeight / h
        ratio = 1
        if wRatio < hRatio:
            ratio = wRatio
        else:
            ratio = hRatio
        image = cv2.resize(image, (int(ratio*w), int(ratio*h)), interpolation=cv2.INTER_AREA)
        h, w, c = image.shape
        vPad = targetHeight - h
        hPad = targetWidth - w
        return cv2.copyMakeBorder(image, 0, vPad, 0, hPad, borderType=cv2.BORDER_CONSTANT, value=[0,0,0])


    def drawRoi(self, image, width, height):
        t, l, b, r = self.roi.getAbsoluteValue(width, height)
        pts = np.array([[l, t],\
            [r, t], \
            [r, b], \
            [l, b], \
            ], np.int32)

        pts = pts.reshape((-1,1,2))

        cv2.polylines(image,[pts],True,(0,255,255), thickness=5)
        return image

    def setRoi(self, top=None, left=None, bottom=None, right=None):
        if top is not None:
            self.roi.top = top
        if left is not None:
            self.roi.left = left
        if bottom is not None:
            self.roi.bottom = bottom
        if right is not None:
            self.roi.right = right

        self.reprocessFrame()