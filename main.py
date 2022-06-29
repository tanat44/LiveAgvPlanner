from PyQt5.QtWidgets import QApplication
from ManagerUi.ManagerUi import ManagerUi
import sys
from MotionPlanner.MotionPlannerP import MotionPlannerP
from MotionPlanner.MotionPlannerT import MotionPlannerT
from RabbitMq.RabbitMqReceiver import RabbitMqReceiver
from time import sleep
import numpy as np

USE_RMQ = False

def onClose(receiver):
    if receiver is not None:
        receiver.stop()

if __name__=="__main__":

    # RMQ
    receiver = None
    if USE_RMQ:
        receiver = RabbitMqReceiver("hello")


    # Motion planner
    motionPlanner = MotionPlannerP("Assets/map.png")
    # motionPlanner = MotionPlannerT("Assets/map.png")

    # UI
    app = QApplication(sys.argv)
    ui = ManagerUi(motionPlanner, onClose, USE_RMQ=USE_RMQ)
    if receiver is not None:
        receiver.onMessage = ui.onRabbitMessage
    ui.show()
    sys.exit(app.exec_())