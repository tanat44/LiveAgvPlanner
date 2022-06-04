from PyQt5.QtWidgets import QApplication
from ManagerUi.ManagerUi import ManagerUi
import sys
from MotionPlanner.MotionPlanner import MotionPlanner

if __name__=="__main__":
    app = QApplication(sys.argv)

    motionPlanner = MotionPlanner("Assets/map.png")
    a = ManagerUi(motionPlanner)
    a.show()
    sys.exit(app.exec_())