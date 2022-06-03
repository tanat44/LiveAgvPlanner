from PyQt5.QtWidgets import QApplication
from ManagerUi.ManagerUi import ManagerUi
import sys

if __name__=="__main__":
    app = QApplication(sys.argv)
    a = ManagerUi()
    a.show()
    sys.exit(app.exec_())