# create pyqt6 gui  application
import sys
from PyQt6.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QLineEdit, QTextEdit, QGridLayout, QHBoxLayout, QVBoxLayout, QMainWindow, QFileDialog, QMessageBox
from PyQt6.QtGui import QIcon, QPixmap
from PyQt6.QtCore import pyqtSlot


class MainApp():
    def __init__(self) -> None:
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Lazer Verisi")
        self.setGeometry(300, 300, 300, 300)
        self.setWindowIcon(QIcon("icon.png"))
        self.show()

    def lazerCallback(self, mesaj):
        sol_on = list(mesaj.ranges[0:9])
        sag_on = list(mesaj.ranges[350:359])
        on = sol_on + sag_on
        sol = list(mesaj.ranges[80:100])
        sag = list(mesaj.ranges[260:280])
        arka = list(mesaj.ranges[170:190])
        print("sol:", sol)
        print("sag:", sag)
        print("arka:", arka)
        print("on:", on)
        min_on = min(on)
        min_sol = min(sol)
        min_sag = min(sag)
        min_arka = min(arka)
        print(min_on, min_sol, min_sag, min_arka)
        if min_on < 1.0:
            self.hiz_mesaji.linear.x = 0.0
            self.pub.publish(self.hiz_mesaji)
        else:
            self.hiz_mesaji.linear.x = 0.25
            self.pub.publish(self.hiz_mesaji)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainApp()
    sys.exit(app.exec())
