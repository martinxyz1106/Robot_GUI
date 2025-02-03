# This Python file uses the following encoding: utf-8
import os
from pathlib import Path
import sys

from PyQt6.QtWidgets import QApplication, QWidget, QMainWindow 
from PyQt6.QtCore import QFile, pyqtSlot
from robot_gui.gui_set.gui_form import Ui_Widget
from collections import deque
# from gui_launcher.DBconstants import DBFieldName
class MyApp(QMainWindow):
    def __init__(self):
        super().__init__("MYAPP")
        self.ui = Ui_Widget()
        self.ui.setupUi(self)

        # 버튼 클릭 시 실행할 함수를 연결합니다.
        self.ui.pickup_cof1.clicked.connect(self.send_pic_cof1)
        self.ui.pickup_cof2.clicked.connect(self.send_pic_cof2)
        self.ui.pickup_bev1.clicked.connect(self.send_pic_bev1)
        self.ui.pickup_bev2.clicked.connect(self.send_pic_bev2)
        self.ui.pickup_zone.clicked.connect(self.send_pic_zone)
        self.cmd_list = deque()
    @pyqtSlot()
    def send_pic_cof1(self):
        print("버튼이 눌렸습니다!")
        cmd = {'request_id': 'pickup_coffee', 'no': 0}
        self.cmd_list.appendleft(cmd)
    @pyqtSlot()
    def send_pic_cof2(self):
        cmd = {'request_id': 'pickup_coffee', 'no': 1}
        print("버튼이 눌렸습니다!")
        self.cmd_list.appendleft(cmd)
    @pyqtSlot()
    def send_pic_bev1(self):
        print("버튼이 눌렸습니다!")
        cmd = {'request_id': 'pickup_powder', 'no': 0}
        self.cmd_list.appendleft(cmd)
    @pyqtSlot()
    def send_pic_bev2(self):
        print("버튼이 눌렸습니다!")
        cmd = {'request_id': 'pickup_powder', 'no': 1}
        self.cmd_list.appendleft(cmd)
    @pyqtSlot()
    def send_pic_zone(self):
        spin_value = self.ui.zone_num.value()  # SpinBox의 값을 가져옵니다.
        print(f"버튼이 눌렸습니다! SpinBox 값: {spin_value}")
        cmd = {'request_id': 'pickup_order','no': spin_value}
        self.cmd_list.appendleft(cmd)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyApp()
    window.show()
    sys.exit(app.exec())
