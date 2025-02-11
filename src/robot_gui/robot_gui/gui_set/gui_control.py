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
        super().__init__()
        self.ui = Ui_Widget()
        self.ui.setupUi(self)

        # 버튼 클릭 시 실행할 함수를 연결합니다.
        self.ui.pickup_cof1.clicked.connect(self.send_pic_cof1)
        self.ui.pickup_cof2.clicked.connect(self.send_pic_cof2)
        self.ui.pickup_bev1.clicked.connect(self.send_pic_bev1)
        self.ui.pickup_bev2.clicked.connect(self.send_pic_bev2)
        self.ui.pickup_zone.clicked.connect(self.send_pick_zone)
        self.ui.unhold_cup.clicked.connect(self.send_pic_cup)
        self.ui.place_zone.clicked.connect(self.send_place_zone)
        self.ui.hold_ice1.clicked.connect(self.send_pic_ice1)
        self.ui.hold_ice2.clicked.connect(self.send_pic_ice2)
        self.ui.ExitButton.clicked.connect(self.exit)
        self.ui.go_home.clicked.connect(self.send_go_home)
        self.ui.move_home.clicked.connect(self.robot_home)
        self.ui.move_approach.clicked.connect(self.robot_approach)
        self.ui.cup_back.clicked.connect(self.robot_cup_back)
        self.ui.unlock_protective.clicked.connect(self.unlock_protective)

        self.cmd_list = deque()
    
    @pyqtSlot()
    def send_pic_cof1(self):
        print("버튼이 눌렸습니다!")
        cmd = {'request_id': 'pickup_coffee', 'no': 0}
        self.cmd_list.appendleft(cmd)
    
    def send_pic_cof2(self):
        cmd = {'request_id': 'pickup_coffee', 'no': 1}
        print("버튼이 눌렸습니다!")
        self.cmd_list.appendleft(cmd)
    
    def send_pic_bev1(self):
        print("버튼이 눌렸습니다!")
        cmd = {'request_id': 'pickup_powder', 'no': 0}
        self.cmd_list.appendleft(cmd)
    
    def send_pic_bev2(self):
        print("버튼이 눌렸습니다!")
        cmd = {'request_id': 'pickup_powder', 'no': 1}
        self.cmd_list.appendleft(cmd)
    
    def send_pick_zone(self):
        spin_value = self.ui.zone_num_1.value()  # SpinBox의 값을 가져옵니다.
        print(f"버튼이 눌렸습니다! SpinBox 값: {spin_value}")
        cmd = {'request_id': 'pickup_order','no': spin_value-1}
        self.cmd_list.appendleft(cmd)
    def send_place_zone(self):
        spin_value = self.ui.zone_num_2.value()  # SpinBox의 값을 가져옵니다.
        print(f"버튼이 눌렸습니다! SpinBox 값: {spin_value}")
        cmd = {'request_id': 'place_order','no': spin_value-1}
        self.cmd_list.appendleft(cmd)
    
    def send_pic_cup(self):
        spin_value = self.ui.channel_num.value()
        print("버튼이 눌렸습니다! SpinBox 값: {spin_value}")
        cmd = {'request_id': 'unhold_cup', 'no': spin_value-1}
        self.cmd_list.appendleft(cmd)
    
    def send_pic_ice1(self):
        print("버튼이 눌렸습니다!")
        cmd = {'request_id': 'unhold_ice', 'no': 0}
        self.cmd_list.appendleft(cmd)
    
    def send_pic_ice2(self):
        print("버튼이 눌렸습니다!")
        cmd = {'request_id': 'unhold_ice', 'no': 1}
        self.cmd_list.appendleft(cmd)
    
    def send_go_home(self):
        print("버튼이 눌렸습니다!")
        cmd = {'request_id': 'home', 'no': 1}
        self.cmd_list.appendleft(cmd)
    def robot_cup_back(self):
        print("ROBCUP 버튼이 눌렸습니다!")
        cmd = {'request_id': 'ROBCUP', 'no': 0}
        self.cmd_list.appendleft(cmd)
        
    def robot_home(self):
        print("ROBHOME 버튼이 눌렸습니다!")
        cmd = {'request_id': 'ROBHOME', 'no': 0}
        self.cmd_list.appendleft(cmd)

    def robot_approach(self):
        print("approach 버튼이 눌렸습니다!")
        cmd = {'request_id': 'APPROACH', 'no': 0}
        self.cmd_list.appendleft(cmd)
    
    def unlock_protective(self):
        print("unlock_버튼이 눌렸습니다!")
        cmd = {'request_id': 'UNLOCK_PROTECT', 'no': 0}
        self.cmd_list.appendleft(cmd)
    def exit(self):
        print("종료 합니다.")
        cmd = {'request_id': 'exit', 'no': 0}
        self.cmd_list.appendleft(cmd)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyApp()
    window.show()
    sys.exit(app.exec())
