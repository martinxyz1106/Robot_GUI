import threading
import rclpy
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication,QMainWindow
import sys

from gui_launcher.robot_client import RobotServiceClient
from robot_gui.gui_set.gui_control import MyApp

class GUIlaunch(Node):
    def __init__(self):
        super().__init__('gui_launch')
        self.gui = None
        self.app = None
        self.service_client = None
        self.gui_thread = None
        self.queue_thread = None
    def launch(self):
        
        # 서비스 클라이언트 노드와 GUI 노드 생성
        self.service_client = RobotServiceClient()
        self.app = QApplication(sys.argv)
        self.gui = MyApp(QMainWindow)
        self.gui.show()

        # MultiThreadedExecutor 생성 및 노드 추가
        executor = MultiThreadedExecutor()
        executor.add_node(self.service_client)
        
        # GUI 실행을 별도의 스레드에서 시작
        self.gui_thread = threading.Thread(target=self.app.exec)
        self.gui_thread.start()

        self.queue_thread = threading.Thread(target=self.process_commands,args=(self,self.service_client))
        self.queue_thread.start()
        try:
            executor.spin()
        finally:
            executor.shutdown()
            self.gui_thread.join()
            self.queue_thread.join()
    def process_commands(self,robot_client):
        while True:
            if self.gui.cmd_list:
                command = self.gui.cmd_list.popleft()
                robot_client.execute_node(command)
            time.sleep(0.1)  # CPU 사용을 방지하기 위해 약간의 대기시간 추가
        

def main(args=None):
    rclpy.init(args=args)
    gui = GUIlaunch()
    gui.launch()

if __name__ == '__main__':
   main()