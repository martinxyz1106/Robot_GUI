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
    def __init__(self, client):
        super().__init__('gui_launch')
        self.gui = None
        self.app = None
        self.service_client = None
        self.queue_thread = None
        self.robot_client = client
    def launch(self):
        
        # GUI 실행을 메인 스레드에서 시작
        self.app = QApplication(sys.argv)
        self.gui = MyApp()
        self.gui.show()

        # 명령을 처리하는 스레드 시작
        self.queue_thread = threading.Timer(interval=0.1, function=self.process_commands)
        self.queue_thread.start()
        # self.executor_thread = threading.Thread(target=self.executor.spin)
        # self.executor_thread.start()
        sys.exit(self.app.exec())
            
        # finally:
        #     # self.executor.shutdown()
        #     # self.queue_thread.join()
        #     # self.executor_thread.join()
            
    def process_commands(self):
        while True:
            if self.gui.cmd_list:
                command = self.gui.cmd_list.popleft()
                print(f"Input Command Cmd: {command['request_id'], command['no']}")
                self.robot_client.get_command(command)
            time.sleep(0.1)  # CPU 사용을 방지하기 위해 약간의 대기시간 추가
        

def main(args=None):
    rclpy.init(args=args)
    robot_client = RobotServiceClient()
    gui = GUIlaunch(robot_client)
    executor = MultiThreadedExecutor()
    executor.add_node(gui)
    executor.add_node(robot_client)
    try:
        executor.spin()
    except Exception as error:
        executor.shutdown()
        gui.queue_thread.join()
        
    
if __name__ == '__main__':
   main()