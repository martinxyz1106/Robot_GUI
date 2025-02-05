import threading
import rclpy
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication,QMainWindow
import sys

from gui_launcher.robot_client import RobotServiceClient
from robot_gui.gui_set.gui_control import MyApp

class GUIlaunch():
    def __init__(self):
        # super().__init__('gui_launch')
        self.gui = None
        self.app = None
        self.service_client = None
        self.queue_thread = None
        self.robot_client = RobotServiceClient()
        self.executor = MultiThreadedExecutor()
    def launch(self):
        
        # GUI 실행을 메인 스레드에서 시작
        self.app = QApplication(sys.argv)
        self.gui = MyApp()
        self.gui.show()

        self.executor.add_node(self.robot_client)

        # 명령을 처리하는 스레드 시작
        self.queue_thread = threading.Timer(interval=0.1, function=self.process_commands)
        
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.queue_thread.start()
        self.executor_thread.start()
        sys.exit(self.app.exec())

        # finally:
        #     # self.executor.shutdown()
        #     # self.queue_thread.join()
        #     # self.executor_thread.join()
        
    def process_commands(self):
        try:
            while True:
                if self.gui.cmd_list:
                    command = self.gui.cmd_list.popleft()
                    print(f"Input Command Cmd: {command['request_id'], command['no']}")

                    if (command['request_id'] == 'exit'):
                        self.queue_thread.cancel()
                        QApplication.instance().quit()
                    else :
                        self.robot_client.get_command(command)
                else :
                    print("GUI Queue is Empty")
                time.sleep(0.5)  # CPU 사용을 방지하기 위해 약간의 대기시간 추가
                
        except KeyboardInterrupt:
                # self.queue_thread.join()
                raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    # robot_client = RobotServiceClient()
    gui = GUIlaunch()
    try:
        gui.launch()
    except Exception as error:
        pass
    except KeyboardInterrupt :
        raise SystemExit

    # executor = MultiThreadedExecutor()
    # executor.add_node(gui)
    # executor.add_node(gui.robot_client)
    # try:
    #     executor.spin()
    # except Exception as error:
    #     executor.remove_node(gui)
    #     executor.shutdown()
    #     gui.queue_thread.join()
        
    
if __name__ == '__main__':
   main()