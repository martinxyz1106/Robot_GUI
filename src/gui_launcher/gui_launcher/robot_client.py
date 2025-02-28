import rclpy as rp
from message.srv import RobotService, RtdeService
from rclpy.node import Node
from rclpy.qos import QoSProfile
import datetime
import threading
from gui_launcher.RobotRequestObject import RobotRequestObject
from gui_launcher.DBConstants import DBFieldName
import dashboard_client
import rtde_control
import time 
import math
import os, subprocess, signal
class RobotServiceClient(Node):
    RTDE_IP = '192.168.0.11'
    RTDE_COMMAND = ['UNLOCK_PROTECT','APPROACH','ROBCUP','ROBHOME']
    def __init__(self):
        super().__init__('RobotTestClient')
        self.qos_profile = QoSProfile(depth=25)
        self.robot_client = self.create_client(RobotService, 'robot/service', qos_profile=self.qos_profile)
        self.rtde_client = self.create_client(RtdeService, 'rtde/service',qos_profile=self.qos_profile)
        self.queue_timer = self.create_timer(1.0, self.queue_checker)
        self.pickup_index = 0
        self.robot_queue = []
        self.dashboard_interface = None
        self.control_interface = None
        while not self.robot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        print("Robot Serivce client Setup Complete")

    def get_command(self, cmd=None):   
        srv_req = RobotService.Request()
        request_object = RobotRequestObject()

        if cmd != type(None):

            if cmd[DBFieldName.REQUEST_ID] in RobotServiceClient.RTDE_COMMAND:
                # request = RtdeService.Request()
                
                
                self.kill_rtde_node()
                
                if self.dashboard_interface is None:
                    self.kill_rtde_port()
                    self.dashboard_interface = dashboard_client.DashboardClient(RobotServiceClient.RTDE_IP)
                    self.control_interface = rtde_control.RTDEControlInterface(RobotServiceClient.RTDE_IP)
            
                print("Ur RTDE Control is Connected ?? ")
                rtde_cmd = cmd[DBFieldName.REQUEST_ID]
                print(f'USE RTDE CMD : {rtde_cmd}')
                if rtde_cmd == RTDECOMMAND.APPROACH:
                    # self.dashboard_interface.reconnect()
                    if not self.control_interface.isConnected() :
                        self.control_interface.reconnect()
                    position = [3.1539, -1.8879, -2.3347, 4.2216, -1.5568, 0.0055]
                    self.control_interface.moveJ(position)
                    
                elif rtde_cmd == RTDECOMMAND.ROBHOME:
                    # 6.280567313301596
                    # self.dashboard_interface.reconnect()
                    if not self.control_interface.isConnected():
                        self.control_interface.reconnect()
                    position = [1.570970859720096, -0.787318025574642, -1.654921196741023, 2.440144827213272, -1.5707963267948966, 0.0055]
                    self.control_interface.moveJ(position)
                    
                elif rtde_cmd == RTDECOMMAND.ROBCUP:
                    # self.dashboard_interface.reconnect()
                    if not self.control_interface.isConnected() :
                        self.control_interface.reconnect()
                    position = [3.224670325984723, -2.475924076879156, -1.844289420582408, 4.318817234059968, -1.4885913190259639, 0.004537856055185257]
                    self.control_interface.moveJ(position)
                    
                elif rtde_cmd == RTDECOMMAND.UNLOCK_PROTECT:
                    self.dashboard_interface.connect()
                    # self.control_interface.reconnect()
                    self.dashboard_interface.unlockProtectiveStop()
                    self.dashboard_interface.stop()
                    # MayBe Script Play???
                
            else: 
            
                robot_request = request_object.get_robot_request(cmd[DBFieldName.REQUEST_ID], cmd[DBFieldName.NO])

                srv_req.seq_no = str(datetime.datetime.now())
                srv_req.cmd = robot_request.command
                srv_req.rail_pos = robot_request.rail_pos
                srv_req.par1 = robot_request.param1
                srv_req.par2 = robot_request.param2
                srv_req.par3 = robot_request.param3
                srv_req.par4 = robot_request.param4
                srv_req.par5 = robot_request.param5

                print(f"{srv_req=}")
                self.robot_queue.append(srv_req)
                
    def deg2rad(self, deg):
        rad = 0.0
        rad = deg * math.pi/180

    def kill_rtde_port(self):
        pids = []
        contorl_port = 30004
        board_port = 29999
        contorl_result = subprocess.run(['lsof', '-i', f':{contorl_port}'], capture_output=True, text=True)
        control_lines = contorl_result.stdout.splitlines()
        if len(control_lines) > 1:
            pid = control_lines[1].split()[1]
            pids.append(pid)

        # board_result = subprocess.run(['lsof', '-i', f':{board_port}'], capture_output=True, text=True)
        # board_lines = board_result.stdout.splitlines()

        # if len(board_lines) > 1:
        #     pid = board_lines[1].split()[1]
        #     pids.append(pid)

        if pids:
            contorl_pid = pids[0]
            # board_pid = pids[1]
            os.system(f'kill -9 {contorl_pid}')
            print(f"PID : {contorl_pid} 가 종료되었습니다.")

            # os.system(f'kill -9 {board_pid}')
            # print(f"PID : {board_pid} 가 종료되었습니다.")

    def kill_rtde_node(self):
        output = os.popen('ps -ef | grep Rtde').read()
        pids = []
        # 출력에서 PID를 추출
        for line in output.split('\n'):
            if 'Rtde' in line:
                parts = line.split()
                pid = int(parts[1])
                pids.append(pid)

        if pids:
            node_pid = pids[0]
            # python_pid = pids[1]
            os.kill(node_pid, signal.SIGTERM)
            print(f" PID : {node_pid} is Killed")
            # os.kill(python_pid, signal.SIGTERM)
            # print(f" PID : {python_pid} is Killed")
    def queue_checker(self):
        try:
            if len(self.robot_queue) != 0:
                srv_req = self.robot_queue.pop()
                print(f"Serivce Request : {srv_req.cmd}, {srv_req.rail_pos}, {srv_req.par1}")
                self.send_robot_request(srv_req)
            # print(f"Serivce Response : {response.seq_no}, {response.result}")
            else :
                print("No Input Data Found")
        except KeyboardInterrupt :
            raise SystemExit
        
    def send_robot_request(self, request):
        self.future = self.robot_client.call_async(request)
        self.future.add_done_callback(self.robot_done)
        # rp.spin_until_future_complete(node = self, future=self.future, timeout_sec=0.5)
        # return self.future.result()
    
    def send_rtde_request(self,request):
        self.future = self.rtde_client.call_async(request)
        self.future.add_done_callback(self.rtde_done)
    
    def robot_done(self, tmp):
        if tmp.done():
            response = tmp.result()
            print(f"Temp : {type(tmp)}")
            print(f"Serivce Response : {response.component_cd}, {response.result}")

    def rtde_done(self, tmp):
        if tmp.done():
            response = tmp.result()
            print(f"Temp : {type(tmp)}")
            print(f"Serivce Response : {response.component_cd}, {response.result}")
# def main(args=None):
#     rp.init(args=args)
#     robot_srv_client = RobotServiceClient()
#     try:
#         rp.spin(robot_srv_client)
#     except Exception as ex:
#         error_msg = traceback.format_exc(ex)
#         robot_srv_client.get_logger().info(ex)


# if __name__ == '__main__':
#     main()
class RTDECOMMAND:

    UNLOCK_PROTECT= 'UNLOCK_PROTECT'
    APPROACH = 'APPROACH'
    ROBCUP = 'ROBCUP'
    ROBHOME ='ROBHOME'