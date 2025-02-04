import rclpy as rp
from robot_msg.srv import RobotService
from rclpy.node import Node
from rclpy.qos import QoSProfile
import traceback
import datetime
import time
from gui_launcher.RobotRequestObject import RobotRequestObject
from gui_launcher.DBConstants import DBFieldName

class RobotServiceClient(Node):
    HOLD_CUP = 'hold_cup'
    UNHOLD_CUP = 'unhold_cup'
    PLACE_COFFEE = 'place_coffee'
    PICKUP_COFFEE = 'pickup_coffee'
    UNHOLD_ICE = 'unhold_ice'
    HOLD_ICE = 'hold_ice'
    PICKUP_POWDER = 'pickup_powder'
    PLACE_POWDER = 'place_powder'
    PLACE_ORDER = 'place_order'
    PICKUP_ORDER = 'pickup_order'
    PLACE_HOLDER = 'place_holder'
    PICKUP_HOLDER = 'pickup_holder'
    GRIPPER_INIT = 'gripper_init'
    SHAKE = 'shake'
    HOME = 'home'
    GREET = 'greet'
    MOVE = 'move'
    GESTURE = 'gesture'
    PICKUP_LID = 'pickup_lid'
    CLOSE_LID = 'close_lid'
    PICKUP_SYRUP = 'pickup_syrup'
    PLACE_SYRUP = 'place_syrup'
    PUMP = 'pump'
    UNHOLD_BOX = 'unhold_box'
    HOLD_BOX = 'hold_box'
    PLACE_BOX = 'place_box'
    def __init__(self):
        super().__init__('RobotTestClient')
        self.qos_profile = QoSProfile(depth=25)
        self.client = self.create_client(RobotService, 'robot/service', qos_profile=self.qos_profile)
        self.queue_timer = self.create_timer(timer_period_sec=0.5, callback=self.queue_checker)
        self.pickup_index = 0
        self.robot_queue = []

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        print("Robot Serivce client Setup Complete")
    def get_command(self, cmd=None):   
        srv_req = RobotService.Request()
        request_object = RobotRequestObject()
        if cmd != type(None):
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

    def queue_checker(self):
        if len(self.robot_queue) != 0:
            srv_req = self.robot_queue.pop()
            print(f"Serivce Request : {srv_req.cmd}, {srv_req.rail_pos}, {srv_req.par1}")
            response = self.send_request(srv_req)
            print(f"Serivce Response : {response.seq_no}, {response.result}")
        else :
            print("No Input Data Found")
    def send_request(self, request):
        self.future = self.client.call_async(request)
        rp.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rp.init(args=args)
    robot_srv_client = RobotServiceClient()
    try:
        rp.spin(robot_srv_client)
    except Exception as ex:
        error_msg = traceback.format_exc(ex)
        robot_srv_client.get_logger().info(ex)


if __name__ == '__main__':
    main()