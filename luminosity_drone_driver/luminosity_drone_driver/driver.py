#!/usr/bin/env python3

"""
Driver for the luminosity drone
"""

# standard imports
import sys
import threading
import time
from timeit import default_timer as timer

# third-party imports
import rclpy
from rclpy.node import Node
from msg_srv.msg import RCMessage
from msg_srv.srv import CommandBool
from yamspy import MSPy

# app imports
from .utils import FCDriver

SERIAL_PORT = "/dev/ttyS0"
BAUDRATE = 500000
LOGLEVEL = "WARNING"
ELAPSED_TIME_FOR_AUTO_DISARM_ON_INACTIVITY = 1

DEFAULT_ROLL_VALUE = 1500
DEFAULT_PITCH_VALUE = 1500
DEFAULT_YAW_VALUE = 1500
DEFAULT_THROTTLE_VALUE = 900

MSP_LOCK = threading.Lock()


print("Staring script...")


class DroneDriver(Node):
    def __init__(self, board):
        super().__init__('luminosity_drone')

        self.board = board
        self.PUSH_TO_FC_COUNT = 0
        self.PUSH_TO_FC_START = timer()

        self.CMDS = {
            "roll": DEFAULT_ROLL_VALUE,
            "pitch": DEFAULT_PITCH_VALUE,
            "throttle": DEFAULT_THROTTLE_VALUE,
            "yaw": DEFAULT_YAW_VALUE,
            "aux1": 900,
            "aux2": 1500,
            "aux3": 1500,
            "aux4": 1500,
        }

        self.get_logger().info(f"Node started /{'luminosity_drone'}")

        self.last_msg_received_time = self.get_clock().now().nanoseconds

        self.rc_sub = self.create_subscription(
            RCMessage,
            "/luminosity_drone/rc_command",
            self.rc_command_topic_callback,
            10
        )

        self.arming_srv = self.create_service(
            CommandBool, "/luminosity_drone/cmd/arming", self.arming_service_callback
        )

        self.reboot_srv = self.create_service(
            CommandBool, "/luminosity_drone/cmd/reboot", self.reboot_service_callback
        )

        
        try:
            if board == 1:
                reason = "Unable to connect with board. Check your SERIAL_PORT"
                raise ValueError(reason)

            self.FC = FCDriver(self.board)
            print("connected")

        except Exception as err:
            self.get_logger().error(str(err))
            sys.exit(1)
    
    def __del__(self):
        self.disarm()

    def rc_command_topic_callback(self, msg):
        self.last_msg_received_time = self.get_clock().now().nanoseconds

        self.CMDS["roll"] = msg.rc_roll
        self.CMDS["pitch"] = msg.rc_pitch
        self.CMDS["throttle"] = msg.rc_throttle
        self.CMDS["yaw"] = msg.rc_yaw

        self.get_logger().info(str(self.CMDS))

    def read_diagnostic_data(self, req):
        diagnostics = self.FC.read_data()
        self.get_logger().info(str(diagnostics))

    def reboot_service_callback(self, request, response):
        if request.value:
            self.FC.reboot()
        # TODO: return appropriate error values
        response.success = True
        response.result = 0
        return response
    
    def arming_service_callback(self, request, response):
        self.last_msg_received_time = self.get_clock().now().nanoseconds
        if request.value:
            self.arm()
        else:
            self.disarm()
        # TODO: return appropriate error values
        response.success = True
        response.result = 0
        return response

    def shutdown_hook(self):
        self.get_logger().info("Calling shutdown hook")
       # self.disarm()

    def arm(self):
        self.get_logger().info("Arming drone")
        self.CMDS["roll"] = DEFAULT_ROLL_VALUE
        self.CMDS["pitch"] = DEFAULT_PITCH_VALUE
        self.CMDS["throttle"] = DEFAULT_THROTTLE_VALUE
        self.CMDS["yaw"] = DEFAULT_YAW_VALUE
        self.CMDS["aux1"] = 1800
        self.get_logger().info("Drone armed")
        self.get_logger().info(str(self.CMDS))
        # self.read_from_fc()

    def reboot_fc(self):
        self.get_logger().info("Reboot drone")
        self.FC.reboot()

    def disarm(self):
        self.get_logger().info("Disarming drone")
        self.CMDS["aux1"] = 1000
        self.push_to_fc(None)

    def push_to_fc(self, event):
        # if MSP_LOCK.locked():
        #     return
        # MSP_LOCK.acquire()
        if hasattr(self, 'FC'):
            self.FC.push_data(self.CMDS)
        self.PUSH_TO_FC_COUNT += 1
        time_diff = timer() - self.PUSH_TO_FC_START
        if timer() - self.PUSH_TO_FC_START >= 10:
            self.get_logger().info(f"Average Rate of Pushing to FC: {self.PUSH_TO_FC_COUNT / time_diff}")
            self.PUSH_TO_FC_START = timer()
            self.PUSH_TO_FC_COUNT = 0
        # MSP_LOCK.release()


    def read_from_fc(self):
        #MSP_LOCK.acquire()
        if hasattr(self, 'FC'):
            self.FC.read_data(self.CMDS)
        #MSP_LOCK.release()


    @property
    def is_armed(self):
        arm_status = self.board.bit_check(self.board.CONFIG["mode"], 0)
        #self.get_logger().debug("ARMED: %s", arm_status)
        return arm_status


def main(args=None):
    rclpy.init()
    print("""
Interfacing with FC. If it is stuck on this line, potentially it is unable to communicate with the FC. In such a case:

1. check your baudrate
2. check if your batteries are connected to the FC
    """)
    with MSPy(device=SERIAL_PORT, loglevel=LOGLEVEL, baudrate=BAUDRATE) as board:
        luminosity_drone = DroneDriver(board)
        luminosity_drone.create_rate(100)
        

        while rclpy.ok():
            luminosity_drone.read_from_fc()
            luminosity_drone.push_to_fc(event=None)
            
            if luminosity_drone.is_armed:
                time_diff = (
                    luminosity_drone.get_clock().now().nanoseconds - luminosity_drone.last_msg_received_time
                )
                if time_diff > ELAPSED_TIME_FOR_AUTO_DISARM_ON_INACTIVITY:
                    luminosity_drone.get_logger().info("Inactivity detected")
                    luminosity_drone.disarm()
            rclpy.spin_once(luminosity_drone)
                 
          
            
    luminosity_drone.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

