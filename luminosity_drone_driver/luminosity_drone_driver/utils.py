#!/usr/bin/env python3

"""
Utilities to run the drone
"""
from itertools import cycle

import rclpy
from msg_srv.msg import RCMessage
from yamspy import MSPy

CMDS_ORDER = ["roll", "pitch", "throttle", "yaw", "aux1", "aux2", "aux3", "aux4"]


class FCDriver():
    def __init__(self, board):
        #super().__init__('fc_driver')
        self.board = board

        command_list = [
            "MSP_API_VERSION",
            "MSP_FC_VARIANT",
            "MSP_FC_VERSION",
            "MSP_BUILD_INFO",
            "MSP_BOARD_INFO",
            "MSP_UID",
            "MSP_ACC_TRIM",
            "MSP_NAME",
            "MSP_STATUS",
            "MSP_STATUS_EX",
            "MSP_BATTERY_CONFIG",
            "MSP_BATTERY_STATE",
            "MSP_BOXNAMES",
        ]
        if self.board.INAV:
            command_list.append("MSPV2_INAV_ANALOG")
            command_list.append("MSP_VOLTAGE_METER_CONFIG")

        for msg in command_list:
            if self.board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                dataHandler = self.board.receive_msg()
                self.board.process_recv_data(dataHandler)

    '''def read_data(self):
        # Read info from the FC
        slow_msgs = ["MSP_ANALOG", "MSP_STATUS_EX", "MSP_MOTOR", "MSP_RC"]
        diagnostics = {}
        for msg in slow_msgs:
            if self.board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                dataHandler = self.board.receive_msg()
                self.board.process_recv_data(dataHandler)
            if msg == "MSP_ANALOG":
                diagnostics['voltage'] = self.board.ANALOG["voltage"]
            elif msg == "MSP_STATUS_EX":
                diagnostics['armed'] = self.board.bit_check(self.board.CONFIG["mode"], 0)
                diagnostics['armingDisableFlags'] = self.board.process_armingDisableFlags(self.board.CONFIG['armingDisableFlags'])
                diagnostics['cpuload'] = self.board.CONFIG['cpuload']
                diagnostics['cycleTime'] = self.board.CONFIG['cycleTime']
                diagnostics['mode'] = self.board.CONFIG['mode']
                diagnostics['flightMode'] = self.board.process_mode(self.board.CONFIG['mode'])
            elif msg == "MSP_MOTOR":
                diagnostics['motorValues'] = self.board.MOTOR_DATA
            elif msg == "MSP_RC":
                diagnostics['rcChannelsValues'] = self.board.RC['channels']
        return diagnostics'''


    def reboot(self):
        self.board.reboot()

    def push_data(self, CMDS):
        if self.board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
            dataHandler = self.board.receive_msg()
            self.board.process_recv_data(dataHandler)


    def read_data(self):
        msgs = ['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC']

        cell_count = 2  # 3S battery
        warn_voltage = 3.4 * cell_count
        min_voltage = 3.3 * cell_count

        for msg in msgs:
            if self.board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                dataHandler = self.board.receive_msg()
                self.board.process_recv_data(dataHandler)

            if self.board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                dataHandler = self.board.receive_msg()
                self.board.process_recv_data(dataHandler)

            if msg == 'MSP_ANALOG':
                voltage = self.board.ANALOG['voltage']
                voltage_msg = ""
                if min_voltage < voltage <= warn_voltage:
                    voltage_msg = "LOW BATT WARNING"
                elif voltage <= min_voltage:
                    voltage_msg = "ULTRA LOW BATT!!!"

                self.get_logger().info("Battery Voltage: {:2.2f}V".format(self.board.ANALOG['voltage']))
                self.get_logger().info(voltage_msg)

            elif msg == 'MSP_STATUS_EX':
                ARMED = self.board.bit_check(self.board.CONFIG['mode'],0)
                self.get_logger().info("ARMED: {}".format(ARMED))

                self.get_logger().info("armingDisableFlags: {}".format(self.board.process_armingDisableFlags(self.board.CONFIG['armingDisableFlags'])))

                self.get_logger().info("cpuload: {}".format(self.board.CONFIG['cpuload']))
                self.get_logger().info("cycleTime: {}".format(self.board.CONFIG['cycleTime']))

                self.get_logger().info("mode: {}".format(self.board.CONFIG['mode']))

                self.get_logger().info("Flight Mode: {}".format(self.board.process_mode(self.board.CONFIG['mode'])))


            elif msg == 'MSP_MOTOR':
                self.get_logger().info("Motor Values: {}".format(self.board.MOTOR_DATA))

            elif msg == 'MSP_RC':
                self.get_logger().info("RC Channels Values: {}".format(self.board.RC['channels']))
