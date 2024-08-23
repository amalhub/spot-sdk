#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math
import time
import threading

import bosdyn.client
import bosdyn.client.util
import bosdyn.api.power_pb2 as PowerServiceProto
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.client.power import PowerClient

# ... (keep other necessary imports)

class SpotInterface(object):
    def __init__(self, robot):
        self._robot = robot
        self._lease_client = robot.ensure_client(LeaseClient.default_service_name)
        self._estop_client = robot.ensure_client(EstopClient.default_service_name)
        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        
        self._lease_keepalive = None
        self._estop_keepalive = None

    def start(self):
        self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True, return_at_exit=True)
        self._estop_endpoint = EstopEndpoint(self._estop_client, 'RosSpotClient', 9.0)
        self._estop_endpoint.force_simple_setup()
        self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)

    def shutdown(self):
        if self._estop_keepalive:
            self._estop_keepalive.shutdown()
        if self._lease_keepalive:
            self._lease_keepalive.shutdown()

    def power_on(self):
        request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
        self._power_client.power_command_async(request)

    def move_robot(self, v_x, v_y, v_rot):
        command = RobotCommandBuilder.synchro_velocity_command(
            v_x=v_x, v_y=v_y, v_rot=v_rot)
        self._robot_command_client.robot_command(command=command,
                                                 end_time_secs=time.time() + 0.5)

class RosSpotBridge(object):
    def __init__(self, robot):
        self.spot_interface = SpotInterface(robot)
        self.spot_interface.start()
        self.spot_interface.power_on()

        rospy.Subscriber("/spot/command", Twist, self.command_callback)

    def command_callback(self, msg):
        v_x = msg.linear.x
        v_y = msg.linear.y
        v_rot = msg.angular.z
        self.spot_interface.move_robot(v_x, v_y, v_rot)

    def shutdown(self):
        self.spot_interface.shutdown()

def main():
    import argparse

    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args()

    sdk = bosdyn.client.create_standard_sdk('RosSpotBridge')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    rospy.init_node('ros_spot_bridge')
    bridge = RosSpotBridge(robot)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()

if __name__ == '__main__':
    main()