import rclpy
from rclpy.node import Node
import libddsbridge as bridge
from rclpy.executors import MultiThreadedExecutor

from autoware_vehicle_msgs.msg import ControlModeReport
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_vehicle_msgs.msg import VelocityReport
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from autoware_perception_msgs.msg import DetectedObjects

TopicList = [
    ( '/vehicle/status/control_mode', ControlModeReport,"r",1024),
    ( '/vehicle/status/steering_status', SteeringReport,"r",1024),
    ( '/vehicle/status/velocity_status', VelocityReport,"r",1024),
    ( '/sensing/radar/detected_objects',DetectedObjects,"r",102400),  
    ( '/control/command/control_cmd',Control,"s",1024),
    ( '/control/command/turn_indicators_cmd', TurnIndicatorsCommand,"s",1024),
    
]


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()

    nodes = []
    for topic_name, topic_type, node_type, dlen in TopicList:
        node = bridge.CreateNode(topic_name, topic_type, node_type,dlen)
        nodes.append(node)
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

