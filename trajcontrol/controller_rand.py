import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from stage_control_interfaces.action import MoveStage
from trajcontrol.sensor_processing import INSERTION_STEP

FINAL_LENGTH = -20.0    # Expected insertion lenght
DEPTH_MARGIN = 1.5      # Final insertion length margin [mm]
SAFE_LIMIT = 5.0        # Maximum control output delta from entry point [mm]

class ControllerRand(Node):

    def __init__(self):
        super().__init__('controller_rand')

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning
        
        #Topics from sensor processing node
        self.subscription_tip = self.create_subscription(PoseStamped, '/sensor/tip', self.tip_callback, 10)
        self.subscription_tip  # prevent unused variable warning
        self.subscription_robot = self.create_subscription(PoseStamped, '/sensor/base', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Topics from UI (currently sensor processing node is doing the job)
        self.subscription_entry_point = self.create_subscription(PointStamped, '/subject/state/skin_entry', self.entry_callback, 10)
        self.subscription_entry_point  # prevent unused variable warning
        self.subscription_target = self.create_subscription(PointStamped, '/subject/state/target', self.target_callback, 10)
        self.subscription_target  # prevent unused variable warning

        #Published topics
        self.publisher_control = self.create_publisher(PointStamped, '/stage/control/cmd', 10)

        #Action client 
        #Check the correct action name and msg type from John's code
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        # Stored values
        self.entry_point = np.empty(shape=[0,3])    # Entry point
        self.target = np.empty(shape=[0,3])         # Target 
        self.tip = np.empty(shape=[0,3])            # Current needle tip x and z (from aurora)
        self.stage = np.empty(shape=[0,3])          # Current stage pose
        self.cmd = np.zeros((1,3))                  # Control output to the robot stage
        self.depth = 0.0
        self.robot_idle = True                      # Robot free to new command
   
    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        # Check if experiment is ready to begin (all topics received)
        # Only takes new inputs if robot finished previous action (robot IDLE)
        if (msg.data == 32) and (self.robot_idle == True) and (self.target.size != 0) and (self.tip.size != 0) and (self.entry_point.size != 0): # Hit SPACE and robot is free
            self.send_cmd()         # Calls routine to calculate and send new control signal

    # Get current entry point (only once)
    def entry_callback(self, msg):
        if (self.entry_point.size == 0):
            entry_point = msg.point
            self.entry_point = np.array([entry_point.x, entry_point.y, entry_point.z])

    # Get current target (only once)
    def target_callback(self, msg):
        if (self.target.size == 0):
            target = msg.point
            self.target = np.array([target.x, target.y, target.z])

    # Get current tip pose
    def tip_callback(self, msg):
        tip = msg.pose
        self.tip = np.array([tip.position.x, tip.position.y, tip.position.z])

    # Get current base pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        self.stage = np.array([robot.position.x, robot.position.y, robot.position.z])
        self.depth = robot.position.y

    # Send MoveStage action to robot
    def send_cmd(self):
        # Calculate control output
        new_rand = np.random.uniform(-SAFE_LIMIT, SAFE_LIMIT, 3)
        self.cmd = self.entry_point + new_rand

        # Test for stage limits
        self.cmd[0] = min(self.cmd[0], 0.0)
        self.cmd[0] = max(self.cmd[0], -90.0)
        self.cmd[2] = min(self.cmd[2], 90.0)
        self.cmd[2] = max(self.cmd[2], 0.0)

        # Prepare service message
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(self.cmd[0]*0.001)
        goal_msg.z = float(self.cmd[2]*0.001)
        goal_msg.eps = 0.0001
        self.robot_idle = False
        self.get_logger().info('Control: x=%f, z=%f' % (self.cmd[0], self.cmd[2]))

        # MoveStage call - send command to stage
        self.action_client.wait_for_server() # Waiting for action server        
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        # Publish control output
        msg = PointStamped()
        msg.point.x = float(self.cmd[0])
        msg.point.z = float(self.cmd[2])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_control.publish(msg)

    # Check if MoveStage action was accepted 
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Get MoveStage action finish message (Result)
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: %f, %f' %(result.x, result.z))
            # Check if max depth reached
            if (abs(self.tip[1]-self.target[1]) <= DEPTH_MARGIN): 
                self.robot_idle = False
                self.get_logger().info('ATTENTION: Depth margin reached! Please stop insertion')                
            else:
                self.robot_idle = True
                self.get_logger().info('Depth count: %.1fmm. Please insert %.1fmm more, then hit SPACE' % (self.stage[1], INSERTION_STEP))      
        else:
            self.get_logger().info('Goal failed with status: %s' %(result.status))

def main(args=None):
    rclpy.init(args=args)

    controller_rand = ControllerRand()

    rclpy.spin(controller_rand)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_rand.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()