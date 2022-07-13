import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from stage_control_interfaces.action import MoveStage

FINAL_LENGTH = -20.0
DEPTH_MARGIN = 1.5   # Final insertion length margin [mm]

class ControllerRand(Node):

    def __init__(self):
        super().__init__('controller_rand')

        #Topics from sensor processing node
        self.subscription_entry_point = self.create_subscription(PoseStamped, '/subject/state/skin_entry', self.entry_callback, 10)
        self.subscription_entry_point  # prevent unused variable warning

        #Topics from robot node
        self.subscription_tip = self.create_subscription(PoseStamped, '/sensor/tip', self.robot_callback, 10)
        self.subscription_tip # prevent unused variable warning
        self.subscription_base = self.create_subscription(PoseStamped, '/sensor/base', self.robot_callback, 10)
        self.subscription_base # prevent unused variable warning

        #Published topics
        self.publisher_control = self.create_publisher(PointStamped, '/stage/control/cmd', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_move_robot)

        #Action client 
        #Check the correct action name and msg type from John's code
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        # Stored values
        self.entry_point = np.empty(shape=[2,0])    # Initial needle tip pose
        self.stage = np.empty(shape=[2,0])          # Current stage pose
        self.cmd = np.zeros((2,1))                  # Control output to the robot stage
        self.robot_idle = True                      # Robot free to new command
        self.depth = 0.0
   
    # Save entry point (only once)
    def entry_callback(self, msg):
        if (self.entry_point.size == 0):
            entry_point = msg.pose
            self.entry_point = np.array([[entry_point.position.x, entry_point.position.z]]).T

    # Get current base pose
    def robot_callback(self, msg_robot):
        # Save base pose only after getting entry point
        if (self.entry_point.size != 0):
            # Get pose from PoseStamped
            robot = msg_robot.pose
            # Get robot position
            self.stage = np.array([[robot.position.x, robot.position.z]]).T
            self.depth = robot.position.y
            # Check if max depth reached
            if (abs(self.depth-FINAL_LENGTH) <= DEPTH_MARGIN): 
                self.get_logger().info('Max depth: %f' % (self.depth))
                self.robot_idle = False

    # Move robot to new random position
    def timer_move_robot(self):
        # Send control signal only if robot is ready and after getting entry point (SPACE was hit by user)
        if (self.robot_idle == True) and (self.entry_point.size != 0):

            new_rand = np.random.uniform(-2, 2, (2,1))
            new_rand[1] = min(new_rand[1], 1.0)
            new_rand[1] = max(new_rand[1],-1.0)

            self.cmd = self.entry_point + new_rand

            # Send command to stage
            self.send_cmd(float(self.cmd[0]), float(self.cmd[1]))
            self.robot_idle = False

            self.get_logger().info('Control: x=%f, z=%f' % (self.cmd[0], self.cmd[1]))

            # Publish control output
            msg = PointStamped()
            msg.point.x = float(self.cmd[0])
            msg.point.z = float(self.cmd[1])
            msg.header.stamp = self.get_clock().now().to_msg()

            self.publisher_control.publish(msg)

    # Send MoveStage action to Stage node (Goal)
    def send_cmd(self, x, z):
        self.robot_idle = False
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(x*0.001)
        goal_msg.z = float(z*0.001)
        goal_msg.eps = 0.0001
        self.get_logger().debug('Send goal request... Control u: x=%f, z=%f' % ((goal_msg.x)*1000, (goal_msg.z)*1000))      

        self.action_client.wait_for_server()  # Waiting for action server   
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

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
            self.robot_idle = True                      #put robot in IDLE state
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.x*1000))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

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