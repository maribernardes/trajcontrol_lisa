import rclpy
import numpy as np
import math

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import Int8
from stage_control_interfaces.action import MoveStage
from trajcontrol.sensor_processing import INSERTION_STEP

SAFE_LIMIT = 5.0    # Maximum control output delta from entry point [mm]
DEPTH_MARGIN = 1.5   # Final insertion length margin [mm]

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        #Declare node parameters
        self.declare_parameter('K', 0.001) #Controller gain

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning
        self.listen_keyboard = False

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PoseStamped, '/sensor/base', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Topics from estimator node
        self.subscription_estimator = self.create_subscription(PoseStamped, '/needle/state/jacobian', self.robot_callback, 10)
        self.subscription_estimator # prevent unused variable warning

        #Topics from sensor processing node
        self.subscription_tip = self.create_subscription(PoseStamped, '/sensor/tip', self.tip_callback, 10)
        self.subscription_tip  # prevent unused variable warning

        #Topics from UI (currently sensor processing node is doing the job)
        self.subscription_entry_point = self.create_subscription(PointStamped, '/subject/state/skin_entry', self.entry_callback, 10)
        self.subscription_entry_point  # prevent unused variable warning
        self.subscription_target = self.create_subscription(PointStamped, '/subject/state/target', self.target_callback, 10)
        self.subscription_target  # prevent unused variable warning

        #Topics from estimator node
        self.subscription_jacobian = self.create_subscription(PoseStamped, '/needle/state/jacobian', self.tip_callback, 10)
        self.subscription_jacobian  # prevent unused variable warning

        #Action client 
        #Check the correct action name and msg type from John's code
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        # Stored values
        self.entry_point = np.empty(shape=[0,3])    # Entry point
        self.target = np.empty(shape=[0,3])         # Target 
        self.tip = np.empty(shape=[0,3])            # Current needle tip x and z (from aurora)
        self.stage = np.empty(shape=[0,3])          # Current stage pose
        self.cmd = np.zeros((1,3))                  # Control output to the robot stage
        self.depth = 0.0                            # Current insertion depth
        self.robot_idle = True                     # Stage move action status
        self.J = np.array([(0.9906,-0.1395,-0.5254),
                    ( 0.0588, 1.7334,-0.1336),
                    (-0.3769, 0.1906, 0.2970),
                    ( 0.0004,-0.0005, 0.0015),
                    ( 0.0058,-0.0028,-0.0015)])

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        # Check if experiment is ready to begin (tip and target received)
        # Only takes new inputs if robot finished previous action (robot IDLE)
        if (msg.data == 32) and (self.robot_idle == True) and (self.target.size != 0) and (self.tip.size != 0): # Hit SPACE and robot is free
            if (abs(self.tip[1]-self.target[1]) <= DEPTH_MARGIN): 
                self.get_logger().info('ATTENTION: Depth margin reached! Please stop insertion')                
            else:
                self.send_cmd()         # Calls routine to calculate and send new control signal

    # Get current entry point
    def entry_callback(self, msg):
        entry_point = msg.point
        self.entry_point = np.array([entry_point.x, entry_point.y, entry_point.z])

    # Get current target
    def target_callback(self, msg):
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

    # Calculates control output and send MoveStage action to robot
    def send_cmd(self):
        Jc = self.J[0:3,:]

        K = self.get_parameter('K').get_parameter_value().double_value        # Get K value          
        error = self.target - self.tip                                       # Calculate control error
        deltaX = self.stage + K*np.matmul(np.linalg.pinv(Jc), error)          # Calculate control output

        self.cmd = self.stage + deltaX
        # Limit control output to maximum SAFE_LIMIT[mm] around entry point
        self.cmd[0] = min(self.cmd[0], self.entry_point[0]+SAFE_LIMIT)
        self.cmd[2] = min(self.cmd[2], self.entry_point[2]+SAFE_LIMIT)
        self.cmd[0] = max(self.cmd[0], self.entry_point[0]-SAFE_LIMIT)
        self.cmd[2] = max(self.cmd[2], self.entry_point[2]-SAFE_LIMIT)

        self.get_logger().info('Applying trajectory compensation... DO NOT insert the needle now\nTip: (%f, %f, %f) \nTarget: (%f, %f, %f) \nError: (%f, %f, %f) \nDeltaX: (%f, %f)' % (self.tip[0],\
             self.tip[1], self.tip[2], self.target[0], self.target[1], self.target[2], error[0], error[1], error[2], deltaX[0], deltaX[2]))    
   

        # REMOVE AFTER TESTS
        self.cmd[0] = 0.0
        self.cmd[2] = 0.0

        # Send command to stage
        self.robot_idle = False
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(self.cmd[0]*0.001)
        goal_msg.z = float(self.cmd[2]*0.001)
        goal_msg.eps = 0.0001

        self.action_client.wait_for_server() # Waiting for action server        
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
            self.get_logger().debug('Goal succeeded! Result: {0}'.format(result.x))
            self.robot_idle = True
            self.get_logger().info('Depth count: %.1fmm. Please insert %.1fmm more, then hit SPACE' % (self.stage[1], INSERTION_STEP))      
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
