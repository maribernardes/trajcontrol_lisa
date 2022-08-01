from curses import keyname
import rclpy
import numpy as np
import time
import math

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PointStamped, Point
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from stage_control_interfaces.action import MoveStage
from scipy.optimize import minimize
from trajcontrol.sensor_processing import INSERTION_STEP

SAFE_LIMIT = 6.0    # Maximum control output delta from entry point [mm]
DEPTH_MARGIN = 1.5   # Final insertion length margin [mm]

class ControllerMPC(Node):

    def __init__(self):
        super().__init__('controller_mpc')

        #Declare node parameters
        # self.declare_parameter('P', 10) #Prediction Horizon
        # self.declare_parameter('C', 3)  #Control Horizon
        self.declare_parameter('insertion_length', -100.0) #Insertion length parameter

        #Topics from sensor processing node
        self.subscription_tip = self.create_subscription(PoseStamped, '/sensor/tip', self.tip_callback, 10)
        self.subscription_tip  # prevent unused variable warning
        self.subscription_robot = self.create_subscription(PoseStamped, '/sensor/base', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Topics from sensor processing node
        #Topics from UI (currently sensor processing node is doing the job)
        self.subscription_target = self.create_subscription(PointStamped, '/subject/state/target', self.target_callback, 10)
        self.subscription_target  # prevent unused variable warning

        #Topics from estimator node
        self.subscription_estimator = self.create_subscription(Image, '/needle/state/jacobian', self.jacobian_callback, 10)
        self.subscription_estimator  # prevent unused variable warning

        #Action client 
        #Check the correct action name and msg type from John's code
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        #Published topics
        self.publisher_control = self.create_publisher(PointStamped, '/stage/control/cmd', 10)

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        # Stored values
        self.tip = np.empty(shape=[0,3])            # Current needle tip x and z (from aurora)
        self.stage_initial = np.empty(shape=[0,3])  # Stage initial position
        self.stage = np.empty(shape=[0,3])          # Current stage position
        self.target = np.empty(shape=[0,3])         # Target 
        self.cmd = np.zeros((1,3))                  # Control output to the robot stage
        self.depth = 0.0                            # Current insertion depth
        self.robot_idle = True                      # Stage move action status
        self.Jc = np.zeros((3,3))

        # self.P = self.get_parameter('P').get_parameter_value().double_value      # Get P value          
        # self.C = self.get_parameter('C').get_parameter_value().double_value      # Get C value          
        # self.get_logger().info('MPC paramenters for this trial: P = %f, C = %f' %(self.P, self.C))
        self.insertion_length = self.get_parameter('insertion_length').get_parameter_value().double_value
        self.ns = math.floor(self.insertion_length/INSERTION_STEP)
        self.get_logger().info('MPC horizon for this trial: H = %f' %(self.ns))

    # Get current base pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        self.stage = np.array([robot.position.x, robot.position.y, robot.position.z])
        self.depth = robot.position.y
        if (self.stage_initial.size == 0):
            self.stage_initial = np.array([robot.position.x, robot.position.y, robot.position.z])
            self.get_logger().info('Stage initial: (%f, %f, %f) ' % (self.stage_initial[0], self.stage_initial[1], self.stage_initial[2]))
            # Control output limits
            limit_x = (float(self.stage_initial[0])-SAFE_LIMIT, float(self.stage_initial[0])+SAFE_LIMIT)
            limit_z = (float(self.stage_initial[2])-SAFE_LIMIT, float(self.stage_initial[2])+SAFE_LIMIT)
            self.limit = [limit_x, limit_z]

    # Get current target (only once)
    def target_callback(self, msg):
        if (self.target.size == 0):
            target = msg.point
            self.target = np.array([target.x, target.y, target.z])

    # Get current tip pose
    def tip_callback(self, msg):
        tip = msg.pose
        self.tip = np.array([tip.position.x, tip.position.y, tip.position.z])

    # Get current Jacobian matrix from Estimator node
    def jacobian_callback(self, msg):
        J = np.asarray(CvBridge().imgmsg_to_cv2(msg))
        self.Jc = J[0:3,:]
       
        # Send control signal only if robot is ready and after first readings (target and current needle tip)
        if (self.robot_idle == True) and (self.target.size != 0) and (self.tip.size != 0):
            self.send_cmd()

    def send_cmd(self):
    ########################################################################
                ## MPC Functions
    ########################################################################
        # Define process model
        # y = y0 + J(u-u0)
        # where y = tip, u = base
        def process_model(y0, u0, u, Jc):
            delta_u = np.array([float(u[0]-u0[0]), INSERTION_STEP, float(u[1]-u0[1])])
            y = y0 + np.matmul(Jc,delta_u)
            return y

        # Define objective function
        def objective(u_hat):
            # Reshape u_hat (minimize flattens it)
            H = math.floor(u_hat.size/2)
            u_hat = np.reshape(u_hat,(H,2), order='C')
            # # Repeat last control output until covers prediction horizon
            # u_hat = np.vstack((u_hat, np.repeat([u_hat[H-1]], P-C, axis=0)))
            y_hat = np.zeros((H,3))

            # Initialize prediction
            y_hat0 = self.tip
            u_hat0 = self.stage

            # Simulate prediction horizon
            for k in range(0,H):
                yp = process_model(y_hat0, u_hat0, u_hat[k], self.Jc)
                # Save predicted variable
                y_hat[k] = yp
                # Update initial condition for next prediction step
                y_hat0 = y_hat[k]
                u_hat0 = u_hat[k]

            # Minimization objectives
            wu = 0.2
            tg_xz = np.array([self.target[0],self.target[2]])   # Build tg without depth
            y_hat_xz = np.array([y_hat[:,0],y_hat[:,2]])        # Build y_hat and tg without depth

            obj1 = np.linalg.norm(tg_xz-y_hat_xz)               # Tip error to target
            obj2 = np.sum(np.amax(np.absolute(u_hat), axis=0))  # Total base displacement around entry point
            obj = obj1 + wu*obj2
            return obj 

    ########################################################################

        # Calculate error to target
        error = self.tip - self.target  

        # MPC Initialization
        H = self.ns - math.floor(self.depth/INSERTION_STEP)    # Horizon size
        if(self.cmd.size == 0):
            u0 = np.array([self.stage_initial[0], self.stage_initial[2]])
        else:
            u0 = np.array([self.cmd[0], self.cmd[2]])
        u_hat = np.tile(u0, (H,1))   # Initial control guess using last cmd value (vector with remaining horizon size)

        # Initial objective
        self.get_logger().info('Initial SSE Objective: %f' % (objective(u_hat)))  # calculate cost function with initial guess

        # MPC calculation
        start_time = time.time()
        solution = minimize(objective, u_hat, method='SLSQP', bounds=self.limit*H)    # optimizes the objective function
        u = np.reshape(solution.x,(H,2), order='C')                                 # reshape solution (minimize flattens it)
        end_time = time.time()
        
        cost = objective(u)
        self.get_logger().info('Final SSE Objective: %f' % (cost)) # calculate cost function with optimization result
        self.get_logger().info('Elapsed time: %f' % (end_time-start_time))
            
        # Update controller output
        self.cmd[0] = u[0]
        self.cmd[1] = self.cmd[1]+INSERTION_STEP
        self.cmd[2] = u[1]

        ## Keeping this just to be on the safe side (but should not be necessary)
        # Limit control output to maximum SAFE_LIMIT[mm] around entry stage_initial
        self.cmd[0] = min(self.cmd[0], self.stage_initial[0]+SAFE_LIMIT)
        self.cmd[0] = max(self.cmd[0], self.stage_initial[0]-SAFE_LIMIT)
        self.cmd[2] = min(self.cmd[2], self.stage_initial[2]+SAFE_LIMIT)
        self.cmd[2] = max(self.cmd[2], self.stage_initial[2]-SAFE_LIMIT)

        # Test for stage limits
        self.cmd[0] = min(self.cmd[0], 0.0)
        self.cmd[0] = max(self.cmd[0], -90.0)
        self.cmd[2] = min(self.cmd[2], 90.0)
        self.cmd[2] = max(self.cmd[2], 0.0)

        # Print values
        self.get_logger().info('Applying trajectory compensation... DO NOT insert the needle now\nTip: (%f, %f, %f) \
            \nTarget: (%f, %f, %f) \nError: (%f, %f, %f) \nDeltaU: (%f, %f)  \nCmd: (%f, %f) \nStage: (%f, %f)' % (self.tip[0],\
            self.tip[1], self.tip[2], self.target[0], self.target[1], self.target[2], error[0], error[1], error[2],\
            u[0] - self.stage[0], u[1] - self.stage[2], self.cmd[0], self.cmd[2], self.stage[0], self.stage[2]))    

        # Send command to stage
        self.robot_idle = False
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(self.cmd[0]*0.001)
        goal_msg.z = float(self.cmd[2]*0.001)
        goal_msg.eps = 0.0001

        self.action_client.wait_for_server() # Waiting for action server        
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        # Publish cmd
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stage"
        msg.point = Point(x=self.cmd[0], y=self.cmd[1], z=self.cmd[2])
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
            self.get_logger().info('Goal succeeded! Result: %f, %f' %(result.x*1000, result.z*1000))
            self.get_logger().info('Tip: (%f, %f, %f)'   % (self.tip[0], self.tip[1], self.tip[2]))
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

    controller_mpc = ControllerMPC()

    # global P
    # global C
    # P = controller_mpc.get_parameter('P').get_parameter_value().integer_value
    # C = controller_mpc.get_parameter('C').get_parameter_value().integer_value

    rclpy.spin(controller_mpc)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_mpc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()