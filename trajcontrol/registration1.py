import os
import rclpy
import numpy as np
import time
import numpy.matlib 
import quaternion

from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from stage_control_interfaces.action import MoveStage

from rclpy.node import Node
from ros2_igtl_bridge.msg import Transform
from numpy import asarray, savetxt, loadtxt
# from std_msgs.msg import Int8
# from geometry_msgs.msg import PoseStamped, Point, Quaternion
from scipy.ndimage import median_filter

class Registration(Node):

    def __init__(self):
        super().__init__('registration')

        #Topics from Aurora sensor node
        self.subscription_sensor = self.create_subscription(Transform, 'IGTL_TRANSFORM_IN', self.aurora_callback, 10)
        self.subscription_sensor # prevent unused variable warning

        #Action client 
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        # Stored values
        self.robot_idle = True                      # Stage move action status
        self.current_point = 0                      # Current registration point
        self.transform = np.empty(shape=[0,7])      # Registration transform (from aurora to stage)
        self.auroraZ = np.empty(shape=[0,7])        # All stored Aurora tip readings as they are sent
        self.Z_sensor = np.empty(shape=[0,7])       # Aurora tip sensor value as sent
        self.Z = np.empty(shape=[0,7])              # Filtered aurora tip value in robot frame

        # Registration points A (aurora) and B (stage)
        ###############################################################################
        ### ATTENTION: Define B: Registration points in the stage frame             ###
        ### B = np.array([[x0, x1, ..., xn], [y0, y1, ..., yn], [z0, z1, ..., zn]]) ###
        ###############################################################################
        #Registration points in aurora frame
        self.A = np.empty(shape=[3,0])          
        #Registration points in stage frame
        self.B = np.array([[0, 0, 0, 0, -20, -20, -20, -20, -40, -40, -40, -40, -60, -60, -60, -60, -80, -80, -80, -80],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                           [0, 2.5, 5, 7.5, 7.5, 5, 2.5, 0, 0, 2.5, 5, 7.5, 7.5, 5, 2.5, 0, 0, 2.5, 5, 7.5]])   

    # Get current Aurora sensor measurements and filters
    def aurora_callback(self, msg_sensor):
        # Get needle shape from Aurora IGTL
        name = msg_sensor.name      
        if name=="NeedleToTracker": # Name is adjusted in Plus .xml
            # Get aurora new reading
            self.Z_sensor = np.array([[msg_sensor.transform.translation.x, msg_sensor.transform.translation.y, msg_sensor.transform.translation.z, \
                msg_sensor.transform.rotation.w, msg_sensor.transform.rotation.x, msg_sensor.transform.rotation.y, msg_sensor.transform.rotation.z]])
            self.auroraZ = np.row_stack((self.auroraZ, self.Z_sensor))

            # Smooth the measurements with a median filter 
            n = self.auroraZ.shape[0]
            size_win = min(n, 500) #array window size
            if (size_win>0): 
                Z_filt = median_filter(self.auroraZ[n-size_win:n,:], size=(40,1))   # use 40 samples median filter (column-wise)
                self.Z_sensor = Z_filt[size_win-1,:]                                # get last value   

    def get_registration(self):  
        # Get points until A is same size as B
        if (self.A.shape[1] < self.B.shape[1]): 
            self.get_next_point()             
        else:
            # Calculate registration transform
            self.transform = np.array(find_registration(self.A, self.B))   #Store registration transform
            # Save matrix to file
            self.get_logger().debug('Save registration transform %s' %(self.transform))
            savetxt(os.path.join(os.getcwd(),'src','trajcontrol','files','registration.csv'), asarray(self.transform), delimiter=',')
            regitration_points = np.row_stack((self.A, self.B))
            savetxt(os.path.join(os.getcwd(),'src','trajcontrol','files','registration_points.csv'), asarray(regitration_points), delimiter=',')
            # Send robot back to home position
            self.get_logger().info('Returning robot to home position... %s' %(self.transform))
            self.robot_idle = False
            self.send_cmd(0.0, 0.0)

    def get_next_point(self):
        # Robot finished moving
        if (self.robot_idle == True):
            # New point reached but not stored
            if (self.A.shape[1] < self.current_point):
                time_end = self.get_clock().now()
                delta_time = time_end - self.time_begin
                # Waited a little for Aurora to converge
                if (delta_time.nanoseconds*1e-9 >= 3.0):
                    if (self.Z_sensor.size == 0):
                        self.get_logger().info('There is no sensor reading to store. Check Aurora')
                    else:
                        P = self.Z_sensor[0:3] # Store next registration point
                        self.get_logger().info('Stored A Point #%i = (%f, %f, %f)' % (self.current_point, P[0], P[1], P[2]))
                        self.A = np.column_stack((self.A, P.T))
            # Select next point
            else:
                # Get next B point
                P = self.B[:, self.A.shape[1]]
                # Send robot to next point
                self.current_point = self.current_point +1
                self.send_cmd(P[0], P[2])          
                self.get_logger().info('Sending robot to B Point #%i = (%f, %f, %f)' % (self.current_point, P[0], P[1], P[2]))

    # Send MoveStage action to Stage
    def send_cmd(self, x, z):

        # Send command to stage (convert mm to m)
        self.robot_idle = False
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(x*0.001)
        goal_msg.z = float(z*0.001)
        goal_msg.eps = 0.0001
        self.get_logger().info('Send goal request... Control u: x=%f, z=%f' % (x, z))

        # Wait for action server
        self.action_client.wait_for_server()
        
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        goal_msg.x = goal_msg.x*1000
        goal_msg.z = goal_msg.z*1000
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
            self.robot_idle = True
            self.time_begin = self.get_clock().now()
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.x*1000))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

                                 
########################################################################
### Auxiliar functions ###
########################################################################

# Function: find_registration
# DO: From two sets of N 3D points in two different reference frames, find the best fit
#       in the LS-sense for the transformation between them (translation and rotation in quaternion)
# Inputs: 
#   A: set of N 3D points in first frame (numpy array 3xN)
#   B: set of N 3D points in second frame (numpy array 3xN)
# Output:
#   x_reg: transformation from first to second frame (numpy array [x, y, z, qw, qx, qy, qz])
def find_registration(A, B):
    [d, n] = np.shape(A)

    #Mean Center Data
    Ac = np.mean(A.T,axis=0)
    Bc = np.mean(B.T,axis=0)
    A = A - np.matlib.repmat(Ac[:,None], 1, n)
    B = B - np.matlib.repmat(Bc[:,None], 1, n)

    #Calculate Optimal Rotation
    M = np.matmul(A, B.T)
    N = np.array([[M[0,0]+M[1,1]+M[2,2], M[1,2]-M[2,1],        M[2,0]-M[0,2],        M[0,1]-M[1,0]],\
                  [M[1,2]-M[2,1],        M[0,0]-M[1,1]-M[2,2], M[0,1]+M[1,0],        M[2,0]+M[0,2]],\
                  [M[2,0]-M[0,2],        M[0,1]+M[1,0],        M[1,1]-M[0,0]-M[2,2], M[1,2]+M[2,1]],\
                  [M[0,1]-M[1,0],        M[2,0]+M[0,2],        M[1,2]+M[2,1],        M[2,2]-M[0,0]-M[1,1]]])
    [w,v]=np.linalg.eig(N)
    ind=np.argmax(w)
    q = v[:,ind]                                                                #Rotation quaternion
    R = (q[0]**2-np.inner(q[1:4],q[1:4]))*np.eye(3) + 2*np.outer(q[1:4],q[1:4]) + \
        2*q[0]*np.array([[0,-q[3],q[2]],[q[3],0,-q[1]],[-q[2],q[1],0]])         #Rotation matrix

    #Calculate Optimal Translation
    t = Bc - np.matmul(R,Ac)

    x_reg = [t[0], t[1], t[2], q[0], q[1], q[2], q[3]] #final registration transform
    return x_reg

########################################################################

def main(args=None):
    rclpy.init(args=args)

    registration = Registration()

    # Initialize registration
    registration.get_logger().info('Registration Node' %  (registration.transform))
    while rclpy.ok():
        rclpy.spin_once(registration)
        if (registration.transform.size == 0): #No registration yet
            registration.get_registration()
        else:
            if (registration.robot_idle == True):
                registration.get_logger().info('Registration procedure successfully completed! Press Ctrl + C to quit' )
                input()
                break
   
    rclpy.spin(registration)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    registration.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
