import rclpy
import math
import numpy as np
import quaternion

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge.core import CvBridge

class Estimator(Node):

    def __init__(self):
        super().__init__('estimator')

        #Declare node parameters
        self.declare_parameter('alpha', 0.65) #Jacobian update parameter

        #Topics from sensor processing node
        self.subscription_tip = self.create_subscription(PoseStamped, '/sensor/tip', self.tip_callback, 10)
        self.subscription_tip # prevent unused variable warning

        #Topics from robot node
        self.subscription_base = self.create_subscription(PoseStamped, '/sensor/base', self.base_callback, 10)
        self.subscription_base # prevent unused variable warning

        #Published topics
        timer_period_jacobian = 0.3  # seconds
        self.timer = self.create_timer(timer_period_jacobian, self.timer_jacobian_callback)
        self.publisher_jacobian = self.create_publisher(Image, '/needle/state/jacobian', 10)
        
        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        # Initialize Jacobian with estimated values from previous experiments
        # (Alternative: initialize with values from first two sets of sensor and robot data)
        self.J = np.array([(0.9906,-0.1395,-0.5254),
                    ( 0.0588, 1.7334,-0.1336),
                    (-0.3769, 0.1906, 0.2970),
                    ( 0.0004,-0.0005, 0.0015),
                    ( 0.0058,-0.0028,-0.0015)])
        self.Z = np.empty(shape=[5,0])                  # Current needle tip pose Z = [x_tip, y_tip, z_tip, yaw, pitch] 
        self.X = np.empty(shape=[3,0])                  # Current needle base pose X = [x_robot, y_needle_depth, z_robot]
        self.Xant = np.empty(shape=[3,0])               # Previous X 
        self.Zant = np.empty(shape=[5,0])               # Previous Z 
        self.TX = self.get_clock().now().to_msg()       # Current X instant (time)
        self.TZ = self.get_clock().now().to_msg()       # Current Z instant (time)        
        self.TXant = self.TX                            # Previous X instant (time)
        self.TZant = self.TZ                            # Previous Z instant (time)
        
    # Get current needle tip from sensor processing node
    # Z = [x_tip, y_tip, z_tip, yaw, pitch]  
    def tip_callback(self, msg_tip):
        # Get filtered sensor in robot frame   
        quat = np.array([msg_tip.pose.orientation.w, msg_tip.pose.orientation.x, msg_tip.pose.orientation.y, msg_tip.pose.orientation.z])
        angles = get_angles(quat)
        self.get_logger().info('quat = %s' %  (quat))
        self.get_logger().info('angles: h = %f, v = %f [deg]' %  (math.degrees(angles[0]), math.degrees(angles[1])))
        self.Z = np.array([[msg_tip.pose.position.x, msg_tip.pose.position.y, msg_tip.pose.position.z, angles[0], angles[1]]]).T
        self.TZ = msg_tip.header.stamp
        if (self.Zant.size == 0):
            self.Zant = self.Z
            self.TZant = self.TZ

    # Get current base from sensor processing node
    # X = [x_robot, y_needle_depth, z_robot]
    def base_callback(self, msg_base):
        # Get robot x and z from PoseStamped
        # From robot, get input X
        self.X = np.array([[msg_base.pose.position.x, msg_base.pose.position.y, msg_base.pose.position.z]]).T
        self.TX = msg_base.header.stamp
        if (self.Xant.size == 0):
            self.Xant = self.X
            self.TXant = self.TX

    # Update Jacobian from current base inputs and tip pose
    def timer_jacobian_callback(self):
            # Calculate deltaTX and deltaTZ between current and previous values         
            deltaTX = ((self.TX.sec*1e9 + self.TX.nanosec) - (self.TXant.sec*1e9 + self.TXant.nanosec))*1e-9    
            deltaTZ = ((self.TZ.sec*1e9 + self.TZ.nanosec) - (self.TZant.sec*1e9 + self.TZant.nanosec))*1e-9    

            # Do Jacobian calculation only when current and previous are different
            if (deltaTX != 0) and (deltaTZ != 0):
                deltaX = (self.X - self.Xant)/deltaTX
                deltaZ = (self.Z - self.Zant)/deltaTZ
                # Update Jacobian
                alpha = self.get_parameter('alpha').get_parameter_value().double_value
                self.J = self.J + alpha*np.outer((deltaZ-np.matmul(self.J, deltaX))/(np.matmul(np.transpose(deltaX), deltaX)+1e-9), deltaX)

                # Publish new Jacobian
                msg = CvBridge().cv2_to_imgmsg(self.J)
                msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher_jacobian.publish(msg)

                # Save previous values for next estimation
                self.Zant = self.Z
                self.TZant = self.TZ
                self.Xant = self.X
                self.TXant = self.TX

########################################################################
### Auxiliar functions ###
########################################################################

# Function: get_angles(q)
# DO: Get needle angles in horizontal and vertical plane
# Inputs: 
#   q: quaternion (numpy array [qw, qx, qy, qz])
# Output:
#   angles: angle vector (numpy array [horiz, vert])
def get_angles(q):

    #Define rotation quaternion
    q_tf= np.quaternion(q[0], q[1], q[2], q[3])

    #Get needle current Z axis vector (needle insertion direction)
    u_z = np.quaternion(0, 0, 0, 1)
    v = q_tf*u_z*q_tf.conj()

    #Angles are components in x (horizontal) and z(vertical)
    horiz = math.atan2(v.x, -v.y)
    vert = math.atan2(v.z, math.sqrt(v.x**2+v.y**2))
    return np.array([horiz, vert])


########################################################################
def main(args=None):
    rclpy.init(args=args)

    estimator = Estimator()

    rclpy.spin(estimator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()