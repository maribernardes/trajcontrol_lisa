import os
import rclpy
import numpy as np
import time
import keyboard
import numpy.matlib 
import quaternion

from rclpy.node import Node
from ros2_igtl_bridge.msg import Transform
from numpy import asarray, savetxt, loadtxt
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from scipy.ndimage import median_filter

INSERTION_STEP = 5.0    #5mm insertion step

class SensorProcessing(Node):

    def __init__(self):
        super().__init__('sensor_processing')

        #Topics from Aurora sensor node (OpenIGTLink Bridge)
        self.subscription_sensor = self.create_subscription(Transform, 'IGTL_TRANSFORM_IN', self.aurora_callback, 10)
        self.subscription_sensor # prevent unused variable warning

        #Topics from robot node (LISA robot)
        self.subscription_robot = self.create_subscription(PoseStamped, 'stage/state/pose', self.robot_callback, 10) #CAUTION: no '\'  before topic name
        self.subscription_robot # prevent unused variable warning

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning
        self.listen_keyboard = False

        #Published topics
        timer_period_entry = 0.8  # seconds
        self.timer = self.create_timer(timer_period_entry, self.timer_entry_point_callback)        
        self.publisher_entry_point = self.create_publisher(PoseStamped, '/subject/state/skin_entry', 10)

        timer_period_tip = 0.2 # seconds
        self.timer = self.create_timer(timer_period_tip, self.timer_tip_callback)
        self.publisher_tip = self.create_publisher(PoseStamped, '/sensor/tip', 10)

        # When keyboard SPACE is pressed (no timer)
        self.publisher_base = self.create_publisher(PoseStamped,'/sensor/base', 10)

        # Load registration from file
        try:
            self.registration = np.array(loadtxt(os.path.join(os.getcwd(),'src','trajcontrol','files','registration.csv'), delimiter=','))
        except IOError:
            self.get_logger().info('Could not find registration.csv file')

        #Stored values
        self.entry_point = np.empty(shape=[0,7])    # Tip position at begining of insertion

        self.auroraZ = np.empty(shape=[0,7])        # All stored Aurora tip readings as they are sent (for median filter)
        self.Z = np.empty(shape=[0,7])              # Current tip value (filtered) in robot frame

        self.stage = np.empty(shape=[0,2])          # Current stage pose
        self.depth = 0.0                            # Current insertion depth
        self.X = np.empty(shape=[0,3])              # Needle base position


    # Get current robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        # Get current robot position
        self.stage = np.array([robot.position.x*1000, robot.position.z*1000])

    # Get current Aurora sensor measurements
    def aurora_callback(self, msg_sensor):
        # Get needle tip from Aurora IGTL
        name = msg_sensor.name      
        if name=="NeedleToTracker": # Needle tip sensor (name is adjusted in PlusServer .xml file)
            # Get aurora new reading
            Z_new = np.array([[msg_sensor.transform.translation.x, msg_sensor.transform.translation.y, msg_sensor.transform.translation.z, \
                msg_sensor.transform.rotation.w, msg_sensor.transform.rotation.x, msg_sensor.transform.rotation.y, msg_sensor.transform.rotation.z]])
            # Filter and transform Aurora data only after registration was loaded from file
            if (self.registration.size != 0): 
                self.auroraZ = np.row_stack((self.auroraZ, Z_new))
                # Smooth the measurements with a median filter 
                n = self.auroraZ.shape[0]
                size_win = min(n, 500) #array window size
                if (size_win>0): 
                    Z_filt = median_filter(self.auroraZ[n-size_win:n,:], size=(40,1))   # use 40 samples median filter (column-wise)
                    Z_new = Z_filt[size_win-1,:]                                     # get last value
                # Transform from sensor to robot frame
                self.Z = pose_transform(Z_new, self.registration)

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        if (self.listen_keyboard == True) and (msg.data == 32): # If listerning to keyboard and hit SPACE key
            if (self.entry_point.size == 0):    # At needle entry point
                self.entry_point = self.Z       # Store entry point
                self.depth = 0.0
            else:
                self.depth = self.depth + INSERTION_STEP
            # Store current base value
            if (self.stage.size != 0):
                self.X = np.array([self.stage[0], self.depth, self.stage[1]])
                self.get_logger().info('X = %s' %(self.X))
                # Publish last base filtered pose in robot frame
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'stage'
                msg.pose.position = Point(x=self.X[0], y=self.X[1], z=self.X[2])
                msg.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
                self.publisher_base.publish(msg)

    def get_entry_point(self):
        # Get entry point if nothing was stored
        if (self.entry_point.size == 0 and self.listen_keyboard == False):
            #Listen to keyboard
            self.get_logger().info('Please, place the needle at the Entry Point and hit SPACE bar')
            self.listen_keyboard = True

    # Publishes entry point
    def timer_entry_point_callback(self):
        # Publishes only after experiment started (stored entry point is available)
        if (self.entry_point.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "stage"
            msg.pose.position = Point(x=self.entry_point[0], y=self.entry_point[1], z=self.entry_point[2])
            msg.pose.orientation = Quaternion(w=self.entry_point[3], x=self.entry_point[4], y=self.entry_point[5], z=self.entry_point[6])
            self.publisher_entry_point.publish(msg)

    # Publishes aurora readings filtered and transformed to robot frame
    def timer_tip_callback (self):
        # Publish last needle filtered pose in robot frame
        if (self.Z.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            msg.pose.position = Point(x=self.Z[0], y=self.Z[1], z=self.Z[2])
            msg.pose.orientation = Quaternion(w=self.Z[3], x=self.Z[4], y=self.Z[5], z=self.Z[6])
            self.publisher_tip.publish(msg)

########################################################################

# Function: pose_transform
# DO: Transform pose to new reference frame
# Inputs: 
#   x_origin: pose in original reference frame (numpy array [x, y, z, qw, qx, qy, qz])
#   x_tf: transformation from original to new frame (numpy array [x, y, z, qw, qx, qy, qz])
# Output:
#   x_new: pose in new reference frame (numpy array [x, y, z, qw, qx, qy, qz])
def pose_transform(x_orig, x_tf):

    #Define frame transformation
    p_tf = np.quaternion(0, x_tf[0], x_tf[1], x_tf[2])
    q_tf= np.quaternion(x_tf[3], x_tf[4], x_tf[5], x_tf[6])

    #Define original position and orientation
    p_orig = np.quaternion(0, x_orig[0], x_orig[1], x_orig[2])
    q_orig = np.quaternion(x_orig[3], x_orig[4], x_orig[5], x_orig[6])

    #Transform to new frame
    q_new = q_tf*q_orig
    p_new = q_tf*p_orig*q_tf.conj() + p_tf

    x_new = np.array([p_new.x, p_new.y, p_new.z, q_new.w, q_new.x, q_new.y, q_new.z])
    return x_new

########################################################################

def main(args=None):
    rclpy.init(args=args)

    sensor_processing = SensorProcessing()
    
    # Initialize entry point position
    while rclpy.ok():
        rclpy.spin_once(sensor_processing)
        if sensor_processing.entry_point.size == 0: #No entry point yet
            sensor_processing.get_entry_point()
        else:
            sensor_processing.get_logger().info('Entry Point in (%f, %f, %f)' %(sensor_processing.entry_point[0], sensor_processing.entry_point[1], sensor_processing.entry_point[2]))
            break

    rclpy.spin(sensor_processing)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_processing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
