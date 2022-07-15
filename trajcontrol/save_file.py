import rclpy
import os
import csv
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from ros2_igtl_bridge.msg import Transform
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from numpy import asarray


class SaveFile(Node):

    def __init__(self):
        super().__init__('save_file')
        
        #Declare node parameters
        self.declare_parameter('filename', 'my_data') #Name of file where data values are saved
        self.filename = os.path.join(os.getcwd(),'src','trajcontrol','data',self.get_parameter('filename').get_parameter_value().string_value + '.csv') #String with full path to file

        #Topics from aurora node (NeedleToTracker and BaseToTracker sensors)
        self.subscription_aurora = self.create_subscription(Transform, 'IGTL_TRANSFORM_IN', self.aurora_callback, 10)
        self.subscription_aurora # prevent unused variable warning

        #Topics from sensor processing node
        self.subscription_sensortip = self.create_subscription(PoseStamped, '/sensor/tip', self.sensortip_callback, 10)
        self.subscription_sensortip # prevent unused variable warning

        self.subscription_sensorbase = self.create_subscription(PoseStamped, '/sensor/base', self.sensorbase_callback, 10)
        self.subscription_sensorbase # prevent unused variable warning

        #Topics from UI (currently sensor processing is doing the job)
        self.subscription_entry = self.create_subscription(PointStamped, '/subject/state/skin_entry', self.entry_point_callback, 10)
        self.subscription_entry  # prevent unused variable warning

        self.subscription_target = self.create_subscription(PointStamped, '/subject/state/target', self.target_callback, 10)
        self.subscription_target  # prevent unused variable warning

        #Topics from estimator_node
        self.subscription_estimator = self.create_subscription(Image, '/needle/state/jacobian', self.estimator_callback, 10)
        self.subscription_estimator  # prevent unused variable warning

        #Topics from controller_node
        self.subscription_controller = self.create_subscription(PointStamped, '/stage/control/cmd', self.control_callback, 10)
        self.subscription_controller  # prevent unused variable warning

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PoseStamped, 'stage/state/pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Published topics
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.write_file_callback)

        #Array of data
        header = ['Timestamp sec', 'Timestamp nanosec', \
            'Target x', 'Target y', 'Target z', ' Target sec', ' Target nanosec', \
            'Entry_point x', 'Entry_point y', 'Entry_point z', ' Entry_point sec', ' Entry_point nanosec', \
            'AuroraTip x', 'AuroraTip y', 'AuroraTip z', 'AuroraTip qw', 'AuroraTip qx', 'AuroraTip qy', 'AuroraTip qz', 'AuroraTip sec', 'AuroraTip nanosec', \
            'Tip x', 'Tip y', 'Tip z', 'Tip qw', 'Tip qx', 'Tip qy', 'Tip qz', 'Tip sec', 'Tip nanosec', \
            'AuroraBase x', 'AuroraBase y', 'AuroraBase z', 'AuroraBase qw', 'AuroraBase qx', 'AuroraBase qy', 'AuroraBase qz', 'AuroraBase sec', 'AuroraBase nanosec', \
            'Base x', 'Base y', 'Base z', 'Base qw', 'Base qx', 'Base qy', 'Base qz', 'Base sec', 'Base nanosec',
            'J00', 'J01', 'J02', \
            'J10', 'J11', 'J12', \
            'J20', 'J21', 'J22', \
            'J30', 'J31', 'J32', \
            'J40', 'J41', 'J42', 'J sec', 'J nanosec', \
            'Control x', 'Control y', 'Control z', 'Control sec', 'Control nanosec', \
            'Robot x', 'Robot y', 'Robot z', 'Robot sec', 'Robot nanosec'
        ]
        
        with open(self.filename, 'w', newline='', encoding='UTF8') as f: # open the file in the write mode
            writer = csv.writer(f)  # create the csv writer
            writer.writerow(header) # write a row to the csv file

        #Last data received
        self.target = [0,0,0, 0,0]                  #target point + sec nanosec
        self.entry_point = [0,0,0, 0,0]             #skin entry point + sec nanosec
        self.aurora_tip = [0,0,0,0,0,0,0, 0,0]      #aurora tip data + sec nanosec
        self.aurora_base = [0,0,0,0,0,0,0, 0,0]     #aurora base data + sec nanosec
        self.Z = [0,0,0,0,0,0,0, 0,0]       #/sensor/tip_filtered (filtered and transformed to robot frame)
        self.X = [0,0,0,0,0,0,0, 0,0]       #/robot/needle_pose (transformed to robot frame)
        self.J = [0,0,0, \
                  0,0,0, \
                  0,0,0, \
                  0,0,0, \
                  0,0,0]            #Jacobian matrix
        self.Jtime = [0,0]          #Jacobian sec nanosec
        self.cmd = [0,0,0, 0,0]       #Control output (x,y,z) + sec nanosec
        self.stage = [0,0,0, 0,0]     #Robot output (x,y,z) + sec nanosec
        self.get_logger().info('Log data will be saved at %s' %(self.filename))   


    #Get current target
    def target_callback(self, msg):
        self.target = [msg.point.x, msg.point.y, msg.point.z, int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]

    #Get current entry_point
    def entry_point_callback(self, msg):
        self.entry_point = [msg.point.x, msg.point.y, msg.point.z, int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]

    #Get Aurora data
    def aurora_callback(self, msg):
        aurora = msg.transform
        now = self.get_clock().now().to_msg()
        if msg.name=="NeedleToTracker": # Name is adjusted in Plus .xml
            self.aurora_tip = [aurora.translation.x, aurora.translation.y, aurora.translation.z, \
                aurora.rotation.w, aurora.rotation.x, aurora.rotation.y, aurora.rotation.z, int(now.sec), int(now.nanosec)]
        if msg.name=="BaseToTracker": # Name is adjusted in Plus .xml
            self.aurora_base = [aurora.translation.x, aurora.translation.y, aurora.translation.z, \
                aurora.rotation.w, aurora.rotation.x, aurora.rotation.y, aurora.rotation.z, int(now.sec), int(now.nanosec)]

    #Get current Z (filtered and in robot frame)
    def sensortip_callback(self, msg):
        tip = msg.pose
        self.Z = [tip.position.x, tip.position.y, tip.position.z, \
            tip.orientation.w, tip.orientation.x, tip.orientation.y, tip.orientation.z, int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]
        #self.get_logger().info('Received Z = %s in %s frame' % (self.Z, msg.header.frame_id))

    #Get current X
    def sensorbase_callback(self, msg):
        base = msg.pose
        self.X = [base.position.x, base.position.y, base.position.z, \
            base.orientation.w, base.orientation.x, base.orientation.y, base.orientation.z, int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]
        #self.get_logger().info('Received X = %s in %s frame' % (self.X, msg.header.frame_id))
        
    #Get current J
    def estimator_callback(self,msg):
        self.J = np.array(CvBridge().imgmsg_to_cv2(msg)).flatten()
        self.Jtime = [int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]

    #Get current control output
    def control_callback(self,msg):
        self.cmd = [msg.point.x, msg.point.y, msg.point.z, int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]        

    #Get current robot pose
    def robot_callback(self,msg):
        self.stage = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]     

    #Save data do file
    def write_file_callback(self):
        now = self.get_clock().now().to_msg()
       
        data = [now.sec, now.nanosec, \
            self.target[0], self.target[1], self.target[2], self.target[3], self.target[4],\
            self.entry_point[0], self.entry_point[1], self.entry_point[2], self.entry_point[3], self.entry_point[4],\
            self.aurora_tip[0], self.aurora_tip[1], self.aurora_tip[2], self.aurora_tip[3], self.aurora_tip[4], self.aurora_tip[5], self.aurora_tip[6], self.aurora_tip[7], self.aurora_tip[8], \
            self.Z[0], self.Z[1], self.Z[2], self.Z[3], self.Z[4], self.Z[5], self.Z[6], self.Z[7], self.Z[8], \
            self.aurora_base[0], self.aurora_base[1], self.aurora_base[2], self.aurora_base[3], self.aurora_base[4], self.aurora_base[5], self.aurora_base[6], self.aurora_base[7], self.aurora_base[8],\
            self.X[0], self.X[1], self.X[2], self.X[3], self.X[4], self.X[5], self.X[6], self.X[7], self.X[8],\
            self.J[0], self.J[1], self.J[2], \
            self.J[3], self.J[4], self.J[5], \
            self.J[6], self.J[7], self.J[8], \
            self.J[9], self.J[10], self.J[11], \
            self.J[12], self.J[13], self.J[14], self.Jtime[0] , self.Jtime[1], \
            self.cmd[0], self.cmd[1], self.cmd[2], self.cmd[3], self.cmd[4], \
            self.stage[0], self.stage[1], self.stage[2], self.stage[3], self.stage[4] \
            ]
        
        with open(self.filename, 'a', newline='', encoding='UTF8') as f: # open the file in append mode
            writer = csv.writer(f) # create the csv writer
            writer.writerow(data)  # append a new row to the existing csv file     

def main(args=None):
    rclpy.init(args=args)

    save_file = SaveFile()

    rclpy.spin(save_file)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    save_file.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
