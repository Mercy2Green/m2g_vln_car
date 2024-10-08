

class Livox_laser_interface(object):
    def __init__(self, laser_name, pub_laser_command_name="/livox/lidar", sub_laser_odom_name="/livox/odom"):
        self.laser_name = laser_name
        self.odom = Pose()
        self.odom_sub = rospy.Subscriber("/livox/odom", Odometry, self.odom_callback)
        self.pub_laser_command = rospy.Publisher(
            name=pub_laser_command_name,
            data_class=Twist,
            queue_size=1,
        )
        self.sub_laser_odom = rospy.Subscriber(
            name=sub_laser_odom_name,
            data_class=Odometry,
            callback=self.laser_odom_cb,
        )
        rospy.sleep(0.5)
        print("Initialized Livox_laser_interface!\n")

    def laser_odom_cb(self, msg):
        self.odom = msg.pose.pose

    def odom_callback(self, msg):
        self.odom = msg.pose.pose

    def move_laser(self, linear_speed, angular_speed):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.pub_laser_command.publish(twist)
        rospy.sleep(0.1)

    def move_laser_for(self, linear_speed, angular_speed, duration):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.pub_laser_command.publish(twist)
        rospy.sleep(duration)
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub_laser_command.publish(twist)
        rospy.sleep(0.1)

    def stop_laser(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub_laser_command.publish(twist)
        rospy.sleep(0.1)

    def get_laser_odom(self):
        return self.odom