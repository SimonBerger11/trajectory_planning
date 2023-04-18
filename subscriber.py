#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from autoware_msgs.msg import Lane, Waypoint
import std_msgs.msg


lane_msg = Lane
global i
i = 0


def callback(data):
    
    position_x = data.pose.position.x
    position_y = data.pose.position.y
    position_z = data.pose.position.z
    orientation_x = data.pose.orientation.x
    orientation_y = data.pose.orientation.y
    orientation_z = data.pose.orientation.z
    orientation_w = data.pose.orientation.w

    rospy.loginfo("Updated Pose")

def listener():
  
    rospy.init_node('pose_node', anonymous=True)

    rospy.Subscriber("/current_pose", PoseStamped, callback)

    rospy.spin()

def talker():

    global i
    pub = rospy.Publisher('/waypoints_pub', Lane, queue_size=10)
    rospy.init_node('wp_pub', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        header = std_msgs.msg.Header()
        header.seq = 13
        header.stamp = rospy.Time.now()
        header.frame_id = 13
        lane_msg.header = header

        lane_msg.increment = i
        i += 1 
        lane_msg.lane_id = 4711
        # defining first waypoint
        first_wp = Waypoint()
        first_wp.gid = 13
        first_wp.lid = 13
        pose_message = PoseStamped()
        pose_message.header = header # same header as on the top
        pose_message.pose.position.x = 13
        pose_message.pose.position.y = 13
        pose_message.pose.position.z = 13
        pose_message.pose.orientation.x = 13
        pose_message.pose.orientation.y = 13
        pose_message.pose.orientation.z = 13
        pose_message.pose.orientation.w = 13
        twist_message = TwistStamped()
        twist_message.header = header # same header as on the top
        twist_message.twist.linear.x = 13
        twist_message.twist.linear.y = 13
        twist_message.twist.linear.z = 13
        twist_message.twist.angular.x=13
        twist_message.twist.angular.y=13
        twist_message.twist.angular.z =13
        first_wp.pose = pose_message
        first_wp.twist=twist_message 
        first_wp.dtlane = 13
        first_wp.change_flag = 13
        first_wp.wpstate.aid = 13
        #first_wp.wpstate.NULLSTATE = 13
        first_wp.wpstate.lanechange_state = 13
        first_wp.wpstate.steering_state = 13
        #first_wp.wpstate.STR_LEFT = 1
        #first_wp.wpstate.STR_RIGHT = 2
        #first_wp.wpstate.STR_STRAIGHT = 3
        #first_wp.wpstate.STR_BACK = 4
        first_wp.wpstate.accel_state = 13
        #first_wp.wpstate.TYPE_STOPLINE=1
        #first_wp.wpstate.TYPE_STOP=2
        first_wp.wpstate.event_state = 13
        #first_wp.wpstate.TYPE_EVENT_NULL=0
        #first_wp.wpstate.TYPE_EVENT_GOAL = 1
        #first_wp.wpstate.TYPE_EVENT_MIDDLE_GOAL=2
        #first_wp.wpstate.TYPE_EVENT_POSITION_STOP =3
        #first_wp.wpstate.TYPE_EVENT_BUS_STOP=4
        #first_wp.wpstate.TYPE_EVENT_PARKING=5
        first_wp.lane_id = 13
        first_wp.left_lane_id = 13
        first_wp.right_lane_id = 13
        first_wp.stop_line_id = 13
        first_wp.cost = 13
        first_wp.time_cost = 13
        first_wp.direction = 13
        # end of defining a waypoint
        waypointarray = [first_wp]
        lane_msg.waypoints = waypointarray 
        lane_msg.lane_index = 0
        lane_msg.cost =0
        lane_msg.closest_object_distance=0
        lane_msg.closest_object_velocity=0
        lane_msg.is_blocked=False
        
        pub.publish(lane_msg)
        rate.sleep()


if __name__ == '__main__':
    listener() 
    try:
        talker()
        #listener()
    except rospy.ROSInterruptException:
        pass