#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Vector3, Pose, Quaternion
from autoware_msgs.msg import Lane, Waypoint, DTLane #, WPState
import std_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray



global numberofwaypoints
numberofwaypoints = 3


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
    global numberofwaypoints
    i=0
    pub = rospy.Publisher('/waypoints_pub', Lane, queue_size=10)
    rospy.init_node('wp_pub', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    lane_msg = Lane()
    while not rospy.is_shutdown():
        header = std_msgs.msg.Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        lane_msg.header = header
        lane_msg.increment = i
        i += 1 
        lane_msg.lane_id = 10
        waypointarray = []
        for j in range(numberofwaypoints): 
            # defining first waypoint
            first_wp = Waypoint()
            first_wp.gid = j
            first_wp.lid = j
            pose_message = PoseStamped()
            pose_message.header = header # same header as on the top
            pose_message.pose.position.x = 57
            pose_message.pose.position.y = -11
            pose_message.pose.position.z = -0.2
            pose_message.pose.orientation.x = 0.0
            pose_message.pose.orientation.y = 0.0
            pose_message.pose.orientation.z = 0.999359580809
            pose_message.pose.orientation.w = 0.0357830720618
            twist_message = TwistStamped()
            twist_message.header = header # same header as on the top
            twist_message.twist.linear.x = 0.833333333
            twist_message.twist.linear.y = 0.0
            twist_message.twist.linear.z = 0.0
            twist_message.twist.angular.x=0.0
            twist_message.twist.angular.y=0.0
            twist_message.twist.angular.z =0.0
            first_wp.pose = pose_message
            first_wp.twist=twist_message 
            dt_lane = DTLane()
            dt_lane.dist = 0.0
            dt_lane.dir = 0.0
            dt_lane.apara = 0.0
            dt_lane.r = 0.0
            dt_lane.slope = 0.0 
            dt_lane.cant = 0.0
            dt_lane.lw = 0.0
            dt_lane.rw = 0.0
            first_wp.dtlane = dt_lane
            first_wp.change_flag = 0
            #wp_state= WPState()
            #wp_state.aid = 0
            #wp_state.lanechange_state= 0
            #wp_state.steering_state= 3
            #wp_state.accel_state=0
            #wp_state.stop_state=0
            #wp_state.event_state =0
            #first_wp.wpstate = wp_state
            #first_wp.wpstate.NULLSTATE = 13
            #first_wp.wpstate.STR_LEFT = 1
            #first_wp.wpstate.STR_RIGHT = 2
            #first_wp.wpstate.STR_STRAIGHT = 3
            #first_wp.wpstate.STR_BACK = 4
                #first_wp.wpstate.accel_state = 13
            #first_wp.wpstate.TYPE_STOPLINE=1
            #first_wp.wpstate.TYPE_STOP=2
                #first_wp.wpstate.event_state = 13
            #first_wp.wpstate.TYPE_EVENT_NULL=0
            #first_wp.wpstate.TYPE_EVENT_GOAL = 1
            #first_wp.wpstate.TYPE_EVENT_MIDDLE_GOAL=2
            #first_wp.wpstate.TYPE_EVENT_POSITION_STOP =3
            #first_wp.wpstate.TYPE_EVENT_BUS_STOP=4
            #first_wp.wpstate.TYPE_EVENT_PARKING=5
            first_wp.lane_id = 8
            first_wp.left_lane_id = 0
            first_wp.right_lane_id = 0
            first_wp.stop_line_id = 4294967295
            first_wp.cost = 0.0
            first_wp.time_cost = 0.0
            first_wp.direction = 0
            # end of defining a waypoint
            waypointarray.append(first_wp)
        lane_msg.waypoints = waypointarray 
        lane_msg.lane_index = 0
        lane_msg.cost =0
        lane_msg.closest_object_distance=0
        lane_msg.closest_object_velocity=0
        lane_msg.is_blocked=False
        
        pub.publish(lane_msg)
        rate.sleep()

def marker():
    global numberofwaypoints
    i=0
    pub = rospy.Publisher('/global_waypoints_rviz2', MarkerArray, queue_size=10)
    rospy.init_node('wp_pub_rviz', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    markerarray_msg = MarkerArray()
    marker_msg = Marker()
    while not rospy.is_shutdown():
        header = std_msgs.msg.Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        marker_msg.header = header
        marker_msg.ns = "global_velocity_lane_1"
        marker_msg.id = 0
        marker_msg.type = 0
        marker_msg.action = 0

        pose_message = Pose()
        pose_message.position.x = 0
        pose_message.position.y = 0
        pose_message.position.z = -0.2
        pose_message.orientation.x = 0.0
        pose_message.orientation.y = 0.0
        pose_message.orientation.z = 0.999359580809
        pose_message.orientation.w = 0.0357830720618
        marker_msg.pose = pose_message

        vector_message = Vector3()
        vector_message.x = 10
        vector_message.y = 1
        vector_message.z = 1
        marker_msg.scale = vector_message

        color_message = std_msgs.msg.ColorRGBA()
        color_message.r = 1.0
        color_message.g = 1.0
        color_message.b = 1.0
        color_message.a = 0.5
        marker_msg.color = color_message

        #marker_msg.lifetime = rospy.Time.now()
        marker_msg.frame_locked = False
        marker_msg.points = []
        marker_msg.colors  =[]
        marker_msg.text = "TEST"
        marker_msg.mesh_resource = "TEST"
        marker_msg.mesh_use_embedded_materials = False
        markerarray_msg = [marker_msg]
        pub.publish(markerarray_msg)
        rate.sleep()

if __name__ == '__main__':
    #listener() 
    try:
        #talker()
        marker()
        #listener()
    except rospy.ROSInterruptException:
        pass