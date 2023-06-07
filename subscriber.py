########################################################################
#FileName: subscriber.py
#Author : Christian Göller, Jonas Bäuml, Simon Berger, Elias Häring 
#Last Modified On : 09.06.2023
##Description : Trajectory planner with start and end position for searching a parking space 
########################################################################

#!/usr/bin/env python
# coding=utf-8
import rospy
import time
import math
from pylab import *
import std_msgs.msg
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Vector3, Pose, Quaternion
from autoware_msgs.msg import Lane, LaneArray,  Waypoint, DTLane 
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt
import parking_parser
import Subscriber_Goal



global goal_position
goal_position = (0,0,0)
global current_position
current_position = (0,0,0,0,0,0,0)
global wp_array_or
wp_array_or={}
global start_point
start_point = (44, 10.5401,0)
global osmfile
osmfile = "Parkhaus_2.osm" 

# listen to current position of vehicle
def current_pos_callback(data):
    global current_position, start_point
    position_x = data.pose.position.x
    position_y = data.pose.position.y
    position_z = data.pose.position.z
    orientation_x = data.pose.orientation.x
    orientation_y = data.pose.orientation.y
    orientation_z = data.pose.orientation.z
    orientation_w = data.pose.orientation.w
    current_position = (position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w)
    start_point = (position_x, position_y, 0)                           # current position is start point for trajectory planner

# initialize the trajectory planner when a goal position is given from rviz
def goal_pos_callback(data):
    global goal_position, start_point, wp_array_or,osmfile
    position_x = data.pose.position.x
    position_y = data.pose.position.y
    position_z = data.pose.position.z
    goal_position = (position_x, position_y, position_z)
    result_no_orientation = astar(start_point,goal_position,osmfile)     # trajecotry is planned 
    wp_array_or = addOrientation(result_no_orientation)                  # wp_array_or contains array of waypoints for trajectory 
    marker_rviz()                                                        # waypoints are visualised in rviz 


# subscriber for goal position 
def goal():
    rospy.init_node('goal_node', anonymous=True)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_pos_callback)
    #rospy.Subscriber("/current_pose", PoseStamped, current_pos_callback)
    rospy.spin()


# publisher for publishing waypoint array of trajectory to vehicle car 
def wp_publisher():
    global wp_array_or           
    incre=0
    pub2 = rospy.Publisher('/waypoints', Lane, queue_size=10)
    rate = rospy.Rate(1)        # 1hz
    lane_array_msg = LaneArray()
    # begin LaneArray Message
    lane_array_msg.id=0
    lane_msg = Lane()
    while not rospy.is_shutdown():
        # begin Lane Message
        header = std_msgs.msg.Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        lane_msg.header = header
        lane_msg.increment = incre
        incre += 1 
        lane_msg.lane_id = 10
        waypointarray = []
        for wp in wp_array_or: 
            # begin Waypoint
            first_wp = Waypoint()
            first_wp.gid = zaehler
            first_wp.lid = zaehler
            pose_message = PoseStamped()
            pose_message.header = header        # same header as on the top
            pose_message.pose.position.x = wp["x"]
            pose_message.pose.position.y = wp["y"]
            pose_message.pose.position.z = wp["z"]
            pose_message.pose.orientation.x = wp["qx"]
            pose_message.pose.orientation.y = wp["qy"]
            pose_message.pose.orientation.z = wp["qz"]
            pose_message.pose.orientation.w = wp["qw"]
            twist_message = TwistStamped()
            twist_message.header = header       # same header as on the top
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
            first_wp.lane_id = 8
            first_wp.left_lane_id = 0
            first_wp.right_lane_id = 0
            first_wp.stop_line_id = 312
            first_wp.cost = 0.0
            first_wp.time_cost = 0.0
            first_wp.direction = 0
            # end Waypoint
            waypointarray.append(first_wp)
            rate.sleep()
        lane_msg.waypoints = waypointarray 
        lane_msg.lane_index = 0
        lane_msg.cost =0
        lane_msg.closest_object_distance=0
        lane_msg.closest_object_velocity=0
        lane_msg.is_blocked=False
        # end Lane Message
        lane_array_msg.lanes.append(lane_msg)
        # end LaneArray Message
        pub2.publish(lane_array_msg)
        break

# publisher for visualisizing waypoint array of trajectory to rviz
def marker_rviz():
    global wp_array_or
    count=0
    pub = rospy.Publisher('/global_waypoints_rviz2', MarkerArray, queue_size=10)
    rate = rospy.Rate(150) # (1/150) hz
    # begin MarkerArray Message
    markerarray_msg = MarkerArray()
    while not rospy.is_shutdown():
        for wp in wp_array_or:
            #Begin Marker Message
            marker_msg = Marker()
            header = std_msgs.msg.Header()
            header.seq = 0
            header.stamp = rospy.Time.now()
            header.frame_id = "world"
            marker_msg.header = header
            marker_msg.ns = "global_velocity_lane_1"
            marker_msg.id = count
            marker_msg.type = 0
            marker_msg.action = 0
            pose_message = Pose()
            pose_message.position.x = wp["x"]
            pose_message.position.y = wp["y"]
            pose_message.position.z = wp["z"]
            pose_message.orientation.x = wp["qx"]
            pose_message.orientation.y = wp["qy"]
            pose_message.orientation.z = wp["qz"]
            pose_message.orientation.w = wp["qw"]
            marker_msg.pose = pose_message
            # defining the size of arrows 
            vector_message = Vector3()
            vector_message.x = 1.0
            vector_message.y = 0.4
            vector_message.z = 0.4
            marker_msg.scale = vector_message
            # defining color of arrows
            color_message = std_msgs.msg.ColorRGBA()
            color_message.r = 1.0
            color_message.g = 0.0
            color_message.b = 0.0
            color_message.a = 1.0
            marker_msg.color = color_message
            marker_msg.frame_locked = False
            marker_msg.points = []
            marker_msg.colors  =[]
            marker_msg.text = "TEST"
            marker_msg.mesh_resource = "TEST"
            marker_msg.mesh_use_embedded_materials = False
            #end Marker Message
            markerarray_msg.markers.append(marker_msg)
            rate.sleep()
            count=count+1
            pub.publish(markerarray_msg)
        # end MarkerArray Message
        break


# convert euler angels (x,y,z) to quaterions 
def get_quaternion_from_euler(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  return [qx, qy, qz, qw]


class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.costStart = 0
        self.minimumCost = 0
        self.sum = 0
        
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

fig = plt.figure()
ax = fig.add_subplot(111)        
way_dict = parking_parser.parse_osm("Parkhaus_2.osm");
way = []
for wa in way_dict:
    way.append([])
    for w in wa:
        #print(type(len(way)-1))
        way[len(way)-1].append((float(w["x"]), float(w["y"]),0))#, float(w["z"])))

way_x = []
way_y = []
for cnt,w in enumerate(way):
    if cnt != 17:        #nur fürs testen
        for wl in w:
            way_x.append(wl[0])
            way_y.append(wl[1])
scatter(way_x, way_y, color= "blue", s= 10)


# Funktion zur Berechnung des nächsten Punktes auf der Mittellinie
def next_point(point, point_list):
    distance = 1000
    point_ruck = None
    i = 0
    for pl1 in point_list:
        if len(pl1) == 0:
            continue
        for p in pl1:
            d1 = math.sqrt((point.x - p[0]) ** 2 + (point.y - p[1]) ** 2 + (point.z - p[2]) ** 2)
            if d1 < distance:
                distance = d1
                point_ruck = Node(p[0],p[1],p[2])
    return point_ruck


def astar(start, end, way_name):
    open_list = []      # Alle noch zu untersuchenden nodes
    closed_list = []    # Alle schon untersuchten nodes
    way_dict = parking_parser.parse_osm(way_name);
    way_p = []
    for wa in way_dict:
        way_p.append([])
        for w in wa:
            way_p[len(way_p)-1].append((float(w["x"]), float(w["y"]),0))#, float(w["z"])))
    start_node1 = Node(start[0], start[1], start[2])
    end_node1 = Node(end[0], end[1], end[2])
    start_node = next_point(start_node1, way_p)
    end_node = next_point(end_node1, way_p)
    # start_node wird erster knoten in der open_list
    open_list.append(start_node)
    cnt = 0
    while len(open_list) >0:
        # Suche des nächsten current_node mit der kleinsten Gesamtsumme
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.sum < current_node.sum:           
                current_node = item
                current_index = index

        ax.scatter(current_node.x, current_node.y, color = "red", s = 10)
        open_list.pop(current_index)
        closed_list.append(current_node)
        # Wenn Ziel gefunden wurde dann Weg zurückgeben
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                ax.scatter(current.x, current.y, color = "green", s = 20)
                #plt.pause(0.2)
                #path.append((current.x, current.y, current.z))
                path.insert(0,({"x": current.x, "y": current.y, "z": current.z}))
                current = current.parent
            return path
        # Nächste nodes berechnen
        children = []
        current_koord = (current_node.x, current_node.y, current_node.z)
        # Alle Linien die der Punkt berührt und die Stelle in der jeweiligen Linie berechnen
        currentLine = []
        for cnt, wp in enumerate(way_p):
            for cnt1, wp1 in enumerate(wp):
                if current_koord == wp1:
                    currentLine.append((cnt,cnt1)) 
        # nächste nodes berechnen 
        for cl in currentLine:
            if (cl[1]+ 1) > len(way_p[cl[0]])-1 :
                continue
            next_pos = way_p[cl[0]][cl[1]+1] 
            next_node = Node(next_pos[0], next_pos[1], next_pos[2])
            next_node.parent = current_node
            children.append(next_node)
        # Prüfen ob children schon behandelt wurden, wenn nicht -> Hinzufügen zur Open_List
        for count, child in enumerate(children):
            flag = False
            for closed in closed_list:
                if child.x == closed.x and child.y == closed.y:
                    flag = True
                    continue    
            if count <4:
                child.costStart = current_node.costStart + 1
            else:
                child.costStart = current_node.costStart + 1.4
            for open in open_list:
                if child.x == open.x and child.y == open.y and child.costStart >= open.costStart:
                    flag = True
                    break
            if flag == True:
                continue
            #child.costStart = current_node.costStart + 1
            child.minimumCost = math.sqrt((child.x - end_node.x) ** 2 + (child.y - end_node.y) ** 2 + (child.z - end_node.z) ** 2)
            child.sum = child.costStart + child.minimumCost
            for open_node in open_list:
                if child == open_node and child.costStart > open_node.costStart:
                    continue
            #ax.scatter(child.x,child.y, color = "orange", s = 10)
            open_list.append(child)
        #plt.pause(0.0001)
        cnt += 1        # for visualization
        #fig.canvas.draw()
    return None


# Add orientation in q to the following waypoint
# Last waypoint has same orientation as second last waypoint
def addOrientation(waypoints):
    pointctr = 0
    for point in waypoints:
        pointctr += 1
        # Last two points have the same orientation
        if pointctr >= len(waypoints):
            point["qx"] = waypoints[pointctr-2]["qx"]
            point["qy"] = waypoints[pointctr-2]["qy"]
            point["qz"] = waypoints[pointctr-2]["qz"]
            point["qw"] = waypoints[pointctr-2]["qw"]
        else:
            following_point = waypoints[pointctr]
            #Angel counter clockwise 
            yaw_euler_angel = math.atan2(following_point["y"]-point["y"], following_point["x"]-point["x"])
            quaternions = get_quaternion_from_euler(0,0, yaw_euler_angel)
            print(yaw_euler_angel)
            point["qx"] = quaternions[0]
            point["qy"] = quaternions[1]
            point["qz"] = quaternions[2]
            point["qw"] = quaternions[3]
    return waypoints



if __name__ == '__main__':
    try:
        goal()       
    except rospy.ROSInterruptException:
        pass