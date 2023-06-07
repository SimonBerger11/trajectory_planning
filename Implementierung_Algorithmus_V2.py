import math
from pylab import *
import matplotlib.pyplot as plt
#import analyse_map as analyse
#import csv_reader
import parking_parser

# Class Node
# attributes: x,y,z: coordinates
# parent: predecessor point
# costStart: cost to reach this point
# minimumCost: distance to the endpoint
# sum: sum of costStart and minimumCost
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

# For testing the functionality
'''
fig = plt.figure()
ax = fig.add_subplot(111)

        
start_point = (44, 10.5401,0)
end_point = (16, 17.417,0)

scatter(start_point[0], start_point[1], color = "orange", s = 40)
scatter(end_point[0], end_point[1], color = "orange", s = 40)

way_dict = parking_parser.parse_osm("Parkhaus_2.osm")       
way = []
for wa in way_dict:   
    way.append([])   
    for w in wa:     
        way[len(way)-1].append((float(w["x"]), float(w["y"]),0))#, float(w["z"])))

# plotten aller Wege    
way_x = []
way_y = []
for cnt,w in enumerate(way):
    
    if cnt != 17:       # nur fürs testen
        
        for wl in w:
            way_x.append(wl[0])
            way_y.append(wl[1])

scatter(way_x, way_y, color= "blue", s= 10)

'''


# Function for calculation the next point on the center line
# input: point (class Node) and point_list (output of the osm-file)
# Output: Node
# Functionality: iterating over all points in the point_list, calculate distance to input point, if the distance is less
# the current smallest distance then the current point becomes the point ruck, after iteration over all points ->
# Return point_ruck 
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


# astar-function:
# Calculates the shortest way from the defined start_point to a defined end_point via prescribed ways
# Input: start(tuple), end(tuple), way_name(string with the pathname of the osm-file)
# Functionality:
# Definitions of terminology:
# open_list: all not analysed but familiar nodes
# closed_list: all analysed nodes, which should not be analysed again
# 1. Add start_point to list open_list
# 2. Choose the node in the open_list with the lowest distance to the destination (Iteration 1: start_point)
# 3. Check if the current_point is the end_point
# yes: return the current point wiht all predecessors
# no: continue with 4 
# 4. Check on which ways the current point is located on the ways of the osm-file 
# 5. Find for each case the next point on the ways concerned, check if the point was already handled
# yes: dont handle it again
# 6. Handling of the points concerned: Calculation of the distance, Adding to a total distance, Setting the current point
# as predecessor and adding to the open_list
# 7. Continue with step 2

def astar(start, end, way_name):
    open_list = []     
    closed_list = []   

    way_dict = parking_parser.parse_osm(way_name);
    way_p = []
    for wa in way_dict:
    
        way_p.append([])
    
        for w in wa:
            way_p[len(way_p)-1].append((float(w["x"]), float(w["y"]),0))

    start_node1 = Node(start[0], start[1], start[2])
    end_node1 = Node(end[0], end[1], end[2])

    # For calculation the next_point on the center_line
    start_node = next_point(start_node1, way_p)
    end_node = next_point(end_node1, way_p)

    # start_node added to the open_list
    open_list.append(start_node)
    
    cnt = 0
            

    while len(open_list) >0:
        
        # Searching for the next current_node with the lowest total sum
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.sum < current_node.sum:           
                current_node = item
                current_index = index

        ax.scatter(current_node.x, current_node.y, color = "red", s = 10)
        

        open_list.pop(current_index)
        closed_list.append(current_node)

        # if the destination is found the return the total path
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                ax.scatter(current.x, current.y, color = "green", s = 20)
                path.insert(0,({"x": current.x, "y": current.y, "z": current.z}))
                current = current.parent
            return path
        
        
        # Calculate next_nodes
        children = []
        current_koord = (current_node.x, current_node.y, current_node.z)

        # Calculate all lines in which the current point is a part and calculate the position in the line
        currentLine = []
        for cnt, wp in enumerate(way_p):
            for cnt1, wp1 in enumerate(wp):
                if current_koord == wp1:
                    currentLine.append((cnt,cnt1))
                
        # Calculate next nodes
        for cl in currentLine:
            if (cl[1]+ 1) > len(way_p[cl[0]])-1 :
                continue
            next_pos = way_p[cl[0]][cl[1]+1] 

            next_node = Node(next_pos[0], next_pos[1], next_pos[2])
            next_node.parent = current_node
            
            children.append(next_node)

        # Check if the node was already handled, if yes: add that node to the open_list
        for count, child in enumerate(children):
            flag = False
            for closed in closed_list:
                if child.x == closed.x and child.y == closed.y:
                    flag = True
                    continue
            
            if count <4:
                child.costStart = current_node.costStart + 1
            #else:
            #    child.costStart = current_node.costStart + 1.4

            for open in open_list:
                if child.x == open.x and child.y == open.y and child.costStart >= open.costStart:
                    flag = True
                    break

            if flag == True:
                continue

           
            child.minimumCost = math.sqrt((child.x - end_node.x) ** 2 + (child.y - end_node.y) ** 2 + (child.z - end_node.z) ** 2)
            
            child.sum = child.costStart + child.minimumCost
            
            for open_node in open_list:
                if child == open_node and child.costStart > open_node.costStart:
                    continue
            

            open_list.append(child)
        
        cnt += 1        # for visualization

    return None

def get_quaternion_from_euler(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


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
            yaw_euler_angel = 90 - rad2deg(math.atan2(following_point["y"]-point["y"], following_point["x"]-point["x"])) 
            quaternions = get_quaternion_from_euler(0, 0, yaw_euler_angel)
            point["qx"] = quaternions[0]
            point["qy"] = quaternions[1]
            point["qz"] = quaternions[2]
            point["qw"] = quaternions[3]

    return waypoints


result_no_orientation = astar(start_point,end_point,"Parkhaus_2.osm")
result = addOrientation(result_no_orientation)


for r in result:
   print(r)

#plt.show()        


