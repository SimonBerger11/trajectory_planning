import math
from pylab import *
import matplotlib.pyplot as plt
#import analyse_map as analyse
#import csv_reader
import parking_parser

# Klasse Node
# Attribute: x,y,z: Koordinaten
# parent: Vorgänger-Punkt
# costStart: Kosten um diesen Punkt zu erreichen
# minimumCost: Entfernung zum Endpunkt
# sum: Summe von costStart und minimumCost
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

        
start_point = (44, 10.5401,0)
end_point = (16, 17.417,0)

scatter(start_point[0], start_point[1], color = "orange", s = 40)
scatter(end_point[0], end_point[1], color = "orange", s = 40)

way_dict = parking_parser.parse_osm("Parkhaus_2.osm")       # informationen der wege aus dem osm-file holen
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




# Funktion zur Berechnung des nächsten Punktes auf der Mittellinie
# input: Point (Klasse Node) und point_list (output des osm-files)
# Output: Node
# Funktionsweise: durchlaufen aller Punkte in der point_list, Abstand zu point berechnen, wenn abstand kleiner 
# dem bisherigen geringsten abstand ist dann wird der aktuelle point zu point ruck, nachdem alle punkte behandelt wurden
# Rückgabe von point_ruck 
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


# astar-Funktion:
# Berechnet kürzesten Weg von einem festgelegten Startpunkt zu einem festgelegten Endpunkt über vorgegebene Straßen 
# Input: start(tuple), end(tuple), way_name(string mit pfadname zum osm-file)
# Funktionsweise:
# Definition von Begrifflichkeiten:
# Open-List: alle noch nicht analysierten aber bekannten Knoten
# Closed-List: alle analysierten und nicht erneut zu analysierenden Knoten
# 1. Start-Punkt wird zu einer Liste Open List hinzugefügt
# 2. Aus der Open_List wird der Knoten mit dem geringsten Abstand zum Ziel ausgewählt (in der 1. Iteration der Startpunkt)
# 3. Überprüfen ob aktueller Punkt der End-Punkt ist, wenn ja: Rückgabe des aktuellen Punkts mit allen Vorgängern, 
# wenn nein: zu 4.
# 4. Prüfen auf welchen Wegen der aktuelle Punkt in den vom Osm-File herausgelesenen Wegen liegt 
# 5. den jeweils nachfolgenden Punkt auf den betroffenen Wegen herausfinden, prüfen ob Weg schon behandelt wurde, wenn ja: 
# nicht weiter behandeln
# 6. Behandlung der jeweiligen Punkte: Berechnung der Entfernung, Hinzufügen zu einer Gesamtentfernung, Setzen des 
# aktuellen Punkts als Vorgänger und Hinzufügen zur Open-List   
# 7. Zurück zu 2. 

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

    # Zur Berechnung des nächsten Punktes auf der Mittellinie 
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
            #else:
            #    child.costStart = current_node.costStart + 1.4

            for open in open_list:
                if child.x == open.x and child.y == open.y and child.costStart >= open.costStart:
                    flag = True
                    break

            if flag == True:
                continue

           
            child.minimumCost = math.sqrt((child.x - end_node.x) ** 2 + (child.y - end_node.y) ** 2 + (child.z - end_node.z) ** 2)
            # Sum: Entfernung zum Ziel+ anzahl an übersprungenen Knoten
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


