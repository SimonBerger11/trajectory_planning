#from lanelet2_parser import Lanelet2Parser
#from collections import Iterable 

import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
tree = ET.parse('Vorabkarte_mit_Parkplaetze_3.xml')
root = tree.getroot()


points = {}

for node in root.findall("node"):
    #print(node.tag, node.attrib)
    point = {}
    point["x"] = node.attrib["lat"]
    point["y"] = node.attrib["lon"]
    #points["Typ"] = ""
    points[node.attrib["id"]] = point

for way in root.findall("way"):
    type = ""
    for tag in way.findall("tag"):
        
        if tag.attrib["k"] == "type":
            type = tag.attrib["v"]
    if type == "parking_space":
        for nd in way.findall("nd"):
            #print(nd.attrib["ref"])
            points[nd.attrib["ref"]]["Typ"] = type


for point in points:
    print(point, points[point])


# plot
fig, ax = plt.subplots()
x = list()
y = list()
x_park = list()
y_park = list()
for point in points:
    try:
        if points[point]['Typ'] == 'parking_space':
            x_park.append(float(points[point]['x']))
            y_park.append(float(points[point]['y']))
    except:
        x.append(float(points[point]['x']))
        y.append(float(points[point]['y']))

#print 
fig, ax = plt.subplots()
ax.scatter(x, y)
ax.scatter(x_park, y_park, color='red')
plt.show()

