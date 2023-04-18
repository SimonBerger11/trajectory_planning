#from lanelet2_parser import Lanelet2Parser
#from collections import Iterable 

import xml.etree.ElementTree as ET
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



