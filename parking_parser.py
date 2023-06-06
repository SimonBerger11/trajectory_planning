import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt

def parse_osm(name):
    tree = ET.parse(name)
    root = tree.getroot()

    points = []
    ref = []
    i = 0
    #extract from xml
    for way in root.findall("way"):
        type = ""
        for tag in way.findall("tag"):
            if tag.attrib["k"] == "type":
                type = tag.attrib["v"]
        if type == "parking_space" or type == "line_thin":
            ref.append([])
            for nd in way.findall("nd"):
                #print(nd.attrib["ref"])
                ref[i].append(nd.attrib["ref"])
                
            i += 1
    
    lines = []
    for re in ref:
        lines.append([])
        for r in re:
            for node in root.findall("node"):
                point = {}
                #print(r)
                if r == node.attrib["id"]:
                    for tag in node.findall("tag"):
                        if tag.attrib["k"] == "local_x":
                            point["x"] = tag.attrib["v"]
                        elif tag.attrib["k"] == "local_y":
                            point["y"] = tag.attrib["v"]
                        elif tag.attrib["k"] == "local_z":
                            point["z"] = tag.attrib["v"]
                    lines[len(lines) -1].append(point)
    
   
    return lines
  

#t = parse_osm();

#for f in t:
#    print(f)
