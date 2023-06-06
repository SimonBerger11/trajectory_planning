import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt


# Funktion zum parsen eines Osm-Files für die von uns benötigten Informationen
# Hier: Suche nach den Punkten der jeweiligen Straßen-Linien und Parkplatz-Linien 
# Input: Pfad zum osm-File als String
# Output: Liste (Gesamtliste aller Wege) an Listen (Liste aller Punkte in jeweiligem Weg) von Dictionaries (Punkt 
# mit x-, y- und z-Koord) 

# Funktionsweise:
# 1. Alle wege mit gesuchten Wegtypen heraussuchen 
# 2. ids der jeweiligen Punkte in den wegen heraussuchen 
# 3. nach ids suchen, um die koordinaten der Punkte herauszusuchen

def parse_osm(name):
    tree = ET.parse(name)
    root = tree.getroot()

    points = []
    ref = []
    i = 0
    # Schritt 1
    for way in root.findall("way"):
        type = ""
        for tag in way.findall("tag"):
            if tag.attrib["k"] == "type":
                type = tag.attrib["v"]
        if type == "parking_space" or type == "line_thin":          # Wegtyp nach denen im Osm-File gesucht wird
            ref.append([])
            # Schritt 2
            for nd in way.findall("nd"):
                ref[i].append(nd.attrib["ref"])
                
            i += 1
    
    # Schritt 3
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
  

