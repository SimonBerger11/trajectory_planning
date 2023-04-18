#from lanelet2_parser import Lanelet2Parser
#from collections import Iterable 

import xml.etree.ElementTree as ET
tree = ET.parse('Vorabkarte_mit_Parkplaetze_3.xml')
root = tree.getroot()


points = []

for child in root:
    print(child.tag, child.attrib)
    if child.tag == "node":


