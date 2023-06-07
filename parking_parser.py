#!/usr/bin/env python
# coding=utf-8

########################################################################
#FileName: parking_parser.py
#Author : Christian Göller, Jonas Bäuml, Simon Berger, Elias Häring 
#Last Modified On : 09.06.2023
#Description : Parser for osm file to retrieve waypoints 
########################################################################



import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt


# Function to parse an osm-file for the for us relevant information
# In this case: Searching for points of street-lines and parking-lines 
# input: path to the osm-file as a string
# output: list (total list of all ways) of lists (liste of all points of a way) of dictionaries (point
# with x-, y- und z-coordinates) 

# Functionality:
# 1. Search for all ways with the way types concerned
# 2. Search for the ids of the points in the way
# 3. search for ids, to get the coordinates of the points

def parse_osm(name):
    tree = ET.parse(name)
    root = tree.getroot()

    points = []
    ref = []
    i = 0
    # step 1
    for way in root.findall("way"):
        type = ""
        for tag in way.findall("tag"):
            if tag.attrib["k"] == "type":
                type = tag.attrib["v"]
        if type == "parking_space" or type == "line_thin":
            ref.append([])
            # step 2
            for nd in way.findall("nd"):
                ref[i].append(nd.attrib["ref"]) 
            i += 1
     # step 3
    lines = []
    for re in ref:
        lines.append([])
        for r in re:
            for node in root.findall("node"):
                point = {}
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
