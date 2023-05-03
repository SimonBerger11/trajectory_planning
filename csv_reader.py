import csv
from pylab import *
import matplotlib.pyplot as plt

def filter_string(str1, str2):
    for c in str2:
        str1 = str1.replace(c, ' ')
    return str1

def get_data(doc):
    elements = []
    with open(doc, newline='') as csvfile:

        spamreader = csv.reader(csvfile, delimiter=',')#, quotechar='|')

        for row in spamreader:
            next = []
            for elem in row:
                elem1 = filter_string(elem,"\r\n")
                elems = elem1.split(" ")
                if len(elems)==0:
                    continue
                if elems[0] == "x:":
                    if elems[1] == " " or elems[4] == " " or elems[7] == " " :
                        continue
                    
                    #next.append((float(elems[1]), float(elems[4]), float(elems[7])))
                    next.append((float(elems[1]), float(elems[4]), 0))
            elements.append(next)

    return elements

    #fig = plt.figure()
    #ax = fig.add_subplot(111)

    #way_x = []
    #way_y = []
    #for w in elements:
     #   print("Neue Zeile")
      #  for wl in w:
       #     print(wl[1], wl[0])
        
        #    way_x.append(wl[0])
         #   way_y.append(wl[1])


    #scatter(way_x, way_y)
    #plt.yscale= "linear"
    #plt.xscale = "linear"
    #plt.show()
