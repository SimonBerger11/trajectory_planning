"""
Graphen Aufbauen mit Einträgen:

Struktur:
('235', {'x': '1', 'y': '1','z' 'Typ': 'parking_space'})

Typen: 
parking_space
object
way

Aufbau:
Grobe Darstellung des Parkplatzes
"""

"""
Bemerkung Elias:
- Fahrbahnmarkierungen zwischen dem Weg entfernen
- (evtl als Punkte statt linien)
- Punkte_all liste

"""
import numpy as np
import matplotlib.pyplot as plt
from statistics import mean

def generate_trajectory(height, length):
    """
    function generate_trajectory takes 2 input parameters height and length, 
    which describe the possible geometry of the trajectory, and creates a sample
    trajectories trajectory_coordinates_circle and trajectory_coordinates_middle_sec 
    which could result from driving in a parking house
    """    
    n = 4
    
    #resolution
    num_points = 1000
    
    # create the array of angles
    theta = np.linspace(0, 2*np.pi, num_points)
    
    # create the x and y coordinates of the trajectory using the parametric equation
    x = (np.sign(np.cos(theta)) * np.abs(np.cos(theta)) ** (2 / n) * height).tolist()
    y = (np.sign(np.sin(theta)) * np.abs(np.sin(theta)) ** (2 / n) * length).tolist()
    
    
    
    # create the x and y coordinates of the trajectory in the middle section of a parking house
    # horizontal driving trajectory in the middle of the parking house 
    x_middle_section_hor = np.linspace(-length, length, num=num_points)
    y_middle_section_hor = np.zeros(num_points)
    
    # vertical driving trajectory in the middle of the parking house
    x_middle_section_vert = np.zeros(num_points)
    y_middle_section_vert = np.linspace(-height, height, num=num_points)
    
    #trajectories to return
    trajectory_coordinates_circle = []
    trajectory_coordinates_middle_sec = []    
    
    # contains all possible driving trajectories incl. circle and middle sec
    full_trajectory = []
    
    # create lists of possible trajectories
    for i in range(0, num_points):
        trajectory_coordinates_circle.append((x[i], y[i]))
        trajectory_coordinates_middle_sec.append((x_middle_section_hor[i], y_middle_section_hor[i]))
        trajectory_coordinates_middle_sec.append((x_middle_section_vert[i], y_middle_section_vert[i]))
        full_trajectory.append((x[i], y[i]))
        full_trajectory.append((x_middle_section_hor[i], y_middle_section_hor[i]))
        full_trajectory.append((x_middle_section_vert[i], y_middle_section_vert[i]))


    return full_trajectory

def generate_parking_house(height, length):
    """
    function generate_parking_house takes 2 input parameters height and length,
    which describe the geometry of a parking house, and creates list of tuples coordinates_list
    with coordinates of the parking house for further graphic representation
    """
    n = 4

    # define the number of points used to create the squircle
    num_points = 1000
    
    # create the array of angles
    theta = np.linspace(0, 2*np.pi, num_points)
    # coordianates of the inner boundaries of the parking house excluding middle section
    x_in = np.sign(np.cos(theta)) * np.abs(np.cos(theta)) ** (2 / n) * (length - 3)
    y_in = np.sign(np.sin(theta)) * np.abs(np.sin(theta)) ** (2 / n) * (height - 6)
    
    # coordianates of the outer boundaries of the parking house excluding middle section
    x_out = np.sign(np.cos(theta)) * np.abs(np.cos(theta)) ** (2 / n) * (length + 3)
    y_out = np.sign(np.sin(theta)) * np.abs(np.sin(theta)) ** (2 / n) * (height + 6)
    
    
    # coordianates of the middle section boundaries of the parking house
    x_middle = np.linspace(-27, 27, num=1000)
    y_middle_top = np.linspace(6, 6, num=1000)
    y_middle_down = np.linspace(-6, -6, num=1000)
    
    # complete list of all coordinates relevant for the parking house
    coordinates_list = [(x_in, y_in), (x_out, y_out), (x_middle, y_middle_top), (x_middle, y_middle_down)]
    
    return coordinates_list
    
    
    
    
def generate_parking_spaces(height, length):
    """
    function generate_parking_house takes 2 input parameters height and length,
    which describe the geometry of a parking house, and returns an array of x coordinates
    parking_spaces_x for all boundaries of possible parking space
    """
    parking_spaces_x = np.arange(-length, length, 3).tolist()[4:-4]
    parking_spaces_traj_x = np.arange(-length + 1.5, length - 1.5, 3).tolist()[4:-4]
    del parking_spaces_x[int(len(parking_spaces_x)/2)]
    del parking_spaces_traj_x[int(len(parking_spaces_traj_x)/2)]
    
    return parking_spaces_x, parking_spaces_traj_x

def generate_graph(height, length):
    """
    function generate_graph takes 2 input parameters height and length,
    which describe the geometry of a parking house, and plots a graph of a parking house
    """
    
    # calculate complete driving trajectory incl. circular and middle section of the parking house
    full_trajectory = generate_trajectory(height, length)
    
    #trajectory_circle = generate_trajectory(height, length)[1]
    #trajectory_middle_section = generate_trajectory(height, length)[2]
    
    coordinates_full_traj_x =[]
    coordinates_full_traj_y = []

    # unpack tuples from a list of trajectories
    for item in full_trajectory:
        x = item[0]
        y = item[1]
        coordinates_full_traj_x.append(x)
        coordinates_full_traj_y.append(y)

    # calculate coordinates for parking house and parking spaces
    coordinates_parking_house = generate_parking_house(height, length)
    coordinates_parking_spaces = generate_parking_spaces(height, length)[0]
    
    # calculate trajectories to parking spaces
    trajectory_to_parking_spaces = generate_parking_spaces(height, length)[1]

    fig, ax = plt.subplots()
    
    # plot the trajectories
    plt.scatter(coordinates_full_traj_x, coordinates_full_traj_y, color='blue')

    # plot parking spaces
    for x_var in coordinates_parking_spaces:
        ax.plot([x_var, x_var], [coordinates_parking_house[1][1].max(), coordinates_parking_house[1][1].max() + 8], '-r')
        ax.plot([x_var, x_var], [coordinates_parking_house[0][1].max(), coordinates_parking_house[0][1].max() - 8], '-r')
        ax.plot([x_var, x_var], [coordinates_parking_house[2][1].max(), coordinates_parking_house[2][1].max() + 8], '-r')
        ax.plot([x_var, x_var], [coordinates_parking_house[3][1].max(), coordinates_parking_house[3][1].max() - 8], '-r')
        ax.plot([x_var, x_var], [coordinates_parking_house[0][1].min(), coordinates_parking_house[0][1].min() + 8], '-r')

    # plot trajectories to parking spaces
    for x_var in trajectory_to_parking_spaces:
        
        plt.scatter(np.linspace(x_var, x_var, 1000), np.linspace(max(coordinates_full_traj_y), max(coordinates_full_traj_y) + 8, 1000), color='blue')
        plt.scatter(np.linspace(x_var, x_var, 1000), np.linspace(max(coordinates_full_traj_y), max(coordinates_full_traj_y) - 8, 1000), color='blue')
        plt.scatter(np.linspace(x_var, x_var, 1000), np.linspace(mean(coordinates_full_traj_y), mean(coordinates_full_traj_y) + 8, 1000), color='blue')
        plt.scatter(np.linspace(x_var, x_var, 1000), np.linspace(0, -8, 1000), color='blue')
        plt.scatter(np.linspace(x_var, x_var, 1000), np.linspace(0, 8, 1000), color='blue')
        plt.scatter(np.linspace(x_var, x_var, 1000), np.linspace(min(coordinates_full_traj_y), min(coordinates_full_traj_y) + 8, 1000), color='blue')

        
generate_graph(30, 30)
plt.show()


    