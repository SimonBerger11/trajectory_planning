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
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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
    x_middle_section = np.linspace(-length, length, num=num_points)
    y_middle_section = np.zeros(num_points)
    trajectory_coordinates_circle = []
    trajectory_coordinates_middle_sec = []
    
    # create lists of possible trajectories
    for i in range(0, num_points):
        trajectory_coordinates_circle.append(('id_number', {'x' : x[i], 'y' : y[i], 'z' : 0, 'Type' : 'way'}))
        trajectory_coordinates_middle_sec.append(('id_number', {'x' : x_middle_section[i], 'y' : y_middle_section[i], 'z' : 0, 'Type' : 'way'}))
    return trajectory_coordinates_circle, trajectory_coordinates_middle_sec


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
    
    return parking_spaces_x

def generate_graph(height, length):
    """
    function generate_graph takes 2 input parameters height and length,
    which describe the geometry of a parking house, and plots a graph of a parking house
    """
    
    trajectory_circle, trajectory_middle_section = generate_trajectory(height, length)
    
    coordinates_traj_circle_x = []
    coordinates_traj_circle_y = []
    coordinates_traj_middle_sec_x = []
    coordinates_traj_middle_sec_y = []

    for item in trajectory_circle:
        x = item[1]['x']
        y = item[1]['y']
        coordinates_traj_circle_x.append(x)
        coordinates_traj_circle_y.append(y)
        
    for item in trajectory_middle_section:
        x = item[1]['x']
        y = item[1]['y']
        coordinates_traj_middle_sec_x.append(x)
        coordinates_traj_middle_sec_y.append(y) 


    coordinates_parking_house = generate_parking_house(height, length)
    coordinates_parking_spaces = generate_parking_spaces(height, length)
    
    fig, ax = plt.subplots()
    
    # plot the points and connect them with lines
    plt.plot(coordinates_traj_circle_x, coordinates_traj_circle_y,'-b', label='trajectory')
    plt.plot(coordinates_traj_middle_sec_x, coordinates_traj_middle_sec_y, '-b')
    plt.plot(coordinates_parking_house[0][0], coordinates_parking_house[0][1], '#000000', label='parking house boundaries')
    plt.plot(coordinates_parking_house[1][0], coordinates_parking_house[1][1], '#000000')
    plt.plot(coordinates_parking_house[2][0], coordinates_parking_house[2][1], '#000000')
    plt.plot(coordinates_parking_house[3][0], coordinates_parking_house[3][1], '#000000')
    
    for x_var in coordinates_parking_spaces:
        ax.plot([x_var, x_var], [coordinates_parking_house[1][1].max(), coordinates_parking_house[1][1].max() + 8], '-r')
        ax.plot([x_var, x_var], [coordinates_parking_house[0][1].max(), coordinates_parking_house[0][1].max() - 8], '-r')
        ax.plot([x_var, x_var], [coordinates_parking_house[2][1].max(), coordinates_parking_house[2][1].max() + 8], '-r')
        ax.plot([x_var, x_var], [coordinates_parking_house[3][1].max(), coordinates_parking_house[3][1].max() - 8], '-r')
        ax.plot([x_var, x_var], [coordinates_parking_house[0][1].min(), coordinates_parking_house[0][1].min() + 8], '-r')
    plt.legend(loc='best')

generate_graph(30, 30)


    