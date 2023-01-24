import math
import os.path as op
import yaml
import numpy as np
import matplotlib.pyplot as plt
thymio_config_file = op.join(op.dirname(op.dirname(__file__)),'config.yaml')


""" Read the config file and return a dict with the config """
def read_config_file(config_file:str=thymio_config_file)->dict:
    with open(config_file, "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            raise Exception("Can't read config file") from exc


def extend_config_file(new_data:dict, config_file:str=thymio_config_file)->None:
    config_file_dict = read_config_file(config_file)
    config_file_dict.update(new_data)
    with open(config_file,'w') as stream:
        yaml.safe_dump(config_file_dict, stream, sort_keys=False)


"""Compute the angle between two points"""
def angle_between(p1, p2):
    ang1 = np.arctan2(-p1[1], p1[0])
   
    ang2 = np.arctan2(-p2[1], p2[0])
    
    return (ang2 - ang1) % (2 * np.pi)

"""Compute the distance between two points"""
def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

"""Create a plot with the grid"""
def create_empty_plot(max_valx, max_valy):
    fig, ax = plt.subplots(figsize=(7, 7))

    major_ticksy = np.arange(0, max_valx + 1, 5)
    major_ticksx = np.arange(0, max_valy + 1, 5)
    minor_ticksy = np.arange(0, max_valx + 1, 1)
    minor_ticksx = np.arange(0, max_valy + 1, 1)
    ax.set_xticks(major_ticksx)
    ax.set_xticks(minor_ticksx, minor=True)
    ax.set_yticks(major_ticksy)
    ax.set_yticks(minor_ticksy, minor=True)
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)
    #ax.set_ylim([-1, max_valy+1])
    #ax.set_xlim([-1, max_valx+1])
    ax.grid(True)

    return fig, ax
