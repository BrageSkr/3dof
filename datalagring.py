import multiprocessing as mp
from multiprocessing import Queue
import cvzone
import numpy as np
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import math
from tkinter import *
import csv

# ------------------------------ Save Data Function ------------------------------------
# This function saves the data generated from the controller to a CSV file.
# NOTE : we need to create a directory (Folder) named "Gen_Data" in the root folder
""" There are two steps in saving the data to a CSV,
1- Initialize a CSV file in a folder inside the python directory.
2- Give in the required field names of the data to be saved
3- Write the data with a function call 
"""
import os
import csv
import numpy as np

counter = 0

# Initialization of the CSV file:
fieldnames = ["num", "x", "y", "z"]
output_dir = 'Gen_Data'
output_file = f'{output_dir}/saved_data.csv'

# Check if directory exists, create if not
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Initialize CSV file with header if it doesn't exist
if not os.path.exists(output_file):
    with open(output_file, 'w') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        csv_writer.writeheader()


# Saving Data to the CSV file:
def save_data(ball_loc):
    global counter
    with open(output_file, 'a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        info = {
            "num": counter,
            "x": ball_loc[0],
            "y": ball_loc[1],
            "z": ball_loc[2],
        }
        csv_writer.writerow(info)
        counter += 1


test = np.array([1, 1, 2])
for i in range(10):
    save_data(test)
    test = test * 2


