import multiprocessing as mp
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import math
from tkinter import *
import numpy as np
import time

# -------------------------------------------Both programs(Servo Control and Ball Tracker) in one -------------------------------------------
"""
For running both programs simultaneously we can use multithreading or multiprocessing
"""

camera_port = 0
cap = cv2.VideoCapture(camera_port)
cap.set(3, 1280)
cap.set(4, 720)

get, img = cap.read()
h, w, _ = img.shape

port_id = '/dev/cu.usbmodem21101'
# initialise serial interface
arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)

# define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = 0
all_angle = 0

# Set a limit to upto which you want to rotate the servos (You can do it according to your needs)
servo1_angle_limit_positive = 90
servo1_angle_limit_negative = -90

servo2_angle_limit_positive = 90
servo2_angle_limit_negative = -90

servo3_angle_limit_positive = 90
servo3_angle_limit_negative = -90




def ball_track(key1, queue):

    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    hsvVals = {'hmin': 10, 'smin': 170, 'vmin': 200, 'hmax': 30, 'smax': 255, 'vmax': 255}

    center_point = [626, 337, 2210] # center point of the plate, calibrated


    while True:
        get, img = cap.read()
        imgColor, mask = myColorFinder.update(img, hsvVals)
        imgContour, countours = cvzone.findContours(img, mask)

        if countours:

            data = round((countours[0]['center'][0] - center_point[0]) / 10), \
                   round((h - countours[0]['center'][1] - center_point[1]) / 10), \
                   round(int(countours[0]['area'] - center_point[2])/100)

            queue.put(data)
            print("The got coordinates for the ball are :", data)

        else:
            data = 'nil' # returns nil if we cant find the ball
            queue.put(data)

        imgStack = cvzone.stackImages([imgContour], 1, 1)
        # imgStack = cvzone.stackImages([img,imgColor, mask, imgContour],2,0.5) #use for calibration and correction
        cv2.imshow("Image", imgStack)
        cv2.waitKey(1)


def servo_control(key2, queue):
    if key2:
        print('Servo controls are initiated')


    def all_angle_assign(angle_passed1,angle_passed2,angle_passed3):
        global servo1_angle, servo2_angle, servo3_angle
        servo1_angle = math.radians(float(angle_passed1))
        servo2_angle = math.radians(float(angle_passed2))
        servo3_angle = math.radians(float(angle_passed3))
        write_servo()

    root = Tk()
    root.resizable(0, 0)

    def writeCoord():
        """
        Here in this function we get both coordinate and servo control, it is an ideal place to implement the controller
        """
        corrd_info = queue.get()

        if corrd_info == 'nil': # Checks if the output is nil
            print('cant fins the ball :(')
        else:
            print('The position of the ball : ', corrd_info[2])

            if (-90 < corrd_info[0] < 90) and (-90 < corrd_info[1] < 90) and (-90 < corrd_info[2] < 90):

                all_angle_assign(corrd_info[0],corrd_info[1],corrd_info[2])
            else:
                all_angle_assign(0,0,0)

    def ballpos_to_servo_angle(x_cord, y_cord):
        # convert the distance to center to angle.
        x_ang = map_x_to_y(x_cord, x_min=28, x_max=-28, y_min=-20, y_max=20)
        y_ang = map_x_to_y(y_cord, x_min=28, x_max=-28, y_min=-20, y_max=20)

        # x and y angle(deg) in and servoangle out(rad)
        servo_ang1, servo_ang2, servo_ang3 = kinematics(0, 22, y_ang, x_ang)
        print("angle", servo_ang1, " ", (servo_ang2), " ", (servo_ang3))

        return servo_ang1, servo_ang2, servo_ang3

    def map_x_to_y(value, x_min, x_max, y_min, y_max):
        return y_min + (((value - x_min) / (x_max - x_min)) * (y_max - y_min))
    def filter_write_angle_servo(servo1_angle_deg, servo2_angle_deg, servo3_angle_deg):
        if (-90 < servo1_angle_deg < 90) and (-90 < servo2_angle_deg < 90) and (-90 < servo3_angle_deg < 90):
            write_servo(servo1_angle_deg, servo2_angle_deg, servo3_angle_deg)
        else:
            write_servo(0, 0, 0)
    def write_arduino(data):
        print('The angles send to the arduino : ', data)

        arduino.write(bytes(data, 'utf-8'))

    def kinematics(Z, rotZdeg, rotYdeg, rotXdeg):
        L = 24
        R = 4

        # Convert degrees to radians
        rotZ = np.deg2rad(rotZdeg)
        rotY = np.deg2rad(rotYdeg)
        rotX = np.deg2rad(rotXdeg)

        # Define the position matrix
        pos = np.array([
            [L / 2, -(L / 2), 0],
            [L / (2 * np.sqrt(3)), L / (2 * np.sqrt(3)), -(L / np.sqrt(3))],
            [Z, Z, Z]
        ])

        # Define the rotation matrix for Y and X axes
        rotations_matrix_yx = np.array([
            [np.cos(0) * np.cos(rotY), np.cos(0) * np.sin(rotY) * np.sin(rotX) - np.sin(0) * np.cos(rotX),
             np.cos(0) * np.sin(rotY) * np.cos(rotX) + np.sin(0) * np.sin(rotX)],
            [np.sin(0) * np.cos(rotY), np.sin(0) * np.sin(rotY) * np.sin(rotX) + np.cos(0) * np.cos(rotX),
             np.sin(0) * np.sin(rotY) * np.cos(rotX) - np.cos(0) * np.sin(rotX)],
            [-np.sin(rotY), np.cos(rotY) * np.sin(rotX), np.cos(rotY) * np.cos(rotX)]
        ])

        # Define the rotation matrix for Z axis
        rotations_matrix_z = np.array([
            [np.cos(rotZ), -np.sin(rotZ), 0],
            [np.sin(rotZ), np.cos(rotZ), 0],
            [0, 0, 1]
        ])

        # Apply rotations in sequence
        newpos = np.dot(rotations_matrix_z, pos)
        all_the_rot = np.dot(rotations_matrix_yx, newpos)

        # Calculate angles using arcsin and normalize them
        angle1 = np.rad2deg(np.arcsin(np.clip(all_the_rot[2, 0] / R, -1, 1)))
        angle2 = np.rad2deg(np.arcsin(np.clip(all_the_rot[2, 1] / R, -1, 1)))
        angle3 = np.rad2deg(np.arcsin(np.clip(all_the_rot[2, 2] / R, -1, 1)))

        # Clip angles to be within -60 and 60 degrees
        angle1 = np.clip(angle1, -60, 60)
        angle2 = np.clip(angle2, -60, 60)
        angle3 = np.clip(angle3, -60, 60)

        return angle1, angle2, angle3

    def write_servo(ang1, ang2, ang3):
        angles: tuple = (round(ang1, 1),
                         round(ang2, 1),
                         round(ang3, 1))
        write_arduino(str(angles))

    kp = 1
    ki = 1
    kd = 3
    reff_val_x = 0
    reff_val_y = 0
    integral_error_x = 0
    integral_error_y = 0
    last_error_x = 0
    last_error_y = 0
    start_time = 0

    def get_ball_pos():
        cord_info = queue.get()
        return cord_info
    while key2:
        cord_info = get_ball_pos()  # Ballpos
        if cord_info == 'nil':
            error_x = 0
            error_y = 0
            integral_error_x = 0
            integral_error_y = 0
            last_error_x = 0
            last_error_y = 0
            pos_x = 0
            pos_y = 0
        else:
            pos_x = cord_info[0]
            pos_y = cord_info[1]

        dt = time.time() - start_time
        # print(dt)
        error_x = reff_val_x - pos_x
        error_y = reff_val_y - pos_y
        integral_error_x += error_x * dt
        integral_error_y += error_y * dt
        # print("integral error:  ", integral_error_x, "   ", integral_error_y)
        deriv_error_x = (error_x - last_error_x) / dt
        deriv_error_y = (error_y - last_error_y) / dt
        # print("deriv error:  ", deriv_error_x, "   ", deriv_error_y)
        last_error_x = error_x
        last_error_y = error_y
        output_x = error_x * kp + ki * integral_error_x + kd * deriv_error_x
        output_y = error_y * kp + ki * integral_error_y + kd * deriv_error_y
        # print(output_x, "   ", output_y)

        servo_ang1, servo_ang2, servo_ang3 = ballpos_to_servo_angle(output_x, output_y)  # Ballpos to servo angle
        filter_write_angle_servo(servo_ang1, servo_ang2, servo_ang3)  # Servo angle to arduino

        start_time = time.time()
    root.mainloop()  # running loop
    # running loop

if __name__ == '__main__':

    queue = Queue() # The queue is done inorder for the communication between the two processes.
    key1 = 1 # just two dummy arguments passed for the processes
    key2 = 2
    p1 = mp.Process(target= ball_track, args=(key1, queue)) # initiate ball tracking process
    p2 = mp.Process(target=servo_control,args=(key2, queue)) # initiate servo controls
    p1.start()
    p2.start()
    p1.join()
    p2.join()