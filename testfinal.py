import multiprocessing as mp
from multiprocessing import Queue
import cvzone
import numpy as np
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import time
import array
from scipy.signal import butter, filtfilt
from tkinter import *
import os
import csv

hmin_v = 0
hmax_v = 50
smin_v = 0
smax_v = 50
vmin_v = 170
vmax_v = 255
# hmin_v = 30
# hmax_v = 90
# smin_v = 30
# smax_v = 255
# vmin_v = 30
# vmax_v = 255
counter = 0
filter_on = True
center = False
circle = True
eight = False
plot = False
camera_port = 0
cap = cv2.VideoCapture(camera_port)
cap.set(3, 960)
cap.set(4, 540)
get, img = cap.read()
h, w, _ = img.shape
# Initialization of the CSV file:
fieldnames = ["num", "x", "y", "targetX", "targetY", "errorX", "errorY", "tot_error", "PID_x", "PID_y"]
output_dir = 'CSV_sourcecode/Gen_Data'
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
def save_data(xpos, ypos, targetx, targety, errorx, errory, PID_x, PID_y, plot):
    tot_error = np.sqrt(errorx ** 2 + errory ** 2)
    global counter
    if plot:
        with open(output_file, 'a') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            info = {
                "num": counter,
                "x": xpos,
                "y": ypos,
                "targetX": targetx,
                "targetY": targety,
                "errorX": errorx,
                "errorY": errory,
                "tot_error": tot_error,
                "PID_x": PID_x,
                "PID_y": PID_y,
            }
            csv_writer.writerow(info)
            counter += 1


def ball_track(key1, queue, reff_queue):

    class button:
        def __init__(self, sizeX, sizeY, cordX, cordY):
            self.size = sizeX, sizeY
            self.cord = cordX, cordY
            self.BtmRight = cordX+sizeX, cordY+sizeY
            self.TxtPos = cordX+20, int(cordY+sizeY/1.75)
        def getCord(self):
            return self.cord

        def getSize(self):
            return self.size

        def TopLeft(self):
            return self.cord

        def BottomRight(self):
            return self.BtmRight

        def TextPos(self):
            return self.TxtPos

    CenterButton = button(160,60,30,30)
    CircleButton = button(160,60,30,120)
    EightButton = button(160, 60, 30, 210)
    PlotButton = button(120,60,30,440)
    StopPlotButton = button(120, 60, 160, 440)
    def mouse_callback(event, x, y, flags, param):
        global center, circle, eight, plot

        if event == cv2.EVENT_LBUTTONDOWN:
            if (CenterButton.TopLeft()[0] <= x <= CenterButton.BottomRight()[0]) and (
                    CenterButton.TopLeft()[1] <= y <= CenterButton.BottomRight()[1]):
                center = True
                circle = False
                eight = False

            elif (CircleButton.TopLeft()[0] <= x <= CircleButton.BottomRight()[0]) and (
                    CircleButton.TopLeft()[1] <= y <= CircleButton.BottomRight()[1]):
                center = False
                circle = True
                eight = False

            elif (EightButton.TopLeft()[0] <= x <= EightButton.BottomRight()[0]) and (
                    EightButton.TopLeft()[1] <= y <= EightButton.BottomRight()[1]):
                center = False
                circle = False
                eight = True

            elif (PlotButton.TopLeft()[0] <= x <= PlotButton.BottomRight()[0]) and (
                    PlotButton.TopLeft()[1] <= y <= PlotButton.BottomRight()[1]):
                print("plot:  true")
                plot = True

            elif (StopPlotButton.TopLeft()[0] <= x <= StopPlotButton.BottomRight()[0]) and (
                    StopPlotButton.TopLeft()[1] <= y <= StopPlotButton.BottomRight()[1]):
                print("plot:  false")
                plot = False

    global center, circle, eight, plot
    prevX = 0
    prevY = 0


    if key1:
        print('Ball tracking is initiated')


    myColorFinder = ColorFinder(
        False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    hsvVals = {'hmin': hmin_v, 'smin': smin_v, 'vmin': vmin_v, 'hmax': hmax_v, 'smax': smax_v,
               'vmax': vmax_v}

    center_point = [480, 270, 2210]  # center point of the plate, calibrated

    while True:
        get, img = cap.read()
        mask_plat = np.zeros(img.shape[:2], dtype='uint8')
        cv2.circle(mask_plat, (480, 270), 220, (255, 255, 255), -1)

        # Make circular mask
        masked = cv2.bitwise_and(img, img, mask=mask_plat)

        imgColor, mask = myColorFinder.update(masked, hsvVals)
        imgContour, countours = cvzone.findContours(masked, mask, minArea=2000, maxArea=5500)

        x = -120
        y = -120

        boolData = [center, circle, eight, plot]
        reff_queue.put(boolData)

        if countours:
            x = round((countours[0]['center'][0]))
            y = round((countours[0]['center'][1]))

            data = round((countours[0]['center'][0] - center_point[0]) / 1), \
                round((h - countours[0]['center'][1] - center_point[1]) / 1), \
                round(int(countours[0]['area'] - center_point[2]) / 100)

            queue.put(data)
        else:
            data = 'nil'  # returns nil if we cant find the ball
            queue.put(data)

        imgStack = cvzone.stackImages([imgContour], 1, 1)

        #Ball Track stuff
        cv2.circle(imgStack, (x, y), 5, (20, 20, 255), 2)
        cv2.circle(imgStack, (x, y), 40, (180, 120, 255), 2)

        #Ball velocity vector
        vector = [prevX - x, prevY - y]
        cv2.arrowedLine(imgStack, (x, y), (x - vector[0] * 10, y - vector[1] * 10), (39, 237, 250), 4)

        if center:
            cv2.circle(imgStack, (center_point[0], center_point[1]), 2, (20, 20, 255), 2)

        if circle:
            cv2.circle(imgStack,
                   (center_point[0] + int(80 * np.cos(time.time())), center_point[1] - int(80 * np.sin(time.time()))),
                   5, (20, 20, 255), 2)

        if eight:
            cv2.circle(imgStack,
                   (center_point[0] + int(80 * np.sin(time.time())), center_point[1] - int(80 * np.sin(2*time.time()))),
                   5, (20, 20, 255), 2)

        # Center Button
        cv2.rectangle(imgStack, CenterButton.TopLeft(), CenterButton.BottomRight(), (40, 160, 40), -1)
        cv2.putText(imgStack, "Center Position", CenterButton.TextPos(),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Circle Button
        cv2.rectangle(imgStack, CircleButton.TopLeft(), CircleButton.BottomRight(), (40, 160, 40), -1)
        cv2.putText(imgStack, "Circle Ball", CircleButton.TextPos(),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # 8-figure Button
        cv2.rectangle(imgStack, EightButton.TopLeft(), EightButton.BottomRight(), (40, 160, 40), -1)
        cv2.putText(imgStack, "Figure Eight", EightButton.TextPos(),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Plot Buttons
        cv2.rectangle(imgStack, PlotButton.TopLeft(), PlotButton.BottomRight(), (40, 160, 40), -1)
        cv2.putText(imgStack, "Start Plot", PlotButton.TextPos(),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.rectangle(imgStack, StopPlotButton.TopLeft(), StopPlotButton.BottomRight(), (40, 40, 160), -1)
        cv2.putText(imgStack, "Stop Plot", StopPlotButton.TextPos(),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        prevX = x
        prevY = y

        cv2.imshow("Image", imgStack)
        cv2.setMouseCallback("Image", mouse_callback)
        cv2.waitKey(1)


def servo_control(key2, queue, reff_queue):
    port_id = '/dev/cu.usbmodem1101'
    # initialise serial interface
    arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)

    if key2:
        print('Servo controls are initiated')

    class PID:
        def __init__(self, Kp, Ki, Kd, setpoint):
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd
            self.setpoint = setpoint
            self.error = 0
            self.lastError = 0
            self.IntegralError = 0
            self.DerivativeError = 0
            self.start_time = 0
            self.dt = 0

        def compute(self, systemValue):
            self.dt = time.time() - self.start_time
            self.error = self.setpoint - systemValue
            self.IntegralError += self.error * self.dt
            self.DerivativeError = (self.error - self.lastError) / self.dt

            output = (-self.Kp * self.error) + (-self.Ki * self.IntegralError) + (-self.Kd * self.DerivativeError)

            self.lastError = self.error
            self.start_time = time.time()

            return output

        def compute_filtered(self, systemValue, error, IntegralError, DerivativeError):
            self.dt = time.time() - self.start_time

            output = (-self.Kp * error) + (-self.Ki * IntegralError) + (-self.Kd * DerivativeError)

            self.lastError = self.error
            self.start_time = time.time()

            return output

        def updateSetpoint(self, newsetpoint):
            self.setpoint = newsetpoint

        def getSetpoint(self):
            return self.setpoint

        def resetErrors(self):
            self.lastError = 0
            self.IntegralError = 0
            self.DerivativeError = 0

        def getError(self):
            return self.error

        def getIntegralError(self):
            return self.IntegralError

        def getDerivativeError(self):
            return self.DerivativeError

    def kinematics(Z, rotZdeg, rotYdeg, rotXdeg):

        # Physical properties
        L = 19
        R = 4

        rotZ = np.deg2rad(rotZdeg)
        rotY = np.deg2rad(rotYdeg)
        rotX = np.deg2rad(rotXdeg)

        # Position matrix
        pos = np.array([
            [L / 2, -(L / 2), 0],
            [L / (2 * np.sqrt(3)), L / (2 * np.sqrt(3)), -(L / np.sqrt(3))],
            [Z, Z, Z]
        ])

        # Rotation matrix for Y and X axes
        rotations_matrix_yx = np.array([
            [np.cos(0) * np.cos(rotY), np.cos(0) * np.sin(rotY) * np.sin(rotX) - np.sin(0) * np.cos(rotX),
             np.cos(0) * np.sin(rotY) * np.cos(rotX) + np.sin(0) * np.sin(rotX)],
            [np.sin(0) * np.cos(rotY), np.sin(0) * np.sin(rotY) * np.sin(rotX) + np.cos(0) * np.cos(rotX),
             np.sin(0) * np.sin(rotY) * np.cos(rotX) - np.cos(0) * np.sin(rotX)],
            [-np.sin(rotY), np.cos(rotY) * np.sin(rotX), np.cos(rotY) * np.cos(rotX)]
        ])

        # Rotation matrix for Z axis
        rotations_matrix_z = np.array([
            [np.cos(rotZ), -np.sin(rotZ), 0],
            [np.sin(rotZ), np.cos(rotZ), 0],
            [0, 0, 1]
        ])

        # Apply rotations in sequence
        newpos = np.dot(rotations_matrix_z, pos)
        all_the_rot = np.dot(rotations_matrix_yx, newpos)

        # Calculate angles
        angle1 = np.rad2deg(np.arcsin(np.clip(all_the_rot[2, 0] / R, -1, 1)))
        angle2 = np.rad2deg(np.arcsin(np.clip(all_the_rot[2, 1] / R, -1, 1)))
        angle3 = np.rad2deg(np.arcsin(np.clip(all_the_rot[2, 2] / R, -1, 1)))

        # Clip angles to be within -50 and 50 degrees
        angle1 = np.clip(angle1, -43, 43)
        angle2 = np.clip(angle2, -43, 43)
        angle3 = np.clip(angle3, -43, 43)
        return angle1, angle2, angle3

    root = Tk()

    # root.resizable(0,0)

    def butter_lowpass_filter(data, cutoff, fs,
                              order):  # https://medium.com/analytics-vidhya/how-to-filter-noise-with-a-low-pass-filter-python-885223e5e9b7
        normal_cutoff = cutoff / (0.5 * fs)
        # Get the filter coefficients
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        y = filtfilt(b, a, data)
        return y

    def map_x_to_y(value, x_min, x_max, y_min, y_max):
        return y_min + (((value - x_min) / (x_max - x_min)) * (y_max - y_min))

    def get_ball_pos():
        cord_info = queue.get()
        return cord_info

    def ballpos_to_servo_angle(x_cord, y_cord):
        """ convert the distance to center to angle.
        x_ang = map_x_to_y(x_cord/10, x_min=28.00, x_max=-28.00, y_min=-35.00, y_max=35.00)
        y_ang = map_x_to_y(y_cord/10, x_min=28.00, x_max=-28.00, y_min=-35.00, y_max=35.00)
        """
        # x and y angle(deg) in and servoangle out(rad)
        servo_ang1, servo_ang2, servo_ang3 = kinematics(0, 22, y_cord, x_cord)

        return servo_ang1, servo_ang2, servo_ang3

    def write_servo(ang1, ang2, ang3):
        angles: tuple = (round(ang1, 1),
                         round(ang2, 1),
                         round(ang3, 1))
        write_arduino(str(angles))

    def write_arduino(data):
        # print('The angles send to the arduino : ', data)
        arduino.write(bytes(data, 'utf-8'))

    # kp =  0.39 # 20s
    kp = 0.482  # 9sek
    # kp = 0.51 #brageverdi
    # ki =  0.64 # 20s
    ki = 0.346  # 9sek
    # ki = 0.31 #brageverdi
    # kd =  0.345 # 20s
    kd = 0.33  # 9sek
    # kd = 0.25 #brageverdi 9sek

    PID_X = PID(kp, ki, kd, 0)
    PID_Y = PID(kp, ki, kd, 0)
    PID_filter_data_x = array.array('f', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    PID_filter_data_y = array.array('f', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    while key2:
        center, circle, eight, plot = reff_queue.get()
        cord_info = get_ball_pos()  # Ballpos

        if center:
            PID_X.updateSetpoint(0)
            PID_Y.updateSetpoint(0)

        if circle:
            PID_X.updateSetpoint(int(80 * np.cos(time.time()) / 10))
            PID_Y.updateSetpoint(int(80 * np.sin(time.time()) / 10))

        if eight:
            PID_X.updateSetpoint(int(80 * np.sin(time.time())/10))
            PID_Y.updateSetpoint(int(80 * np.sin(2*time.time())/10))

        if cord_info == 'nil':
            PID_X.resetErrors()
            PID_Y.resetErrors()
            pos_x = 0
            pos_y = 0
            PID_X.updateSetpoint(0)
            PID_Y.updateSetpoint(0)
            PID_filter_data_x = array.array('f', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
            PID_filter_data_y = array.array('f', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        else:
            pos_x = (float(cord_info[0]) / 10)
            pos_y = (float(cord_info[1]) / 10)

        output_x = PID_X.compute(pos_x)
        output_y = PID_Y.compute(pos_y)

        if output_x != 0:
            cutoff_x = np.abs(3)
            PID_filter_data_x.append(output_x)
        else:
            cutoff_x = 2

        if output_y != 0:
            cutoff_y = np.abs(3)
            PID_filter_data_y.append(output_y)
        else:
            cutoff_y = 2

        if filter_on:
            if ((len(PID_filter_data_x) >= 15) and (len(PID_filter_data_y) >= 15)):
                print("cutoff_x: ", cutoff_x)
                print("cutoff_y: ", cutoff_y)
                filtered_error_data_x = butter_lowpass_filter(data=PID_filter_data_x, cutoff=cutoff_x, fs=100, order=2)
                filtered_error_data_y = butter_lowpass_filter(data=PID_filter_data_y, cutoff=cutoff_y, fs=100, order=2)
                filter_deriv_error_x = filtered_error_data_x[len(filtered_error_data_x) - 1]
                filter_deriv_error_y = filtered_error_data_y[len(filtered_error_data_y) - 1]
                output_x = filter_deriv_error_x
                output_y = filter_deriv_error_y

        servo_ang1, servo_ang2, servo_ang3 = ballpos_to_servo_angle(output_x, output_y)  # Ballpos to servo angle
        write_servo(servo_ang1, servo_ang2, servo_ang3)  # Servo angle to arduino
        save_data(pos_x, pos_y, PID_X.getSetpoint(), PID_Y.getSetpoint(), PID_X.getError(), PID_Y.getError(), output_x, output_y, plot)
    root.mainloop()  # running loop


if __name__ == '__main__':
    queue = Queue()  # The queue is done inorder for the communication between the two processes.
    reff_queue = Queue()
    key1 = 1  # just two dummy arguments passed for the processes
    key2 = 2
    p1 = mp.Process(target=ball_track, args=(key1, queue, reff_queue))  # initiate ball tracking process
    p2 = mp.Process(target=servo_control, args=(key2, queue, reff_queue))  # initiate servo controls
    p1.start()
    p2.start()
    p1.join()
    p2.join()