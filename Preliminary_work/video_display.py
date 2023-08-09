import os
from scipy.io import loadmat
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import time
from moviepy.editor import VideoFileClip, TextClip
from moviepy.video.io.ffmpeg_tools import ffmpeg_extract_subclip
from distance_calculation import calculate_distance
import cv2
import numpy as np
import datetime
from movement_detection import *


"""""""""""""""""""""""""""
    Fuego coordinates
"""""""""""""""""""""""""""

lat_fuego = 14.4747
lgn_fuego = -90.8806

""""""""""""""""""""""""""""
        Files Import
"""""""""""""""""""""""""""

###### Flight Video Import ######

videos_path = r'path to the video folder'
nom_sortie = 'file_name.mp4'

###### Telemetry Import ######
logs_path = r"path_to_flight_logs_file.mat"
logs_data = loadmat(logs_path)

"""""""""""""""""""""""""""""""""""""""""""""
        Processing of flight data
"""""""""""""""""""""""""""""""""""""""""""""

###### Extract GPS Data, IMU_Data, ARH Data ######
GPS = logs_data['GPS']
GPS_Time = GPS[:, 1]
GPS_Lat = GPS[:, 7]
GPS_Lng = GPS[:, 8]
GPS_Alt = GPS[:, 9]
GPS_Spd = GPS[:, 10]

IMU = logs_data['IMU']
IMU_Time = IMU[:, 1]
IMU_Accx = IMU[:, 5]

AHR = logs_data['ATT']
AHR_Time = AHR[:, 1]
AHR_Pitch = AHR[:, 5]
AHR_Roll = AHR[:, 3]


IMU_Time, GPS_Time = IMU_Time * 1e-6, GPS_Time * 1e-6
AHR_Time = AHR_Time * 1e-6

IMU_Time_display, GPS_Time_display = [], []
GPS_Time_display = []
GPS_Lat_display = []
GPS_Lng_display = []
GPS_Alt_display = []
GPS_Spd_display = []
IMU_Accx_display = []
AHR_Time_display = []
AHR_Pitch_display = []
AHR_Roll_display = []

## The average of sensor values over one second is calculated. Not all sensors are recorded at the same frequency, which is why it's not possible to process them simultaneously. It would be possible to make the code more elegant by writing a general function for this task. However, due to the limited amount of data to process, generalization has not been implemented. ##

################# IMU

IMU_Base_Time = np.around(IMU_Time[0], 0)
temp_Accx = []

for k in range(len(IMU_Time)):
    if np.around(IMU_Time[k], 0) == IMU_Base_Time:
        temp_Accx.append(IMU_Accx[k])
    else:
        IMU_Accx_display.append(np.mean(temp_Accx))
        IMU_Time_display.append(IMU_Base_Time)
        IMU_Base_Time = np.around(IMU_Time[k], 0)
        temp_Accx = []

################# Pitch, Roll, Yaw
GPS_Base_Time = np.around(GPS_Time[0], 0)
temp_Lat = []
temp_Lng = []
temp_Alt = []
temp_Spd = []

for k in range(len(GPS_Time)):
    if np.around(GPS_Time[k], 0) == GPS_Base_Time:
        temp_Lat.append(GPS_Lat[k])
        temp_Lng.append(GPS_Lng[k])
        temp_Alt.append(GPS_Alt[k])
        temp_Spd.append(GPS_Spd[k])
    else:
        if len(temp_Lat + temp_Lng + temp_Alt + temp_Spd) == 0:
            GPS_Base_Time = np.around(GPS_Time[k], 0)
        else:
            GPS_Time_display.append(GPS_Base_Time)
            GPS_Base_Time = np.around(GPS_Time[k], 0)
            GPS_Lat_display.append(np.mean(temp_Lat))
            GPS_Lng_display.append(np.mean(temp_Lng))
            GPS_Alt_display.append(np.mean(temp_Alt))
            GPS_Spd_display.append(np.mean(temp_Spd))

        temp_Lat, temp_Lng, temp_Alt, temp_Spd = [], [], [], []

###### Take off time determinatation, logs synchronizing

# Using the accelerometer, we aim to determine the moment in the logs when Leila is catapulted
max_value = np.max(IMU_Accx)
max_index = np.argmax(IMU_Accx)
Departure_Time = IMU_Time[max_index]
Departure_Time = float(int(Departure_Time))
index_logs_departure = IMU_Time_display.index(Departure_Time)

# Machine Vision Method to detect, the timecode in the video where the drone starts to move
cap = cv2.VideoCapture(videos_path + '\\' + nom_sortie)

if (cap.isOpened()== False):
  print("Error opening video stream or file")
ret, frame = cap.read()
prev_image = frame
compteur = 0
# Read until video is completed
while(cap.isOpened()):
  # Capture frame-by-frame
  compteur += 1
  ret, frame = cap.read()
  if compteur%20 == 0:
    print('Looking for the drone departure in the video, frame n°' + str(compteur))
  if ret == True:
    # Display the resulting frame
    #cv2.imshow('Frame',frame)

    if detect_movement(frame, prev_image) == True:
        print("Motion detected")
        take_off_video = cap.get(cv2.CAP_PROP_POS_MSEC)
        break
        # When everything done, release the video capture object
        cap.release()
        # Closes all the frames
        cv2.destroyAllWindows()
    else:
        prev_image = frame
    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
  # Break the loop
  else:
    break
# When everything done, release the video capture object
cap.release()
# Closes all the frames
cv2.destroyAllWindows()


take_off_video = round(take_off_video*1e-3)
print(take_off_video)


###### Video display

# Create a VideoCapture object and read from input file
cap = cv2.VideoCapture(videos_path + '\\' + nom_sortie)

# Check if camera opened successfully
if not cap.isOpened():
    print("Error opening video stream or file")

# Read until video is completed
previous_time = -1
AH_init_done = False

while cap.isOpened():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if ret:
        # Describe the type of font to be used
        font = cv2.FONT_HERSHEY_SIMPLEX
        current_time = cap.get(cv2.CAP_PROP_POS_MSEC)
        current_time = current_time * 1e-3

        if int(current_time) != int(previous_time):
            previous_time = int(current_time)

        if current_time >= take_off_video:

            flight_time_display = str(datetime.timedelta(seconds=int(current_time - take_off_video)))[2:]
            index_data = previous_time - int(take_off_video) + index_logs_departure

            Spd = GPS_Spd_display[index_data]
            Alt = GPS_Alt_display[index_data]
            Lat = GPS_Lat_display[index_data]
            Lng = GPS_Lng_display[index_data]


            distance_to_fuego = calculate_distance(lat_fuego, lgn_fuego, float(Lat), float(Lng), Alt)

            # Use putText() method for inserting text on video
            cv2.putText(frame, 'Flight Information', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.putText(frame, f'Speed: {Spd:.2f} m/s', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, f'Altitude: {Alt:.2f} m', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, f'Distance to the summit: {distance_to_fuego:.2f} m', (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # Display the current time on the frame
            cv2.putText(frame, 'Flight Time: ' + flight_time_display, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 255),
                        2)
            # Display the resulting frame
            cv2.imshow('Frame', frame)
        else:
            # Use putText() method for inserting text on video
            cv2.putText(frame, 'Waiting for flight logs', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            # Display the countdown time on the frame
            countdown = take_off_video - current_time
            countdown_display = str(datetime.timedelta(seconds=int(countdown)))[2:]
            cv2.putText(frame, f'Take-off countdown: ' + countdown_display, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 255), 2)
            # Display the resulting frame
            cv2.imshow('Frame', frame)

        # Press 'Q' on keyboard to exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    # Break the loop if no more frames
    else:
        break

# When everything is done, release the video capture object
cap.release()
# Closes all the frames
cv2.destroyAllWindows()