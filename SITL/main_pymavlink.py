from pymavlink import mavutil
from Route_E_CPI import  cpi_trajectory, E_wind_frame, extract_circle_points
from STIL_change_param import *
import time
from distance_calculation import convert_coordinates_ll2xy, convert_coordinates_xy2ll
import numpy as np
import matplotlib.pyplot as plt
from STIL_start_point import set_home
from STIL_send_WP import send_waypoints
from STIL_quick_takeoff import start_mission
from pymavlink import mavutil


################ SET UP THE COORDINATE INTERCEPTION TRAJECTORY #################

W_speed = 10
W_direction = 20
d_P1P2 = 10 * 1e3
V_a = 21.9
N_waypoints = 12
t_start = 0
t = 0
x_plane = -574.0251485



y_plane = -1385.819299



Yaw_plane = 22.5



N=12

x_c, y_c, d_r = E_wind_frame(W_direction, W_speed, d_P1P2, V_a, (0,0))
x_point, y_point = extract_circle_points(x_c, y_c, d_r, N, W_direction)


x,y,t, lat, long, wf_x, wf_y = cpi_trajectory(d_P1P2, V_a, W_speed, N, x_point, y_point, t_start, x_plane, y_plane, Yaw_plane, W_direction)
altitude = [4600 for _ in range (len(lat))]
coordinates = list(zip(lat, long, altitude))
coordinates.insert(0, coordinates[0])
print(coordinates)



################# SET UP HOME AND TAKE OFF #################
P1 = coordinates[1]
P2 = coordinates[2]

x1,y1 = convert_coordinates_ll2xy(P1[0], P1[1])
x2,y2 = convert_coordinates_ll2xy(P2[0], P2[1])

def find_home(d,x1,y1,x2,y2):
    try :
        a = (y2 - y1)/(x2 - x1)
    except :
        a = 0
    b = y1 - a*x1
    A = 1 + a**2
    B = -2*x1 - 2*y1*a + 2*a*b
    C = -d**2 +b**2 - 2*y1*b - x1**2 - y1**2
    coefficients = [A, B, C]
    racines = np.roots(coefficients)

    xh = racines[0]
    yh = a*xh + b
    return xh,yh

xh, yh = find_home(15e3, x1, y1, x2, y2)
latitude_h, longitude_h = convert_coordinates_xy2ll(14.4747, -90.8806, -xh,-yh)
set_home(latitude_h, longitude_h, 1131, 130)
print(xh)
STIL_change_weather(W_speed, W_direction)
STIL_set_plane(2*8000, 14.8)
send_waypoints(coordinates)
start_mission()

################# LISTEN WHILE IN FLIGHT #################


the_connection = mavutil.mavlink_connection("172.29.80.1:14550")

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

turning_point = the_connection.recv_match(type='MISSION_CURRENT', blocking=True)



while turning_point.seq == 1:
    turning_point = the_connection.recv_match(type='MISSION_CURRENT', blocking=True)
    time_tp = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    time_detection_ms = time_tp.time_boot_ms
print(time_detection_ms)


current_waypoint = 2

while 1:
    turning_point = the_connection.recv_match(type='MISSION_CURRENT', blocking=True)
    waypoint_seq = turning_point.seq

    if current_waypoint != waypoint_seq:
        current_waypoint = waypoint_seq
        time_tp = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        time_detection_ms_2 = time_tp.time_boot_ms
        print(time_detection_ms_2)

    if turning_point.mission_mode == 0:
        time_tp = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        time_detection_ms_2 = time_tp.time_boot_ms
        print(time_detection_ms_2)
        break

print(time_detection_ms_2 - time_detection_ms)



