import numpy as np
import matplotlib.pyplot as plt
from distance_calculation import *


class Plume:
    """This class allows to define in an easy way the plume and its properties"""
    def __init__(self, t_start, W_speed, W_direction):
        self.__t_start = float(t_start)
        self.__W_speed = float(W_speed)
        self.__W_direction = float(W_direction)

    def in_flight_zone(self, t):  # Check if it is still legal to fly to the plume
        flight_radius = 20 * 1e3
        distance_travelled = self.__W_speed * (t - self.__t_start)
        if distance_travelled >= flight_radius:
            return False
        else:
            return True

    def where_meters(self, t):

        x = - (t - self.__t_start) * self.__W_speed * np.sin(np.deg2rad(self.__W_direction))
        y = - (t - self.__t_start) * self.__W_speed * np.cos(np.deg2rad(self.__W_direction))
        return (x, y)

    def where_meters_wind_shift(self, t):

        error = 2 *(t - self.__t_start) + 96.36 - 330

        x = - (t - self.__t_start) * self.__W_speed * np.sin(np.deg2rad(self.__W_direction))
        y = - (t - self.__t_start) * self.__W_speed * np.cos(np.deg2rad(self.__W_direction))

        x = x + error * np.sin(np.deg2rad(self.__W_direction))
        y = y + error * np.cos(np.deg2rad(self.__W_direction))

        return (x,y)


    def where_geo(self, t):
        if self.where_meters(t) == None:
            return None
        else:
            x, y = self.where_meters(t)
            lat, lgn = convert_coordinates_xy2ll(14.4747, -90.8806, x, y)
            return lat, lgn

    def trajectory(self, t):
        if self.in_flight_zone(t) == True:
            t_max = 20 * 1e3 / self.__W_speed
            A = self.where_geo(t_max - 1)
            return [(14.4747, -90.8806), A]
        else:
            print(
                "Plume out of legal flight zone")  # If the plume is outside the flight zone, it is useless to plot its trajectory.
            return None

if __name__ == '__main__':
    p = Plume(0,10,20)
    print(p.where_meters(400))