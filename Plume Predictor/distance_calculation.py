import math
import geopy.distance
import numpy as np
from geopy.point import Point
import geopy.distance
from geopy import distance
from geopy.point import Point

def distance_entre_deux_points(x1, y1, x2, y2):
    point1 = np.array([x1, y1])
    point2 = np.array([x2, y2])
    return np.linalg.norm(point2 - point1)


def calculate_distance(lat1, lon1, lat2, lon2, alt_plane):
    c1 = (lat1, lon1)
    c2 = (lat2, lon2)
    alt_fuego = 3768
    intem = (alt_fuego - alt_plane)**2 + (geopy.distance.distance(c1, c2).m)**2

    return np.sqrt(intem)


def convert_coordinates_xy2ll(latitude_origin, longitude_origin, x, y):
    origin_point = Point(latitude_origin, longitude_origin)  # We define the orign point
    new_point = distance.distance(meters=y).destination(origin_point, 0)  # We generate a new point on the y axis
    new_point = distance.distance(meters=x).destination(new_point, 90)  # We move this point then on this x axis
    new_latitude = new_point.latitude
    new_longitude = new_point.longitude
    return new_latitude, new_longitude


def convert_coordinates_ll2xy(new_latitude, new_longitude):

    latitude_origin = 14.4747
    longitude_origin = -90.8806

    origin_point = distance.distance(meters=0).destination(Point(latitude_origin, longitude_origin), 0)
    distance_y = distance.distance(origin_point, Point(new_latitude, longitude_origin)).meters
    distance_x = distance.distance(origin_point, Point(latitude_origin, new_longitude)).meters

    if new_latitude - latitude_origin < 0:
        distance_y = -distance_y

    if  new_longitude- longitude_origin < 0:
        distance_x = -distance_x

    return distance_x, distance_y

if __name__ == "__main__":

    latitude_origin = 14.4747
    longitude_origin = -90.8806

    lat, lgn = convert_coordinates_xy2ll(latitude_origin, longitude_origin, -1000, -1000)
    x,y = convert_coordinates_ll2xy(lat, lgn)




