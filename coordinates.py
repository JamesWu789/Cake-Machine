#from __future__ import division
import math
from logging_config import *

class Coordinates(object):
    """ This object represent machine coordinates.
        Machine supports 3 axis, so there are X, Y and Z.
    """
    def __init__(self, x, y, z, e):
        """ Create object.
        :param x: x coordinated.
        :param y: y coordinated.
        :param z: z coordinated.
        """
        self.x = round(x, 10)
        self.y = round(y, 10)
        self.z = round(z, 10)
        self.e = round(e, 10)

    def coordinates(self, x,y,z,e): 
        """ Get X, Y and Z values as Coord object.
        :param default: Default values, if any of coordinates is not specified.
        :param multiply: If value exist, multiply it by this value.
        :return: Coord object.
        """
        x *= multiply_x #get各軸g-code的位置
        y *= multiply_y
        z *= multiply_z
        e *= multiply_e
        return Coordinates(x, y, z, e)  

    def is_in_aabb(self, p1, p2):  ###限制範圍要再修改 (p1到p2的範圍)
        """ Check coordinates are in aabb(Axis-Aligned Bounding Box).
            aabb is specified with two points. E is ignored.
        :param p1: First point in Coord object.
        :param p2: Second point in Coord object.
        :return: boolean value.
        """
        min_x, max_x = sorted((p1.x, p2.x))
        min_y, max_y = sorted((p1.y, p2.y))
        min_z, max_z = sorted((p1.z, p2.z))
        if self.x < min_x or self.y < min_y or self.z < min_z:
            return False
        if self.x > max_x or self.y > max_y or self.z > max_z:
            return False
        return True

    def length(self):
        """ Calculate the length of vector.
        :return: Vector length.
        """
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z
                         + self.e * self.e)

    def round(self, base_x, base_y, base_z, base_e):
        """ Round values to specified base, ie 0.49 with base 0.25 will be 0.5.
        :param base_x: Base for x axis.
        :param base_y: Base for y axis.
        :param base_z: Base for z axis.
        :param base_e: Base for e axis.
        :return: New rounded object.
        """
        return Coordinates(round(self.x / base_x) * base_x,
                           round(self.y / base_y) * base_y,
                           round(self.z / base_z) * base_z,
                           round(self.e / base_e) * base_e)
    
    def is_zero(self):
        """ Check if all coordinates are zero.
        :return: boolean value.
        """
        return (self.x == 0.0 and self.y == 0.0 and self.z == 0.0 and self.e == 0.0)

    def __add__(self, other):
        return Coordinates(self.x + other.x, self.y + other.y,
                           self.z + other.z, self.e + other.e)    
    
    def __sub__(self, other):
        return Coordinates(self.x - other.x, self.y - other.y,
                           self.z - other.z, self.e - other.e)
    
    def __truediv__(self, other):
        return Coordinates(self.x / other.x, self.y / other.y, self.z / other.z, self.e / other.e)
    
    def __mul__(self, other):
        return Coordinates(self.x * other.x, self.y * other.y, self.z * other.z, self.e * other.e)

    # build in function implementation  #讓參數能以coordinate(x,y,z,e)的形式，加減乘除...

#    def __truediv__(self, v):
#        return Coordinates(self.x / v, self.y / v, self.z / v, self.e / v)

    def __eq__(self, other):
        return self.x == other.x, self.y == other.y, self.z == other.z, self.e == other.e

    def __str__(self):
        return '(' + str(self.x) + ', ' + str(self.y) + ', ' + str(self.z) + ', ' + str(self.e) + ')'

    def __abs__(self):
        return Coordinates(abs(self.x), abs(self.y), abs(self.z),  abs(self.e))
    
    def __ceil__(self):
        return Coordinates(math.ceil(self.x), math.ceil(self.y), math.ceil(self.z),  math.ceil(self.e))
    
    def __floor__(self):
        return Coordinates(math.floor(self.x), math.floor(self.y), math.floor(self.z),  math.floor(self.e))
    
    def __gt__(self, other):
        return self.x > other.x, self.y > other.y, self.z > other.z, self.e > other.e

    def __lt__(self, other):
        return self.x < other.x, self.y < other.y, self.z < other.z, self.e < other.e
    
