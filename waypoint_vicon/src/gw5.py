#!/usr/bin/env python
from generate_waypoint import Generate_Point
import math

if __name__ == '__main__':
    gw3 = Generate_Point(2000,3*math.pi/4,0.8*1.414)
    gw3.run()