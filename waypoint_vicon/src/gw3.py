#!/usr/bin/env python
from generate_waypoint import Generate_Point
import math

if __name__ == '__main__':
    gw2 = Generate_Point(2000,math.pi/4,0.8*1.414)
    gw2.run()