#!/usr/bin/env python
from follower import Follower

if __name__ == '__main__':
    follower = Follower("/crazyflie1/controller/frame")
    follower.run()
