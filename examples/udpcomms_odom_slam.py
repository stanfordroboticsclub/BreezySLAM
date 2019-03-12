#!/usr/bin/env python3

'''
rpslam.py : BreezySLAM Python with SLAMTECH RP A1 Lidar
                 
Copyright (C) 2018 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

MAP_SIZE_PIXELS         = 2000
MAP_SIZE_METERS         = 20

# Ideally we could use all 250 or so samples that the RPLidar delivers in one 
# scan, but on slower computers you'll get an empty map and unchanging position
# at that rate.
MIN_SAMPLES   = 50

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel

from roboviz import MapVisualizer

from UDPComms import Subscriber
import time, math


# globals are extra jank but work as a demo
last_odom = time.time()
wheel_l = 0
wheel_r = 0

WHEEL_BASE = 19 * 25.4
WHEEL_RAD = 6.5 * 25.4 /2

def get_odom(odom):
        print("running")
        global last_odom
        global wheel_r
        global wheel_l
        l, r = odom.get()

        # http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/odometry.html

        dist_l = (l - wheel_l) * 2 * math.pi * WHEEL_RAD
        dist_r = (r - wheel_r) * 2 * math.pi * WHEEL_RAD

        wheel_l = l
        wheel_r = r

        v_right   = 0
        v_forward = (dist_l + dist_r) / 2
        v_th      = (dist_r - dist_l) / WHEEL_BASE

        # we don't do our own position tracking becasue breezy does it already
        # delta_x = (v_forward * math.cos(self.th + v_th/2) ) # - v_right * math.sin(th)) # needed fro omni robots
        # delta_y = (v_forward * math.sin(self.th + v_th/2) ) # + v_right * math.cos(th))

        # self.x += delta_x;
        # self.y += delta_y;
        # self.th += v_th;

        dt = time.time() - last_odom

        last_odom = time.time()

        print((v_forward, v_th, dt))
        return (-v_forward, -v_th, dt)



if __name__ == '__main__':
    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Set up a SLAM display
    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

    # Initialize an empty trajectory
    trajectory = []

    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    # Create an iterator to collect scan data from the RPLidar
    # iterator = lidar.iter_scans()

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles    = None


    sub = Subscriber(8110, timeout=4)
    odom = Subscriber(8820, timeout=4)

    sub.recv()
    wheel_l, wheel_r = odom.recv()


    while True:

        # Extract (quality, angle, distance) triples from current scan
        # items = [item for item in next(iterator)]
        items = sub.get()

        # Extract distances and angles from triples
        distances = [item[2] for item in items]
        angles    = [item[1] for item in items]

        # Update SLAM with current Lidar scan and scan angles if adequate
        if len(distances) > MIN_SAMPLES:
            slam.update(distances, get_odom(odom), scan_angles_degrees=angles)
            previous_distances = distances.copy()
            previous_angles    = angles.copy()

        # If not adequate, use previous
        elif previous_distances is not None:
            print("not enoguh")
            slam.update(previous_distances, get_odom(odom), scan_angles_degrees=previous_angles)

        # Get current robot position
        x, y, theta = slam.getpos()

        # Get current map bytes as grayscale
        slam.getmap(mapbytes)

        # Display map and robot pose, exiting gracefully if user closes it
        if not viz.display(x/1000., y/1000., theta, mapbytes):
            exit(0)
 
