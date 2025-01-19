#!/usr/bin/env python3

import pygame as pg
import numpy as np
import numpy.typing as npt
import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import braitenbug.geometry as geo


class NormalizedDistanceArray:

    def __init__(self,
                 angles: list[float],
                 distances: list[float]
                 )-> None:
        self.angles = angles
        self.distances = distances


class NormalizedLidarScan:

    __EPSILON: float = 0.0001

    def __init__(self, scan: LaserScan)->None:
        scan_range = scan.angle_max - scan.angle_min
        if scan_range - 2 * np.pi < self.__EPSILON:
            raise ValueError('NormalizedLidarScan is intended for use with 360 lidars,' +
                             f' insted lidar wiht range {scan_range}rad was used.')
        self.__angle_min = scan.angle_min
        distances = np.asarray(scan.ranges, dtype=np.float64)
        self.__distances = (distances - scan.range_min) / (scan.range_max-scan.range_min)


