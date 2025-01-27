#!/usr/bin/env python3

import pygame as pg
import numpy as np
import numpy.typing as npt
import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import braitenbug.geometry as geo


class NormalizedDistance:

    def __init__(self,
                 angle: float,
                 distance: float
                 )-> None:
        self.angle = angle
        self.distance = distance


class NormalizedLidarScan:

    __EPSILON: float = 0.0001

    def __init__(self, scan: LaserScan)->None:
        self.__validate_scan(scan)
        self.__angle_min = scan.angle_min
        self.__step = scan.angle_increment
        distances = np.asarray(scan.ranges, dtype=np.float64)
        self.__distances = (distances - scan.range_min) / (scan.range_max-scan.range_min)

    def __getitem__(self, key:float|slice)-> NormalizedDistance| list[NormalizedDistance]:
        if isinstance(key, float):
            return self.__get_readouts_from_range(key, key)
        if isinstance(key, slice):
            return self.__get_readouts_from_range(key.start, key.stop)
        raise KeyError(f'NormalizedLidarScan must be indexed with float or foat based slice')

    def __validate_scan(self, scan: LaserScan)-> None:
        scan_range = scan.angle_max - scan.angle_min
        if abs(scan_range - 2 * np.pi) > self.__EPSILON:
            raise ValueError('NormalizedLidarScan is intended for use with 360 lidars,' +
                             f' insted lidar wiht range {scan_range}rad was used.')
        calculated_angle_max = self.__angle_min + self.__step * len(self.__distances)
        if abs(calculated_angle_max - self.__angle_max) > self.__EPSILON:
            raise ValueError(f'Calculated max angle is different than one given by lidar')

    def __get_index_for_angle(self, angle: float)-> int:
        angle -= self.__angle_min
        return angle // self.__step

    def __get_single_readout(self, angle: float) -> NormalizedDistance:
        return NormalizedDistance(angle, self.__distances[self.__get_index_for_angle(angle)])

    def __get_readouts_from_range(self, start: float, end: float)-> list[NormalizedDistance]:
        distances = []
        start = self.__to_basic_angle(start)
        end = self.__to_basic_angle(end)
        angle = start
        if start <= end:
            while angle <= end:
                distances.append(self.__get_single_readout(angle))
                angle += self.__step
        else:
            angle_max = self.__angle_min + self.__step * (len(self.__distances)-1)
            while angle <= angle_max:
                distances.append(self.__get_single_readout(angle))
                angle += self.__step
            angle = self.__angle_min
            while angle <= end:
                distances.append(self.__get_single_readout(angle))
                angle += self.__step


        return distances
        

    @staticmethod
    def __to_basic_angle(angle: float)-> float:
        while angle > 2*np.pi:
            angle -= 2*np.pi
        while angle < 0:
            angle += 2*np.pi
        return angle

#==================================================================================================
#                                        NODE
#==================================================================================================

class LidarMonitor(Node):
    def __init__(self):
        super().__init__('whiskers_experiment')
        self._subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.max_distance = 1.0
        self.scan = LaserScan()
    
    def scan_callback(self, msg: LaserScan):
        self.scan = msg
