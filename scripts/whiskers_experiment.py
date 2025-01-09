#!/usr/bin/env python3

import pygame as pg
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def polar_to_cartesian_matrix(polar_matrix):
    """
    Convert a matrix of polar coordinates to Cartesian coordinates.
    
    Parameters:
    polar_matrix (numpy.ndarray): A 2xN matrix where the first row is r (radius) 
                                   and the second row is theta (angle in radians).
    
    Returns:
    numpy.ndarray: A 2xN matrix where the first row is x and the second row is y.
    """
    r = polar_matrix[0, :]
    theta = polar_matrix[1, :]
    
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    
    cartesian_matrix = np.vstack((x, y))
    return cartesian_matrix



class Wheel:

    def __init__(self,
                 position: tuple[int,int],
                 radius: float = 100,
                 )-> None:
        self.__position: list[int,int] = list(position)
        self.__radius: float = radius
        self.scan: LaserScan = LaserScan()

    def render(self, surface: pg.Surface)-> None:
        if not self.scan.ranges:
            return
        
        print('dupa')
        for i, distance in enumerate(self.scan.ranges):
            angle = i * self.scan.angle_increment
            angle = 0 - angle + math.pi/2 # From ROS2 to pygame
            scale = (distance + self.scan.range_min) / (self.scan.range_max - self.scan.range_min)
            line_length = self.__radius * scale
            end_x = self.__position[0] + line_length * math.cos(angle)
            end_y = self.__position[1] + line_length * math.sin(angle)
            pg.draw.line(surface,
                         (0, 255, 0),
                         (self.__position[0], self.__position[1]),
                         (end_x, end_y),
                         2)

class LidarUI:

    def __init__(self, node: Node)-> None:
        self.__running: bool = False
        self.__display: pg.Surface|None = None
        self.__size: tuple[int, int] = (640, 400)
        self.__clock: pg.time.Clock = pg.time.Clock()
        self.__node: Node = node

        center: tuple[int,int] = (self.__size[0]//2, self.__size[1]//2)
        self.__wheel:Wheel = Wheel(center)

    def run(self)-> None:
        self.__initialize()
        while(self.__running):
            for event in pg.event.get():
                self.__handle_event(event)
            self.__update_logic()
            self.__render()
        self.__cleanup()

    def __initialize(self)-> None:
        pg.init()
        self.__display = pg.display.set_mode(self.__size,
                                            pg.HWSURFACE | pg.DOUBLEBUF)
        self.__running = True

    def __handle_event(self, event: pg.event.Event)-> None:
        if event.type == pg.QUIT:
            self.__running = False
        elif event.type == pg.KEYDOWN:
            if event.key == pg.K_q or event.key == pg.K_ESCAPE:
                self.__running = False

    def __update_logic(self)-> None:
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.__wheel.scan = self.__node.scan

    def __render(self)-> None:
        self.__display.fill((0,0,0))
        self.__wheel.render(self.__display)
        pg.display.flip()
        self.__clock.tick(60) # FPS cap

    def __cleanup(self)-> None:
        pg.quit()


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
        

def main(args=None):
    rclpy.init(args=args)

    monitor = LidarMonitor()
    ui = LidarUI(monitor)
    ui.run()

    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()