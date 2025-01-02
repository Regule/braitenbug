#!/usr/bin/env python3

import pygame as pg
import math
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class Wheel:

    def __init__(self,
                 position: tuple[int,int],
                 radius: float = 100,
                 )-> None:
        self.__position: list[int,int] = list(position)
        self.__radius: float = radius
        self.measurements: list[float] = [0.0, 0.0, 0.0, 0.0, 0.0]

    def render(self, surface: pg.Surface)-> None:
        for i, distance in enumerate(self.measurements):
            angle = 0 - math.radians(i * (360 / (len(self.measurements))) - 90 )
            line_length = self.__radius * distance
            end_x = self.__position[0] + line_length * math.cos(angle)
            end_y = self.__position[1] + line_length * math.sin(angle)
            print(f'{end_x}({type(end_x)})')
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
        self.__wheel.measurements = self.__node.scan

    def __render(self)-> None:
        self.__display.fill((0,0,0))
        self.__wheel.render(self.__display)
        pg.display.flip()
        self.__clock.tick(60) # FPS cap

    def __cleanup(self)-> None:
        pg.quit()


class LidarMonitor(Node):
    def __init__(self):
        super().__init__('lidar_monitor')
        self._subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.scan = np.ones(10)
        self.max_distance = 1.0
    
    def scan_callback(self, msg: LaserScan):
        self.max_distance = msg.range_max
        self.scan = np.asarray(msg.ranges, dtype=np.int64)
        

def main(args=None):
    rclpy.init(args=args)

    monitor = LidarMonitor()
    ui = LidarUI(monitor)
    ui.run()

    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()