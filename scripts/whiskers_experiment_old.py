#!/usr/bin/env python3

import pygame as pg
import numpy as np
import numpy.typing as npt
import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import braitenbug.geometry as geo


class WrappedList:

    def __init__(self, contents: list)-> None:
        self.__contents = contents
    
    def __getitem__(self, idx:int):
        if not self.__contents:
            raise IndexError(f'Attempted to use a empty wrapped list')
        while idx < 0:
            idx += len(self.__contents)
        while idx >= len(self.__contents):
            idx -= len(self.__contents)
        return self.__contents[idx]

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
        ranges = np.asarray(self.scan.ranges)
        line_lengths = (ranges + self.scan.range_min) / (self.scan.range_max - self.scan.range_min)
        line_lengths *= self.__radius
        angles = np.linspace(self.scan.angle_min, self.scan.angle_max, len(self.scan.ranges))
        angles = geo.angle_ros2_to_pygame(angles)
        endpoints = geo.polar_to_cartesian_matrix(line_lengths, angles)

        
        for x, y in endpoints.T:
            x = int(x+self.__position[0])
            y = int(y+self.__position[1])
            pg.draw.line(surface,
                         (0, 255, 0),
                         (self.__position[0], self.__position[1]),
                         (x, y),
                         2)
            
class Cone:

    def __init__(self,
                 position: tuple[int,int],
                 radius: float = 100,
                 angle: float = 0.0,
                 width: float = np.pi/2
                 )-> None:
        self.position: list[int,int] = list(position)
        self.radius: float = radius
        self.angle: float = angle
        self.width: float = width
        self.scan: LaserScan = LaserScan()
        self.start = 0
        self.end = 0

    @staticmethod    
    def calculate_indexes(cone_angle: float, cone_width:float,
                          angle_min:float, angle_increment: float):
        start = int((cone_angle-cone_width/2 - angle_min)/angle_increment)
        end = int((cone_angle+cone_width/2 - angle_min)/angle_increment)
        return start, end

    def render(self, surface: pg.Surface)-> None:
        if not self.scan.ranges:
            return
        self.start, self.end = Cone.calculate_indexes(self.angle,
                                                      self.width,
                                                      self.scan.angle_min,
                                                      self.scan.angle_increment)
        if self.start<0 or self.end>=len(self.scan.ranges):
            rclpy.logging.get_logger('global').warn(f'RANGE {self.start} -> {self.end}')
            return

    
        ranges = np.asarray(self.scan.ranges[self.start:self.end])
        line_lengths = (ranges + self.scan.range_min) / (self.scan.range_max - self.scan.range_min)
        line_lengths *= self.radius
        angles = np.linspace(self.angle-self.width/2, self.angle+self.width/2, len(line_lengths))

    
        
        angles = geo.angle_ros2_to_pygame(angles)
        endpoints = geo.polar_to_cartesian_matrix(line_lengths, angles)

        for x, y in endpoints.T:
            x = int(x+self.position[0])
            y = int(y+self.position[1])
            pg.draw.line(surface,
                         (255, 0, 0),
                         (self.position[0], self.position[1]),
                         (x, y),
                         2)

class LidarUI:

    def __init__(self, node: Node)-> None:
        self.__running: bool = False
        self.__display: pg.Surface|None = None
        self.__size: tuple[int, int] = (640, 400)
        self.__clock: pg.time.Clock = pg.time.Clock()
        self.__node: Node = node
        self.__angle_step: float = np.pi/64

        center: tuple[int,int] = (self.__size[0]//2, self.__size[1]//2)
        self.__wheel:Wheel = Wheel(center)
        self.__cone:Cone = Cone(center)
        self.__font =  None

    def run(self)-> None:
        self.__initialize()
        while(self.__running):
            for event in pg.event.get():
                self.__handle_event(event)
            self.__handle_keyboard()
            self.__update_logic()
            self.__render()
        self.__cleanup()

    def __initialize(self)-> None:
        pg.init()
        self.__display = pg.display.set_mode(self.__size,
                                            pg.HWSURFACE | pg.DOUBLEBUF)
        self.__running = True
        self.__font = pg.font.Font(pg.font.get_default_font(), 20)

    def __handle_event(self, event: pg.event.Event)-> None:
        if event.type == pg.QUIT:
            self.__running = False
        elif event.type == pg.KEYDOWN:
            if event.key == pg.K_q or event.key == pg.K_ESCAPE:
                self.__running = False

    def __handle_keyboard(self):
        keys = pg.key.get_pressed()
        if keys[pg.K_UP]:
            self.__cone.width += self.__angle_step
        if keys[pg.K_DOWN]:
            self.__cone.width -= self.__angle_step
        if keys[pg.K_LEFT]:
            self.__cone.angle += self.__angle_step
        if keys[pg.K_RIGHT]:
            self.__cone.angle -= self.__angle_step


    def __update_logic(self)-> None:
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.__wheel.scan = self.__node.scan
        self.__cone.scan = self.__node.scan

    def __render(self)-> None:
        self.__display.fill((0,0,0))
        self.__wheel.render(self.__display)
        self.__cone.render(self.__display)
        msg = f'Start={self.__cone.start}  End={self.__cone.end}'
        label = self.__font.render(msg, 1, (255,255,0))
        self.__display.blit(label, (0, self.__size[1]-30))
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
