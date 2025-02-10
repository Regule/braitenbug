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


class NormalizedLaserScan:

    __EPSILON: float = 0.0001

    def __init__(self, scan: LaserScan|None= None)->None:
        if scan is None:
            self.__distances = None
            return
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
        if self.__distances is None:
            return NormaizedDistance(angle, 1.0)
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
        self.scan = NormalizedLaserScan()
    
    def scan_callback(self, msg: LaserScan):
        self.scan = msg
#==================================================================================================
#                                    USER INTERFACE 
#==================================================================================================

class ConeConfig:

    def __init__(self,
                 position: tuple[int,int],
                 radius: float = 100,
                 angle: float = 0.0,
                 width: float = np.pi/2
                 )->None:
        self.position: list[int,int] = list(position)
        self.radius: float = radius
        self.angle: float = angle
        self.width: float = width


class Cone:

    def __init__(self,
                 position: tuple[int,int],
                 radius: float = 100,
                 angle: float = 0.0,
                 width: float = np.pi/2
                 )-> None:
        self.__config = ConeConfig(position, radius, angle, width)
        self.scan: NormalizedLaserScan = NormalizedLaserScan()
        self.start = 0
        self.end = 0


    def render(self, surface: pg.Surface)-> None:

    
        measurements = self.scan[self.start:self.end]
        for measurement in measurements:
            line_lengths *= measurement.distance * self.radius
            angle = geo.angle_ros2_to_pygame(measirement.angle)
            endpoint = geo.polar_to_cartesian(line_length, angle)
            endpoint[0] = int(endpoint[0] + self.position[0])
            endpoint[1] = int(endpoint[1] + self.position[1])
            pg.draw.line(surface,
                         (255, 0, 0),
                         (self.position[0], self.position[1]),
                         (endpoint[0], endpoint[1]),
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

#==================================================================================================
#                                        MAIN 
#==================================================================================================

def main(args=None):
    rclpy.init(args=args)

    monitor = LidarMonitor()
    #ui = LidarUI(monitor)
    #ui.run()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
