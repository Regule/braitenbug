#!/usr/bin/env python3

import pygame as pg
import math
import rclpy
from rclpy.node import Node
from braitenbug_msgs.msg import Whiskers


class Wheel:

    def __init__(self,
                 position: tuple[int,int],
                 radius: float = 100,
                 )-> None:
        self.__position: list[int,int] = list(position)
        self.__radius: float = radius
        self._measurements: list[float] = [0.0, 0.0, 0.0, 0.0, 0.0]

    def render(self, surface: pg.Surface)-> None:
        for i, val in enumerate(self._measurements):
            angle = math.radians(i * (360 / len(self._measurements)))
            start_x = self.__position[0] + (self.__radius - val) * math.cos(angle)
            start_y = self.__position[1] + (self.__radius - val) * math.sin(angle)
            end_x = self.__position[0] + self.__radius * math.cos(angle)
            end_y = self.__position[1] + self.__radius * math.sin(angle)
            pg.draw.line(surface,
                         (0, 255, 0),
                         (self.__position[0], self.__position[1]),
                         (end_x, end_y),
                         2)
            pg.draw.line(surface,
                         (255, 0, 0),
                         (start_x, start_y),
                         (end_x, end_y),
                         2)

class WhiskersUI:

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

    def __render(self)-> None:
        self.__display.fill((0,0,0))
        self.__wheel.render(self.__display)
        pg.display.flip()
        self.__clock.tick(60) # FPS cap

    def __cleanup(self)-> None:
        pg.quit()

class WhiskersSubscriber(Node):

    def __init__(self):
        super().__init__('whiskers_subscriber')
        self._subscription = self.create_subscription(
            Whiskers,
            'whiskers',
            self.listener_callback,
            10)
        
        self.whisker_data = [0.0, 0.0, 0.0, 0.0, 0.0]

    def listener_callback(self, msg: Whiskers):
        self.whisker_data = [
            msg.side_right,
            msg.front_right,
            msg.center,
            msg.front_left,
            msg.side_left
        ]
        self.get_logger().info(f'Whiskers data = {self.whisker_data}')

def main(args=None):
    rclpy.init(args=args)

    whiskers = WhiskersSubscriber()
    ui = WhiskersUI(whiskers)
    ui.run()

    whiskers.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()