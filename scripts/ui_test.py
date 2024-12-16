#!/usr/bin/env python3

import pygame as pg
import math


class Wheel:

    def __init__(self,
                 position: tuple[int,int],
                 radius: float = 10.0,
                 angle: float = 0.0,
                 lines: int = 8)-> None:
        self.__position: list[int,int] = list(position)
        self.__radius: float = radius
        self.__angle: float = angle
        self.__lines:int = lines

    def render(self, surface: pg.Surface)-> None:
        for i in range(self.__lines):
            angle = math.radians(i * (360 / self.__lines) + self.__angle)
            end_x = self.__position[0] + self.__radius * math.cos(angle)
            end_y = self.__position[1] + self.__radius * math.sin(angle)
            pg.draw.line(surface,
                         (255, 255, 255),
                         (self.__position[0], self.__position[1]),
                         (end_x, end_y),
                         2)
    
    def turn(self, angle: float = 1.0)-> None:
        self.__angle += angle

    def grow(self, amount: float = 1.0)-> None:
        self.__radius += amount

    def move(self, amount: tuple[int, int])-> None:
        self.__position[0] += amount[0]
        self.__position[1] += amount[1]



class DemoUi:

    def __init__(self)-> None:
        self.__running: bool = False
        self.__display: pg.Surface|None = None
        self.__size: tuple[int, int] = (640, 400)
        self.__clock = pg.time.Clock()

        center: tuple[int,int] = (self.__size[0]//2, self.__size[1]//2)
        self.__wheel:Wheel = Wheel(center)

    def run(self)-> None:
        self.__initialize()
        while(self.__running):
            self.__handle_keys()
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

    def __handle_keys(self)-> None:
        keys = pg.key.get_pressed()
        if keys[pg.K_UP]:
            self.__wheel.grow(1)
        elif keys[pg.K_DOWN]:
            self.__wheel.grow(-1)
        if keys[pg.K_LEFT]:
            self.__wheel.turn(-1)
        elif keys[pg.K_RIGHT]:
            self.__wheel.turn(1)
        if keys[pg.K_w]:
            self.__wheel.move((0,-1))
        elif keys[pg.K_s]:
            self.__wheel.move((0,1))
        if keys[pg.K_a]:
            self.__wheel.move((-1,0))
        elif keys[pg.K_d]:
            self.__wheel.move((1,0))

    def __update_logic(self)-> None:
        pass

    def __render(self)-> None:
        self.__display.fill((0,0,0))
        self.__wheel.render(self.__display)
        pg.display.flip()
        self.__clock.tick(60) # FPS cap

    def __cleanup(self)-> None:
        pg.quit()

def main():
    user_interface = DemoUi()
    user_interface.run()


if __name__ == '__main__':
    main()
