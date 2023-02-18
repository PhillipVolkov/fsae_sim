from cones import yellow_cones, blue_cones
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = 'hide'
import pygame
from pygame import gfxdraw
import numpy as np
from time import perf_counter
from car import Car, PERCEPT_R

GREY = (200, 200, 200)
YELLOW = (255, 235, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
WHITE = (255, 255, 255)
CANVAS_W = 700
CANVAS_H = 700

class Sim:
  def __init__(self):
    pygame.init()
    pygame.display.set_caption('FSAE Simulation')
    self.canvas = pygame.display.set_mode((CANVAS_W, CANVAS_H))
    self.running = True
    self.prev_time = perf_counter()
    self.events = {'quit': False, 'r': False, 'q': False}
    self.car = Car()

  def update_events(self):
    for event in pygame.event.get():
      self.events['quit'] = event.type == pygame.QUIT
      if event.type == pygame.KEYDOWN:
        if event.key == pygame.K_r:
          self.events['r'] = True
        if event.key == pygame.K_q:
          self.events['q'] = True
      if event.type == pygame.KEYUP:
        if event.key == pygame.K_r:
          self.events['r'] = False
        if event.key == pygame.K_q:
          self.events['q'] = False

  def update(self):
    curr_time = perf_counter()
    dt = curr_time - self.prev_time
    self.prev_time = curr_time
    self.update_events()
    self.car.update(dt)
    if self.events['r']:
      self.car.reset()
    if self.events['quit'] or self.events['q']:
      self.running = False

  def rot_mat(self, theta):
    cos = np.cos(theta)
    sin = np.sin(theta)
    return np.array([(cos, -sin), (sin, cos)])

  def render(self):
    self.canvas.fill(GREY)
    for x, y in yellow_cones:
      x = int(0.01 * CANVAS_W * x)
      y = int(0.01 * CANVAS_H * y)
      color = WHITE if (x - self.car.x) * (x - self.car.x) + (y - self.car.y) * (y - self.car.y) < PERCEPT_R * PERCEPT_R else YELLOW
      gfxdraw.aacircle(self.canvas, int(CANVAS_W / 2 + x), int(CANVAS_H / 2 - y), 2, color)
      gfxdraw.filled_circle(self.canvas, int(CANVAS_W / 2 + x), int(CANVAS_H / 2 - y), 2, color)
    for x, y in blue_cones:
      x = int(0.01 * CANVAS_W * x)
      y = int(0.01 * CANVAS_H * y)
      color = WHITE if (x - self.car.x) * (x - self.car.x) + (y - self.car.y) * (y - self.car.y) < PERCEPT_R * PERCEPT_R else BLUE
      gfxdraw.aacircle(self.canvas, int(CANVAS_W / 2 + x), int(CANVAS_H / 2 - y), 2, color)
      gfxdraw.filled_circle(self.canvas, int(CANVAS_W / 2 + x), int(CANVAS_H / 2 - y), 2, color)
    car_poly = np.array([
      (0.5 * self.car.l, 0.5 * self.car.w),
      (-0.5 * self.car.l, 0.5 * self.car.w),
      (-0.5 * self.car.l, -0.5 * self.car.w),
      (0.5 * self.car.l, -0.5 * self.car.w)])
    car_points = car_poly @ self.rot_mat(self.car.theta) + (CANVAS_W / 2 + self.car.x, CANVAS_H / 2 - self.car.y)
    gfxdraw.aapolygon(self.canvas, car_points, RED)
    gfxdraw.filled_polygon(self.canvas, car_points, RED)
    pygame.display.update()
