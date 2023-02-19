import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = 'hide'
import pygame
from pygame import gfxdraw

import numpy as np
from time import perf_counter

from cones import yellow_cones, blue_cones
from car import Car, PERCEPT_RADIUS

GREY = (200, 200, 200)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 235, 0)
GREEN = (0, 160, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

CANVAS_WIDTH = 700
CANVAS_HEIGHT = 700

class Sim:
  def __init__(self):
    pygame.init()
    pygame.display.set_caption('FSAE Simulation')
    self.canvas = pygame.display.set_mode((CANVAS_WIDTH, CANVAS_HEIGHT))
    self.running = True
    self.prev_time = perf_counter()
    self.events = {'quit': False, 'r': False, 'q': False, 'h': False}
    self.car = Car()
    self.debug = False
    self.hide_cones = False

  def get_events(self):
    events = {'space': False, 'quit': False, 'r': False, 'q': False, 'h': False}
    for event in pygame.event.get():
      events['quit'] = event.type == pygame.QUIT
      if event.type == pygame.KEYDOWN:
        if event.key == pygame.K_SPACE:
          events['space'] = True
        if event.key == pygame.K_r:
          events['r'] = True
        if event.key == pygame.K_q:
          events['q'] = True
        if event.key == pygame.K_h:
          events['h'] = True
    return events

  def update(self):
    curr_time = perf_counter()
    dt = curr_time - self.prev_time
    self.prev_time = curr_time
    self.ref_pt = self.car.update(dt)
    events = self.get_events()
    if events['space']:
      self.debug = not self.debug
    if events['r']:
      self.car.reset()
    if events['quit'] or events['q']:
      self.running = False
    if events['h']:
      self.hide_cones = not self.hide_cones

  def rot_mat(self, theta):
    cos = np.cos(theta)
    sin = np.sin(theta)
    return np.array([(cos, sin), (-sin, cos)])
  
  def canvas_coords(self, points):
    new_points = points.copy()
    new_points[:, 0] = CANVAS_WIDTH / 2 + points[:, 0]
    new_points[:, 1] = CANVAS_HEIGHT / 2 - points[:, 1]
    return new_points
  
  def render_cones(self, cones, color):
    for x, y in cones:
      x *= 0.01 * CANVAS_WIDTH
      y *= 0.01 * CANVAS_HEIGHT
      if self.hide_cones and not (x - self.car.x) * (x - self.car.x) + (y - self.car.y) * (y - self.car.y) < PERCEPT_RADIUS * PERCEPT_RADIUS:
        c = WHITE
      else:
        if self.debug and (x - self.car.x) * (x - self.car.x) + (y - self.car.y) * (y - self.car.y) < PERCEPT_RADIUS * PERCEPT_RADIUS:
          c = WHITE
        else:
          c = color
        x, y = self.canvas_coords(np.array([[x, y]]))[0]
        gfxdraw.aacircle(self.canvas, int(x), int(y), 2, c)
        gfxdraw.filled_circle(self.canvas, int(x), int(y), 2, c)

  def render_ref(self):
    if self.debug:
      x, y = self.canvas_coords(np.array([self.ref_pt]))[0]
      gfxdraw.aacircle(self.canvas, int(x), int(y), 2, GREEN)
      gfxdraw.filled_circle(self.canvas, int(x), int(y), 2, GREEN)

  def render_car(self):
    wheel_poly = np.array([(2, 1), (-2, 1), (-2, -1), (2, -1)])
    # front left wheel
    front_left = wheel_poly.copy() @ self.rot_mat(self.car.steer) + (0.3 * self.car.l, 0.6 * self.car.w)
    front_left = front_left @ self.rot_mat(self.car.theta) + (self.car.x, self.car.y)
    front_left = self.canvas_coords(front_left)
    gfxdraw.aapolygon(self.canvas, front_left, BLACK)
    gfxdraw.filled_polygon(self.canvas, front_left, BLACK)
    # front right wheel
    front_right = wheel_poly.copy() @ self.rot_mat(self.car.steer) + (0.3 * self.car.l, -0.6 * self.car.w)
    front_right = front_right @ self.rot_mat(self.car.theta) + (self.car.x, self.car.y)
    front_right = self.canvas_coords(front_right)
    gfxdraw.aapolygon(self.canvas, front_right, BLACK)
    gfxdraw.filled_polygon(self.canvas, front_right, BLACK)
    # back left wheel
    back_left =  wheel_poly.copy() + (-0.3 * self.car.l, 0.6 * self.car.w)
    back_left = back_left @ self.rot_mat(self.car.theta) + (self.car.x, self.car.y)
    back_left = self.canvas_coords(back_left)
    gfxdraw.aapolygon(self.canvas, back_left, BLACK)
    gfxdraw.filled_polygon(self.canvas, back_left, BLACK)
    # back right wheel
    back_right =  wheel_poly.copy() + (-0.3 * self.car.l, -0.6 * self.car.w)
    back_right = back_right @ self.rot_mat(self.car.theta) + (self.car.x, self.car.y)
    back_right = self.canvas_coords(back_right)
    gfxdraw.aapolygon(self.canvas, back_right, BLACK)
    gfxdraw.filled_polygon(self.canvas, back_right, BLACK)
    # car body
    car_poly = np.array([(0.5 * self.car.l, 0.5 * self.car.w), (-0.5 * self.car.l, 0.5 * self.car.w), (-0.5 * self.car.l, -0.5 * self.car.w), (0.5 * self.car.l, -0.5 * self.car.w)])
    car_points = car_poly @ self.rot_mat(self.car.theta) + (self.car.x, self.car.y)
    car_points = self.canvas_coords(car_points)
    gfxdraw.aapolygon(self.canvas, car_points, RED)
    gfxdraw.filled_polygon(self.canvas, car_points, RED)

  def render(self):
    self.canvas.fill(GREY)
    self.render_cones(blue_cones, BLUE)
    self.render_cones(yellow_cones, YELLOW)
    self.render_ref()
    self.render_car()
    pygame.display.update()
