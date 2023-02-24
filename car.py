import numpy as np

from cones import yellow_cones, blue_cones
from ai import AI, CAR_LENGTH, CAR_WIDTH

CANVAS_WIDTH = 700
CANVAS_HEIGHT = 700

STEERING_THRESHOLD = 1e-4
PERCEPT_RADIUS = 50
NOISE = 0.01

class Car:
  def __init__(self):
    self.l = CAR_LENGTH
    self.w = CAR_WIDTH
    self.ai = AI()
    self.reset()

  def reset(self):
    # state
    self.x = 0
    self.y = 50
    self.theta = 0
    # controls
    self.steer = 0
    self.vel = 0

  def update(self, dt):
    percept = self.detect_cones()
    self.steer, self.vel = self.ai.compute(percept)
    self.step(dt)

  def detect_cones(self):
    left_cones = []
    right_cones = []
    for x, y in blue_cones:
      x *= 0.01 * CANVAS_WIDTH
      y *= 0.01 * CANVAS_HEIGHT
      cone = self.relative_pos((x, y))
      if np.linalg.norm(cone) < PERCEPT_RADIUS:
        left_cones.append(cone)
    for x, y in yellow_cones:
      x *= 0.01 * CANVAS_WIDTH
      y *= 0.01 * CANVAS_HEIGHT
      cone = self.relative_pos((x, y))
      if np.linalg.norm(cone) < PERCEPT_RADIUS:
        right_cones.append(cone)
    return left_cones, right_cones

  def step(self, dt):
    self.steer += np.random.normal(0, NOISE)
    self.vel += np.random.normal(0, NOISE)
    if np.abs(self.steer) < STEERING_THRESHOLD:
      self.x += self.vel * np.cos(self.theta) * dt
      self.y += self.vel * np.sin(self.theta) * dt
    else:
      tan = np.tan(self.steer)
      new_theta = self.theta + self.vel / self.l * tan * dt
      self.x += self.l / tan * (np.sin(new_theta) - np.sin(self.theta))
      self.y += self.l / tan * (np.cos(self.theta) - np.cos(new_theta))
      self.theta = new_theta
    self.x += np.random.normal(0, NOISE)
    self.y += np.random.normal(0, NOISE)
    self.theta += np.random.normal(0, NOISE)

  def relative_pos(self, pt):
    x, y = pt
    cos = np.cos(self.theta)
    sin = np.sin(self.theta)
    R = np.array([[cos, sin], [-sin, cos]])
    return R @ np.array([x - self.x, y - self.y])
