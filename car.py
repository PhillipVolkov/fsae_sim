import numpy as np
from cones import yellow_cones, blue_cones
from ai import AI

STEERING_THRESHOLD = 1e-2
PERCEPT_R = 50
CANVAS_W = 700
CANVAS_H = 700
STATE_NOISE_STD = 0.01
CONTORL_NOISE_STD = 0.05

class Car:
  def __init__(self):
    self.w = 5
    self.l = 10
    self.ai = AI()
    self.reset()

  def reset(self):
    self.x = 0
    self.y = 50
    self.theta = 0

  def update(self, dt):
    perception = self.detect_cones()
    state = (self.x, self.y, self.theta)
    control = self.ai.compute(perception, state)
    self.prop_dynamics(control, dt)

  def detect_cones(self):
    left_cones = []
    right_cones = []
    for x, y in blue_cones:
      x *= 0.01 * CANVAS_W
      y *= 0.01 * CANVAS_H
      if (x - self.x) * (x - self.x) + (y - self.y) * (y - self.y) < PERCEPT_R * PERCEPT_R:
        left_cones.append(np.array([x, y]))
    for x, y in yellow_cones:
      x *= 0.01 * CANVAS_W
      y *= 0.01 * CANVAS_H
      if (x - self.x) * (x - self.x) + (y - self.y) * (y - self.y) < PERCEPT_R * PERCEPT_R:
        right_cones.append(np.array([x, y]))
    return left_cones, right_cones

  def prop_dynamics(self, control, dt):
    steer, vel = control
    steer += np.random.normal(0, CONTORL_NOISE_STD)
    if np.abs(steer) < STEERING_THRESHOLD:
      self.x += vel * np.cos(self.theta) * dt
      self.y += vel * np.sin(self.theta) * dt
    else:
      tan = np.tan(steer)
      new_theta = self.theta + vel / self.l * tan * dt
      self.x += self.l / tan * (np.sin(new_theta) - np.sin(self.theta))
      self.y += self.l / tan * (np.cos(self.theta) - np.cos(new_theta))
      self.theta = new_theta
    self.x += np.random.normal(0, STATE_NOISE_STD)
    self.y += np.random.normal(0, STATE_NOISE_STD)
    self.theta += np.random.normal(0, STATE_NOISE_STD)
