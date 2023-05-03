import numpy as np
import time

from cones import yellow_cones, blue_cones
from ai import AI, CAR_LENGTH, CAR_WIDTH

STEERING_THRESHOLD = 1e-4
PERCEPT_RADIUS = 10
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
    self.y = 45
    self.theta = 0
    # controls
    self.steer = 0
    self.steerAngle = 0
    self.vel = 0
    self.target_vel = 0
    self.fc = 0
    self.accel = 0
    self.throttle = 0
    self.prevTime = time.time()

  def update(self, dt):
    percept = self.detect_cones()
    self.steer, self.steerAngle, self.throttle, self.vel, self.target_vel, self.fc, self.accel = self.ai.compute(percept, self.vel, self.accel, time.time()-self.prevTime)
    self.prevTime = time.time()
    self.step(dt)

  def detect_cones(self):
    left_cones = []
    right_cones = []
    for x, y in blue_cones:
      cone = self.relative_pos((x, y))
      if np.linalg.norm(cone) < PERCEPT_RADIUS:
        left_cones.append(cone)
    for x, y in yellow_cones:
      cone = self.relative_pos((x, y))
      if np.linalg.norm(cone) < PERCEPT_RADIUS:
        right_cones.append(cone)
    return left_cones, right_cones

  def step(self, dt):
    self.steerAngle += np.random.normal(0, NOISE)
    self.vel += np.random.normal(0, NOISE)
    if np.abs(self.steerAngle) < STEERING_THRESHOLD:
      self.x += self.vel * np.cos(self.theta) * dt
      self.y += self.vel * np.sin(self.theta) * dt
    else:
      tan = np.tan(self.steerAngle)
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
