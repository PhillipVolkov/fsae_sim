import numpy as np

CAR_LENGTH = 15
CAR_WIDTH = 5

AGGRESSION = 2.5

class AI:
  def __init__(self):
    self.reference = np.array([0, 0])

  def compute(self, perception, state):
    left_cones, right_cones = perception

    target_left = self.get_target(state, left_cones)
    target_right = self.get_target(state, right_cones)

    min_left_pairwise_dist = self.min_pairwise_dist(left_cones)
    min_right_pairwise_dist = self.min_pairwise_dist(right_cones)

    target_cross_track_dist = np.linalg.norm(target_left - target_right)
    adjusted_target_left = target_left + (target_right - target_left) / target_cross_track_dist * CAR_WIDTH
    adjusted_target_right = target_right + (target_left - target_right) / target_cross_track_dist * CAR_WIDTH

    if min_left_pairwise_dist < min_right_pairwise_dist:
      ratio = min_left_pairwise_dist / min_right_pairwise_dist
      p = 1 / (1 + np.exp(AGGRESSION * (ratio - 1)))
    else:
      ratio = min_right_pairwise_dist / min_left_pairwise_dist
      p = 1 - 1 / (1 + np.exp(AGGRESSION * (ratio - 1)))

    self.reference = p * adjusted_target_left + (1 - p) * adjusted_target_right

    ref_x, ref_y = self.transform(state, self.reference)
    turn_radius = (ref_x * ref_x + ref_y * ref_y) / (2 * ref_y)
    steer = np.arctan(CAR_LENGTH / turn_radius)

    # can be whateva lol
    vel = 200

    return steer, vel
  
  def min_pairwise_dist(self, cones):
    min_dist = np.inf

    for i in range(0, len(cones) - 1):
      dist = np.linalg.norm(cones[i] - cones[i + 1])

      if dist < min_dist:
        min_dist = dist

    return min_dist
  
  def get_target(self, state, cones):
    car_x, car_y, car_theta = state
    car_pos = np.array((car_x, car_y))

    target_cone = car_pos.copy()

    for cone in cones:
      if self.transform(state, cone)[0] > 0 and np.linalg.norm(cone - car_pos) > np.linalg.norm(target_cone - car_pos):
        target_cone = cone

    return target_cone

  def transform(self, state, pt):
    car_x, car_y, car_theta = state
    x, y = pt

    cos = np.cos(car_theta)
    sin = np.sin(car_theta)
    R = np.array([[cos, sin], [-sin, cos]])

    return R @ np.array([x - car_x, y - car_y])
