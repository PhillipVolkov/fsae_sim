import numpy as np

CAR_LENGTH = 15

class AI:
  def compute(self, perception, state):
    left_cones, right_cones = perception

    target_left = self.get_target(state, left_cones)
    target_right = self.get_target(state, right_cones)

    min_left_pairwise_dist = self.min_pairwise_dist(left_cones)
    min_right_pairwise_dist = self.min_pairwise_dist(right_cones)

    if min_left_pairwise_dist < min_right_pairwise_dist:
      p = 1 - 0.5 * min_left_pairwise_dist / min_right_pairwise_dist
    else:
      p = 0.5 * min_right_pairwise_dist / min_left_pairwise_dist
    ref_pt = p * target_left + (1 - p) * target_right

    ref_x, ref_y = self.transform(state, ref_pt)
    turn_radius = (ref_x * ref_x + ref_y * ref_y) / (2 * ref_y)
    steer = np.arctan(CAR_LENGTH / turn_radius)

    # can be whateva lol
    vel = 160

    return steer, vel, ref_pt
  
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

  def transform(self, state, ref_pt):
    car_x, car_y, car_theta = state
    ref_x, ref_y = ref_pt
    cos = np.cos(car_theta)
    sin = np.sin(car_theta)
    R = np.array([[cos, sin], [-sin, cos]])
    dx = ref_x - car_x
    dy = ref_y - car_y
    return R @ np.array([dx, dy])
