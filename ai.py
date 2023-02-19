import numpy as np

MAX_VEL = 250
MAX_STEER = 45
CAR_L = 10

class AI:
  def compute(self, perception, state):
    left_cones, right_cones = perception
    car_x, car_y, car_theta = state

    target_left_x = car_x
    target_left_y = car_y
    for x, y in left_cones:
      if self.transform((x, y), state)[0] > 0 and (x - car_x) * (x - car_x) + (y - car_y) * (y - car_y) > (target_left_x - car_x) * (target_left_x - car_x) + (target_left_y - car_y) * (target_left_y - car_y):
        target_left_x = x
        target_left_y = y

    target_right_x = car_x
    target_right_y = car_y
    for x, y in right_cones:
      if self.transform((x, y), state)[0] > 0 and (x - car_x) * (x - car_x) + (y - car_y) * (y - car_y) > (target_right_x - car_x) * (target_right_x - car_x) + (target_right_y - car_y) * (target_right_y - car_y):
        target_right_x = x
        target_right_y = y

    target_left = np.array([target_left_x, target_left_y])
    target_right = np.array([target_right_x, target_right_y])

    min_left_dist = np.inf
    for i in range(0, len(left_cones) - 1):
      cone1 = left_cones[i]
      cone2 = left_cones[i + 1]
      dist = np.linalg.norm(cone1 - cone2)
      if dist < min_left_dist:
        min_left_dist = dist
    
    min_right_dist = np.inf
    for i in range(0, len(right_cones) - 1):
      cone1 = right_cones[i]
      cone2 = right_cones[i + 1]
      dist = np.linalg.norm(cone1 - cone2)
      if dist < min_right_dist:
        min_right_dist = dist

      if min_left_dist < min_right_dist:
        p = 1 - min_left_dist / min_right_dist / 2
      else:
        p = min_right_dist / min_left_dist / 2
      reference_pt = p * target_left + (1 - p) * target_right

      ref_x, ref_y = self.transform(reference_pt, state)
      turn_radius = (ref_x * ref_x + ref_y * ref_y) / (2 * ref_y)
      steer = np.arctan(CAR_L / turn_radius)

    return steer, MAX_VEL

  def transform(self, reference_pt, state):
    ref_x, ref_y = reference_pt
    x, y, theta = state
    cos = np.cos(theta)
    sin = np.sin(theta)
    R = np.array([[cos, sin], [-sin, cos]])
    dx = ref_x - x
    dy = ref_y - y
    return R @ np.array([dx, dy])
