import numpy as np

CAR_LENGTH = 15
CAR_WIDTH = 5

AGGRESSION = 2.5

class AI:
  def compute(self, perception, state):
    left_cones, right_cones = perception

    # can be whatever lol
    vel = 200

    if not left_cones or not right_cones:
      return 0, vel

    # get target cones on either side
    target_left = self.get_target(state, left_cones)
    target_right = self.get_target(state, right_cones)

    # add cross-track padding to target cones (of size car width)
    target_cross_track_dist = np.linalg.norm(target_left - target_right)
    adjusted_target_left = target_left + (target_right - target_left) / target_cross_track_dist * CAR_WIDTH
    adjusted_target_right = target_right + (target_left - target_right) / target_cross_track_dist * CAR_WIDTH

    # compute min pairwise distance on either side
    min_left_pairwise_dist = self.min_pairwise_dist(left_cones)
    min_right_pairwise_dist = self.min_pairwise_dist(right_cones)

    # compute weighted average coefficient p, higher aggression = greater commitment on turns
    if min_left_pairwise_dist < min_right_pairwise_dist:
      ratio = min_left_pairwise_dist / min_right_pairwise_dist
      p = 1 / (1 + np.exp(AGGRESSION * (ratio - 1)))
    else:
      ratio = min_right_pairwise_dist / min_left_pairwise_dist
      p = 1 - 1 / (1 + np.exp(AGGRESSION * (ratio - 1)))

    # calculate reference point from weighted average of target cones
    self.reference = p * adjusted_target_left + (1 - p) * adjusted_target_right

    # compute steering angle corresponding to trajectory that would hit the reference point
    ref_x, ref_y = self.transform(state, self.reference)
    turn_radius = (ref_x * ref_x + ref_y * ref_y) / (2 * ref_y)
    steer = np.arctan(CAR_LENGTH / turn_radius)

    return steer, vel

  def min_pairwise_dist(self, cones):
    min_dist = np.inf

    # get minimum pairwise distance
    for i in range(0, len(cones) - 1):
      dist = np.linalg.norm(cones[i] - cones[i + 1])
      if dist < min_dist:
        min_dist = dist

    return min_dist

  def get_target(self, state, cones):
    car_x, car_y, car_theta = state
    car_pos = np.array((car_x, car_y))

    # find furthest cone in front of the car
    target_cone = car_pos.copy()
    for cone in cones:
      if self.transform(state, cone)[0] > 0 and np.linalg.norm(cone - car_pos) > np.linalg.norm(target_cone - car_pos):
        target_cone = cone

    return target_cone

  def transform(self, state, pt):
    car_x, car_y, car_theta = state
    x, y = pt

    # compute inverse rotation matrix
    cos = np.cos(car_theta)
    sin = np.sin(car_theta)
    R = np.array([[cos, sin], [-sin, cos]])

    # transform point to refence frame of state
    return R @ np.array([x - car_x, y - car_y])
