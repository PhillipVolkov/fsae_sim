import numpy as np

CAR_LENGTH = 15
CAR_WIDTH = 5

AGGRESSION = 2.5

class AI:
  def compute(self, percept):
    left_cones, right_cones = percept

    # can be whatever lol
    vel = 200

    if not left_cones or not right_cones:
      return 0, vel

    # get target cones on either side
    target_left = self.get_target(left_cones)
    target_right = self.get_target(right_cones)

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
    ref_x, ref_y = self.reference
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

  def get_target(self, cones):
    target_cone = np.array([0, 0])

    # find furthest cone in front of the car
    for cone in cones:
      if cone[0] > 0 and np.linalg.norm(cone) > np.linalg.norm(target_cone):
        target_cone = cone

    return target_cone
