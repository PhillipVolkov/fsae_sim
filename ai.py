import numpy as np

MAX_VEL = 500
MAX_STEER = 45
CAR_L = 10
CANVAS_W = 700
CANVAS_H = 700

class AI:
  def compute(self, perception, state):
    left_cones, right_cones = perception
    car_x, car_y, car_theta = state

    # closest_left = 0
    # left_x, left_y = left_cones[0]
    # for i in range(len(left_cones)):
    #   x, y = left_cones[i]
    #   if (x - car_x) * (x - car_x) + (y - car_y) * (y - car_y) < (left_x - car_x) * (left_x - car_x) + (left_y - car_y) * (left_y - car_y):
    #     closest_left = i
    #     left_x = x
    #     left_y = y

    # closest_right = 0
    # right_x, right_y = right_cones[0]
    # for i in range(len(right_cones)):
    #   x, y, = right_cones[i]
    #   if (x - car_x) * (x - car_x) + (y - car_y) * (y - car_y) < (right_x - car_x) * (right_x - car_x) + (right_y - car_y) * (right_y - car_y):
    #     closest_right = i
    #     right_x = x
    #     right_y = y

    # left_radius = self.get_radius(left_cones[closest_left - 1], left_cones[closest_left], left_cones[closest_left + 1])
    # right_radius = self.get_radius(right_cones[closest_right - 1], right_cones[closest_right], right_cones[closest_right + 1])

    # turn_radius = (left_radius + right_radius) / 2
    # new_steer = - np.arctan2(l, turn_radius)
    # print(new_steer)

    # angle_to_first = np.arctan2(left_cones[0][0] - car_x, left_cones[0][1] - car_y)
    # angle_to_last = np.arctan2(left_cones[-1][0] - car_x, left_cones[-1][1] - car_y)
    # if np.abs(angle_to_last - car_theta) < np.abs(angle_to_first - car_theta):
    #   left_idx = -1
    # else:
    #   left_idx = 0
      
    # angle_to_first = np.arctan2(right_cones[0][0] - car_x, right_cones[0][1] - car_y)
    # angle_to_last = np.arctan2(right_cones[-1][0] - car_x, right_cones[-1][1] - car_y)
    # if np.abs(angle_to_last - car_theta) < np.abs(angle_to_first - car_theta):
    #   right_idx = -1
    # else:
    #   right_idx = 0

    target_left_x = car_x
    target_left_y = car_y
    for x, y in left_cones:
      ref_x, ref_y = self.transform((x, y), state)
      if ref_x > 0 and (x - car_x) * (x - car_x) + (y - car_y) * (y - car_y) > (target_left_x - car_x) * (target_left_x - car_x) + (target_left_y - car_y) * (target_left_y - car_y):
        target_left_x = x
        target_left_y = y

    target_right_x = car_x
    target_right_y = car_y
    for x, y in right_cones:
      ref_x, ref_y = self.transform((x, y), state)
      if ref_x > 0 and (x - car_x) * (x - car_x) + (y - car_y) * (y - car_y) > (target_right_x - car_x) * (target_right_x - car_x) + (target_right_y - car_y) * (target_right_y - car_y):
        target_right_x = x
        target_right_y = y

    reference_pt = ((target_left_x + target_right_x) / 2, (target_left_y + target_right_y) / 2)
    # reference_pt = (left_cones[-1] + right_cones[-1]) / 2

    ref_x, ref_y = self.transform(reference_pt, state)
    turn_radius = (ref_x * ref_x + ref_y * ref_y) / (2 * ref_y)
    steer = np.arctan(CAR_L / turn_radius)
    
    # target_theta_left = np.arctan2(left_cones[last_idx][0] - left_cones[first_idx][0], left_cones[last_idx][1] - left_cones[first_idx][1])
    # target_theta_right = np.arctan2(right_cones[last_idx][0] - right_cones[first_idx][0], right_cones[last_idx][1] - right_cones[first_idx][1])
    # target_theta = (target_theta_left + target_theta_right) / 2

    # if car_theta > target_theta:
    #   new_steer = -15
    # else:
    #   new_steer = 15

    # return new_steer, MAX_VEL
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


  # def get_radius(self, p_1, p_2, p_3):
  #   coeffs_1 = self.perp_bisect(p_1, p_2)
  #   coeffs_2 = self.perp_bisect(p_2, p_3)
  #   center = self.intersection(coeffs_1, coeffs_2)
  #   return np.sqrt((p_1[0] - center[0]) * (p_1[0] - center[0]) + (p_1[1] - center[1]) * (p_1[1] - center[1]))

  # # ax + by + c = 0
  # def perp_bisect(self, p_1, p_2):
  #   x_1, y_1 = p_1
  #   x_2, y_2 = p_2
  #   dx = x_2 - x_1
  #   dy = y_2 - y_1
  #   a = -dx
  #   b = -dy
  #   c = dx * a + dy * b
  #   return a, b, c

  # def intersection(self, coeffs_1, coeffs_2):
  #   a_1, b_1, c_1 = coeffs_1
  #   a_2, b_2, c_2 = coeffs_2
  #   x = (b_1 * c_2 / b_2 - c_1) / (a_1 - a_2 * b_1 / b_2)
  #   y = (-a_1 * x - c_1) / b_1
  #   return x, y
