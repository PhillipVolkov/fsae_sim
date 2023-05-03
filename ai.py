import math
import numpy as np

#FSAE SIM car model dimensions (in meters)
CAR_LENGTH = 1.8
CAR_WIDTH = 1

AGGRESSION = 5

CAR_MASS = 230 #kg
MAX_FRICTION = 0.7*CAR_MASS*9.81 #Ff = µ*m*g N
MAX_STEERING = 25 #degrees
MAX_ACCEL = 7.5 #m/s^2
KP_THROTTLE = 1
KD_THROTTLE = 0.0
MAX_VEL = 15 #m/s^2

class AI:
  def compute(self, percept, currVel, currAccel, dt):
    left_cones, right_cones = percept

    # get target cones on either side
    target_left = self.get_target(left_cones)
    target_right = self.get_target(right_cones)
    
    #calculate turn radius if cones on both sides are detected
    target_cross_track_dist = np.linalg.norm(target_left - target_right)
    if not math.isnan(target_cross_track_dist) and target_cross_track_dist != 0:
        # add cross-track padding to target cones (of size car width)
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
        steerAngle = np.arctan(CAR_LENGTH / turn_radius)

        #clamp steering angle
        if (steerAngle > MAX_STEERING):
            steerAngle = MAX_STEERING * steerAngle/abs(steerAngle)
        
        #normalize steering
        steer = math.degrees(steerAngle) / MAX_STEERING

        #calculate desired velocity based on turning radius
        #Fc = mv^2/r => v=sqrt(Fc*r / m)  => r = mv^2/Fc
        target_vel = math.sqrt(MAX_FRICTION * abs(turn_radius) / CAR_MASS)

        #clamp target_vel
        if (target_vel > MAX_VEL):
            target_vel = MAX_VEL * target_vel/abs(target_vel)
    else:
      target_vel = -5
      turn_radius = 0
      steerAngle = 0
      steer = 0

    #calculate desired throttle based on PID principles (but just P is enough for this sim as theres no control lag modeled yet)
    error = (target_vel-currVel)
    #pTerm = KP * error
    #dTerm = KD * de(t)/dt = KD * d(dv)/dt = KD * d(accel) = KD * accel/dt
    P = KP_THROTTLE * error
    D = KD_THROTTLE * currAccel/dt
    throttle = P + D #D is currently zero
    
    #clamp the throttle
    if (abs(throttle) > 1):
        throttle = throttle/abs(throttle)

    #calculate acceleration based on applied throttle
    accel = MAX_ACCEL * throttle

    #updates current velocity
    vel = currVel + accel*dt

    #computes centripetal force if the car is turning
    if turn_radius == 0:
        fc = 0
    else:
        fc = CAR_MASS * pow(vel, 2) / abs(turn_radius);

    #if centripetal force exceeds maximum friction, simulate a skid
    #(currently just has the car overshoot)
    if (fc > MAX_FRICTION and turn_radius != 0):
        fc = MAX_FRICTION
        turn_radius = CAR_MASS*pow(vel,2) / MAX_FRICTION * abs(turn_radius)/turn_radius
        steerAngle = np.arctan(CAR_LENGTH / turn_radius)

    return steer, steerAngle, throttle, vel, target_vel, fc, accel

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
