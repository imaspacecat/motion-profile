import math


class MotionState:
    def __init__(self, pose, vel: float):
        self.pose = pose
        self.vel = vel


class Profile:
    def __init__(self, initial_state: MotionState, final_state: MotionState):
        self.initial_state = initial_state
        self.final_state = final_state
        self.pose_delta = final_state.pose - initial_state.pose


class Trapezoid(Profile):
    def get_time(self, max_vel, max_accel):
        accel_t = max_vel / max_accel
        d = self.pose_delta

        accel_d = 0.5 * max_accel * accel_t**2

        if accel_d > (d / 2):
            accel_t = math.sqrt((d / 2) / (0.5 * max_accel))
            accel_d = d / 2
        max_vel = max_accel * accel_t

        decel_t = accel_t

        cruise_d = d - 2 * accel_d
        cruise_t = cruise_d / max_vel

        t_sum = accel_t + cruise_t + decel_t
        return t_sum

    def calculate_distance(self, max_vel, max_accel, elapsed_time):
        d = self.pose_delta

        accel_t = max_vel / max_accel
        accel_d = 0.5 * max_accel * accel_t**2

        if accel_d > (d / 2):
            accel_t = math.sqrt((d / 2) / (0.5 * max_accel))
            accel_d = d / 2
            max_vel = max_accel * accel_t

        decel_t = accel_t

        cruise_d = d - 2 * accel_d
        cruise_t = cruise_d / max_vel

        total_profile_t = accel_t + cruise_t + decel_t

        decel_start = accel_t + cruise_t

        if elapsed_time > total_profile_t:
            return d

        if elapsed_time < accel_t:
            return 0.5 * max_accel * elapsed_time**2
        elif elapsed_time < decel_start:
            accel_d = 0.5 * max_accel * accel_t**2
            current_cruise_t = elapsed_time - accel_t
            return accel_d + max_vel * current_cruise_t
        else:
            accel_d = 0.5 * max_accel * accel_t**2
            cruise_d = max_vel * cruise_t
            decel_t = elapsed_time - decel_start

            return (
                accel_d + cruise_d + max_vel * decel_t - 0.5 * max_accel * decel_t**2
            )
         
    def calculate_distance_2(self, max_vel, max_accel, elapsed_time):
        d = self.pose_delta
        i_pose = self.initial_state.pose
        i_v = self.initial_state.vel
        f_v = self.final_state.vel

        # v = v0 + at
        # t = (v - v0) / t
        accel_t = (max_vel - i_v) / max_accel
        # d = v0t + 0.5at^2
        accel_d = i_v * accel_t + 0.5 * max_accel * accel_t**2

        if accel_d > (d / 2):
            # d = v0t + 0.5at^2
            # sqrt((d - v0t) / (0.5a)) = t 
            accel_t = math.sqrt(((d / 2) - i_v * max_accel) / (0.5 * max_accel))
            accel_d = d / 2
            max_vel = max_accel * accel_t

        # decel_t = accel_t
        decel_t = (max_vel - f_v) / max_accel
        decel_d = f_v * decel_t + 0.5 * max_accel * decel_t**2

        cruise_d = d - accel_d - decel_d
        cruise_t = cruise_d / max_vel

        total_profile_t = accel_t + cruise_t + decel_t

        decel_start = accel_t + cruise_t

        if elapsed_time > total_profile_t:
            return d + i_pose

        if elapsed_time < accel_t:
            return i_v * elapsed_time + 0.5 * max_accel * elapsed_time**2 + i_pose
        elif elapsed_time < decel_start:
            # accel_d = 0.5 * max_accel * accel_t**2
            current_cruise_t = elapsed_time - accel_t
            return accel_d + max_vel * current_cruise_t + i_pose
        else:
            # accel_d = 0.5 * max_accel * accel_t**2
            # cruise_d = max_vel * cruise_t
            decel_t = elapsed_time - decel_start

            return (
                accel_d 
                + cruise_d 
                + max_vel * decel_t
                - 0.5 * max_accel * decel_t**2
                + i_pose
            )
