from scipy.spatial.transform import Rotation as Rot
import numpy as np


class DubinsPath():

    def __init__(self) -> None:
        self._PATH_TYPE_MAP = {"LSL": self._LSL, "RSR": self._RSR, "LSR": self._LSR, "RSL": self._RSL,
                               "RLR": self._RLR, "LRL": self._LRL, }

    def rot_mat_2d(self, angle):
        return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]

    def angle_mod(self, x, zero_2_2pi=False, degree=False):
        if isinstance(x, float):
            is_float = True
        else:
            is_float = False

        x = np.asarray(x).flatten()
        if degree:
            x = np.deg2rad(x)

        if zero_2_2pi:
            mod_angle = x % (2 * np.pi)
        else:
            mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

        if degree:
            mod_angle = np.rad2deg(mod_angle)

        if is_float:
            return mod_angle.item()
        else:
            return mod_angle

    def plan_dubins_path(self, s_x, s_y, s_yaw, g_x, g_y, g_yaw, curvature,
                         step_size=0.1, selected_types=None):
        if selected_types is None:
            planning_funcs = self._PATH_TYPE_MAP.values()
        else:
            planning_funcs = [self._PATH_TYPE_MAP[ptype]
                              for ptype in selected_types]

        # calculate local goal x, y, yaw
        l_rot = self.rot_mat_2d(s_yaw)
        le_xy = np.stack([g_x - s_x, g_y - s_y]).T @ l_rot
        local_goal_x = le_xy[0]
        local_goal_y = le_xy[1]
        local_goal_yaw = g_yaw - s_yaw

        lp_x, lp_y, lp_yaw, modes, lengths = self._dubins_path_planning_from_origin(
            local_goal_x, local_goal_y, local_goal_yaw, curvature, step_size,
            planning_funcs)

        # Convert a local coordinate path to the global coordinate
        rot = self.rot_mat_2d(-s_yaw)
        converted_xy = np.stack([lp_x, lp_y]).T @ rot
        x_list = converted_xy[:, 0] + s_x
        y_list = converted_xy[:, 1] + s_y
        yaw_list = self.angle_mod(np.array(lp_yaw) + s_yaw)

        return x_list, y_list, yaw_list, modes, lengths

    def _mod2pi(self, theta):
        return self.angle_mod(theta, zero_2_2pi=True)

    def _calc_trig_funcs(self, alpha, beta):
        sin_a = np.sin(alpha)
        sin_b = np.sin(beta)
        cos_a = np.cos(alpha)
        cos_b = np.cos(beta)
        cos_ab = np.cos(alpha - beta)
        return sin_a, sin_b, cos_a, cos_b, cos_ab

    def _LSL(self, alpha, beta, d):
        sin_a, sin_b, cos_a, cos_b, cos_ab = self._calc_trig_funcs(alpha, beta)
        mode = ["L", "S", "L"]
        p_squared = 2 + d ** 2 - (2 * cos_ab) + (2 * d * (sin_a - sin_b))
        if p_squared < 0:  # invalid configuration
            return None, None, None, mode
        tmp = np.arctan2((cos_b - cos_a), d + sin_a - sin_b)
        d1 = self._mod2pi(-alpha + tmp)
        d2 = np.sqrt(p_squared)
        d3 = self._mod2pi(beta - tmp)
        return d1, d2, d3, mode

    def _RSR(self, alpha, beta, d):
        sin_a, sin_b, cos_a, cos_b, cos_ab = self._calc_trig_funcs(alpha, beta)
        mode = ["R", "S", "R"]
        p_squared = 2 + d ** 2 - (2 * cos_ab) + (2 * d * (sin_b - sin_a))
        if p_squared < 0:
            return None, None, None, mode
        tmp = np.arctan2((cos_a - cos_b), d - sin_a + sin_b)
        d1 = self._mod2pi(alpha - tmp)
        d2 = np.sqrt(p_squared)
        d3 = self._mod2pi(-beta + tmp)
        return d1, d2, d3, mode

    def _LSR(self, alpha, beta, d):
        sin_a, sin_b, cos_a, cos_b, cos_ab = self._calc_trig_funcs(alpha, beta)
        p_squared = -2 + d ** 2 + (2 * cos_ab) + (2 * d * (sin_a + sin_b))
        mode = ["L", "S", "R"]
        if p_squared < 0:
            return None, None, None, mode
        d1 = np.sqrt(p_squared)
        tmp = np.arctan2((-cos_a - cos_b), (d + sin_a + sin_b)
                         ) - np.arctan2(-2.0, d1)
        d2 = self._mod2pi(-alpha + tmp)
        d3 = self._mod2pi(-self._mod2pi(beta) + tmp)
        return d2, d1, d3, mode

    def _RSL(self, alpha, beta, d):
        sin_a, sin_b, cos_a, cos_b, cos_ab = self._calc_trig_funcs(alpha, beta)
        p_squared = d ** 2 - 2 + (2 * cos_ab) - (2 * d * (sin_a + sin_b))
        mode = ["R", "S", "L"]
        if p_squared < 0:
            return None, None, None, mode
        d1 = np.sqrt(p_squared)
        tmp = np.arctan2((cos_a + cos_b), (d - sin_a - sin_b)
                         ) - np.arctan2(2.0, d1)
        d2 = self._mod2pi(alpha - tmp)
        d3 = self._mod2pi(beta - tmp)
        return d2, d1, d3, mode

    def _RLR(self, alpha, beta, d):
        sin_a, sin_b, cos_a, cos_b, cos_ab = self._calc_trig_funcs(alpha, beta)
        mode = ["R", "L", "R"]
        tmp = (6.0 - d ** 2 + 2.0 * cos_ab + 2.0 * d * (sin_a - sin_b)) / 8.0
        if abs(tmp) > 1.0:
            return None, None, None, mode
        d2 = self._mod2pi(2 * np.pi - np.arccos(tmp))
        d1 = self._mod2pi(alpha - np.arctan2(cos_a - cos_b,
                                             d - sin_a + sin_b) + d2 / 2.0)
        d3 = self._mod2pi(alpha - beta - d1 + d2)
        return d1, d2, d3, mode

    def _LRL(self, alpha, beta, d):
        sin_a, sin_b, cos_a, cos_b, cos_ab = self._calc_trig_funcs(alpha, beta)
        mode = ["L", "R", "L"]
        tmp = (6.0 - d ** 2 + 2.0 * cos_ab + 2.0 * d * (- sin_a + sin_b)) / 8.0
        if abs(tmp) > 1.0:
            return None, None, None, mode
        d2 = self._mod2pi(2 * np.pi - np.arccos(tmp))
        d1 = self._mod2pi(-alpha - np.arctan2(cos_a - cos_b,
                                              d + sin_a - sin_b) + d2 / 2.0)
        d3 = self._mod2pi(self._mod2pi(beta) - alpha - d1 + self._mod2pi(d2))
        return d1, d2, d3, mode

    def _dubins_path_planning_from_origin(self, end_x, end_y, end_yaw, curvature,
                                          step_size, planning_funcs):
        dx = end_x
        dy = end_y
        d = np.hypot(dx, dy) * curvature

        theta = self._mod2pi(np.arctan2(dy, dx))
        alpha = self._mod2pi(-theta)
        beta = self._mod2pi(end_yaw - theta)

        best_cost = float("inf")
        b_d1, b_d2, b_d3, b_mode = None, None, None, None

        for planner in planning_funcs:
            d1, d2, d3, mode = planner(alpha, beta, d)
            if d1 is None:
                continue

            cost = (abs(d1) + abs(d2) + abs(d3))
            if best_cost > cost:  # Select minimum length one.
                b_d1, b_d2, b_d3, b_mode, best_cost = d1, d2, d3, mode, cost

        lengths = [b_d1, b_d2, b_d3]
        x_list, y_list, yaw_list = self._generate_local_course(lengths, b_mode,
                                                               curvature, step_size)

        lengths = [length / curvature for length in lengths]

        return x_list, y_list, yaw_list, b_mode, lengths

    def _interpolate(self, length, mode, max_curvature, origin_x, origin_y,
                     origin_yaw, path_x, path_y, path_yaw):
        if mode == "S":
            path_x.append(origin_x + length /
                          max_curvature * np.cos(origin_yaw))
            path_y.append(origin_y + length /
                          max_curvature * np.sin(origin_yaw))
            path_yaw.append(origin_yaw)
        else:  # curve
            ldx = np.sin(length) / max_curvature
            ldy = 0.0
            if mode == "L":  # left turn
                ldy = (1.0 - np.cos(length)) / max_curvature
            elif mode == "R":  # right turn
                ldy = (1.0 - np.cos(length)) / -max_curvature
            gdx = np.cos(-origin_yaw) * ldx + np.sin(-origin_yaw) * ldy
            gdy = -np.sin(-origin_yaw) * ldx + np.cos(-origin_yaw) * ldy
            path_x.append(origin_x + gdx)
            path_y.append(origin_y + gdy)

            if mode == "L":  # left turn
                path_yaw.append(origin_yaw + length)
            elif mode == "R":  # right turn
                path_yaw.append(origin_yaw - length)

        return path_x, path_y, path_yaw

    def _generate_local_course(self, lengths, modes, max_curvature, step_size):
        p_x, p_y, p_yaw = [0.0], [0.0], [0.0]

        for (mode, length) in zip(modes, lengths):
            if length == 0.0:
                continue

            # set origin state
            origin_x, origin_y, origin_yaw = p_x[-1], p_y[-1], p_yaw[-1]

            current_length = step_size
            while abs(current_length + step_size) <= abs(length):
                p_x, p_y, p_yaw = self._interpolate(current_length, mode, max_curvature,
                                                    origin_x, origin_y, origin_yaw,
                                                    p_x, p_y, p_yaw)
                current_length += step_size

            p_x, p_y, p_yaw = self._interpolate(length, mode, max_curvature, origin_x,
                                                origin_y, origin_yaw, p_x, p_y, p_yaw)

        return p_x, p_y, p_yaw
