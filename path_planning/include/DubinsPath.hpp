#ifndef DUBINS_PATH_HPP
#define DUBINS_PATH_HPP

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <functional>
#include <vector>
#include "tsl/ordered_map.h"

class DubinsPath
{
public:
    DubinsPath();

    Eigen::Matrix2d rot_mat_2d(double angle);

    double angle_mod(double x, bool zero_2_2pi = false, bool degree = false);

    double mod2pi(double theta);

    std::tuple<double, double, double, double, double> calc_trig_funcs(double alpha, double beta);

    std::tuple<double, double, double, std::vector<std::string>> LSL(double alpha, double beta, double d);

    std::tuple<double, double, double, std::vector<std::string>> RSR(double alpha, double beta, double d);

    std::tuple<double, double, double, std::vector<std::string>> LSR(double alpha, double beta, double d);

    std::tuple<double, double, double, std::vector<std::string>> RSL(double alpha, double beta, double d);

    std::tuple<double, double, double, std::vector<std::string>> RLR(double alpha, double beta, double d);

    std::tuple<double, double, double, std::vector<std::string>> LRL(double alpha, double beta, double d);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<std::string>, std::vector<double>> dubins_path_planning_from_origin(
        double end_x, double end_y, double end_yaw, double curvature,
        double step_size, const std::vector<std::function<std::tuple<double, double, double, std::vector<std::string>>(double, double, double)>> &planning_funcs);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> interpolate(
        double length, const std::string &mode, double max_curvature, double origin_x, double origin_y,
        double origin_yaw, std::vector<double> &path_x, std::vector<double> &path_y, std::vector<double> &path_yaw);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> generate_local_course(
        const std::vector<double> &lengths, const std::vector<std::string> &modes, double max_curvature, double step_size);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<std::string>, std::vector<double>> plan_dubins_path(
        double s_x, double s_y, double s_yaw, double g_x, double g_y, double g_yaw, double curvature,
        double step_size = 0.1, const std::vector<std::string> &selected_types = {});

private:
    tsl::ordered_map<std::string, std::function<std::tuple<double, double, double, std::vector<std::string>>(double, double, double)>> PATH_TYPE_MAP;
};

#endif