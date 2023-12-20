#include "../include/DubinsPath.hpp"

DubinsPath::DubinsPath()
{
    PATH_TYPE_MAP = {
        {"LSL", std::bind(&DubinsPath::LSL, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)},
        {"RSR", std::bind(&DubinsPath::RSR, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)},
        {"LSR", std::bind(&DubinsPath::LSR, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)},
        {"RSL", std::bind(&DubinsPath::RSL, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)},
        {"RLR", std::bind(&DubinsPath::RLR, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)},
        {"LRL", std::bind(&DubinsPath::LRL, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)}};
}

Eigen::Matrix2d DubinsPath::rot_mat_2d(double angle)
{
    Eigen::Rotation2Dd rotation(angle);
    std::cout << rotation.toRotationMatrix() << std::endl;
    return rotation.toRotationMatrix();
}

double DubinsPath::angle_mod(double x, bool zero_2_2pi, bool degree)
{
    if (degree)
    {
        x = x * (M_PI / 180.0); // deg2rad
    }

    double mod_angle;
    if (zero_2_2pi)
    {
        mod_angle = fmod(x, 2.0 * M_PI);
    }
    else
    {
        mod_angle = fmod((x + M_PI), (2.0 * M_PI)) - M_PI;
    }

    if (degree)
    {
        mod_angle = mod_angle * (180.0 / M_PI); // rad2deg
    }

    return mod_angle;
}

double DubinsPath::mod2pi(double theta)
{
    return angle_mod(theta, true);
}

std::tuple<double, double, double, double, double> DubinsPath::calc_trig_funcs(double alpha, double beta)
{
    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);
    double cos_ab = std::cos(alpha - beta);

    return std::make_tuple(sin_a, sin_b, cos_a, cos_b, cos_ab);
}

std::tuple<double, double, double, std::vector<std::string>> DubinsPath::LSL(double alpha, double beta, double d)
{
    auto [sin_a, sin_b, cos_a, cos_b, cos_ab] = calc_trig_funcs(alpha, beta);
    std::vector<std::string> mode = {"L", "S", "L"};
    double p_squared = 2 + std::pow(d, 2) - (2 * cos_ab) + (2 * d * (sin_a - sin_b));
    if (p_squared < 0)
    {
        return {NAN, NAN, NAN, mode};
    }
    double tmp = std::atan2((cos_b - cos_a), (d + sin_a - sin_b));
    double d1 = mod2pi(-alpha + tmp);
    double d2 = std::sqrt(p_squared);
    double d3 = mod2pi(beta - tmp);
    return {d1, d2, d3, mode};
}

std::tuple<double, double, double, std::vector<std::string>> DubinsPath::RSR(double alpha, double beta, double d)
{
    auto [sin_a, sin_b, cos_a, cos_b, cos_ab] = calc_trig_funcs(alpha, beta);
    std::vector<std::string> mode = {"R", "S", "R"};
    double p_squared = 2 + std::pow(d, 2) - (2 * cos_ab) + (2 * d * (sin_b - sin_a));
    if (p_squared < 0)
    {
        return {NAN, NAN, NAN, mode};
    }
    double tmp = std::atan2((cos_a - cos_b), (d - sin_a + sin_b));
    double d1 = mod2pi(alpha - tmp);
    double d2 = std::sqrt(p_squared);
    double d3 = mod2pi(-beta + tmp);
    return {d1, d2, d3, mode};
}

std::tuple<double, double, double, std::vector<std::string>> DubinsPath::LSR(double alpha, double beta, double d)
{
    auto [sin_a, sin_b, cos_a, cos_b, cos_ab] = calc_trig_funcs(alpha, beta);
    std::vector<std::string> mode = {"L", "S", "R"};
    double p_squared = -2 + std::pow(d, 2) + (2 * cos_ab) + (2 * d * (sin_a + sin_b));
    if (p_squared < 0)
    {
        return {NAN, NAN, NAN, mode};
    }
    double d1 = std::sqrt(p_squared);
    double tmp = std::atan2((-cos_a - cos_b), (d + sin_a + sin_b)) - std::atan2(-2.0, d1);
    double d2 = mod2pi(-alpha + tmp);
    double d3 = mod2pi(-mod2pi(beta) + tmp);
    return {d2, d1, d3, mode};
}

std::tuple<double, double, double, std::vector<std::string>> DubinsPath::RSL(double alpha, double beta, double d)
{
    auto [sin_a, sin_b, cos_a, cos_b, cos_ab] = calc_trig_funcs(alpha, beta);
    std::vector<std::string> mode = {"R", "S", "L"};
    double p_squared = std::pow(d, 2) - 2 + (2 * cos_ab) - (2 * d * (sin_a + sin_b));
    if (p_squared < 0)
    {
        return {NAN, NAN, NAN, mode};
    }
    double d1 = std::sqrt(p_squared);
    double tmp = std::atan2((cos_a + cos_b), (d - sin_a - sin_b)) - std::atan2(2.0, d1);
    double d2 = mod2pi(alpha - tmp);
    double d3 = mod2pi(beta - tmp);
    return {d2, d1, d3, mode};
}

std::tuple<double, double, double, std::vector<std::string>> DubinsPath::RLR(double alpha, double beta, double d)
{
    auto [sin_a, sin_b, cos_a, cos_b, cos_ab] = calc_trig_funcs(alpha, beta);
    std::vector<std::string> mode = {"R", "L", "R"};
    double tmp = (6.0 - std::pow(d, 2) + 2.0 * cos_ab + 2.0 * d * (sin_a - sin_b)) / 8.0;
    if (std::abs(tmp) > 1.0)
    {
        return {NAN, NAN, NAN, mode};
    }
    double d2 = mod2pi(2 * M_PI - std::acos(tmp));
    double d1 = mod2pi(alpha - std::atan2(cos_a - cos_b, d - sin_a + sin_b) + d2 / 2.0);
    double d3 = mod2pi(alpha - beta - d1 + d2);
    return {d1, d2, d3, mode};
}

std::tuple<double, double, double, std::vector<std::string>> DubinsPath::LRL(double alpha, double beta, double d)
{
    auto [sin_a, sin_b, cos_a, cos_b, cos_ab] = calc_trig_funcs(alpha, beta);
    std::vector<std::string> mode = {"L", "R", "L"};
    double tmp = (6.0 - std::pow(d, 2) + 2.0 * cos_ab + 2.0 * d * (-sin_a + sin_b)) / 8.0;
    if (std::abs(tmp) > 1.0)
    {
        return {NAN, NAN, NAN, mode};
    }
    double d2 = mod2pi(2 * M_PI - std::acos(tmp));
    double d1 = mod2pi(-alpha - std::atan2(cos_a - cos_b, d + sin_a - sin_b) + d2 / 2.0);
    double d3 = mod2pi(mod2pi(beta) - alpha - d1 + mod2pi(d2));
    return {d1, d2, d3, mode};
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<std::string>, std::vector<double>> DubinsPath::dubins_path_planning_from_origin(
    double end_x, double end_y, double end_yaw, double curvature,
    double step_size, const std::vector<std::function<std::tuple<double, double, double, std::vector<std::string>>(double, double, double)>> &planning_funcs)
{

    double dx = end_x;
    double dy = end_y;
    double d = std::hypot(dx, dy) * curvature;

    double theta = mod2pi(std::atan2(dy, dx));
    double alpha = mod2pi(-theta);
    double beta = mod2pi(end_yaw - theta);

    double best_cost = std::numeric_limits<double>::infinity();
    double b_d1, b_d2, b_d3;
    std::vector<std::string> b_mode;

    for (const auto &planner : planning_funcs)
    {
        double d1, d2, d3;
        std::vector<std::string> mode;
        std::tie(d1, d2, d3, mode) = planner(alpha, beta, d);

        if (std::isnan(d1))
        {
            continue;
        }

        double cost = (std::abs(d1) + std::abs(d2) + std::abs(d3));
        if (best_cost > cost)
        {
            b_d1 = d1;
            b_d2 = d2;
            b_d3 = d3;
            b_mode = mode;
            best_cost = cost;
        }
    }

    std::vector<double> lengths = {b_d1, b_d2, b_d3};
    auto [x_list, y_list, yaw_list] = generate_local_course(lengths, b_mode, curvature, step_size);

    for (double &length : lengths)
    {
        length /= curvature;
    }

    return std::make_tuple(x_list, y_list, yaw_list, b_mode, lengths);
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> DubinsPath::interpolate(
    double length, const std::string &mode, double max_curvature, double origin_x, double origin_y,
    double origin_yaw, std::vector<double> &path_x, std::vector<double> &path_y, std::vector<double> &path_yaw)
{

    if (mode == "S")
    {
        path_x.push_back(origin_x + length / max_curvature * std::cos(origin_yaw));
        path_y.push_back(origin_y + length / max_curvature * std::sin(origin_yaw));
        path_yaw.push_back(origin_yaw);
    }
    else
    { // curve
        double ldx = std::sin(length) / max_curvature;
        double ldy = 0.0;
        if (mode == "L")
        { // left turn
            ldy = (1.0 - std::cos(length)) / max_curvature;
        }
        else if (mode == "R")
        { // right turn
            ldy = (1.0 - std::cos(length)) / -max_curvature;
        }
        double gdx = std::cos(-origin_yaw) * ldx + std::sin(-origin_yaw) * ldy;
        double gdy = -std::sin(-origin_yaw) * ldx + std::cos(-origin_yaw) * ldy;
        path_x.push_back(origin_x + gdx);
        path_y.push_back(origin_y + gdy);

        if (mode == "L")
        { // left turn
            path_yaw.push_back(origin_yaw + length);
        }
        else if (mode == "R")
        { // right turn
            path_yaw.push_back(origin_yaw - length);
        }
    }

    return {path_x, path_y, path_yaw};
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> DubinsPath::generate_local_course(
    const std::vector<double> &lengths, const std::vector<std::string> &modes, double max_curvature, double step_size)
{

    std::vector<double> p_x = {0.0};
    std::vector<double> p_y = {0.0};
    std::vector<double> p_yaw = {0.0};

    for (size_t i = 0; i < lengths.size(); ++i)
    {
        double length = lengths[i];
        auto &mode = modes[i];

        if (length == 0.0)
        {
            continue;
        }

        // set origin state
        double origin_x = p_x.back();
        double origin_y = p_y.back();
        double origin_yaw = p_yaw.back();

        double current_length = step_size;
        while (std::abs(current_length + step_size) <= std::abs(length))
        {
            std::tie(p_x, p_y, p_yaw) = interpolate(current_length, mode, max_curvature,
                                                    origin_x, origin_y, origin_yaw,
                                                    p_x, p_y, p_yaw);
            current_length += step_size;
        }
        std::tie(p_x, p_y, p_yaw) = interpolate(length, mode, max_curvature, origin_x,
                                                origin_y, origin_yaw, p_x, p_y, p_yaw);
    }

    return {p_x, p_y, p_yaw};
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<std::string>, std::vector<double>> DubinsPath::plan_dubins_path(
    double s_x, double s_y, double s_yaw, double g_x, double g_y, double g_yaw, double curvature,
    double step_size, const std::vector<std::string> &selected_types)
{

    std::vector<std::function<std::tuple<double, double, double, std::vector<std::string>>(double, double, double)>> planning_funcs;

    if (selected_types.empty())
    {
        for (const auto &entry : PATH_TYPE_MAP)
        {
            planning_funcs.push_back(entry.second);
        }
    }
    else
    {
        for (const auto &ptype : selected_types)
        {
            if (PATH_TYPE_MAP.find(ptype) != PATH_TYPE_MAP.end())
            {
                planning_funcs.push_back(PATH_TYPE_MAP[ptype]);
            }
        }
    }

    // calculate local goal x, y, yaw
    Eigen::Matrix2d l_rot = rot_mat_2d(s_yaw);
    // l_rot << std::cos(s_yaw), -std::sin(s_yaw),
    //     std::sin(s_yaw), std::cos(s_yaw);

    Eigen::Vector2d le_xy = Eigen::Vector2d(g_x - s_x, g_y - s_y).transpose() * l_rot;
    double local_goal_x = le_xy[0];
    double local_goal_y = le_xy[1];
    double local_goal_yaw = g_yaw - s_yaw;

    auto [lp_x, lp_y, lp_yaw, modes, lengths] = dubins_path_planning_from_origin(
        local_goal_x, local_goal_y, local_goal_yaw, curvature, step_size,
        planning_funcs);

    // Convert a local coordinate path to the global coordinate
    Eigen::Matrix2d rot = rot_mat_2d(-s_yaw);;
    rot << std::cos(-s_yaw), -std::sin(-s_yaw),
        std::sin(-s_yaw), std::cos(-s_yaw);

    Eigen::MatrixXd converted_xy(lp_x.size(), 2);
    for (size_t i = 0; i < lp_x.size(); ++i)
    {
        Eigen::Vector2d local_point(lp_x[i], lp_y[i]);
        Eigen::Vector2d global_point = rot * local_point;
        converted_xy.row(i) = global_point + Eigen::Vector2d(s_x, s_y);
    }

    std::vector<double> x_list(converted_xy.col(0).data(), converted_xy.col(0).data() + converted_xy.rows());
    std::vector<double> y_list(converted_xy.col(1).data(), converted_xy.col(1).data() + converted_xy.rows());

    Eigen::VectorXd mapped_lp_yaw = Eigen::VectorXd::Map(lp_yaw.data(), lp_yaw.size());
    std::vector<double> yaw_list(lp_yaw.size());
    for (size_t i = 0; i < lp_yaw.size(); ++i)
    {
        yaw_list[i] = angle_mod(mapped_lp_yaw[i] + s_yaw);
    }

    return std::make_tuple(x_list, y_list, yaw_list, modes, lengths);
}