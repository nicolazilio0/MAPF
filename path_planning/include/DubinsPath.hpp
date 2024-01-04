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

    Eigen::Matrix2d rotMat2d(double angle);

    double angleMod(double x, bool zero22pi = false, bool degree = false);

    double mod2Pi(double theta);

    std::tuple<double, double, double, double, double> calcTrigFuncs(double alpha, double beta);

    std::tuple<double, double, double, std::vector<std::string>> LSL(double alpha, double beta, double d);

    std::tuple<double, double, double, std::vector<std::string>> RSR(double alpha, double beta, double d);

    std::tuple<double, double, double, std::vector<std::string>> LSR(double alpha, double beta, double d);

    std::tuple<double, double, double, std::vector<std::string>> RSL(double alpha, double beta, double d);

    std::tuple<double, double, double, std::vector<std::string>> RLR(double alpha, double beta, double d);

    std::tuple<double, double, double, std::vector<std::string>> LRL(double alpha, double beta, double d);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<std::string>, std::vector<double>> dubinsPathPlanningFromOrigin(
        double endX, double endY, double endYaw, double curvature,
        double stepSize, const std::vector<std::function<std::tuple<double, double, double, std::vector<std::string>>(double, double, double)>> &planningFuncs);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> interpolate(
        double length, const std::string &mode, double maxCurvature, double originX, double originY,
        double originYaw, std::vector<double> &pathX, std::vector<double> &pathY, std::vector<double> &pathYaw);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> generateLocalCourse(
        const std::vector<double> &lengths, const std::vector<std::string> &modes, double maxCurvature, double stepSize);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<std::string>, std::vector<double>> planDubinsPath(
        double sX, double sY, double sYaw, double gX, double gY, double gYaw, double curvature,
        double stepSize = 0.1, const std::vector<std::string> &selectedTypes = {});

private:
    tsl::ordered_map<std::string, std::function<std::tuple<double, double, double, std::vector<std::string>>(double, double, double)>> PATH_TYPE_MAP;
};

#endif