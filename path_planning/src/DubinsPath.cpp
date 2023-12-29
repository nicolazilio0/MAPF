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

Eigen::Matrix2d DubinsPath::rotMat2d(double angle)
{
    Eigen::Rotation2Dd rotation(angle);
    return rotation.toRotationMatrix();
}

double DubinsPath::angleMod(double x, bool zero22pi, bool degree)
{
    if (degree)
    {
        x = x * (M_PI / 180.0); // deg2rad
    }

    double modAngle;
    if (zero22pi)
    {
        modAngle = fmod(x, 2.0 * M_PI);
        if (modAngle < 0)
        {
            modAngle += 2.0 * M_PI;
        }
    }
    else
    {
        modAngle = fmod((x + M_PI), (2.0 * M_PI)) - M_PI;
    }

    if (degree)
    {
        modAngle = modAngle * (180.0 / M_PI); // rad2deg
    }

    return modAngle;
}

double DubinsPath::mod2Pi(double theta)
{
    return angleMod(theta, true);
}

std::tuple<double, double, double, double, double> DubinsPath::calcTrigFuncs(double alpha, double beta)
{
    double sinA = std::sin(alpha);
    double sinB = std::sin(beta);
    double cosA = std::cos(alpha);
    double cosB = std::cos(beta);
    double cosAB = std::cos(alpha - beta);

    return std::make_tuple(sinA, sinB, cosA, cosB, cosAB);
}

std::tuple<double, double, double, std::vector<std::string>> DubinsPath::LSL(double alpha, double beta, double d)
{
    auto [sinA, sinB, cosA, cosB, cosAB] = calcTrigFuncs(alpha, beta);
    std::vector<std::string> mode = {"L", "S", "L"};
    double pSquared = 2 + std::pow(d, 2) - (2 * cosAB) + (2 * d * (sinA - sinB));
    if (pSquared < 0)
    {
        return {NAN, NAN, NAN, mode};
    }
    double tmp = std::atan2((cosB - cosA), (d + sinA - sinB));
    double d1 = mod2Pi(-alpha + tmp);
    double d2 = std::sqrt(pSquared);
    double d3 = mod2Pi(beta - tmp);
    return {d1, d2, d3, mode};
}

std::tuple<double, double, double, std::vector<std::string>> DubinsPath::RSR(double alpha, double beta, double d)
{
    auto [sinA, sinB, cosA, cosB, cosAB] = calcTrigFuncs(alpha, beta);
    std::vector<std::string> mode = {"R", "S", "R"};
    double pSquared = 2 + std::pow(d, 2) - (2 * cosAB) + (2 * d * (sinB - sinA));
    if (pSquared < 0)
    {
        return {NAN, NAN, NAN, mode};
    }
    double tmp = std::atan2((cosA - cosB), (d - sinA + sinB));
    double d1 = mod2Pi(alpha - tmp);
    double d2 = std::sqrt(pSquared);
    double d3 = mod2Pi(-beta + tmp);
    return {d1, d2, d3, mode};
}

std::tuple<double, double, double, std::vector<std::string>> DubinsPath::LSR(double alpha, double beta, double d)
{
    auto [sinA, sinB, cosA, cosB, cosAB] = calcTrigFuncs(alpha, beta);
    std::vector<std::string> mode = {"L", "S", "R"};
    double pSquared = -2 + std::pow(d, 2) + (2 * cosAB) + (2 * d * (sinA + sinB));
    if (pSquared < 0)
    {
        return {NAN, NAN, NAN, mode};
    }
    double d1 = std::sqrt(pSquared);
    double tmp = std::atan2((-cosA - cosB), (d + sinA + sinB)) - std::atan2(-2.0, d1);
    double d2 = mod2Pi(-alpha + tmp);
    double d3 = mod2Pi(-mod2Pi(beta) + tmp);
    return {d2, d1, d3, mode};
}

std::tuple<double, double, double, std::vector<std::string>> DubinsPath::RSL(double alpha, double beta, double d)
{
    auto [sinA, sinB, cosA, cosB, cosAB] = calcTrigFuncs(alpha, beta);
    std::vector<std::string> mode = {"R", "S", "L"};
    double pSquared = std::pow(d, 2) - 2 + (2 * cosAB) - (2 * d * (sinA + sinB));
    if (pSquared < 0)
    {
        return {NAN, NAN, NAN, mode};
    }
    double d1 = std::sqrt(pSquared);
    double tmp = std::atan2((cosA + cosB), (d - sinA - sinB)) - std::atan2(2.0, d1);
    double d2 = mod2Pi(alpha - tmp);
    double d3 = mod2Pi(beta - tmp);
    return {d2, d1, d3, mode};
}

std::tuple<double, double, double, std::vector<std::string>> DubinsPath::RLR(double alpha, double beta, double d)
{
    auto [sinA, sinB, cosA, cosB, cosAB] = calcTrigFuncs(alpha, beta);
    std::vector<std::string> mode = {"R", "L", "R"};
    double tmp = (6.0 - std::pow(d, 2) + 2.0 * cosAB + 2.0 * d * (sinA - sinB)) / 8.0;
    if (std::abs(tmp) > 1.0)
    {
        return {NAN, NAN, NAN, mode};
    }
    double d2 = mod2Pi(2 * M_PI - std::acos(tmp));
    double d1 = mod2Pi(alpha - std::atan2(cosA - cosB, d - sinA + sinB) + d2 / 2.0);
    double d3 = mod2Pi(alpha - beta - d1 + d2);
    return {d1, d2, d3, mode};
}

std::tuple<double, double, double, std::vector<std::string>> DubinsPath::LRL(double alpha, double beta, double d)
{
    auto [sinA, sinB, cosA, cosB, cosAB] = calcTrigFuncs(alpha, beta);
    std::vector<std::string> mode = {"L", "R", "L"};
    double tmp = (6.0 - std::pow(d, 2) + 2.0 * cosAB + 2.0 * d * (-sinA + sinB)) / 8.0;
    if (std::abs(tmp) > 1.0)
    {
        return {NAN, NAN, NAN, mode};
    }
    double d2 = mod2Pi(2 * M_PI - std::acos(tmp));
    double d1 = mod2Pi(-alpha - std::atan2(cosA - cosB, d + sinA - sinB) + d2 / 2.0);
    double d3 = mod2Pi(mod2Pi(beta) - alpha - d1 + mod2Pi(d2));
    return {d1, d2, d3, mode};
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<std::string>, std::vector<double>> DubinsPath::dubinsPathPlanningFromOrigin(
    double endX, double endY, double endYaw, double curvature,
    double stepSize, const std::vector<std::function<std::tuple<double, double, double, std::vector<std::string>>(double, double, double)>> &planningFuncs)
{

    double dx = endX;
    double dy = endY;
    double d = std::hypot(dx, dy) * curvature;

    double theta = mod2Pi(std::atan2(dy, dx));
    double alpha = mod2Pi(-theta);
    double beta = mod2Pi(endYaw - theta);

    double bestCost = std::numeric_limits<double>::infinity();
    double bD1, bD2, bD3;
    std::vector<std::string> bMode;

    for (const auto &planner : planningFuncs)
    {
        double d1, d2, d3;
        std::vector<std::string> mode;
        std::tie(d1, d2, d3, mode) = planner(alpha, beta, d);

        if (std::isnan(d1))
        {
            continue;
        }

        double cost = (std::abs(d1) + std::abs(d2) + std::abs(d3));
        if (bestCost > cost)
        {
            bD1 = d1;
            bD2 = d2;
            bD3 = d3;
            bMode = mode;
            bestCost = cost;
        }
    }

    std::vector<double> lengths = {bD1, bD2, bD3};
    auto [xList, yList, yawList] = generateLocalCourse(lengths, bMode, curvature, stepSize);

    for (double &length : lengths)
    {
        length /= curvature;
    }

    return std::make_tuple(xList, yList, yawList, bMode, lengths);
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> DubinsPath::interpolate(
    double length, const std::string &mode, double maxCurvature, double originX, double originY,
    double originYaw, std::vector<double> &pathX, std::vector<double> &pathY, std::vector<double> &pathYaw)
{

    if (mode == "S")
    {
        pathX.push_back(originX + length / maxCurvature * std::cos(originYaw));
        pathY.push_back(originY + length / maxCurvature * std::sin(originYaw));
        pathYaw.push_back(originYaw);
    }
    else
    { // curve
        double ldx = std::sin(length) / maxCurvature;
        double ldy = 0.0;
        if (mode == "L")
        { // left turn
            ldy = (1.0 - std::cos(length)) / maxCurvature;
        }
        else if (mode == "R")
        { // right turn
            ldy = (1.0 - std::cos(length)) / -maxCurvature;
        }
        double gdx = std::cos(-originYaw) * ldx + std::sin(-originYaw) * ldy;
        double gdy = -std::sin(-originYaw) * ldx + std::cos(-originYaw) * ldy;
        pathX.push_back(originX + gdx);
        pathY.push_back(originY + gdy);

        if (mode == "L")
        { // left turn
            pathYaw.push_back(originYaw + length);
        }
        else if (mode == "R")
        { // right turn
            pathYaw.push_back(originYaw - length);
        }
    }

    return {pathX, pathY, pathYaw};
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> DubinsPath::generateLocalCourse(
    const std::vector<double> &lengths, const std::vector<std::string> &modes, double maxCurvature, double stepSize)
{

    std::vector<double> pX = {0.0};
    std::vector<double> pY = {0.0};
    std::vector<double> pYaw = {0.0};

    for (size_t i = 0; i < lengths.size(); ++i)
    {
        double length = lengths[i];
        auto &mode = modes[i];

        if (length == 0.0)
        {
            continue;
        }

        // set origin state
        double originX = pX.back();
        double originY = pY.back();
        double originYaw = pYaw.back();

        double currentLength = stepSize;
        while (std::abs(currentLength + stepSize) <= std::abs(length))
        {
            std::tie(pX, pY, pYaw) = interpolate(currentLength, mode, maxCurvature,
                                                 originX, originY, originYaw,
                                                 pX, pY, pYaw);
            currentLength += stepSize;
        }
        std::tie(pX, pY, pYaw) = interpolate(length, mode, maxCurvature, originX,
                                             originY, originYaw, pX, pY, pYaw);
    }

    return {pX, pY, pYaw};
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<std::string>, std::vector<double>> DubinsPath::planDubinsPath(
    double sX, double sY, double sYaw, double gX, double gY, double gYaw, double curvature,
    double stepSize, const std::vector<std::string> &selectedTypes)
{

    std::vector<std::function<std::tuple<double, double, double, std::vector<std::string>>(double, double, double)>> planningFuncs;

    if (selectedTypes.empty())
    {
        for (const auto &entry : PATH_TYPE_MAP)
        {
            planningFuncs.push_back(entry.second);
        }
    }
    else
    {
        for (const auto &ptype : selectedTypes)
        {
            if (PATH_TYPE_MAP.find(ptype) != PATH_TYPE_MAP.end())
            {
                planningFuncs.push_back(PATH_TYPE_MAP[ptype]);
            }
        }
    }

    // calculate local goal x, y, yaw
    Eigen::Matrix2d lRot = rotMat2d(sYaw);

    Eigen::Vector2d leXY = Eigen::Vector2d(gX - sX, gY - sY).transpose() * lRot;
    double localGoalX = leXY[0];
    double localGoalY = leXY[1];
    double localGoalYaw = gYaw - sYaw;

    auto [lpX, lpY, lpYaw, modes, lengths] = dubinsPathPlanningFromOrigin(
        localGoalX, localGoalY, localGoalYaw, curvature, stepSize,
        planningFuncs);

    // Convert a local coordinate path to the global coordinate
    Eigen::Matrix2d rot = rotMat2d(-sYaw);

    Eigen::VectorXd lpXEigen = Eigen::Map<Eigen::VectorXd>(lpX.data(), lpX.size());
    Eigen::VectorXd lpYEigen = Eigen::Map<Eigen::VectorXd>(lpY.data(), lpY.size());

    Eigen::MatrixXd stackedMatrix(2, lpX.size());
    stackedMatrix.row(0) = lpXEigen;
    stackedMatrix.row(1) = lpYEigen;

    // Transpose the matrix and apply the rotation
    Eigen::MatrixXd convertedXY = stackedMatrix.transpose() * rot;

    // Extract columns from converted_xy
    Eigen::VectorXd xListEigen = convertedXY.col(0).array() + sX;
    Eigen::VectorXd yListEigen = convertedXY.col(1).array() + sY;

    // Convert lp_yaw to Eigen::VectorXd
    Eigen::VectorXd lpYawEigen = Eigen::Map<Eigen::VectorXd>(lpYaw.data(), lpYaw.size());

    // Apply angle_mod to each element of lp_yaw_eigen
    Eigen::VectorXd yawListEigen = lpYawEigen.array() + sYaw;
    yawListEigen = yawListEigen.unaryExpr([this](double angle)
                                           { return angleMod(angle); });

    // Convert Eigen::VectorXd to std::vector<double>
    std::vector<double> xList(xListEigen.data(), xListEigen.data() + xListEigen.size());
    std::vector<double> yList(yListEigen.data(), yListEigen.data() + yListEigen.size());
    std::vector<double> yawList(yawListEigen.data(), yawListEigen.data() + yawListEigen.size());

    return std::make_tuple(xList, yList, yawList, modes, lengths);
}