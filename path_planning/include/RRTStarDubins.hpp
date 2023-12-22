#ifndef RRT_STAR_DUBINS_HPP
#define RRT_STAR_DUBINS_HPP

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <random>
#include <limits>

#include "DubinsPath.hpp"

class Node
{
public:
    Node(double _x, double _y, double _yaw) : x(_x), y(_y), yaw(_yaw), cost(0.0), parent(nullptr)
    {
        empty = false;
    };
    Node()
    {
        empty = true;
    };
    ~Node()
    {
        // Clear vectors to release memory
        pathX.clear();
        pathY.clear();
        pathYaw.clear();
    }

    double x, y, yaw, cost;
    std::vector<double> pathX, pathY, pathYaw;
    Node *parent;
    bool empty;
};

class RRTStartDubins
{
public:
    RRTStartDubins(std::vector<double> _start, std::vector<double> _goal, std::vector<std::vector<double>> _obstacleList,
                   double _minRand, double _maxRand, int _goalSampleRate = 10, int _maxIter = 150,
                   double _connectCircleDist = 50.0, double _curvature = 1.0, double _robotRadius = 0.0);

    Node getRandomNode();
    static size_t getNearestNodeIndex(const std::vector<Node> &nodeList, const Node &rndNode);
    Node steer(Node &fromNode, const Node &toNode);
    static bool checkCollision(const Node &node, const std::vector<std::vector<double>> &obstacleList, double robotRadius);
    std::vector<size_t> findNearNodes(const Node &newNode);
    double calcNewCost(Node &fromNode, const Node &toNode);
    Node chooseParent(Node &newNode, const std::vector<size_t> &nearInds);
    void propagateCostToLeaves(Node *parentNode);
    void rewire(Node &newNode, const std::vector<size_t> &nearInds);
    double calcDistToGoal(double x, double y);
    int searchBestGoalNode();
    std::vector<std::vector<double>> generateFinalCourse(int goalIndex);
    std::vector<std::vector<double>> planning(bool searchUntilMaxIter);

private:
    Node start;
    Node end;
    double minRand;
    double maxRand;
    int goalSampleRate;
    int maxIter;
    double curvature;
    double goalYawThr;
    double goalXYThr;
    double robotRadius;
    double connectCircleDist;
    std::vector<std::vector<double>> obstacleList;
    std::vector<Node> nodeList;

    DubinsPath dubinsPlanner;
};

#endif