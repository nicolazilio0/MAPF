#include "../include/RRTStarDubins.hpp"

RRTStartDubins::RRTStartDubins(std::vector<double> _start, std::vector<double> _goal, std::vector<std::vector<double>> _obstacleList,
                               double _minRand, double _maxRand, int _goalSampleRate, int _maxIter,
                               double _connectCircleDist, double _curvature, double _robotRadius)
    : start(_start[0], _start[1], _start[2]),
      end(_goal[0], _goal[1], _goal[2]),
      minRand(_minRand),
      maxRand(_maxRand),
      goalSampleRate(_goalSampleRate),
      maxIter(_maxIter),
      curvature(_curvature),
      goalYawThr(0.0174532),
      goalXYThr(0.5),
      robotRadius(_robotRadius),
      connectCircleDist(_connectCircleDist),
      obstacleList(_obstacleList)
{
}

Node RRTStartDubins::getRandomNode()
{

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_int_distribution<> distrib(0, 100);

    std::uniform_real_distribution<double> minMaxDis(minRand, maxRand);
    std::uniform_real_distribution<double> piDis(-M_PI, M_PI);

    if (distrib(gen) > goalSampleRate)
    {
        Node rnd(minMaxDis(gen), minMaxDis(gen), piDis(gen));
        return rnd;
    }
    else
    {
        Node rnd(end.x, end.y, end.yaw);
        return rnd;
    }
}

size_t RRTStartDubins::getNearestNodeIndex(Node *rndNode)
{
    std::vector<double> dlist;
    for (auto node : nodeList)
    {
        double dx = node.x - rndNode->x;
        double dy = node.y - rndNode->y;
        dlist.push_back(dx * dx + dy * dy);
    }

    auto minind = std::min_element(dlist.begin(), dlist.end()) - dlist.begin();

    return static_cast<size_t>(minind);
}

Node *RRTStartDubins::steer(Node *fromNode, Node *toNode)
{
    auto [px, py, pyaw, mode, courseLenghts] = dubinsPlanner.plan_dubins_path(fromNode->x, fromNode->y, fromNode->yaw,
                                                                              toNode->x, toNode->y, toNode->yaw,
                                                                              curvature);
    if (px.size() <= 1)
    {
        return nullptr;
    }

    Node *newNode = new Node(px.back(), py.back(), pyaw.back());
    newNode->pathX = px;
    newNode->pathY = py;
    newNode->pathYaw = pyaw;
    newNode->cost += std::accumulate(courseLenghts.begin(), courseLenghts.end(), 0.0,
                                     [](double acc, double value)
                                     {
                                         return acc + std::abs(value);
                                     });
    newNode->parent = fromNode;

    return newNode;
}

bool RRTStartDubins::checkCollision(const Node *node, std::vector<std::vector<double>> &obstacleList, double robotRadius)
{
    if (node == nullptr)
    {
        return false;
    }
    for (auto obstacle : obstacleList)
    {
        double ox = obstacle[0];
        double oy = obstacle[1];
        double size = obstacle[2];

        for (size_t i = 0; i < node->pathX.size(); ++i)
        {
            double dx = ox - node->pathX[i];
            double dy = oy - node->pathY[i];
            double d = dx * dx + dy * dy;

            if (d <= std::pow((size + robotRadius), 2))
            {
                return false; // collision
            }
        }
    }

    return true; // safe
}

std::vector<size_t> RRTStartDubins::findNearNodes(Node *newNode)
{
    // Defines a ball centered on newNode
    size_t nNode = nodeList.size() + 1;
    double r = connectCircleDist * std::sqrt(std::log(nNode) / nNode);

    std::vector<double> distList;
    for (auto node : nodeList)
    {
        double dx = node.x - newNode->x;
        double dy = node.y - newNode->y;
        double dist = dx * dx + dy * dy;
        distList.push_back(dist);
    }

    // Find indices of nodes inside the ball of radius r
    std::vector<size_t> nearInds;
    for (size_t i = 0; i < distList.size(); ++i)
    {
        if (distList[i] <= r * r)
        {
            nearInds.push_back(i);
        }
    }

    return nearInds;
}

double RRTStartDubins::calcNewCost(Node *fromNode, Node *toNode)
{
    auto [px, py, pyaw, mode, courseLengths] = dubinsPlanner.plan_dubins_path(
        fromNode->x, fromNode->y, fromNode->yaw,
        toNode->x, toNode->y, toNode->yaw, curvature);

    double cost = std::accumulate(courseLengths.begin(), courseLengths.end(), 0.0,
                                  [](double acc, double value)
                                  {
                                      return acc + std::abs(value);
                                  });

    return fromNode->cost + cost;
}

Node *RRTStartDubins::chooseParent(Node *newNode, std::vector<size_t> &nearInds)
{
    if (nearInds.empty())
    {
        return nullptr; // Return an empty node if near_inds is empty
    }

    // Search for the nearest cost in near_inds
    std::vector<double> costs;
    for (size_t i : nearInds)
    {
        Node &nearNode = nodeList[i];
        Node *tempNode = steer(&nearNode, newNode);

        if (tempNode != nullptr && checkCollision(tempNode, obstacleList, robotRadius))
        {
            costs.push_back(calcNewCost(&nearNode, tempNode));
        }
        else
        {
            costs.push_back(std::numeric_limits<double>::infinity()); // Cost of collision node
        }
    }

    // Find the minimum cost
    double minCost = *std::min_element(costs.begin(), costs.end());

    if (minCost == std::numeric_limits<double>::infinity())
    {
        std::cout << "There is no good path. (minCost is inf)" << std::endl;
        return nullptr; // Return an empty node if there is no good path
    }

    size_t minInd = nearInds[std::distance(costs.begin(), std::min_element(costs.begin(), costs.end()))];
    Node *chosenNode = steer(&nodeList[minInd], newNode);
    chosenNode->cost = minCost;

    return chosenNode;
}

void RRTStartDubins::propagateCostToLeaves(Node *parentNode)
{
    for (auto node : nodeList)
    {
        if (node.parent == parentNode)
        {
            node.cost = calcNewCost(parentNode, &node);
            propagateCostToLeaves(&node);
        }
    }
}

void RRTStartDubins::rewire(Node *newNode, const std::vector<size_t> &nearInds)
{
    for (size_t i : nearInds)
    {
        Node *nearNode = &nodeList[i];
        Node *edgeNode = steer(newNode, nearNode);

        if (!edgeNode)
        {
            continue;
        }

        edgeNode->cost = calcNewCost(newNode, nearNode);

        bool noCollision = checkCollision(edgeNode, obstacleList, robotRadius);
        bool improvedCost = nearNode->cost > edgeNode->cost;

        if (noCollision && improvedCost)
        {
            for (Node &node : nodeList)
            {
                // TODO: Possible error
                if (node.parent == &nodeList[i])
                {
                    node.parent = edgeNode;
                }
            }
            nodeList[i] = *(edgeNode);
            propagateCostToLeaves(&nodeList[i]);
        }
    }
}

double RRTStartDubins::calcDistToGoal(double x, double y)
{
    double dx = x - end.x;
    double dy = y - end.y;
    return std::hypot(dx, dy);
}

int RRTStartDubins::searchBestGoalNode()
{
    std::vector<int> goalIndexes;

    for (std::vector<Node>::size_type i = 0; i < nodeList.size(); ++i)
    {
        auto node = nodeList[i];
        if (calcDistToGoal(node.x, node.y) <= goalXYThr)
        {
            goalIndexes.push_back(i);
        }
    }

    // Angle check
    std::vector<int> finalGoalIndexes;
    for (int i : goalIndexes)
    {
        if (std::abs(nodeList[i].yaw - end.yaw) <= goalYawThr)
        {
            finalGoalIndexes.push_back(i);
        }
    }

    if (finalGoalIndexes.empty())
    {
        return -1;
    }

    double minCost = std::numeric_limits<double>::infinity();
    int bestGoalIndex = -1;

    for (int i : finalGoalIndexes)
    {
        if (nodeList[i].cost < minCost)
        {
            minCost = nodeList[i].cost;
            bestGoalIndex = i;
        }
    }

    return bestGoalIndex;
}

std::vector<std::vector<double>> RRTStartDubins::generateFinalCourse(int goalIndex)
{
    std::vector<std::vector<double>> path;
    path.push_back({end.x, end.y});

    std::cout << "Generating Final course" << std::endl;

    Node currentNode = nodeList[goalIndex];
    std::cout << "goal indx " << goalIndex << std::endl;

    while (currentNode.parent != nullptr)
    {
        std::cout << currentNode.parent << " , " << currentNode.parent->parent << std::endl;

        if (currentNode.pathX.empty() || currentNode.pathY.empty())
        {
            break;
        }

        for (int i = static_cast<int>(currentNode.pathX.size()) - 1; i >= 0; --i)
        {
            path.push_back({currentNode.pathX[i], currentNode.pathY[i]});

            std::cout << currentNode.pathX[i] << " , " << currentNode.pathY[i] << std::endl;

            if (currentNode.pathX[i] == start.x && currentNode.pathY[i] == start.y)
            {
                std::cout << "========================" << std::endl;
            }
        }

        try
        {
            currentNode = *(currentNode.parent);
        }
        catch (...)
        {
            break;
        }
    }

    path.push_back({start.x, start.y});

    return path;
}

std::vector<std::vector<double>> RRTStartDubins::planning(bool searchUntilMaxIter)
{
    nodeList = {start};

    for (int i = 0; i < maxIter; ++i)
    {
        std::cout << "Iter: " << i << ", number of nodes: " << nodeList.size() << std::endl;
        Node rnd = getRandomNode();
        int nearestInd = getNearestNodeIndex(&rnd);
        Node *newNode = steer(&nodeList[nearestInd], &rnd);

        if (checkCollision(newNode, obstacleList, robotRadius))
        {
            std::vector<size_t> nearIndexes = findNearNodes(newNode);
            newNode = chooseParent(newNode, nearIndexes);

            if (newNode != nullptr)
            {
                nodeList.push_back(*newNode);
                rewire(newNode, nearIndexes);
            }
        }

        if ((!searchUntilMaxIter) && newNode != nullptr)
        {
            int lastIndex = searchBestGoalNode();
            if (lastIndex > 0)
            {
                return generateFinalCourse(lastIndex);
            }
        }
    }

    std::cout << "Reached max iteration" << std::endl;

    int lastIndex = searchBestGoalNode();
    if (lastIndex > 0)
    {
        return generateFinalCourse(lastIndex);
    }
    else
    {
        std::cout << "Cannot find path" << std::endl;
    }

    return {};
}
