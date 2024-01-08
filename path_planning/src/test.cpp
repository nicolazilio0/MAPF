#include <cmath>
#include <cstdlib>
#include <iostream>
#include "../include/kdtree.hpp"

using namespace std;

//
// helper function for printing points
//
void print_nodes(const Kdtree::KdNodeVector &nodes)
{

    for (auto &node : nodes)
    {
        cout << "(" << node.point[0] << "," << node.point[1] << ")";
    }
    cout << endl;
}

void print_nodes(const Kdtree::KdNodeVector &nodes, std::vector<double> n)
{

    for (auto &node : nodes)
    {
        float dx = n[0] - node.point[0];
        float dy = n[1] - node.point[1];
        float dist = hypot(dx, dy);
        cout << node.index << " (" << node.point[0] << "," << node.point[1] << " - " << node.dist << ") ";
    }
    cout << endl;
}

//
// main program demonstrating typical use cases
//
int main(int argc, char **argv)
{

    //
    // functionality tests
    //
    cout << "Functionality tests" << endl;
    cout << "-------------------" << endl;

    // 1.1) construction of kd-tree
    Kdtree::KdNodeVector nodes;
    double points[10][2] = {{1, 1}, {2, 1}, {1, 2}, {2, 4}, {3, 4}, {7, 2}, {8, 3}, {8, 5}, {7, 3}, {7, 3}};
    for (int i = 0; i < 10; ++i)
    {
        std::vector<double> point(2);
        point[0] = points[i][0];
        point[1] = points[i][1];
        nodes.push_back(Kdtree::KdNode(point));
    }
    Kdtree::KdTree tree(&nodes);
    cout << "Points in kd-tree:\n  ";
    print_nodes(tree.allnodes);

    // 1.2) kNN search
    Kdtree::KdNodeVector result;
    std::vector<double> test_point(2);
    test_point[0] = 8;
    test_point[1] = 3;
    tree.k_nearest_neighbors(test_point, 10, &result);
    cout << "10NNs of (" << test_point[0] << "," << test_point[1] << "):\n  ";
    print_nodes(result, test_point);
}