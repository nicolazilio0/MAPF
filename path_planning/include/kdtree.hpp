#ifndef __kdtree_HPP
#define __kdtree_HPP

//
// Kd-Tree implementation.
//
// Copyright: Christoph Dalitz, 2018-2023
//            Jens Wilberg, 2018
// Version:   1.3
// License:   BSD style license
//            (see the file LICENSE for details)
//

// Implementation
// https://github.com/cdalitz/kdtree-cpp/tree/master

#include <cstdlib>
#include <queue>
#include <vector>
#include <math.h>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace Kdtree
{

    typedef std::vector<double> CoordPoint;
    typedef std::vector<double> DoubleVector;

    // for passing points to the constructor of kdtree
    struct KdNode
    {
        CoordPoint point;
        void *data;
        int index;
        // modified by Bussola & Zilio to store distrance from nn
        double dist;
        KdNode(const CoordPoint &p, void *d = NULL, int i = -1)
        {
            point = p;
            data = d;
            index = i;
            dist = 0;
        }
        KdNode() { data = NULL; }
    };
    typedef std::vector<KdNode> KdNodeVector;

    // base function object for search predicate in knn search
    // returns true when the given KdNode is an admissible neighbor
    // To define an own search predicate, derive from this class
    // and overwrite the call operator operator()
    struct KdNodePredicate
    {
        virtual ~KdNodePredicate() {}
        virtual bool operator()(const KdNode &) const { return true; }
    };

    //--------------------------------------------------------
    // private helper classes used internally by KdTree
    //
    // the internal node structure used by kdtree
    class kdtree_node;
    // base class for different distance computations
    class DistanceMeasure;
    // helper class for priority queue in k nearest neighbor search
    class nn4heap
    {
    public:
        size_t dataindex; // index of actual kdnode in *allnodes*
        double distance;  // distance of this neighbor from *point*
        nn4heap(size_t i, double d)
        {
            dataindex = i;
            distance = d;
        }
    };
    class compare_nn4heap
    {
    public:
        bool operator()(const nn4heap &n, const nn4heap &m)
        {
            return (n.distance < m.distance);
        }
    };
    typedef std::priority_queue<nn4heap, std::vector<nn4heap>, compare_nn4heap> SearchQueue;
    //--------------------------------------------------------

    // kdtree class
    class KdTree
    {
    private:
        // recursive build of tree
        kdtree_node *build_tree(size_t depth, size_t a, size_t b);
        // helper variable for keeping track of subtree bounding box
        CoordPoint lobound, upbound;
        // helper variable to check the distance method
        int distance_type;
        bool neighbor_search(const CoordPoint &point, kdtree_node *node, size_t k, SearchQueue *neighborheap);
        void range_search(const CoordPoint &point, kdtree_node *node, double r, std::vector<size_t> *range_result);
        bool bounds_overlap_ball(const CoordPoint &point, double dist,
                                 kdtree_node *node);
        bool ball_within_bounds(const CoordPoint &point, double dist,
                                kdtree_node *node);
        // class implementing the distance computation
        DistanceMeasure *distance;
        // search predicate in knn searches
        KdNodePredicate *searchpredicate;

    public:
        KdNodeVector allnodes;
        size_t dimension;
        kdtree_node *root;
        // distance_type can be 0 (max), 1 (city block), or 2 (euklid [squared])
        KdTree(const KdNodeVector *nodes, int distance_type = 2);
        ~KdTree();
        void set_distance(int distance_type, const DoubleVector *weights = NULL);
        void k_nearest_neighbors(const CoordPoint &point, size_t k,
                                 KdNodeVector *result, KdNodePredicate *pred = NULL);
        void range_nearest_neighbors(const CoordPoint &point, double r,
                                     KdNodeVector *result);
    };
    //--------------------------------------------------------------
    // function object for comparing only dimension d of two vecotrs
    //--------------------------------------------------------------
    class compare_dimension
    {
    public:
        compare_dimension(size_t dim) { d = dim; }
        bool operator()(const KdNode &p, const KdNode &q)
        {
            return (p.point[d] < q.point[d]);
        }
        size_t d;
    };

    //--------------------------------------------------------------
    // internal node structure used by kdtree
    //--------------------------------------------------------------
    class kdtree_node
    {
    public:
        kdtree_node()
        {
            dataindex = cutdim = 0;
            loson = hison = (kdtree_node *)NULL;
        }
        ~kdtree_node()
        {
            if (loson)
                delete loson;
            if (hison)
                delete hison;
        }
        // index of node data in kdtree array "allnodes"
        size_t dataindex;
        // cutting dimension
        size_t cutdim;
        // value of point
        // double cutval; // == point[cutdim]
        CoordPoint point;
        //  roots of the two subtrees
        kdtree_node *loson, *hison;
        // bounding rectangle of this node's subtree
        CoordPoint lobound, upbound;
    };

    //--------------------------------------------------------------
    // different distance metrics
    //--------------------------------------------------------------
    class DistanceMeasure
    {
    public:
        DistanceMeasure() {}
        virtual ~DistanceMeasure() {}
        virtual double distance(const CoordPoint &p, const CoordPoint &q) = 0;
        virtual double coordinate_distance(double x, double y, size_t dim) = 0;
    };
    // Maximum distance (Linfinite norm)
    class DistanceL0 : virtual public DistanceMeasure
    {
        DoubleVector *w;

    public:
        DistanceL0(const DoubleVector *weights = NULL)
        {
            if (weights)
                w = new DoubleVector(*weights);
            else
                w = (DoubleVector *)NULL;
        }
        ~DistanceL0()
        {
            if (w)
                delete w;
        }
        double distance(const CoordPoint &p, const CoordPoint &q)
        {
            size_t i;
            double dist, test;
            if (w)
            {
                dist = (*w)[0] * fabs(p[0] - q[0]);
                for (i = 1; i < p.size(); i++)
                {
                    test = (*w)[i] * fabs(p[i] - q[i]);
                    if (test > dist)
                        dist = test;
                }
            }
            else
            {
                dist = fabs(p[0] - q[0]);
                for (i = 1; i < p.size(); i++)
                {
                    test = fabs(p[i] - q[i]);
                    if (test > dist)
                        dist = test;
                }
            }
            return dist;
        }
        double coordinate_distance(double x, double y, size_t dim)
        {
            if (w)
                return (*w)[dim] * fabs(x - y);
            else
                return fabs(x - y);
        }
    };
    // Manhatten distance (L1 norm)
    class DistanceL1 : virtual public DistanceMeasure
    {
        DoubleVector *w;

    public:
        DistanceL1(const DoubleVector *weights = NULL)
        {
            if (weights)
                w = new DoubleVector(*weights);
            else
                w = (DoubleVector *)NULL;
        }
        ~DistanceL1()
        {
            if (w)
                delete w;
        }
        double distance(const CoordPoint &p, const CoordPoint &q)
        {
            size_t i;
            double dist = 0.0;
            if (w)
            {
                for (i = 0; i < p.size(); i++)
                    dist += (*w)[i] * fabs(p[i] - q[i]);
            }
            else
            {
                for (i = 0; i < p.size(); i++)
                    dist += fabs(p[i] - q[i]);
            }
            return dist;
        }
        double coordinate_distance(double x, double y, size_t dim)
        {
            if (w)
                return (*w)[dim] * fabs(x - y);
            else
                return fabs(x - y);
        }
    };
    // Euklidean distance (L2 norm) (squared)
    class DistanceL2 : virtual public DistanceMeasure
    {
        DoubleVector *w;

    public:
        DistanceL2(const DoubleVector *weights = NULL)
        {
            if (weights)
                w = new DoubleVector(*weights);
            else
                w = (DoubleVector *)NULL;
        }
        ~DistanceL2()
        {
            if (w)
                delete w;
        }
        double distance(const CoordPoint &p, const CoordPoint &q)
        {
            size_t i;
            double dist = 0.0;
            if (w)
            {
                for (i = 0; i < p.size(); i++)
                    dist += (*w)[i] * (p[i] - q[i]) * (p[i] - q[i]);
            }
            else
            {
                for (i = 0; i < p.size(); i++)
                    dist += (p[i] - q[i]) * (p[i] - q[i]);
            }
            return dist;
        }
        double coordinate_distance(double x, double y, size_t dim)
        {
            if (w)
                return (*w)[dim] * (x - y) * (x - y);
            else
                return (x - y) * (x - y);
        }
    };
} // end namespace Kdtree

#endif