#ifndef COORDINATE_MAPPER_HPP
#define COORDINATE_MAPPER_HPP

#include <cstdio>
#include <iostream>
#include <cmath>

using namespace std;

class CoordinateMapper
{
public:
    CoordinateMapper(double realWorldWidth, double realWorldHeight, double imageWidth, double imageHeight);

    void gazebo2img(double realX, double realY, int &imageX, int &imageY) const;
    void gazebo2img(double real, int &imageMeasure) const;

    void img2gazebo(int imageX, int imageY, double &realX, double &realY) const;

private:
    double realWorldWidth_;
    double realWorldHeight_;
    double imageWidth_;
    double imageHeight_;

    double scaleX_;
    double scaleY_;
    double offsetX_;
    double offsetY_;
};

#endif