#ifndef COORDINATE_MAPPER_HPP
#define COORDINATE_MAPPER_HPP

#include <cstdio>
#include <iostream>
#include <cmath>

using namespace std;

class CoordinateMapper
{
public:
    CoordinateMapper(double real_world_width_, double real_world_height_, double image_width_, double image_height_)
        : real_world_width(real_world_width_), real_world_height(real_world_height_),
          image_width(image_width_), image_height(image_height_)
    {
        // Calculate scaling factors
        scale_x = image_width / real_world_width;
        scale_y = image_height / real_world_height;

        // Calculate offset
        offset_x = image_width / 2.0;
        offset_y = image_height / 2.0;
    };

    void gazebo2img(double real_x, double real_y, int &image_x, int &image_y) const
    {
        image_x = std::round(real_y * scale_x + offset_x);
        image_y = std::round(real_x * scale_y + offset_y);
    };

    void gazebo2img(double real, int &image_measure) const
    {
        image_measure = std::round(real * scale_x);
    };

    void img2gazebo(int image_x, int image_y, double &real_x, double &real_y) const
    {
        real_x = (image_y - offset_y) / scale_y;
        real_y = (image_x - offset_x) / scale_x;
    };

private:
    double real_world_width;
    double real_world_height;
    double image_width;
    double image_height;

    double scale_x;
    double scale_y;
    double offset_x;
    double offset_y;
};

#endif