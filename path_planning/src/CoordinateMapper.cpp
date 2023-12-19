#include "../include/CoordinateMapper.hpp"

CoordinateMapper::CoordinateMapper(
    double realWorldWidth, double realWorldHeight, double imageWidth, double imageHeight)
    : realWorldWidth_(realWorldWidth), realWorldHeight_(realWorldHeight),
      imageWidth_(imageWidth), imageHeight_(imageHeight)
{
    // Calculate scaling factors
    scaleX_ = imageWidth_ / realWorldWidth_;
    scaleY_ = imageHeight_ / realWorldHeight_;

    // Calculate offset
    offsetX_ = imageWidth_ / 2.0;
    offsetY_ = imageHeight_ / 2.0;
}

void CoordinateMapper::gazebo2img(double realX, double realY, int &imageX, int &imageY) const
{
    imageX = std::round(-realY * scaleX_ + offsetX_);
    imageY = std::round(-realX * scaleY_ + offsetY_);
}

// For symmetric world
void CoordinateMapper::gazebo2img(double real, int &imageMeasure) const
{
    imageMeasure = std::round(real * scaleX_);
}

void CoordinateMapper::img2gazebo(int imageX, int imageY, double &realX, double &realY) const
{
    realX = -(imageY - offsetY_) / scaleY_;
    realY = -(imageX - offsetX_) / scaleX_;
}
