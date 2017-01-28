#include "TagObject.h"
#include <cmath>

TagObject::TagObject()
{
  tag_detected_ = false;
}

void TagObject::initi(double x, double y)
{
  //tag_id_ = tag_id;
  x_ = x;
  y_ = y;
  setDistance();
}

bool TagObject::isTagDetected()
{
  return tag_detected_;
}
void TagObject::setTagDetected()
{
  tag_detected_ = true;
}
double TagObject::getX()
{
  return x_;
}
double TagObject::getY()
{
  return y_;
}

// int TagObject::getId()
// {
//   return tag_id;
// }
double TagObject::getLinearDistance()
{

  return linear_distance_;
}
double TagObject::getAngularDistance()
{
  return angular_distance_;
}
void TagObject::setDistance()
{
  linear_distance_ = sqrt(x_ * x_ + y_ * y_);
  if (x_ == 0 || y_ == 0)
  {
    onAxis();
  }
  else if (x_ > 0 && y_ > 0)
  {
    angular_distance_ = atan(x_ / y_);
  }
  else if (x_ > 0 && y_ < 0)
  {
    angular_distance_ = atan((-y_) / x_) + PI / 2;
  }
  else if (x_ < 0 && y_ < 0)
  {
    angular_distance_ = atan((-x_) / (-y_)) + PI;
  }
  else if (x_ < 0 && y_ > 0)
  {
    angular_distance_ = atan(y_ / (-x_)) + PI * 3 / 2;
  }
}
void TagObject::onAxis()
{
  if (x_ == 0 && y_ == 0)
  {
    angular_distance_ = 0;
  }
  else if (x_ == 0)
  {
    if (y_ > 0)
    {
      angular_distance_ = 0;
    }
    if (y_ < 0)
    {
      angular_distance_ = PI;
    }
  }
  else if (y_ == 0)
  {
    if (x_ > 0)
    {
      angular_distance_ = PI / 2;
    }
    if (x_ < 0)
    {
      angular_distance_ = PI * 3 / 2;
    }
  }
}

bool TagObject::isTagCollected()
{
  return tag_collected_;
}
void TagObject::setTagCollected()
{
  tag_collected_ = true;
}

bool TagObject::isAvailable()
{
  return tag_available_;
}

void TagObject::setAvailable()
{
  tag_available_ = true;
}

void TagObject::setUnavailable()
{
  tag_available_ = false;
}
