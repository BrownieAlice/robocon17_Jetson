#include "../include/pole.hpp"

Pole::Pole(double x, double y, int n)
{
  position << x, y, 0, 1;
  num = n;
}

Eigen::Vector4d Pole::getVector(void) const
{
  return position;
}
