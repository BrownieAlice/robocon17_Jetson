#include "../include/pole.h"

Pole::Pole(double x, double y, int n)
{
  position << x, y, 0, 1;
  num = n;
}

Eigen::Vector4d Pole::getVector(void)
{
  return position;
}
/*
void Pole::setVector(Eigen::Vector4d vec)
{
  position = vec;
}
*/
