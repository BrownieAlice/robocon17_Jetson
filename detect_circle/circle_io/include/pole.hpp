#ifndef INCLUDED_POLE
#define INCLUDED_POLE

#include <Eigen/Dense>

class Pole
{
  private:
    Eigen::Vector4d position;
    int num;
  public:
    Pole(double x, double y, int n);
    Eigen::Vector4d getVector(void) const;
};

#endif
