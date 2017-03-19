#ifndef POLE_CLASS
#define POLE_CLASS

#include <Eigen/Dense>

class Pole
{
  private:
    Eigen::Vector4d position;
    int num;
  public:
    Pole(double x, double y, int n);
    Eigen::Vector4d getVector(void) const;
//    void setVector(Eigen::Vector4d vec);
};

#endif
