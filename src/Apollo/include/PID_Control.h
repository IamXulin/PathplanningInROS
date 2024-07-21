#include <iostream>
#include <cassert>
#include <cmath>
using namespace std;

class PIDController
{
public:
  PIDController(const double kp, const double ki, const double kd);
  ~PIDController() = default;


  double Control(const double error, const double dt);

  void Reset();

protected:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double previous_error_ = 0.0;
  double integral_ = 0.0;
  bool first_hit_ = false;

  double error_sum=0.0;
  double error_sub=0.0;
  double differential_=0.0;
};