#ifndef PID_Controllerh_h
#define PID_Controllerh_h

#include <math.h>

class PID_Controller {
   private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  PID_Controller();

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn);

  // A function to compute the control signal
  void evaluate(int value, int target, float deltaT, int &pwr, int &dir);

};

#endif