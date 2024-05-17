#ifndef THRUSTER_ALLOCATOR_H
#define THRUSTER_ALLOCATOR_H
#include "Arduino.h"
#include "config.h"
#include "BasicLinearAlgebra.h"
#define BKWD_FWD_THRUST_RATIO 0.8

class Thruster_Allocator {
public:
  Thruster_Allocator();
  void defineMatrix(BLA::Matrix<3> COM, BLA::Matrix<NUM_MOTORS, 3> thrusterLocations, BLA::Matrix<NUM_MOTORS, 3> thrusterOrientations);
  void allocate(float control_effort[6], bool verbose = false);
  float output[NUM_MOTORS] = {0};
private:
  BLA::Matrix<NUM_MOTORS, 6> thrust_matrix;
};

#endif