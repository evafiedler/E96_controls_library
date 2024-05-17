#include "Thruster_Allocator.h"

// Template function to find the maximum value in an array
template<typename T>
T findMaxValue(T arr[], int size) {
  T maxValue = arr[0];  // Initialize maxValue with the first element
  for (int i = 1; i < size; ++i) {
    if (abs(arr[i]) > abs(maxValue)) {
      maxValue = arr[i];
    }
  }
  return maxValue;
}
template<typename T>
T findMaxIndex(T arr[], int size) {
  T maxIndex = 0;  // Initialize maxIndex with the first element
  for (int i = 1; i < size; ++i) {
    if (abs(arr[i]) > abs(arr[maxIndex])) {
      maxIndex = i;
    }
  }
  return maxIndex;
}

Thruster_Allocator::Thruster_Allocator() {
}

void Thruster_Allocator::defineMatrix(BLA::Matrix<3> COM, BLA::Matrix<NUM_MOTORS, 3> thrusterLocations, BLA::Matrix<NUM_MOTORS, 3> thrusterOrientations) {

  for (int i = 0; i < NUM_MOTORS; i++) {
    for (int j = 0; j < 3; j++) {
      thrusterLocations(i, j) -= COM(j, 0);
    }
  }

  BLA::Matrix<NUM_MOTORS, 3> Torque;
  for (int i = 0; i < NUM_MOTORS; i++) {
    Torque(i, 0) = thrusterLocations(i, 1) * thrusterOrientations(i, 2) - thrusterLocations(i, 2) * thrusterOrientations(i, 1);
    Torque(i, 1) = thrusterLocations(i, 2) * thrusterOrientations(i, 0) - thrusterLocations(i, 0) * thrusterOrientations(i, 2);
    Torque(i, 2) = thrusterLocations(i, 0) * thrusterOrientations(i, 1) - thrusterLocations(i, 1) * thrusterOrientations(i, 0);
  }
  // Stack force and torque matrices to form the conversion matrix A
  BLA::Matrix<6, NUM_MOTORS> A;
  for (int i = 0; i < NUM_MOTORS; i++) {
    A(0, i) = thrusterOrientations(i, 0);
    A(1, i) = thrusterOrientations(i, 1);
    A(2, i) = thrusterOrientations(i, 2);
    A(3, i) = Torque(i, 0);
    A(4, i) = Torque(i, 1);
    A(5, i) = Torque(i, 2);
  }
  Serial.println("Force-Torque Matrix:");
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < NUM_MOTORS; j++) {
      if(A(i, j) == 0){
        A(i, j) = 0.1;
      }
      Serial.print(A(i, j), 6);
      Serial.print("\t");
    }
    Serial.println();
  }
  // Compute the pseudoinverse of A (Ainv)
  thrust_matrix = ~A * BLA::Inverse(A * ~A);
  Serial.println("Thrust Matrix:");
  for (int i = 0; i < NUM_MOTORS; i++) {
    for (int j = 0; j < 6; j++) {
      if(thrust_matrix(i, j) > 1000 || thrust_matrix(i, j) == NAN){
        thrust_matrix(i, j) = 0;
      }
      Serial.print(thrust_matrix(i, j));
      Serial.print("\t");
    }
    Serial.println();
  }
}

void Thruster_Allocator::allocate(float *control_effort, bool verbose) {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    output[i] = 0;
    for (uint8_t j = 0; j < 6; j++) {
      output[i] += control_effort[j] * thrust_matrix(i, j);
    }
    if (output[i] < 0) {
      output[i] /= BKWD_FWD_THRUST_RATIO;
    }
  }
  float max_thrust = findMaxValue(output, NUM_MOTORS);
  float max_effort = findMaxValue(control_effort, 6);
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    output[i] = output[i] / max_thrust;
    if (abs(max_effort) < 1.0) {
      output[i] *= max_effort * output[i];
    }
    if (verbose) {
      Serial.print(output[i], 3);
      Serial.print(", ");
    }
  }
  if (verbose) {
    Serial.println();
  }
}