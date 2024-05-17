#include "Motor_Driver.h"
#include "Thruster_Allocator.h"
#include "config.h"

// Joystick 1
const int8_t INPUT_up_down;
const int8_t INPUT_yaw;

// Joystick 2
const int8_t INPUT_forward_back;
const int8_t INPUT_pitch;

// Make sure to download the BasicLinearAlgebra Library from the Library manager
// Change value in config to the number of motors you have
Motor_Driver* motor[NUM_MOTORS];
Thruster_Allocator allocator;

float processRawReadings(int raw);

void setup() {
  Serial.begin(115200);
  while(!Serial){
    delay(1);
  }
  // Initialize Motors
  // change these numbers to the appropriate pins for each motor
  uint8_t motor_pin1[NUM_MOTORS] = { 2, 3, 6, 7 };
  uint8_t motor_pin2[NUM_MOTORS] = { 2, 3, 6, 7 };
  uint8_t motor_ena[NUM_MOTORS] = { 2, 3, 6, 7 };
  for(uint8_t i; i < NUM_MOTORS; i++){
    motor[i] = new Motor_Driver(motor_pin1[i], motor_pin2[i], motor_ena[i]);
  }
  allocator = Thruster_Allocator();
  // 
  // EDIT these matrices, use whatever coordinate system you want but be consistent
  // 
  // Center of mass relative to origin of robot
  BLA::Matrix<3> COM = {0.0, 0.0, 0.0};
  // Location of thruster relative to origin of robot
  BLA::Matrix<NUM_MOTORS, 3> thrusterLocations = {3.7, -1.8, 0,
                                                  -3.7, -.1,8, 0,
                                                  0, 1.4, 4.8,
                                                  0, 1.4, -4.8
  };

  // orientation vector of motor relative to origin of robot
  // const float deg = 45.0;
  // const float s = sin(deg * PI / 180.0);
  // const float c = cos(deg * PI / 180.0);
  BLA::Matrix<NUM_MOTORS, 3> thrusterOrientations = {0, 0, -1,
                                                    0, 0, -1,
                                                    0, -1, 0,
                                                    0, -1, 0
    };

  allocator.defineMatrix(COM, thrusterLocations, thrusterOrientations);
}

void loop() {
  int y_raw, z_raw, pitch_raw, yaw_raw;
  float x, y, z, roll, pitch, yaw;

  // Always zero
  roll = 0.0;
  x = 0.0;

  y_raw = analogRead(INPUT_up_down);
  z_raw = analogRead(INPUT_forward_back);
  pitch_raw = analogRead(INPUT_pitch);
  yaw_raw = analogRead(INPUT_yaw);

  y = processRawReadings(y_raw);
  z = processRawReadings(z_raw);
  pitch = processRawReadings(pitch_raw);
  yaw = processRawReadings(yaw_raw);

  // Control effort is how much you want to go in a direction (ex: output from joystick)
  // order is x, y, z, roll, pitch, yaw
  float control_effort[6] = {x, y, z, roll, pitch, yaw};
  allocator.allocate(control_effort, false);
  for(uint8_t i = 0; i < NUM_MOTORS; i++){
    Serial.print(allocator.output[i], 3);
    Serial.print(" ");
    motor[i]->run(allocator.output[i]);
  }
  Serial.println();
}

// TODO: this a linear mapping from {0,...,1023} to {-1,...,1} but should be tested
float processRawReadings(int raw) {
  return (raw - 511) / 511.0;
}