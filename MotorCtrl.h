#ifndef MOTORCTRL_H
#define MOTORCTRL_H


#include <Arduino.h>
// make sure DEBUG_MODE = 0 in mcp_can_dfs.h line 35
#include "CANopen.h"

#define DEBUG

#define PIN_DIN_4 7
#define PIN_DIN_5 6
#define IN_CONTROL_ACITVATE PIN_DIN_5
#define IN_MOTOR_ACTIVATE PIN_DIN_4
#define PIN_DO_0 4
#define PIN_DO_1 5

#define SUCCESS 1
#define FAILURE 0

#define MODE_POSITION 1
#define MODE_VELOCITY 2
#define MODE_TORQUE 3
#define MODE_HOMING 6
#define MODE_INTERPOLATED_POSITION 7

#define RAMP_LINEAR 0
#define RAMP_LIMITED_JERK 2

#define ADR_CONTROLWORD 0x6040
#define ADR_STATUSWORD 0x6041
#define ADR_MODES_OF_OPERATION 0x6060
#define ADR_MODES_OF_OPERATION_DISPLAY 0x6061

#define ADR_TARGET_POSITION 0x607A
#define ADR_PROFILE_VELOCITY 0x6081
#define ADR_END_VELOCITY 0x6082
#define ADR_PROFILE_ACCELERATION 0x6083
#define ADR_PROFILE_DECELERATION 0x6084
#define ADR_MOTION_PROFILE_TYPE 0x6086

class MotorCtrl
{
public:
  MotorCtrl() {}
  void setup();
  void startController();
  uint8_t setMotorReady();
  uint8_t setOperationMode(int8_t mode);
  uint8_t enableOperation();
  uint8_t newSetpoint(int32_t pos, uint32_t vel);
  uint8_t activateNewSetpoint();
  uint8_t motorHalt();

  static CANopen can;

private:

};


#endif /* end of include guard: MOTORCTRL_H */
