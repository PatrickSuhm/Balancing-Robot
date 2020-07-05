#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define BATPIN A0

#define STEP_PIN_R          0b00010000
#define DIR_PIN_R           0b00100000
#define STEP_PIN_L          0b01000000
#define DIR_PIN_L           0b10000000

#define STEPS_PER_REV      200 * 4     

struct RadioData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte switches;
};

struct AckPayload {
  int16_t dt_rx;      
  uint16_t vol_rx;
  uint16_t alt_rx;
  byte flags;
};


#endif // CONTROLLER_H_
