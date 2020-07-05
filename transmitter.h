#ifndef TRANSMITTER_H_
#define TRANSMITTER_H_

#define THROTTLEPIN    A3
#define YAWPIN         A2 
#define PITCHPIN       A1
#define ROLLPIN        A0

#define SWITCHPIN1  0b00010000 //Pin 4
#define SWITCHPIN2  0b00100000 //Pin 5

#define MY_I2C_ADDRESS 8
#define LCD_I2C_ADDRESS 9

// The sizeof this struct should not exceed 32 bytes
struct RadioData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte switches; 
};

struct AckPayload {
  int16_t heading;
  uint16_t voltage;
  uint16_t alt;
  byte flags;
};


#endif //TRANSMITTER_H_
