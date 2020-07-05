#include <SPI.h>
#include <Wire.h>
#include <I2C_Anything.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "transmitter.h"

RF24 radio(6, 7);                                 //CE, CSN
RadioData radioData;                              
byte radioDataSize = sizeof(RadioData);           
AckPayload ackPayload;                            
byte ackPayloadSize = sizeof(AckPayload);         
byte address[] = {"bla"};                       
int pps = 0;                                     
int ppsCounter = 0;                              
unsigned long lastTime = 0;                       
unsigned long lastLoop = 0;  
unsigned long dt = 0;
byte err = 0;

volatile boolean haveI2CData = false;

volatile struct I2cRcvStruct{    //I2C Structs
bool empfangen;
}i2cRcvStruct;  

struct I2cSendStruct{   //I2C Structs
byte err;
uint16_t plane_vol;
int pps;
unsigned long dt;
} i2cSendStruct;

void setup(void)
{
  Wire.begin(MY_I2C_ADDRESS);   //start i2c communication
  TWBR = 12;    //400khz
  Wire.onReceive(receiveEvent);
  
  radio.begin();
  radio.setChannel(2);           
  radio.setPayloadSize(max(ackPayloadSize, radioDataSize)); //  radio.enableDynamicPayloads();           
  radio.setPALevel(RF24_PA_MIN);         
  radio.setDataRate(RF24_250KBPS);      
  radio.enableAckPayload();
  radio.openWritingPipe(address);
  
  resetRadioData();
  resetAckPayload();
  
  Serial.begin(115200);
  printDebugInfo();
  
  DDRD &= ~SWITCHPIN1;     //SWITCHPIN1 Input
  PORTD |= SWITCHPIN1;     //SWITCHPIN! HIGH
  
}


void loop(void) 
{ 
  //Timing
  unsigned long now = micros();
  dt = now - lastLoop;
  lastLoop=now;
    
  if ( (now - lastTime) > 1000000 ) 
  { 
    //printToSerial();
    lastTime = now; 
    pps = ppsCounter;   
    ppsCounter = 0;
     
    i2cSendStruct.plane_vol = ackPayload.voltage;
    i2cSendStruct.err = err;
    i2cSendStruct.pps = pps;
    i2cSendStruct.dt = dt;
     
    Wire.beginTransmission (LCD_I2C_ADDRESS);
    I2C_writeAnything(i2cSendStruct);
    Wire.endTransmission();
    
    if(haveI2CData == true)
    { 
      haveI2CData = false;
    }
  }
  
  readSticks();
    
  if (radio.write(&radioData, radioDataSize))  //senden!
  { 
    if(!radio.available())
    {
        err = 1;    
    }
    else
    {
      while ( radio.isAckPayloadAvailable() ) 
      {
        radio.read(&ackPayload, ackPayloadSize);
        ppsCounter++;  
        err = 0;
      } 
    }   
  }
  else
  {
      err = 2;   
  }
  
  
}


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

void receiveEvent(int numBytes)     //Interrupt Service Routine für I2C
{
  if (numBytes >= sizeof(i2cRcvStruct))
  {
    I2C_readAnything(i2cRcvStruct);
    haveI2CData = true;
  }
}

void readSticks()
{
  radioData.throttle = mapJoystickValues( analogRead(THROTTLEPIN), 220, 480, 690, true);
  radioData.yaw      = mapJoystickValues( analogRead(YAWPIN), 250, 500, 720, false);
  radioData.pitch    = mapJoystickValues( analogRead(PITCHPIN), 260, 512, 730, false);
  radioData.roll     = mapJoystickValues( analogRead(ROLLPIN), 210, 445, 720, false);
  radioData.switches = 0; 
  if ( PIND & SWITCHPIN1 ){ radioData.switches |= 0b00000001; }
  else{radioData.switches &= 0b11111110;}  //erstes Flag zurücksetzen
  if ( PIND & SWITCHPIN2 ){ radioData.switches |= 0b00000010; }
  else{radioData.switches &= 0b11111101;}  //zweites Flag zurücksetzen
}


void resetRadioData() 
{
  radioData.throttle = 0;
  radioData.yaw = 127;
  radioData.pitch = 127;
  radioData.roll = 127;
  radioData.switches = 0;
}

void resetAckPayload() 
{
  ackPayload.heading = 0;
  ackPayload.voltage = 0;
  ackPayload.alt = 0;
  ackPayload.flags = 0;
}

int mapJoystickValues(int val, int lower, int middle, int upper, bool inverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )   val = map(val, lower, middle, 0, 128);
  else                  val = map(val, middle, upper, 128, 255);
  if(inverse==true)     val=255-val;
  return val;
}


void printToSerial(void) 
{
  Serial.print("  Heading ");   Serial.print(ackPayload.heading);
  Serial.print("  Alt ");       Serial.print(ackPayload.alt);
  Serial.print("  Voltage ");    Serial.print(ackPayload.voltage);
  Serial.print("  Flags ");     Serial.print(ackPayload.flags & 0b00000001);
  Serial.print("  PPS ");       Serial.print(pps);
  Serial.print("  dt ");        Serial.println(dt);
}
void printDebugInfo(void)
{
  Serial.print("AckPayloadgroesse: ");
  Serial.println(ackPayloadSize); 
  Serial.print("Payloadgroesse: ");
  Serial.println(radioDataSize);
  Serial.print("Channel: ");
  Serial.println(radio.getChannel()); 
}
