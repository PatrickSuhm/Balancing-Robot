/* Note: this code was heavily influenced by: http://www.brokking.net/yabr_main.html */

#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "balancingRobot.h"

//Radio
RF24 radio(9, 10);                              
RadioData radioData;                              
byte radioDataSize = sizeof(RadioData);    
AckPayload ackPayload;                           
byte ackPayloadSize = sizeof(AckPayload);        
byte address[] = {"blabliblub"};               
byte pipeNr = 0;                         

//Various settings
float pid_p_gain = 10;                                      
float pid_i_gain = 0.33;                                
float pid_d_gain = 60;                                   
float turning_speed = 70;                                  
float max_target_speed = 200;                    

//Motor
int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;

//IMU
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

//PID
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error, self_balance_pid_setpoint;
float pid_output_left, pid_output_right;

//Timing
unsigned long loop_timer;
unsigned long dt_rx = 0;

//Battery
unsigned long lastTime_Bat = 0;
unsigned long lastTime_rx_vol = 0;
double summierteBatSpannung = 0.0;
uint16_t mittelungscounter = 0;
uint16_t rx_vol = 0;


void setup()
{
  Serial.begin(115200);  
  
  //IMU
  Wire.begin();                                                 
  TWBR = 12;                                                        

  setup_mpu_6050_registers();                                         
  Serial.println("Calibrating gyro");                                                                     
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  
    read_mpu_6050_data();                                             
    gyro_x_cal += gyro_x;                                         
    gyro_y_cal += gyro_y;                                             
    gyro_z_cal += gyro_z;                                        
    delay(3);                                                     
  }
  gyro_x_cal /= 2000;                                              
  gyro_y_cal /= 2000;                                                 
  gyro_z_cal /= 2000;                                             
 
  
  //Radio
  radio.begin();
  radio.setChannel(2);                          
  radio.setPALevel(RF24_PA_MIN);                 
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(1);
  radio.enableAckPayload();
  radio.openReadingPipe(pipeNr,address);
  radio.startListening();                       
  resetRadioData();
  resetAckPayload(); 
  
  //Motor
  DDRD |= ( STEP_PIN_R | DIR_PIN_R | STEP_PIN_L | DIR_PIN_L );      
  PORTD &= ( ~STEP_PIN_R & ~DIR_PIN_R & ~STEP_PIN_L & ~DIR_PIN_L);  
  TCCR2A = 0;                                                               
  TCCR2B = 0;                                                               
  TIMSK2 |= (1 << OCIE2A);                                                  
  TCCR2B |= (1 << CS21);                                                   
  OCR2A = 39;                                                              
  TCCR2A |= (1 << WGM21);                                                   
  
  
  //Timing
  loop_timer = micros();                                            

  pid_setpoint = 0.0;

}


void loop()
{
  //IMU
  read_mpu_6050_data();                                               

  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                               
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                  
  angle_roll += gyro_y * 0.0000611;                                   
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);              
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= -0.0;                                              
  angle_roll_acc -= -6.0;                                               

  if(set_gyro_angles){                                                 
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;       
  }
  else{                                                               
    angle_pitch = angle_pitch_acc;                                    
    angle_roll = angle_roll_acc;                                      
    set_gyro_angles = true;                                           
  }

  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;  
  
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      
                                                      


  //PID controller calculations
  pid_error_temp = angle_roll_output - pid_setpoint - self_balance_pid_setpoint;
  if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015 ;

  pid_i_mem += pid_i_gain * pid_error_temp;                                 
  if(pid_i_mem > 400)pid_i_mem = 400;                                       
  else if(pid_i_mem < -400)pid_i_mem = -400;
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if(pid_output > 400)pid_output = 400;                                     
  else if(pid_output < -400)pid_output = -400;

  pid_last_d_error = pid_error_temp;                                       

  if(pid_output < 5 && pid_output > -5)pid_output = 0;                     

  if( angle_roll_output > 25 || angle_roll_output < -25){      
    pid_output = 0.0;                                                       
    pid_i_mem = 0.0;                                                         
    self_balance_pid_setpoint = 0.0;
  }


  //Control calculations
  pid_output_left = pid_output;    
  pid_output_right = pid_output;    

  if(radioData.roll < 110){                                          
    pid_output_left -= turning_speed;                                    
    pid_output_right += turning_speed;                                      
  }
  
  if(radioData.roll > 150){                                               
    pid_output_left += turning_speed;                                      
    pid_output_right -= turning_speed;                                      
  }

  if(radioData.pitch < 110){                                            
    if(pid_setpoint > -2.5)pid_setpoint -= 0.05;                            
    if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005;           
  }
  if(radioData.pitch> 150){                                            
    if(pid_setpoint < 2.5)pid_setpoint += 0.05;                             
    if(pid_output < max_target_speed)pid_setpoint += 0.005;                 
  }   

  if( radioData.pitch > 110 && radioData.pitch < 150){                                        
    if(pid_setpoint > 0.5)pid_setpoint -=0.05;                              
    else if(pid_setpoint < -0.5)pid_setpoint +=0.05;                        
    else pid_setpoint = 0;                                                  
  }
  
 //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if(pid_setpoint == 0){                                                    
    if(pid_output < 0)self_balance_pid_setpoint += 0.0015;                  
    if(pid_output > 0)self_balance_pid_setpoint -= 0.0015;                  
  }

  //Motor pulse calculations
  if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
  else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;

  if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
  else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if(pid_output_left > 0)left_motor = 400 - pid_output_left;
  else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if(pid_output_right > 0)right_motor = 400 - pid_output_right;
  else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
  else right_motor = 0;

  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;
  
  //Battery
  unsigned long now = micros();
  if ( now - lastTime_Bat > 12000 ) {
    lastTime_Bat = now;
    summierteBatSpannung += (double)(analogRead(BATPIN)) / 1024.0 * 5.0 * (20.8 + 6.8) / 6.8;
    mittelungscounter ++;
  }
  if( now - lastTime_rx_vol > 1200000 ) {
    lastTime_rx_vol = now;
    rx_vol = (uint16_t)(1000.0 * summierteBatSpannung / mittelungscounter);
    mittelungscounter = 0;
    summierteBatSpannung = 0.0;    
  }

   //Radio
  ackPayload.dt_rx = dt_rx;
  ackPayload.vol_rx = rx_vol;
  ackPayload.alt_rx = 0;
  ackPayload.flags = 0b00000001; 
  while ( radio.available(&pipeNr) ) {
    radio.writeAckPayload(pipeNr, &ackPayload, ackPayloadSize);
    radio.read(&radioData, radioDataSize);
  }

  //Timing
  dt_rx = micros() - loop_timer;
  while(micros() - loop_timer < 4000);                                
  loop_timer = micros();                                              

}


void read_mpu_6050_data(){                                           
  Wire.beginTransmission(0x68);                                
  Wire.write(0x3B);                                                    
  Wire.endTransmission();                                            
  Wire.requestFrom(0x68,14);                                        
  while(Wire.available() < 14);                                       
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                 
  acc_z = Wire.read()<<8|Wire.read();                                  
  temperature = Wire.read()<<8|Wire.read();                            
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                
}


void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                    
  Wire.endTransmission();                                              
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1C);                                                    
  Wire.write(0x10);                                                    
  Wire.endTransmission();                                              
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1B);                                                    
  Wire.write(0x08);                                                    
  Wire.endTransmission();                                          
}


void debugIMUData(){                                                      
  //Serial.print("pitch: ");
  //Serial.print(angle_pitch_output);
  //Serial.print("  roll: ");
  Serial.println(angle_roll_output);
  //Serial.print("  dt: ");
  //Serial.println(micros()-loop_timer);
}


//Radio
void resetRadioData(){
  radioData.throttle = 0;
  radioData.yaw = 255;
  radioData.pitch = 127;
  radioData.roll = 127;
  radioData.switches = 0;
}


void resetAckPayload(){
  ackPayload.dt_rx = 0;
  ackPayload.vol_rx = 0;
  ackPayload.alt_rx = 0;
  ackPayload.flags = 0;
}


//Interrupt Routine
ISR(TIMER2_COMPA_vect){
  //Left motor pulse calculations
  throttle_counter_left_motor ++;                                           
  if(throttle_counter_left_motor > throttle_left_motor_memory){             
    throttle_counter_left_motor = 0;                                        
    throttle_left_motor_memory = throttle_left_motor;                       
    if(throttle_left_motor_memory < 0){                                    
      PORTD &= ~DIR_PIN_L;                                                 
      throttle_left_motor_memory *= -1;                                     
    }
    else PORTD |= DIR_PIN_L;                                               
  }
  else if(throttle_counter_left_motor == 1)PORTD |= STEP_PIN_L;             
  else if(throttle_counter_left_motor == 2)PORTD &= ~STEP_PIN_L;           
  
  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          
  if(throttle_counter_right_motor > throttle_right_motor_memory){           
    throttle_counter_right_motor = 0;                                       
    throttle_right_motor_memory = throttle_right_motor;                     
    if(throttle_right_motor_memory < 0){                                    
      PORTD |= DIR_PIN_R;                                                   
      throttle_right_motor_memory *= -1;                                   
    }
    else PORTD &= ~DIR_PIN_R;                                               
  }
  else if(throttle_counter_right_motor == 1)PORTD |= STEP_PIN_R;            
  else if(throttle_counter_right_motor == 2)PORTD &= ~STEP_PIN_R;           
}
