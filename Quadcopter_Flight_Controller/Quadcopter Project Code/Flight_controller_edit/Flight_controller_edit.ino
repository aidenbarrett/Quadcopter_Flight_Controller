
#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 0.9;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.06;              //Gain setting for the roll I-controller (0.05)
float pid_d_gain_roll = 15;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 3;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;

unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int cal_int, start;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte highByte, lowByte;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  Wire.begin();                                                //Start the I2C as master.
  
  DDRD |= B11110000;                                           //Configure digital port 4, 5, 6 and 7 as an output.
  DDRB |= B00110000;                                           //Configure digital port 12 and 13 as an output.
  //Arduino pins default to inputs, so they don't need to be declared as inputs.
  
  Wire.beginTransmission(105);                                 //Start communication with the gyro (adress 1101001)
  Wire.write(0x20);                                            //This writes to register 1 (20 hex)
  Wire.write(0x0F);                                            //Sets the register bits as 00001111 (Turn on the gyro and enable all the axis)
  Wire.endTransmission();                                      //End the transmission with the gyro

  Wire.beginTransmission(105);                                 //Start communication with the gyro (adress 1101001)
  Wire.write(0x23);                                            //This writes to register 4 (23 hex)
  Wire.write(0x90);                                            //Sets the register bits as 10010000 (Block Data Update active & 500dps full scale)
  Wire.endTransmission();                                      //End the transmission with the gyro

  delay(250);                                                  //Delay to allow the gyro some time to start.

  //This takes multiple gyro data samples to determine the average gyro offset (for calibration).
  for (cal_int = 0; cal_int < 2000; cal_int ++){               //Take 2000 readings for calibration.
    gyro_signal();                                             //Read the gyro output.
    gyro_roll_cal += gyro_roll;                                //Add roll value to gyro_roll_cal.
    gyro_pitch_cal += gyro_pitch;                              //Add pitch value to gyro_pitch_cal.
    gyro_yaw_cal += gyro_yaw;                                  //Add yaw value to gyro_yaw_cal.
    
    //The esc's beep annoyingly. So this gives them a 1000us pulse while calibrating the gyro.
    PORTD |= B11110000;                                        //Sets digital port 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Sets digital port 4, 5, 6 and 7 low.
    delay(3);                                                  //Wait 3 milliseconds before the next loop.
  }
  //Now that there are 2000 samples, need to divide by 2000 to get the average gyro offset.
  gyro_roll_cal /= 2000;                                       //Divide the roll total by 2000.
  gyro_pitch_cal /= 2000;                                      //Divide the pitch total by 2000.
  gyro_yaw_cal /= 2000;                                        //Divide the yaw total by 2000.
  
  PCICR  |= (1 << PCIE0);                                      //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                     //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

  //Wait until the receiver is active and the throttle is set to the lowest position.
  //reciever_3 is 0 until it is connected, getting a value <976 means it is not connected.
  //976 is the min value so we want the throttle in the lowest position so reciever_3 must be below 1000ish.
  //channel_4, yaw, is so that it doesn't spin on startup.
  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400 || receiver_input_channel_4 > 1540){
    start ++;                                                  //While waiting increment start whith every loop.
    
    //The esc's beep annoyingly. So this gives them a 1000us pulse while waiting for the receiver inputs.
    PORTD |= B11110000;                                        //Sets digital port 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                   //Wait 1000us.  
    PORTD &= B00001111;                                        //Sets digital port 4, 5, 6 and 7 low.
    delay(3);                                                  //Wait 3 milliseconds before the next loop.

  }
  start = 0;                                                   //Set start back to 0.
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
      //Gets the current gyro data and scales it to degrees per second for the pid calculations.
      gyro_signal();
      gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
      gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
      gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw / 57.14286) * 0.2);               //Gyro pid input is deg/sec.
    
      //For starting the motors: throttle low and yaw left (step 1).
      if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
      //When yaw stick is back in the center position start the motors (step 2).
      if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
        start = 2;
        //Reset the pid controllers for a smoother start.
        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
      }
      //Stopping the motors: throttle low and yaw right.
      if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;
      
      //The PID set point in degrees per second is determined by the roll receiver input.
      //In the case of dividing by 3 the max roll rate is approx 164 degrees per second ( (500-8)/3 = 164d/s ).
      pid_roll_setpoint = 0;
      //We need a little dead band of 16us for better results.
      if(receiver_input_channel_1 > 1508)pid_roll_setpoint = (receiver_input_channel_1 - 1508)/3.0;
      else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = (receiver_input_channel_1 - 1492)/3.0;
      
      //The PID set point in degrees per second is determined by the pitch receiver input.
      //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
      pid_pitch_setpoint = 0;
      //Needs a slight dead band of 16us for better results.
      if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = (receiver_input_channel_2 - 1508)/3.0;
      else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = (receiver_input_channel_2 - 1492)/3.0;
      
      //The PID set point in degrees per second is determined by the yaw receiver input.
      //In the case of dividing by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
      pid_yaw_setpoint = 0;
      //Needs a slight dead band of 16us for better results.
      if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
        if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
        else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
      }
      //PID inputs are known. So it can calculate the pid output.
      calculate_pid();

      
      throttle = receiver_input_channel_3;                                      //Need the throttle signal as a base signal.
      
      if (start == 2){                                                          //The motors are started.
        if (throttle > 1800) throttle = 1800;                                   //Need some room to keep full control at full throttle.
        esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
        esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
        esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
        esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
    
        if (esc_1 < 1200) esc_1 = 1200;                                         //Keep the motors running.
        if (esc_2 < 1200) esc_2 = 1200;                                         //Keep the motors running.
        if (esc_3 < 1200) esc_3 = 1200;                                         //Keep the motors running.
        if (esc_4 < 1200) esc_4 = 1200;                                         //Keep the motors running.
        
        if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
        if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
        if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
        if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
      }
      
      else{
        esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for esc-1.
        esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for esc-2.
        esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for esc-3.
        esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for esc-4.
      }
      
      //All the information for controlling the motor's is available.
      //The refresh rate is 250Hz. That means the esc's need a pulse every 4ms.
      while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
      loop_timer = micros();                                                    //Set the timer for the next loop.
    
      PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
      timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the falling edge of the esc-1 pulse.
      timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the falling edge of the esc-2 pulse.
      timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the falling edge of the esc-3 pulse.
      timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the falling edge of the esc-4 pulse.
      
      while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
        esc_loop_timer = micros();                                              //Read the current time.
        if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
        if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
        if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
        if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
      }
}





///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10 or 11 changed state
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                        //Is input 8 high?
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signal(){
  Wire.beginTransmission(105);                                 //Start communication with the gyro (address 1101001)
  Wire.write(168);                                             //Start reading @ register 28h and auto increment with every read
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(105, 6);                                    //Request 6 bytes from the gyro
  while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_roll = ((highByte<<8)|lowByte);                         //Multiply highByte by 256 (shift left by 8) and add lowByte
  if(cal_int == 2000)gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_pitch = ((highByte<<8)|lowByte);                        //Multiply highByte by 256 (shift left by 8) and add lowByte
  gyro_pitch *= -1;                                            //Invert axis
  if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;             //Only compensate after the calibration
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_yaw = ((highByte<<8)|lowByte);                          //Multiply highByte by 256 (shift left by 8) and add lowByte
  gyro_yaw *= -1;                                              //Invert axis
  if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;                 //Only compensate after the calibration
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;    //difference between gyro output and reciever input
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;      //degrees per second to correct error
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
  
  //motor output = error * compensation speed P + error * compensation speed I + compensation speed * (error - last error) 
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);  //output PID equation
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
  
  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}


