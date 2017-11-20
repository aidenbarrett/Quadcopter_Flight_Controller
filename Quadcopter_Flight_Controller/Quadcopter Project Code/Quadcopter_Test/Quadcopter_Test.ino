//Declaring Variables
byte channel_1, channel_2, channel_3, channel_4; // A byte stores an 8-bit unsigned number, from 0 to 255
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4; // Unsigned longs (32-bit) won't store negative numbers.

//Setup routine
void setup(){
  
  // sets up Pin Change Interrupt Enable 0 bit (PCIE0) in the Pin Change Interrups Control Register (PCICR)
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  
  // Pin Change Interrupt 0 - Pin 8
  // set PCINT0 to trigger an interrupt in the Pin Change Mask Register 0 (PCMSK0) on a change of state
  PCMSK0 |= (1 << PCINT0);  // set PCINT0 (digital input 8) to trigger an interrupt on state change
  
  // Pin Change Interrupt 0 - Pin 9
  // set PCINT1 to trigger an interrupt in the Pin Change Mask Register 0 (PCMSK0) on a change of state
  PCMSK0 |= (1 << PCINT1);  // set PCINT1 (digital input 9)to trigger an interrupt on state change
  
  // Pin Change Interrupt 0 - Pin 10
  // set PCINT2 to trigger an interrupt in the Pin Change Mask Register 0 (PCMSK0) on a change of state
  PCMSK0 |= (1 << PCINT2);  // set PCINT2 (digital input 10)to trigger an interrupt on state change
  
  // Pin Change Interrupt 0 - Pin 11
  // set PCINT3 to trigger an interrupt in the Pin Change Mask Register 0 (PCMSK0) on a change of state
  PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 11)to trigger an interrupt on state change
  Serial.begin(9600); 
}

//Main program loop
void loop(){
  delay(250);
  print_signals(); // function call for displaying the signals
}

// The ISR is called every time the input on pins 8, 9, 10 or 11 change state
ISR(PCINT0_vect){
 
  // ***Interrupts must be as short as possible*** 
  // PINB & (1<<0) is the same as digital read but uses less CPU cycles. 
  
  // ********************************************************************************************
  // Checks to see if there is a rising edge on the channel, if so, starts a timer.
  // Stops timer when there is a falling edge.
  // Elapsed time between rising and falling edges is equal to the pulse width.
  // It does this for each of the channels simultaneously, allowing multiple signals to be sent.
  // ********************************************************************************************
  
  // Channel 1 ***************************
  if(channel_1 == 0 && PINB & (1<<0)){   // Input on pin 8 changed from 0 to 1 
    channel_1 = 1;                       // Stores current input state
    timer_1 = micros();                  // Set timer_1 to micros()
  }
  else if(channel_1 == 1 && !(PINB & (1<<0))){     // Input on pin 8 changed from 1 to 0
    channel_1 = 0;                                 // Stores current input state
    receiver_input_channel_1 = micros() - timer_1;  // Channel 1 is micros() - timer_1
  }
  // Channel 2 ***************************
  if(channel_2 == 0 && PINB & (1<<1)){   // Input on pin 9 changed from 0 to 1 
    channel_2 = 1;                       // Stores current input state
    timer_2 = micros();                  // Set timer_2 to micros()
  }
  else if(channel_2 == 1 && !(PINB & (1<<1))){     // Input 9 changed from 1 to 0
    channel_2 = 0;                                 // Stores current input state
    receiver_input_channel_2 = micros() - timer_2; // Channel 2 is micros() - timer_2
  }
  // Channel 3 ***************************
  if(channel_3 == 0 && PINB & (1<<2)){   // Input on pin 10 changed from 0 to 1 
    channel_3 = 1;                       // Stores current input state
    timer_3 = micros();                  // Set timer_3 to micros()
  }
  else if(channel_3 == 1 && !(PINB & (1<<2))){     // Input on pin 10 changed from 0 to 1 
    channel_3 = 0;                                 // Stores current input state
    receiver_input_channel_3 = micros() - timer_3; // Channel 3 is micros() - timer_3
  }
  // Channel 4 ***************************
  if(channel_4 == 0 && PINB & (1<<3)){   // Input on pin 11 changed from 0 to 1 
    channel_4 = 1;                       // Stores current input state
    timer_4 = micros();                  // Set timer_4 to micros()
  }
  else if(channel_4 == 1 && !(PINB & (1<<3))){     // Input on pin 11 changed from 0 to 1 
    channel_4 = 0;                                 // Stores current input state
    receiver_input_channel_4 = micros() - timer_4; // Channel 4 is micros() - timer_4
  }
}

// Displaying the receiver signals
void print_signals(){
  
  Serial.print("Pitch:");
  Serial.print(receiver_input_channel_1);
  
  Serial.print("  Roll:");
  Serial.print(receiver_input_channel_2);
  
  Serial.print("  Yaw:");
  Serial.print(receiver_input_channel_3);
  
  Serial.print("  Throttle:");
  Serial.println(receiver_input_channel_4);
}
