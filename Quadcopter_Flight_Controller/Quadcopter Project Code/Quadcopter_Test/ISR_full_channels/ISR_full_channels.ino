void setup() {
    // sets up Pin Change Interrupt Enable 0 bit (PCIE0) in the Pin Change Interrups Control Register (PCICR)
  PCICR |= (1 << PCIE0); 
  
  // Pin Change Interrupt 0 - Pin 8
  // set PCINT0 to trigger an interrupt in the Pin Change Mask Register 0 (PCMSK0) on a change of state
  PCMSK0 |= (1 << PCINT0); 
  
  // Pin Change Interrupt 0 - Pin 9
  // set PCINT1 to trigger an interrupt in the Pin Change Mask Register 0 (PCMSK0) on a change of state
  PCMSK0 |= (1 << PCINT1); 
  
  // Pin Change Interrupt 0 - Pin 10
  // set PCINT2 to trigger an interrupt in the Pin Change Mask Register 0 (PCMSK0) on a change of state
  PCMSK0 |= (1 << PCINT2); 
  
  // Pin Change Interrupt 0 - Pin 11
  // set PCINT3 to trigger an interrupt in the Pin Change Mask Register 0 (PCMSK0) on a change of state
  PCMSK0 |= (1 << PCINT3);

}

void loop() {
  // put your main code here, to run repeatedly:

}

// The ISR is called every time the input on pins 8, 9, 10 or 11 change state
ISR(PCINT0_vect){
  
  // Channel 1 ***************************
  if(channel_1 == 0 && PINB & (1<<0)){   // Input on pin 8 changed from 0 to 1 
    channel_1 = 1;                       // Stores current input state
    timer_1 = micros();                  // Set timer_1 to micros()
  }
  else if(channel_1 == 1 && !(PINB & (1<<0))){     // Input on pin 8 changed from 1 to 0
    channel_1 = 0;                                 // Stores current input state
    receiver_input_channel1 = micros() - timer_1;  // Channel 1 is micros() - timer_1
  }
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
