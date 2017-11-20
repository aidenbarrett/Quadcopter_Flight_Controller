void setup() {
    // sets up Pin Change Interrupt Enable 0 bit (PCIE0) in the Pin Change Interrupts Control Register (PCICR)
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
  
  // PINB & (1<<0) is the same as digital read but uses less CPU cycles. 
  // ***Interrupts must be as short as possible***
  
  // Compares channel one state to input on PINB bitwise AND'ed with B00000001
  
  if(channel_1 == 0 && PINB & (1<<0)){   // Input on pin 8 changed from 0 to 1 
    channel_1 = 1;                 // Remember current input state
    timer_1 = micros();                 // Set timer_1 to micros()
  }
  else if(channel_1 == 1 && !(PINB & (1<<0))){     // Input on pin 8 changed from 1 to 0
    channel_1 = 0;                                 // Remember current input state
    receiver_input_channel1 = micros() - timer_1; // Channel 1 is micros() - timer_1
  }
}
