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
