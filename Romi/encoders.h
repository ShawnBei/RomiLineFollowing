// These #defines act like a find-and-replace
// in your code, and make your code more readable.

#define E1_A_PIN  7 // left
#define E1_B_PIN  23 
#define E0_A_PIN  26 //right
//E0_B_Pin is defined seperately later

// Volatile Global variables used by Encoder ISR.
volatile long count_e1; // used by encoder to count the rotation
volatile bool oldE1_A;  // used by encoder to remember prior state of A
volatile bool oldE1_B;  // used by encoder to remember prior state of B

volatile long count_e0; // used by encoder to count the rotation
volatile bool oldE0_A;  // used by encoder to remember prior state of A
volatile bool oldE0_B;  // used by encoder to remember prior state of B

volatile unsigned long e0_interval;
volatile unsigned long e0_last_time;
volatile float e0_speed;

volatile unsigned long e1_interval;
volatile unsigned long e1_last_time;
volatile float e1_speed;

// This ISR handles just Encoder 1
// ISR to read the Encoder1 Channel A and B pins
// and then look up based on  transition what kind of
// rotation must have occured.
ISR( INT6_vect ) {

  e1_interval = micros() - e1_last_time;

  e1_speed = 152000 / e1_interval;
  
  e1_last_time = micros();
  
  // First, Read in the new state of the encoder pins.
  // Standard pins, so standard read functions.
  boolean newE1_B = digitalRead( E1_B_PIN );
  boolean newE1_A = digitalRead( E1_A_PIN );

  // Some clever electronics combines the
  // signals and this XOR restores the
  // true value.
  newE1_A ^= newE1_B;

  // Create a bitwise representation of our states
  // We do this by shifting the boolean value up by
  // the appropriate number of bits, as per our table
  // header:
  //
  // State :  (bit3)  (bit2)  (bit1)  (bit0)
  // State :  New A,  New B,  Old A,  Old B.
  byte state = 0;
  state = state | ( newE1_A  << 3 );
  state = state | ( newE1_B  << 2 );
  state = state | ( oldE1_A  << 1 );
  state = state | ( oldE1_B  << 0 );


  // This is an inefficient way of determining
  // the direction.  However it illustrates well
  // against the lecture slides.
  switch ( state ) {
    case 0:              break; // No movement.
    case 1:  count_e1++; break;  // clockwise?
    case 2:  count_e1--; break;  // anti-clockwise?
    case 3:              break;  // Invalid
    case 4:  count_e1--; break;  // anti-clockwise?
    case 5:              break;  // No movement.
    case 6:              break;  // Invalid
    case 7:  count_e1++; break;  // clockwise?
    case 8:  count_e1++; break;  // clockwise?
    case 9:              break;  // Invalid
    case 10:             break;  // No movement.
    case 11: count_e1--; break;  // anti-clockwise?
    case 12:             break;  // Invalid
    case 13: count_e1--; break;  // anti-clockwise?
    case 14: count_e1++; break;  // clockwise?
    case 15:             break;  // No movement.
  }

  // Save current state as old state for next call.
  oldE1_A = newE1_A;
  oldE1_B = newE1_B;

}


// This ISR handles just Encoder 0
// ISR to read the Encoder0 Channel A and B pins
// and then look up based on  transition what kind of
// rotation must have occured.
ISR( PCINT0_vect ) {

  e0_interval = micros() - e0_last_time;

  e0_speed = 152000 / e0_interval;
  
  e0_last_time = micros();

  
  // First, Read in the new state of the encoder pins.

  // Mask for a specific pin from the port.
  // Non-standard pin, so we access the register
  // directly.  
  // Reading just PINE would give us a number
  // composed of all 8 bits.  We want only bit 2.
  // B00000100 masks out all but bit 2
  boolean newE0_B = PINE & (1<<PINE2);
  //boolean newE0_B = PINE & B00000100;  // Does same as above.

  // Standard read fro the other pin.
  boolean newE0_A = digitalRead( E0_A_PIN ); // 26 the same as A8

  // Some clever electronics combines the
  // signals and this XOR restores the 
  // true value.
  newE0_A ^= newE0_B;


  
  // Create a bitwise representation of our states
  // We do this by shifting the boolean value up by
  // the appropriate number of bits, as per our table
  // header:
  //
  // State :  (bit3)  (bit2)  (bit1)  (bit0)
  // State :  New A,  New B,  Old A,  Old B.
  byte state = 0;                   
  state = state | ( newE0_A  << 3 );
  state = state | ( newE0_B  << 2 );
  state = state | ( oldE0_A  << 1 );
  state = state | ( oldE0_B  << 0 );

  // This is an inefficient way of determining
  // the direction.  However it illustrates well
  // against the lecture slides.  
  switch( state ) {
case 0:              break; // No movement.
    case 1:  count_e0++; break;  // clockwise?
    case 2:  count_e0--; break;  // anti-clockwise?
    case 3:              break;  // Invalid
    case 4:  count_e0--; break;  // anti-clockwise?
    case 5:              break;  // No movement.
    case 6:              break;  // Invalid
    case 7:  count_e0++; break;  // clockwise?
    case 8:  count_e0++; break;  // clockwise?
    case 9:              break;  // Invalid
    case 10:             break;  // No movement.
    case 11: count_e0--; break;  // anti-clockwise?
    case 12:             break;  // Invalid
    case 13: count_e0--; break;  // anti-clockwise?
    case 14: count_e0++; break;  // clockwise?
    case 15:             break;  // No movement.
  }
     
  // Save current state as old state for next call.
  oldE0_A = newE0_A;
  oldE0_B = newE0_B; 
}





/*
   This setup routine enables interrupts for
   encoder1.  The interrupt is automatically
   triggered when one of the encoder pin changes.
   This is really convenient!  It means we don't
   have to check the encoder manually.
*/
void setupEncoder1() {

  // Initialise our count value to 0.
  count_e1 = 0;

  // Initialise the prior A & B signals
  // to zero, we don't know what they were.
  oldE1_A = 0;
  oldE1_B = 0;

  // Setup pins for encoder 1
  pinMode( E1_A_PIN, INPUT );
  pinMode( E1_B_PIN, INPUT );

  // Now to set up PE6 as an external interupt (INT6), which means it can
  // have its own dedicated ISR vector INT6_vector

  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  // Disable external interrupts for INT6 first
  // Set INT6 bit low, preserve other bits
  EIMSK = EIMSK & ~(1<<INT6);
  //EIMSK = EIMSK & B1011111; // Same as above.
  
  // Page 89, 11.1.2 External Interrupt Control Register B – EICRB
  // Used to set up INT6 interrupt
  EICRB |= ( 1 << ISC60 );  // using header file names, push 1 to bit ISC60
  //EICRB |= B00010000; // does same as above

  // Page 90, 11.1.4 External Interrupt Flag Register – EIFR
  // Setting a 1 in bit 6 (INTF6) clears the interrupt flag.
  EIFR |= ( 1 << INTF6 );
  //EIFR |= B01000000;  // same as above

  // Now that we have set INT6 interrupt up, we can enable
  // the interrupt to happen
  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  // Disable external interrupts for INT6 first
  // Set INT6 bit high, preserve other bits
  EIMSK |= ( 1 << INT6 );
  //EIMSK |= B01000000; // Same as above

}

void setupEncoder0() {

    // Initialise our count value to 0.
    count_e0 = 0;

    // Initialise the prior A & B signals
    // to zero, we don't know what they were.
    oldE0_A = 0;
    oldE0_B = 0;

    // Setting up E0_PIN_B:
    // The Romi board uses the pin PE2 (port E, pin 2) which is
    // very unconventional.  It doesn't have a standard
    // arduino alias (like d6, or a5, for example).
    // We set it up here with direct register access
    // Writing a 0 to a DDR sets as input
    // DDRE = Data Direction Register (Port)E
    // We want pin PE2, which means bit 2 (counting from 0)
    // PE Register bits [ 7  6  5  4  3  2  1  0 ]
    // Binary mask      [ 1  1  1  1  1  0  1  1 ]
    //    
    // By performing an & here, the 0 sets low, all 1's preserve
    // any previous state.
    DDRE = DDRE & ~(1<<DDE6);
    //DDRE = DDRE & B11111011; // Same as above. 

    // We need to enable the pull up resistor for the pin
    // To do this, once a pin is set to input (as above)
    // You write a 1 to the bit in the output register
    PORTE = PORTE | (1<< PORTE2 );
    //PORTE = PORTE | 0B00000100;

    // Encoder0 uses conventional pin 26
    pinMode( E0_A_PIN, INPUT );
    digitalWrite( E0_A_PIN, HIGH ); // Encoder 0 xor

    // Enable pin-change interrupt on A8 (PB4) for encoder0, and disable other
    // pin-change interrupts.
    // Note, this register will normally create an interrupt a change to any pins
    // on the port, but we use PCMSK0 to set it only for PCINT4 which is A8 (PB4)
    // When we set these registers, the compiler will now look for a routine called
    // ISR( PCINT0_vect ) when it detects a change on the pin.  PCINT0 seems like a
    // mismatch to PCINT4, however there is only the one vector servicing a change
    // to all PCINT0->7 pins.
    // See Manual 11.1.5 Pin Change Interrupt Control Register - PCICR
    
    // Page 91, 11.1.5, Pin Change Interrupt Control Register 
    // Disable interrupt first
    PCICR = PCICR & ~( 1 << PCIE0 );
    // PCICR &= B11111110;  // Same as above
    
    // 11.1.7 Pin Change Mask Register 0 – PCMSK0
    PCMSK0 |= (1 << PCINT4);
    
    // Page 91, 11.1.6 Pin Change Interrupt Flag Register – PCIFR
    PCIFR |= (1 << PCIF0);  // Clear its interrupt flag by writing a 1.

    // Enable
    PCICR |= (1 << PCIE0);
}

void resetEncoder()
{
  count_e0 = 0;
  count_e1 = 0;
}
