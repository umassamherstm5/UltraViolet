/*
  Master control program for UltraViolet
  Charles B. Malloch, PhD  2015-03-31
  
  Reads joystick and buttons from a game controller and transmits their
  state via XBee radio to the UltraViolet platform, on which an Arduino is 
  running slave.ino (slave_devl during development).
  
*/

#define VERSION "0.0.1" 
#define VERDATE "2015-03-31"
#define PROGMONIKER "MDV"

// SCAN_DELAY_ms is the interval at which we scan the controls for changes
#define SCAN_DELAY_ms 50

#define BAUDRATE 19200

#define paJoystickLR A0
#define paJoystickUD A1
#define pdStageUp 10
#define pdBlinkyLED 13

/*

  The messages from the controller will have the following format:
    ">>" 0x3e3e (16 bits) note that 0x3e3e is beyond range of 0x2710=10000
    <joystickLR> signed 16-bit integer, low-order byte first
      -10000 - 10000 ( joystick position pct * 100 0 is ctr, positive right )
    <joystickUD> signed 16-bit integer 
      likewise, positive up
    <status> unsigned 16-bit integer, low-order byte first
      bit 0: 1 -> platform up
      bits 1-15 -> RFU
    checksum (16 bit XOR of cmdLeft, cmdRight, and status)
    
*/

enum { BUFLEN = 80 };
char strBuf [ BUFLEN ];
  
void setup () {
  
  pinMode ( pdStageUp, INPUT_PULLUP );
  pinMode ( pdBlinkyLED, OUTPUT );
  
  Serial.begin ( BAUDRATE );
  
  while ( !Serial && ( millis() < 5000 ) ) {
    // wait for a limited time for Serial port to open
    digitalWrite ( pdBlinkyLED, ( millis() >> 8 ) & 0x00000001 );
  }
  
  // the robot will ignore this, but it helps for program verification
  snprintf(strBuf, BUFLEN, "%s: UltraViolet master v%s (%s)\n", 
    PROGMONIKER, VERSION, VERDATE);
  Serial.print(strBuf);

}

void loop () {
  
  #define joystickJitterTolerance 5
  
  static unsigned long lastMessageAt_ms = 0;
  static int oldJoystickLR = 0, oldJoystickUD = 0;
  static unsigned int oldBits = 0;
  
  int joystickLR, joystickUD;
  uint16_t bits, checksum;
  
  // update the state vectors
  
  joystickLR = analogRead ( paJoystickLR );
  joystickUD = analogRead ( paJoystickUD );
  bits = 0 
       | ( ( ! digitalRead ( pdStageUp ) ) & 0x0001 ) << 0;
        
  /*
    send a message whenever a change in state is detected 
    or every second, whichever happens first
  */
  
  int changed = (
                    ( bits != oldBits )
                  | ( abs ( joystickLR - oldJoystickLR ) > joystickJitterTolerance )
                  | ( abs ( joystickUD - oldJoystickUD ) > joystickJitterTolerance )
                );
  
  if ( ( ( millis() - lastMessageAt_ms ) >= 1000UL ) || changed ) {
    
    // assemble the message before sending so it can be sent all at once
    
    checksum = joystickLR ^ joystickUD ^ bits;
    
    strBuf [ 0 ] = '>';
    strBuf [ 1 ] = '>';
    strBuf [ 2 ] = ( joystickLR >> 0 ) & 0x00ff;
    strBuf [ 3 ] = ( joystickLR >> 8 ) & 0x00ff;
    strBuf [ 4 ] = ( joystickUD >> 0 ) & 0x00ff;
    strBuf [ 5 ] = ( joystickUD >> 8 ) & 0x00ff;
    strBuf [ 6 ] = ( checksum  >> 0 ) & 0x00ff;
    strBuf [ 7 ] = ( checksum  >> 8 ) & 0x00ff;
    
    Serial.write ( ( uint8_t * ) strBuf, 8 );
    
    lastMessageAt_ms = millis();
    
    if ( changed ) {
      
      // update old positions only if change occurred, so the old joystick
      // position can't drift without being detected.
      
      oldBits = bits;
      oldJoystickLR = joystickLR;
      oldJoystickUD = joystickUD;
    }
    
  }
  
  delay ( SCAN_DELAY_ms );
}
