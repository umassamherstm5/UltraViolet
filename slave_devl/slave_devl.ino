/*
  Slave program for UltraViolet ( embedded on UltraViolet platform )
  Charles B. Malloch, PhD  2015-03-31
  
  Receives joystick and buttons state from game controller 
  running master.ino (master_devl during development) 
  via XBee radio. Uses this data to command motor speeds and platform
  pneumatics valve.
  
  Last change was to convert commanded speeds to percent 
    [-100.0, 100.0] from [-1.0, 1.0].
  Once operation is satisfactory, try decreasing verbosity.
  
  Updated verbosity code, separated motor verbosity out
  Removed delay of 50ms from loop, replaced with timeout
  
*/

#define VERSION "0.6.0" 
#define VERDATE "2015-04-02"
#define PROGMONIKER "SLD"

#include <math.h>
#include <Adafruit_NeoPixel.h>

#define BAUDRATE 19200
#define SLD_VERBOSE 8
#define COMMUNICATIONS_TIMEOUT_ms 2000
#define STICK_DEADBAND_cts 10

// Pins
#define pd_MOTOR_ENABLE 4
#define pp_L_FWD 5
#define pp_L_REV 6
#define pp_R_FWD 9
#define pp_R_REV 10
#define pd_TABLE_DIR_UP 7
#define pd_TABLE_EN 8

#define pd_NeoPixel_Control 12

// LED strip works like NeoPixels

#define nPixels 8
unsigned short colors[][3] = { { 127, 0, 0 },        // red
                               { 0, 127, 0 },        // green
                               { 0, 0, 127 },        // blue
                               { 0, 127, 127 },      // cyan
                               { 127, 0, 127 },      // magenta
                               { 127, 127, 0 },      // yellow
                               { 0, 0, 0 },          // black
                               { 127, 127, 127 } };  // white
#define RED 0
#define GREEN 1
#define BLUE 2
#define CYAN 3
#define MAGENTA 4
#define YELLOW 5
#define BLACK 6
#define WHITE 7

#define DIR_FWD 1
#define DIR_REV -1
#define DIR_UP 1
#define DIR_DOWN -1

enum { BUFLEN = 80 };
char strBuf [ BUFLEN ];

int master_enable = 0;

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)

Adafruit_NeoPixel neoPixelStrip = Adafruit_NeoPixel(nPixels, pd_NeoPixel_Control, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

class Motor {
  public:
  
    Motor () {
      reset ();
    }
    
    Motor ( int motor_ID, int pp_FWD, int pp_REV ) {
      _motor_ID = motor_ID;
      _pp_FWD = pp_FWD;
      pinMode ( _pp_FWD, OUTPUT );
      digitalWrite ( _pp_FWD, 0 );
      _pp_REV = pp_REV;
      pinMode ( _pp_REV, OUTPUT );
      digitalWrite ( _pp_REV, 0 );
      reset ();
    }
    
    void reset () {
      digitalWrite ( _pp_FWD, 0 );
      digitalWrite ( _pp_REV, 0 );
      _curr_speed_pct = 0.0;
      _curr_direction = DIR_FWD;
      _cmd_speed_pct = 0.0;
      _cmd_direction = DIR_FWD;
    }
    
    float speed ( float pct = -1.0, int dir = DIR_FWD ) {
      if ( pct < 0.0 ) {
        return ( _curr_speed_pct );
      }
      _cmd_speed_pct = pct;
      _cmd_direction = dir;
      // not handling enable here because it affects both motors
    }
    
    void tick () {
    
      #if _MOTOR_VERBOSE >= 4
        // hack!
        for ( int i = 0; i < ( _motor_ID == 100 ? 0 : 40 ); i++ ) {
          Serial.print ( " " );
        }
        Serial.print ( _motor_ID );
        Serial.print ( " at " );
        Serial.print ( millis() );
        Serial.print ( " speed " );
        Serial.print ( _curr_speed_pct );
        Serial.print ( _curr_direction == DIR_FWD ? " F " : " R " );
        Serial.println ();
      #endif
      
      if (    ( fabs ( _cmd_speed_pct - _curr_speed_pct ) < 1e-3 )
           && ( _cmd_direction == _curr_direction ) ) {
        #if _MOTOR_VERBOSE >= 12
          Serial.print ( "Tick: we're good\n" );
        #endif
        return;
      }
    
      float max_chg_pct = _max_speed_change_per_step_pct;
      
      #if _MOTOR_VERBOSE >= 10
        Serial.print ( "max_chg_pct: " );
        Serial.println ( max_chg_pct );
      #endif
                            
      if ( _last_change_at != 0UL ) {
        // continuing change in progress
        float max_accel_pct = _max_accel_per_sec_pct
                            * ( millis() - _last_change_at )
                            / 1000.0;
        if ( max_accel_pct < max_chg_pct ) {
          max_chg_pct = max_accel_pct;
        }
      }
      
      #if _MOTOR_VERBOSE >= 10
        Serial.print ( "       |-=>  max_chg_pct: " );
        Serial.println ( max_chg_pct );
      #endif
                            
      if ( _curr_direction != _cmd_direction ) {
        // decelerate to zero before changing _curr_cirection
        if ( fabs ( _curr_speed_pct ) < max_chg_pct ) {
          // can come to full stop
          #if _MOTOR_VERBOSE >= 12
            Serial.print ( "full stop\n" );
          #endif                           
          _curr_speed_pct = 0.0;
          _curr_direction = _cmd_direction;
        } else {
          // cannot yet come to full stop
          _curr_speed_pct -= max_chg_pct;
          #if _MOTOR_VERBOSE >= 12
            Serial.print ( "stopping\n" );
          #endif
        }
      } else {
        // same direction, might be accel or decel
        if ( _cmd_speed_pct < _curr_speed_pct ) {
          // decel
          _curr_speed_pct -= max_chg_pct;
          if ( _curr_speed_pct < _cmd_speed_pct ) {
            // please don't overshoot
            _curr_speed_pct = _cmd_speed_pct;
          }
          if ( _curr_speed_pct < 0.0 ) {
            _curr_speed_pct = 0.0;
          }
          #if _MOTOR_VERBOSE >= 10
            Serial.print ( "decel\n" );
          #endif
        } else {
          // accel
          _curr_speed_pct += max_chg_pct;
          if ( _curr_speed_pct > _cmd_speed_pct ) {
            // please don't overshoot
            _curr_speed_pct = _cmd_speed_pct;
          }
          if ( _curr_speed_pct > 100.0 ) {
            _curr_speed_pct = 100.0;
          }
          #if _MOTOR_VERBOSE >= 10
            Serial.print ( "accel\n" );
          #endif
        }
      }
      
      // now enact the new speeds
      switch ( _curr_direction ) {
        case DIR_FWD:
          analogWrite ( _pp_FWD, _curr_speed_pct );
          break;
        case DIR_REV:
          analogWrite ( _pp_REV, _curr_speed_pct );
          break;
      }
      
      if (   ( _curr_direction == _cmd_direction ) 
          && ( _curr_speed_pct == _cmd_speed_pct ) ) {
        // changes are complete
        #if _MOTOR_VERBOSE >= 12
          Serial.print ( _motor_ID );
          Serial.print ( " --> Done" );
          Serial.println ();
        #endif
        _last_change_at = 0UL;
      } else {
        _last_change_at = millis ();
      }
    }  // tick

        
  private:
    int _motor_ID;
    int _pp_FWD;
    int _pp_REV;
    int _pd_ENABLE;
    float _curr_speed_pct;
    unsigned char _curr_direction;
    unsigned long _last_change_at;
    float _cmd_speed_pct;
    int _cmd_direction;
    static const float _max_accel_per_sec_pct = 100.0 / 5.0;
    static const float _max_speed_change_per_step_pct = 0.5;
    enum { _MOTOR_VERBOSE = 6 };
};

Motor left ( 100, pp_L_FWD, pp_L_REV ); 
Motor right ( 200, pp_R_FWD, pp_R_REV ); 

unsigned long lastCmdAt_ms = 0;

void setup() {

  pinMode ( pd_TABLE_EN, OUTPUT );
  digitalWrite ( pd_TABLE_EN, 0 );
  pinMode ( pd_TABLE_DIR_UP, OUTPUT );
  digitalWrite ( pd_TABLE_DIR_UP, 0 );
  pinMode ( pd_MOTOR_ENABLE, OUTPUT );
  digitalWrite ( pd_MOTOR_ENABLE, 0 );

  #ifdef BAUDRATE
    Serial.begin ( BAUDRATE );
    while ( millis() < 5000 && !Serial ) { };
    delay ( 2000 );
    snprintf ( strBuf, BUFLEN, "%s: UltraViolet slave hardware and software v%s (%s)\n", 
      PROGMONIKER, VERSION, VERDATE );
    Serial.println ( strBuf );
  #endif

  neoPixelStrip.begin();
  neoPixelStrip.show(); // Initialize all pixels to 'off'
  
  setNeoPixelColor ( 0, BLUE );
  
}

#define TICK_PERIOD_ms 50

void loop() {
  
  static unsigned int lastDisplayAt_ms = 0;

  handleControl();
  
  if ( ( millis() - lastCmdAt_ms ) < COMMUNICATIONS_TIMEOUT_ms ) {
    setNeoPixelColor ( 0, GREEN );
    enable_movement ();
  } else {
    // comm timeout has occurred
    disable_movement ();
    setNeoPixelColor ( 0, RED );
  }
    
  if ( ( millis() - lastDisplayAt_ms ) >= TICK_PERIOD_ms ) {
    
    tick ();

    showSpeed ( 5, left.speed () );
    showSpeed ( 6, right.speed () );
    lastDisplayAt_ms = millis();
    
  }
  
}

void tick () {
  static int printed = 5;
  if ( ! master_enable ) {
    #ifdef BAUDRATE
      if ( printed ) {
        Serial.println ( F ( "Not enabled" ) );
        printed--;
      }
    #endif
    return;
  }
  left.tick();
  right.tick();
  printed = 5;
}

int enable_movement () {
  master_enable = 1;
  digitalWrite ( pd_MOTOR_ENABLE, master_enable );
  return ( master_enable );
}

int disable_movement () {
  master_enable = 0;
  digitalWrite ( pd_MOTOR_ENABLE, master_enable );
  return ( master_enable );
}

void setNeoPixelColor ( int n, int c ) {
  neoPixelStrip.setPixelColor( n, neoPixelStrip.Color( colors [c] [0],
                                                       colors [c] [1],
                                                       colors [c] [2]
                                                     ) ); // pixel n to color c
  neoPixelStrip.show();
}

void showSpeed ( int n, float speed ) {
  // show a color from red (stopped) to green (full speed)
  neoPixelStrip.setPixelColor( n, neoPixelStrip.Color( 255 - speed * 255 / 100, speed * 255 / 100, 0 ) );
  neoPixelStrip.show();
}


/*

  The messages from the controller will have the following format:
    ">>" 0x3e3e (16 bits) note that 0x3e3e is beyond range of 0x0400=1024
    <stickX> signed 16-bit integer, low-order byte first
      0 - 1024 ( joystick position pct * 100 0 is ctr, positive right )
    <stickY> signed 16-bit integer 
      likewise, positive up
    <bits> unsigned 16-bit integer, low-order byte first
      bit 0: 1 -> platform up
      bits 1-15 -> RFU
    checksum (16 bit XOR of cmdLeft, cmdRight, and bits)
    
*/

const int inBufLen = 20;
char inBuf [ inBufLen ];
int inBufPtr = 0;
  
void handleControl() {
  
  while ( Serial.available() ) {
          
    if ( inBufPtr >= inBufLen ) {
      Serial.println ("BUFFER OVERRUN");
      inBufPtr = 0;
    }
    
    inBuf [ inBufPtr ] = Serial.read();
    
    if ( inBuf [ inBufPtr ] == 0x3e ) {
      // received the start character; clear buffer
      inBufPtr = 0;
      setNeoPixelColor ( 1, BLUE );
    } else {
      inBufPtr++;
      setNeoPixelColor ( 1, MAGENTA );
    }
    
    // inBufPtr is the number of chars received; points to first unused char position
    
    if ( inBufPtr == 8 ) {
      // have received enough characters to be a complete message
      
      int16_t stickLR, stickUD;
      uint16_t bits, cksum, testCkSum;
      
      setNeoPixelColor ( 1, YELLOW );
      
      stickLR = inBuf [ 0 ] + ( inBuf [ 1 ] << 8 );
      stickUD = inBuf [ 2 ] + ( inBuf [ 3 ] << 8 );
      bits = inBuf [ 4 ] + ( inBuf [ 5 ] << 8 );
      cksum = inBuf [ 6 ] + ( inBuf [ 7 ] << 8 );
      testCkSum = stickLR ^ stickUD ^ bits;
      
      if ( testCkSum == cksum ) {
        // good message received
        lastCmdAt_ms = millis ();  // log time of arrival
        setNeoPixelColor ( 1, GREEN );
        if ( SLD_VERBOSE >= 8 ) {
          Serial.print ( " msg OK at " ); Serial.println ( millis() );
        }
        handleCommand ( stickLR, stickUD, bits );
      } else {
        if ( SLD_VERBOSE >= 1 ) {
          Serial.print ( " Bad checksum: " );
          Serial.print ( "L: 0x" ); Serial.print ( stickLR, HEX ); 
          Serial.print ( "; R: 0x" ); Serial.print ( stickUD, HEX ); 
          Serial.print ( "; bits: 0x" ); Serial.print ( bits, HEX ); 
          Serial.print ( "; his cksum: 0x" ); Serial.print ( cksum, HEX ); 
          Serial.print ( "; my cksum: 0x" ); Serial.print ( testCkSum, HEX );
          Serial.println ();
        }
        setNeoPixelColor ( 1, CYAN );
      }
      // whether or not msg was good, clear the buffer for next message
      inBufPtr = 0;
    }
  }  // reading the input buffer
  
  if ( SLD_VERBOSE >= 10 ) {
    for ( int i = 0; i < inBufPtr; i++ ) {
      Serial.print ( inBuf [ inBufPtr ], HEX );
    }
    Serial.println ();
  
  }  // Serial.available
  
}

// handleCommand updates wheel speed and platform commands
void handleCommand ( int16_t stickLR, int16_t stickUD, uint16_t bits ) {
  
  // translate stick position into speed commands
  // if stick position is ( x, y )
  //   L motor speed command is that projected onto vector (  1, 1 )
  //   R                                                   ( -1, 1 )
  
  // begin by imposing dead band; if the stick is within STICK_DEADBAND_cts
  // counts of center, call it centered -- forestalls slow drift
  
  if ( abs ( stickLR - 512 ) < STICK_DEADBAND_cts ) stickLR = 512;
  if ( abs ( stickUD - 512 ) < STICK_DEADBAND_cts ) stickUD = 512;
  
  float sX, sY, cR, cL;
  sX = float ( stickLR ) / 512.0 - 1.0;  // stick x, scaled -1 to 1
  sY = float ( stickUD ) / 512.0 - 1.0;
  
  #if SLD_VERBOSE >= 8
    Serial.print ( "hC: stickLR: " ); Serial.print ( stickLR );
    Serial.print ( "; stickUD: " ); Serial.print ( stickUD );
    Serial.print ( "; sX: " ); Serial.print ( sX );
    Serial.print ( "; sY: " ); Serial.print ( sY );
  #endif
  
  // reduce turning input at higher speeds
  sX *= ( 1.0 - fabs ( sY ) );
  #if SLD_VERBOSE >= 4
    Serial.print ( "   sX reduced to: " ); Serial.print ( sX );
    Serial.println ();
  #endif
  
  // cL and cR are the commanded speeds (in percent) for L and R motors
  cL = ( sX + sY ) * 100.0 / 1.414;
  cR = ( -sX + sY )* 100.0 / 1.414;
  
  if ( cL >= 0 ) {
    left.speed ( cL, DIR_FWD );
  } else {
    left.speed ( -cL, DIR_REV );
  }
  
  if ( cR > 0 ) {
    right.speed ( cR, DIR_FWD );
  } else {
    right.speed ( -cR, DIR_REV );
  }
  
  #if SLD_VERBOSE >= 6
    Serial.print ( "  cmd L: " ); Serial.print ( cL );
    Serial.print ( "  cmd R: " ); Serial.print ( cR );
    Serial.println ();
  #endif
    
  if ( bits & 0x0001 ) {
    // table up
    digitalWrite ( pd_TABLE_DIR_UP, 1 );
    digitalWrite ( pd_TABLE_EN, 1 );
    setNeoPixelColor ( 7, GREEN );
  } else {
    digitalWrite ( pd_TABLE_DIR_UP, 0 );
    digitalWrite ( pd_TABLE_EN, 0 );
    setNeoPixelColor ( 7, BLUE );
  }

}
