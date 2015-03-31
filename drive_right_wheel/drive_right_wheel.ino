#define VERSION "0.3.0" 
#define VERDATE "2015-03-30"
#define PROGMONIKER "DOW"

#include <Adafruit_NeoPixel.h>
#include <math.h>

#define BAUDRATE 115200
#define DOW_VERBOSE 18
#define MOTOR_RUN_TIME_ms 2000

// Pins
#define pd_MOTOR_ENABLE 4
#define pp_L_FWD 5
#define pp_L_REV 6
#define pp_R_FWD 9
#define pp_R_REV 10
#define pd_TABLE_DIR_UP 7
#define pd_TABLE_EN 8

#define pd_NeoPixel_Control 12

// HACK
#define pd_ENABLE_BUTTON 11
int enableButtonState = 0;  // 0 = switch open, 1 = switch closed (button down) DEBOUNCED

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

#define MTR_LEFT 1
#define MTR_RIGHT 2
#define TABLE 4
#define DIR_FWD 1
#define DIR_REV -1
#define DIR_UP 1
#define DIR_DOWN -1

#define bufLen ( 80 )
char strBuf[bufLen];

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

void sc ( int n, int c ) {
  neoPixelStrip.setPixelColor( n, neoPixelStrip.Color( colors [c] [0],
                                                       colors [c] [1],
                                                       colors [c] [2]
                                                     ) ); // pixel n to color c
  neoPixelStrip.show();
}

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
      _max_accel_per_sec_pct = 100.0 / 2.5;
      _max_speed_change_per_step_pct = 2.0;
    }
    
    float speed ( float pct = -1.0, int dir = DIR_FWD ) {
      if ( pct < 0.0 ) {
        return ( _curr_speed_pct );
      }
      _cmd_speed_pct = pct;
      _cmd_direction = dir;
      // not handling enable here because it affects both motors
    }
    
    void _go_to_motors_col ( int motor_ID ) {
      // hack - assuming that motor ID will be 100 or 200!
      for ( int i = 0; i < ( _motor_ID == 100 ? 0 : 40 ); i++ ) {
        Serial.print ( " " );
      }
    }

    void tick () {
    
      #if DOW_VERBOSE > 5
        _go_to_motors_col ( _motor_ID );
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
        #if DOW_VERBOSE >= 12
          _go_to_motors_col ( _motor_ID );
          Serial.print ( "Tick: we're good\n" );
        #endif
        return;
      }
    
      float max_chg_pct = _max_speed_change_per_step_pct;
      
      #if DOW_VERBOSE >= 12
        _go_to_motors_col ( _motor_ID );
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
      
      #if DOW_VERBOSE >= 12
        _go_to_motors_col ( _motor_ID );
        Serial.print ( "       |-=>  max_chg_pct: " );
        Serial.println ( max_chg_pct );
      #endif
                            
      if ( _curr_direction != _cmd_direction ) {
        // decelerate to zero before changing _curr_cirection
        if ( fabs ( _curr_speed_pct ) < max_chg_pct ) {
          // can come to full stop
          #if DOW_VERBOSE >= 10
            _go_to_motors_col ( _motor_ID );
            Serial.print ( "full stop\n" );
          #endif                           
          _curr_speed_pct = 0.0;
          _curr_direction = _cmd_direction;
        } else {
          // cannot yet come to full stop
          _curr_speed_pct -= max_chg_pct;
          #if DOW_VERBOSE >= 10
            _go_to_motors_col ( _motor_ID );
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
          #if DOW_VERBOSE >= 10
            _go_to_motors_col ( _motor_ID );
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
          #if DOW_VERBOSE >= 10
            _go_to_motors_col ( _motor_ID );
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
        #if DOW_VERBOSE > 5
          _go_to_motors_col ( _motor_ID );
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
    float _max_accel_per_sec_pct;
    float _max_speed_change_per_step_pct;
};

Motor left ( 100, pp_L_FWD, pp_L_REV ); 
Motor right ( 200, pp_R_FWD, pp_R_REV ); 

void setup() {

  pinMode ( pd_TABLE_EN, OUTPUT );
  digitalWrite ( pd_TABLE_EN, 0 );
  pinMode ( pd_TABLE_DIR_UP, OUTPUT );
  digitalWrite ( pd_TABLE_DIR_UP, 0 );
  pinMode ( pd_MOTOR_ENABLE, OUTPUT );
  digitalWrite ( pd_MOTOR_ENABLE, 0 );
  pinMode ( pd_ENABLE_BUTTON, INPUT_PULLUP );
  
  pinMode ( pp_L_FWD, OUTPUT );
  digitalWrite ( pp_L_FWD, 0 );
  pinMode ( pp_L_REV, OUTPUT );
  digitalWrite ( pp_L_REV, 0 );
  pinMode ( pp_R_FWD, OUTPUT );
  digitalWrite ( pp_R_FWD, 0 );
  pinMode ( pp_R_REV, OUTPUT );
  digitalWrite ( pp_R_REV, 0 );


  #ifdef BAUDRATE
    Serial.begin(BAUDRATE);
    while ( millis() < 5000 && !Serial ) { };
    delay ( 2000 );
    snprintf(strBuf, bufLen, "%s: Test UltraViolet platform L wheel v%s (%s)\n", 
      PROGMONIKER, VERSION, VERDATE);
    Serial.print(strBuf);
  #endif

  neoPixelStrip.begin();
  neoPixelStrip.show(); // Initialize all pixels to 'off'
  
  while ( ! ( enableButtonState = checkButtonState ( pd_ENABLE_BUTTON ) ) ) {
    // wait for button to be pressed
    delay ( 20 );
  }

  Serial.println ( "About to proceed with test\n\n" );
  enable_movement ();
  
}

// these have to be globals to enable abort subroutine to (side-)affect them!
volatile int state = 0;
volatile unsigned int state_started_at_ms = 0;
volatile unsigned long state_duration_ms;
volatile int abortOnButtonRelease;

void loop() {

  // setup guarantees button will be pressed at the first entry
  
  int newEnableButtonState = checkButtonState ( pd_ENABLE_BUTTON );
  if ( newEnableButtonState != enableButtonState ) {
    Serial.print ( "New button state: " ); 
    Serial.println ( newEnableButtonState ? "on / closed" : "off / open" );
  }
  enableButtonState = newEnableButtonState;
  
  static int oldState = -17;
  if ( state != oldState ) {
    Serial.print ( "New program state " ); 
    Serial.print ( oldState );
    Serial.print ( " -> " );
    Serial.println ( state );
    oldState = state;
  }
  
  #if DOW_VERBOSE > 118
    Serial.print ( state );
    Serial.print ( " / " );
    Serial.print ( enableButtonState );
    Serial.print ( ": " );
  #endif
  
  if ( state == 0 ) {
    static int justEnteredP = 1;
    
    // ========================================= test R FWD

    if ( 1 || justEnteredP ) {  // TESTING
      sc ( 0, RED );
      sc ( 1, YELLOW );
    
      // left.speed ( 100, DIR_FWD );  // TESTING
      int spd = analogRead ( 0 ) / 10;
      Serial.print ( "R: " ); Serial.print ( spd ); Serial.println ( "%" );
      right.speed ( spd );
      sc ( 7, YELLOW );

      state_started_at_ms = millis();
      state_duration_ms = MOTOR_RUN_TIME_ms;
      abortOnButtonRelease = 1;
      justEnteredP = 0;
    }

    // showSpeed ( 5, left.speed () );
    showSpeed ( 6, right.speed () );  // TESTING

    if ( 0 && ( millis() - state_started_at_ms ) > state_duration_ms ) {  // TESTING
      // transition to next state
      state = 1;
    }
   
  } else if ( state == 1 && ( millis() - state_started_at_ms ) > state_duration_ms ) {
    
    static int justEnteredP = 1;
  
    // ========================================= test L STOP

    if ( justEnteredP ) {
      right.speed ( 0 );
      sc ( 7, BLUE );
    
      state_started_at_ms = millis();
      state_duration_ms = MOTOR_RUN_TIME_ms;
      abortOnButtonRelease = 1;
      justEnteredP = 0;
    }

    // showSpeed ( 5, left.speed () );
    showSpeed ( 6, right.speed () );
   
    if ( ( millis() - state_started_at_ms ) > state_duration_ms ) {
      // transition to next state
      state = 1000; // 2;                                              TESTING
    }
  
  } else if ( state == 2 ) {
    static int justEnteredP = 1;
  
    // ========================================= test L REV

    if ( justEnteredP ) {
      right.speed ( 100, DIR_REV );
      sc ( 7, MAGENTA );
  
      state_started_at_ms = millis();
      state_duration_ms = MOTOR_RUN_TIME_ms;
      abortOnButtonRelease = 1;
      justEnteredP = 0;
    }
    
    // showSpeed ( 5, left.speed () );
    showSpeed ( 6, right.speed () );
   
    if ( ( millis() - state_started_at_ms ) > state_duration_ms ) {
      // transition to next state
      state = 3;
    }
        
  } else if ( state == 3 && ( millis() - state_started_at_ms ) > state_duration_ms ) {
    
    static int justEnteredP = 1;
  
    // ========================================= test L STOP

    if ( justEnteredP ) {
      right.speed ( 0 );
      sc ( 7, BLUE );
    
      state_started_at_ms = millis();
      state_duration_ms = MOTOR_RUN_TIME_ms;
      abortOnButtonRelease = 1;
      justEnteredP = 0;
    }

    // showSpeed ( 5, left.speed () );
    showSpeed ( 6, right.speed () );
   
    if ( ( millis() - state_started_at_ms ) > state_duration_ms ) {
      // transition to next state
      state = 10;
    }
  
  } else if ( state == 10 && ( millis() - state_started_at_ms ) > state_duration_ms ) {
    static int justEnteredP = 1;

    // ========================================= test R FWD
      
    if ( justEnteredP ) {
      sc ( 1, GREEN );
      
      right.speed ( 100, DIR_FWD );
      sc ( 7, YELLOW );
      
      state_started_at_ms = millis();
      state_duration_ms = MOTOR_RUN_TIME_ms;
      abortOnButtonRelease = 1;
      justEnteredP = 0;
    }
    
    // showSpeed ( 5, left.speed () );
    showSpeed ( 6, right.speed () );
   
    if ( ( millis() - state_started_at_ms ) > state_duration_ms ) {
      // transition to next state
      state = 11;
    }
  
  } else if ( state == 11 && ( millis() - state_started_at_ms ) > state_duration_ms ) {
    static int justEnteredP = 1;
    
    // ========================================= test R STOP

    if ( justEnteredP ) {
      right.speed ( 0 );
      sc ( 7, BLUE );
  
      state_started_at_ms = millis();
      state_duration_ms = 1000;
      abortOnButtonRelease = 1;
      justEnteredP = 0;
    }
    
    // showSpeed ( 5, left.speed () );
    showSpeed ( 6, right.speed () );
   
    if ( ( millis() - state_started_at_ms ) > state_duration_ms ) {
      state = 12;
    }
  
  } else if ( state == 12 && ( millis() - state_started_at_ms ) > state_duration_ms ) {
    static int justEnteredP = 1;

    // ========================================= test R REV

    if ( justEnteredP ) {
      right.speed ( 20, DIR_REV );
    
      sc ( 7, MAGENTA );
      
      state_started_at_ms = millis();
      state_duration_ms = MOTOR_RUN_TIME_ms;
      abortOnButtonRelease = 1;
      justEnteredP = 0;
    }
    
    // showSpeed ( 5, left.speed () );
    showSpeed ( 6, right.speed () );
   
    if ( ( millis() - state_started_at_ms ) > state_duration_ms ) {
      // transition to next state
      state = 13;
    }
  
  } else if ( state == 13 && ( millis() - state_started_at_ms ) > state_duration_ms ) {
    static int justEnteredP = 1;
    
    // ========================================= test R STOP

    if ( justEnteredP ) {
      right.speed ( 0 );
      sc ( 7, BLUE );
  
      state_started_at_ms = millis();
      state_duration_ms = 1000;
      abortOnButtonRelease = 0;
      justEnteredP = 0;
    }
    
    showSpeed ( 5, left.speed () );
    showSpeed ( 6, right.speed () );
   
    // wait for timeout and then release of button
    if ( ( ( millis() - state_started_at_ms ) > state_duration_ms ) && ! enableButtonState ) {
      state = 20;
    }
  
  } else if ( state == 20 ) {
    static int justEnteredP = 1;

    if ( justEnteredP ) {
      
      while ( ! ( enableButtonState = checkButtonState ( pd_ENABLE_BUTTON ) ) ) delay ( 10 );
      
      // ========================================= stop motion
      disable_movement ();
      sc ( 0, BLACK );
      sc ( 7, BLACK );
  
      // ========================================= test pneumatics UP

      sc ( 1, YELLOW );
      sc ( 7, YELLOW );
      
      Serial.println ( "pneumatics UP" );

      digitalWrite ( pd_TABLE_DIR_UP, 1 );
      digitalWrite ( pd_TABLE_EN, 1 );

      state_started_at_ms = millis();
      state_duration_ms = 1000;
      abortOnButtonRelease = 0;
      justEnteredP = 0;
    }
    
    // Serial.print ( millis() - state_started_at_ms );
    // Serial.print ( " <> " );
    // Serial.print ( state_duration_ms );
    // Serial.print ( " && ! " );
    // Serial.println ( enableButtonState );
    
    // wait for timeout and then release of button
    if ( ( ( millis() - state_started_at_ms ) > state_duration_ms ) && ! enableButtonState ) {
      state = 21;
    }
      
  } else if ( state == 21 ) {
    static int justEnteredP = 1;
  
    if ( justEnteredP ) {
      
      while ( ! ( enableButtonState = checkButtonState ( pd_ENABLE_BUTTON ) ) ) delay ( 10 );

      // ========================================= test pneumatics OFF

      sc ( 7, BLUE );
   
      Serial.println ( "pneumatics OFF" );

      digitalWrite ( pd_TABLE_EN, 0 );
    
      state_started_at_ms = millis();
      state_duration_ms = 0;
      abortOnButtonRelease = 0;
      justEnteredP = 0;
    }
    
    // wait for timeout and then release of button
    if ( ( ( millis() - state_started_at_ms ) > state_duration_ms ) && ! enableButtonState ) {
      state = 22;
    }
    
  } else if ( state == 22 ) {
    static int justEnteredP = 1;
  
    if ( justEnteredP ) {
      
      while ( ! ( enableButtonState = checkButtonState ( pd_ENABLE_BUTTON ) ) ) delay ( 10 );

      // ========================================= test pneumatics DOWN
  
      sc ( 7, MAGENTA );
      
      Serial.println ( "pneumatics DOWN" );

      digitalWrite ( pd_TABLE_DIR_UP, 0 );
      digitalWrite ( pd_TABLE_EN, 1 );
    
      state_started_at_ms = millis();
      state_duration_ms = 0;
      abortOnButtonRelease = 0;
      justEnteredP = 0;
    }
    
    // wait for timeout and then release of button
    if ( ( ( millis() - state_started_at_ms ) > state_duration_ms ) && ! enableButtonState ) {\
      state = 23;
    }
    
  } else if ( state == 23 ) {
    static int justEnteredP = 1;
  
    if ( justEnteredP ) {
      
      while ( ! ( enableButtonState = checkButtonState ( pd_ENABLE_BUTTON ) ) ) delay ( 10 );

      // ========================================= test pneumatics OFF

      sc ( 7, BLUE );
     
      digitalWrite ( pd_TABLE_EN, 0 );
    
      state_started_at_ms = millis();
      state_duration_ms = 0;
      abortOnButtonRelease = 0;
      justEnteredP = 0;
    }
    
    // wait for timeout and then release of button
    if ( ( ( millis() - state_started_at_ms ) > state_duration_ms ) && ! enableButtonState ) {\
      state = 1000;
    }
    
  } else if ( state == 1000 && ( millis() - state_started_at_ms ) > state_duration_ms ) {
    static int justEnteredP = 1;
  
    if ( justEnteredP ) {
      
      // ========================================= hang loose for 10 seconds
    
      for ( int i = 0; i < nPixels; i++ ) {
        sc ( i, GREEN );
      }
      state_started_at_ms = millis();
      state_duration_ms = 10000;
      abortOnButtonRelease = 0;
      justEnteredP = 0;
    }

    // wait for timeout
    if ( ( ( millis() - state_started_at_ms ) > state_duration_ms ) ) {\
      state = 9000;
    }
    
  } else if ( state == 9000 && ( millis() - state_started_at_ms ) > state_duration_ms ) {
    
    // ========================================= ALL DONE

    // turn off pixels, clean up, go home
    
    for ( int i = 0; i < nPixels; i++ ) {
      sc ( i, BLACK );
    }
    
    while ( 1 ) { delay ( 10 ); }
  
  }  // state selector
  
  tick ();
  
  delay ( 50 );
  
}


void tick () {
  static int printed = 5;
  
  if ( ( !enableButtonState ) && abortOnButtonRelease ) {
    disable_movement ();
    for ( int i = 0; i < 4; i++ ) {
      sc ( i , RED );
    }
    state = 1001;
    state_started_at_ms = millis();
    state_duration_ms = 0;
    Serial.println ( "\n\n\n\nABORT ABORT ABORT\n\n\n" );
    delay ( 2000 );
  }

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

void showSpeed ( int n, float speed ) {
  // show a color from red (stopped) to green (full speed)
  neoPixelStrip.setPixelColor( n, neoPixelStrip.Color( 255 - speed * 255 / 100, speed * 255 / 100, 0 ) );
  neoPixelStrip.show();
}
  
int checkButtonState ( int buttonPin ) {
  // will delay for at least inspectionTime_ms milliseconds iff button state has changed
  static const unsigned long inspectionTime_ms = 50;
  static unsigned long lastChangeAt_ms = millis();
  // look for the button being in a steady state for at least the specified inspection time
  // buttonState is global, 1 for closed (logic low) and 0 for open (logic high)
  int buttonLevel = 1 - enableButtonState;
  int newButtonLevel = digitalRead ( buttonPin );
  if ( newButtonLevel != buttonLevel ) {
    // button has changed state at some point
    // wait for it to stabilize
    lastChangeAt_ms = millis();
    while ( ( millis() - lastChangeAt_ms ) < inspectionTime_ms ) {
      newButtonLevel = digitalRead ( buttonPin );
      if ( newButtonLevel != buttonLevel ) {
        lastChangeAt_ms = millis();
        buttonLevel = newButtonLevel;
      }
    }
  }
  // Serial.println ( buttonLevel ? "open" : "closed" );
  return ( 1 - buttonLevel );
}
