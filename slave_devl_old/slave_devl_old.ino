#include <Adafruit_NeoPixel.h>

#define BAUDRATE 115200

// Pins
#define pd_MOTOR_ENABLE 4
#define pd_L_FWD 5
#define pd_L_REV 6
#define pd_R_FWD 9
#define pd_R_REV 10
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

#define LEFT 1
#define RIGHT 0
#define FWD 1
#define REV 0

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

void setDir ( int LR, int FR, int pct = 50 ) {
  switch LR {
    case LEFT:
      switch FR {
        case FWD:
          digitalWrite ( pd_L_REV, 0 );  // make sure not FWD and REV simultaneously
          analogWrite ( pd_L_FWD, pct * 255 / 50 );
          break;
        case REV:
          digitalWrite ( pd_L_FWD, 0 );  // make sure not FWD and REV simultaneously
          analogWrite ( pd_L_REV, pct * 255 / 50 );
          break;
      }
    case RIGHT:
      switch FR {
        case FWD:
          digitalWrite ( pd_R_REV, 0 );  // make sure not FWD and REV simultaneously
          analogWrite ( pd_R_FWD, pct * 255 / 50 );
          break;
        case REV:
          digitalWrite ( pd_R_FWD, 0 );  // make sure not FWD and REV simultaneously
          analogWrite ( pd_R_REV, pct * 255 / 50 );
          break;
      }
  }
          
void setup() {
  Serial.begin ( BAUDRATE );
  
  pinMode ( pd_MOTOR_ENABLE, OUTPUT );
  digitalWrite ( pd_MOTOR_ENABLE, 0 );

  pinMode ( pd_L_REV, OUTPUT );
  pinMode ( pd_L_FWD, OUTPUT );
  pinMode ( pd_R_REV, OUTPUT );
  pinMode ( pd_R_FWD, OUTPUT );
  digitalWrite ( pd_L_REV, 0 );
  digitalWrite ( pd_L_FWD, 0 );
  digitalWrite ( pd_R_REV, 0 );
  digitalWrite ( pd_R_FWD, 0 );

  neoPixelStrip.begin();
  neoPixelStrip.show(); // Initialize all pixels to 'off'
  
  sc ( 0, GREEN );
  
  // ========================================= test L
  
  sc ( 1, YELLOW );
  
  setDir ( LEFT, FWD )
  sc ( 7, MAGENTA );
  
  delay ( 1000 );
  
  // enable H-bridges
  digitalWrite ( pd_MOTOR_ENABLE, 1 );
  sc ( 7, YELLOW );
  delay ( 5000 );
  
  digitalWrite ( pd_MOTOR_ENABLE, 0 );
  
  setDir ( LEFT, REV );
  sc ( 7, MAGENTA );
  
  delay ( 1000 );
  
  // enable H-bridges
  digitalWrite ( pd_MOTOR_ENABLE, 1 );
  sc ( 7, YELLOW );
  delay ( 5000 );
  
  digitalWrite ( pd_MOTOR_ENABLE, 0 );
  setDir ( LEFT, FWD, 0 );
  
  sc ( 7, BLACK );
  sc ( 1, GREEN );
  
  // ========================================= test R
  
  sc ( 2, YELLOW );

  setDir ( RIGHT, FWD );
  sc ( 7, MAGENTA );
  
  delay ( 2000 );
  
  // enable H-bridges
  
  pinMode ( pd_MOTOR_ENABLE, OUTPUT );
  digitalWrite ( pd_MOTOR_ENABLE, 1 );
  sc ( 7, YELLOW );
  delay ( 5000 );
  
  digitalWrite ( pd_MOTOR_ENABLE, 0 );
  
  setDir ( RIGHT, REV );
  sc ( 7, MAGENTA );
  
  delay ( 1000 );
  
  // enable H-bridges
  digitalWrite ( pd_MOTOR_ENABLE, 1 );
  sc ( 7, YELLOW );
  delay ( 5000 );
  
  digitalWrite ( pd_MOTOR_ENABLE, 0 );
  
  setDir ( RIGHT, FWD, 0 );

  sc ( 7, BLACK );
  sc ( 2, GREEN );
  
  delay ( 5000 );
  
  // turn off pixels, clean up, go home
  for ( int i = 0; i < nPixels; i++ ) {
    sc ( i, BLACK );
  }
  
}

void loop() {
}

