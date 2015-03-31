// 1.10.14
// UV Server

#include "UV_Server.h"

#define ENTRY_STATE Wait
#define WAIT_DELAY 10
#define MSG_SIZE 3
#define PWM_CONV 20 //7.96875
#define BAD_DATA 0xFF

// Pins
#define PIN_L_FORWARD 5
#define PIN_L_BACKWARD 6
#define PIN_R_FORWARD 9
#define PIN_R_BACKWARD 10
#define PIN_TABLE_UP 7
#define PIN_TABLE_DOWN 8

unsigned char dataIn[MSG_SIZE];
unsigned char dataOut[MSG_SIZE];
int currentLeft, currentRight;

void setup(){
  Serial.begin(19200);
  pinMode(PIN_L_FORWARD, OUTPUT);
  pinMode(PIN_L_BACKWARD, OUTPUT);
  pinMode(PIN_R_FORWARD, OUTPUT);
  pinMode(PIN_R_BACKWARD, OUTPUT);
  
  // Start motors at stop
  currentLeft = 0;
  currentRight = 0;
}

void loop(){
  enum state_codes cur_state = ENTRY_STATE;
  enum return_codes rc;
  int (*state_fun)(void);
  struct transition passedStruct;
  
  while(1){
    // Pull pointer for state function
    state_fun = state[cur_state];

    // Run state, get return code
    rc = (return_codes)state_fun();
    
    // Pack struct to send to transition lookup
    passedStruct.src_state = cur_state;
    passedStruct.ret_code = rc;
    
    // Lookup next state's return code
    cur_state = (state_codes)lookup_transition(&passedStruct);
  }
}

int lookup_transition(struct transition *curTrans){
  int i;
  
  // If src and ret code match, return dst
  for(i=0; i<MAX_SERVER_TRANSITIONS; i++){
    if((*curTrans).src_state == state_transitions[i].src_state &&
        (*curTrans).ret_code == state_transitions[i].ret_code){
      return state_transitions[i].dst_state;
    }
  }

  Serial.println("Error (lookup_transition): No Proper Destination State!\n");
  exit(1);
}

int Send_state(void){
  /*
    Steps:
      (1) Send dataOut buffer if data isn't bad
  */
  
  if(dataOut[0] == BAD_DATA){
    return (int)ok;
  }
  
  Serial.print(dataOut[0]);
  Serial.print(dataOut[1]);
  Serial.print(dataOut[2]);
  
  return (int)nok;
}

int Wait_state(void){
  /*
    Steps:
      (1) Sit
      (2) If data in, pack data and return ok
      (3) If bad data in/timeout, dump buffer and set bad data flag
  */
  delay(WAIT_DELAY);
  
  // Ok -- Data in 
  if(Serial.available() == MSG_SIZE){
    // Pack data    
    dataIn[0] = Serial.read();
    dataIn[1] = Serial.read();
    dataIn[2] = Serial.read();
    
    return (int)ok;
  }
  
  // NOk - Bad data or timeout
  // Clear data
  while(Serial.available()){
    char dumpMe = Serial.read();
    delay(10);
  }

  // Bad data indication for ProcessData
  dataIn[0] = BAD_DATA;
  dataIn[1] = BAD_DATA;
  
  return (int)nok;
}

int Process_state(void){
  /*
    Steps:
      (1) Check for bad data / process checksum
      (2) Checksum == bad, return nok
      (3) Checksum == good, apply 
  */
  
  // Bad data / timeout from Wait State OR bad cksum
  unsigned char check = (dataIn[0]<<1) & dataIn[1];
  
  if((dataIn[0] == BAD_DATA && dataIn[1] == BAD_DATA) ||
      check != dataIn[2]){
    runFuzzyMotors(31, 31);
    return (int)nok;
  }

  int x = (dataIn[0] & 0xFC) >> 2;
  int y = (dataIn[1] & 0xF0) >> 4;
  y |=(dataIn[0] & 0x3) << 4;
  int button = dataIn[1] & 0xF;
  
  // Run motors
  runFuzzyMotors(x, y);
  
  // Check for voltage return
  if(button == 0xF){
    // NEED TO IMPLEMENT VOLTAGE CHECK HERE
  }
  else{
    // NEED TO IMPLEMENT BUTTON COMMMAND... 
    // ALSO INDICATES VOLTAGE RETURN
    // buttonCommand(button);
    dataOut[1] = NO_PERIODIC;
  }
  
  dataOut[0] = ACK;
  dataOut[2] = (dataOut[0] << 1) & dataOut[1];
  return (int)ok;
}

void runFuzzyMotors(int L_X, int L_Y){
  // Translated value
  int T_X = 0;
  int T_Y = 0;
  
  returnTranslated(&T_X, &T_Y, L_X, L_Y);

  // Find Effective Value
  int E_L = T_Y + (T_X/2);
  int E_R = T_Y - (T_X/2);
   
  // Find Diff values
  int D_L = E_L - currentLeft;
  int D_R = E_R - currentRight;
  
  // Get Delta Values
  int DELTA_L = returnFuzzyChange(D_L);
  int DELTA_R = returnFuzzyChange(D_R);
    
  // Sum motor speeds with deltas
  currentLeft += DELTA_L;
  currentRight += DELTA_R;
  
  // Clamp within valid range
  currentLeft = constrain(currentLeft, -255, 255);
  currentRight = constrain(currentRight, -255, 255);
  
  // Pass currentRight and currentLeft to motorHandler
  motorHandler();
  
  delay(50);
  
  return;
}

void motorHandler(void){
  // Zero out
  if(currentLeft >= -10 && currentLeft <= 10){
    analogWrite(PIN_L_FORWARD, 0);
    analogWrite(PIN_L_BACKWARD, 0);
  } // Forward
  else if(currentLeft > 10){
    analogWrite(PIN_L_FORWARD, currentLeft);
    analogWrite(PIN_L_BACKWARD, 0);
  } // Backward
  else{
    analogWrite(PIN_L_FORWARD, 0);
    analogWrite(PIN_L_BACKWARD, abs(currentLeft));
  }
  
  if(currentRight >= -10 && currentRight <= 10){
    analogWrite(PIN_R_FORWARD, 0);
    analogWrite(PIN_R_BACKWARD, 0);
  }
  else if(currentRight > 10){
    analogWrite(PIN_R_FORWARD, currentRight);
    analogWrite(PIN_R_BACKWARD, 0);
  }
  else{
    analogWrite(PIN_R_FORWARD, 0);
    analogWrite(PIN_R_BACKWARD, abs(currentRight));
  }
  
  return;
}

void returnTranslated(int *T_X, int *T_Y, int L_X, int L_Y){
  if(L_X <= 31){
    *T_X = (L_X - 31) * PWM_CONV;
  }
  else{
    *T_X = (L_X - 31) * PWM_CONV;
  }
  
  if(L_Y <= 31){
    *T_Y = (L_Y - 31) * PWM_CONV;
  }
  else{
    *T_Y = (L_Y - 31) * PWM_CONV;
  }

  return;
}

int returnFuzzyChange(int diff){
  /*
      Thresholds:
        (1) +-20 PWM for difference >= 150
        (2) +-15 PWM for difference >= 100
        (3) +-10 PWM for difference >= 50
        (5) +- 5 PWM for difference < 50
  */
  int returnValue;
  
  if(abs(diff) >= 150){
    returnValue = 20;
  }
  else if(abs(diff) >= 100){
    returnValue = 15;
  }
  else if(abs(diff) >= 50){
    returnValue = 10;
  }
  else{
    returnValue = 5;
  }
  
  // If positive value, just return
  if(diff >= 0){
    return returnValue;
  }
  else // If negative value, translate to negative
  return (0 - returnValue);
}
