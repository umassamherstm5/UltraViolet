// 1.7.2014
// UV Wii Client

#include "Wire.h"
#include "UV_Client.h"
#include "UVWiiClassic.h"

#define ENTRY_STATE Wait
#define MSG_SIZE 3
#define WAIT_DELAY 50
#define PERIODIC_RATE 100

WiiClassic controller = WiiClassic();
char dataIn[MSG_SIZE];
char dataOut[MSG_SIZE];
unsigned int periodicCounter;

void setup(){
  setPowerPins();
  controller.begin();
  Serial.begin(19200);
  periodicCounter = 0;
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
  for(i=0; i<MAX_CLIENT_TRANSITIONS; i++){
    if((*curTrans).src_state == state_transitions[i].src_state &&
        (*curTrans).ret_code == state_transitions[i].ret_code){
      return state_transitions[i].dst_state;
    }
  }

  printf("Error (lookup_transition): No Proper Destination State!\n");
  exit(1);
}

int Wait_state(void){
  /*
      Steps:
        (1) Sit
        (2) If Serial.available() at/before t ms, return ok
        (3) If !Serial.available() after t ms, return nok
  */ 
  // Hard delay
  delay(WAIT_DELAY);  
  
  // OK - Data in
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
    
    return (int)nok;
}

int Process_state(void){
  /*
    Steps:
      (1) Process checksum
      (2) Checksum == bad, return nok
      (3) Checksum == ok, return ok
  */
  unsigned int msg_type = (unsigned int)dataIn[0] - '0';
  unsigned int data = (unsigned int)dataIn[1] - '0';
  unsigned int cksum = (unsigned int)dataIn[2] - '0';

  if((msg_type<<1) & data == cksum){
    switch(msg_type){
    case ACK:
      // do nothing
      dataIn[0] = '\0';
      dataIn[1] = '\0';
      dataIn[2] = '\0';
      return (int)ok;
      
    case NACK:
      // do nothing
      dataIn[0] = '\0';
      dataIn[1] = '\0';
      dataIn[2] = '\0';      
      return (int)nok;
      
    case VOLTAGE:
      // NEED TO IMPLEMENT VOLTAGE DISPLAY
      dataIn[0] = '\0';
      dataIn[1] = '\0';
      dataIn[2] = '\0';
      return (int)ok;
      
    default:
      return (int)nok;
    }
  }
}

int Send_state(void){
  /*
    Steps:
      (1) Pack stick & controller data
      (2) Decide to ask to periodic
      (3) Send command, return ok
  */
  packControllerData();
  
  periodicCounter++;
  
  // Set Periodic
  if(periodicCounter == PERIODIC_RATE){
    periodicCounter = 0;
    dataOut[1] |= VOLTAGE;
    
    //sendData();
    return (int)ok;
  }
  
  // Set no periodic
  dataOut[1] |= NO_PERIODIC;
  
  // Set Checksum
  dataOut[2] = (dataOut[0] << 1) & dataOut[1];
  
  // Send data and return
  sendData();
  return (int)ok;
}

void packControllerData(void){
  /*
    B0, bits 0-4 Left Stick X
    B1, bits 5-7 Left Stick Y (lower)
    B1, bits 0-1, Left Stick Y (upper)
    B1, bits2-5, Buttons (ONLY ONE PER MSG)
    B1, bits6-7, Status Return Indicator
  */
  controller.update();
  
  unsigned int leftStickX = controller.leftStickX();
  unsigned int leftStickY = controller.leftStickY();
  
  dataOut[0] = leftStickX << 2;
  dataOut[0] |= (leftStickY & 0x30) >> 4;
  dataOut[1] = (leftStickY & 0x0F) << 4;
  
  unsigned int button = 0;
  
  // Prioritize Buttons, if one button has been pressed, then return
  if((button = controller.leftZTrigger()) == 1){
    dataOut[1] |= ZL;
    return;
  }
  if((button = controller.rightZTrigger()) == 1){
    dataOut[1] |= ZR;
    return;
  }
  if((button = controller.buttonUp()) == 1){
    dataOut[1] |= Up;
    return;
  }
  if((button = controller.buttonDown()) == 1){
    dataOut[1] |= Down;
    return;
  }
  if((button = controller.buttonRight()) == 1){
    dataOut[1] |= Right;
    return;
  }
  if((button = controller.buttonLeft()) == 1){
    dataOut[1] |= Left;
    return;
  }
  if((button = controller.rightTrigger()) == 1){
    dataOut[1] |= RightTrigger;
    return;
  }
  if((button = controller.leftTrigger()) == 1){
    dataOut[1] |= LeftTrigger;
    return;
  }
  if((button = controller.buttonA()) == 1){
    dataOut[1] |= A;
    return;
  }
  if((button = controller.buttonB()) == 1){
    dataOut[1] |= B;
    return;
  }
  if((button = controller.buttonX()) == 1){
    dataOut[1] |= X;
    return;
  }
  if((button = controller.buttonY()) == 1){
    dataOut[1] |= Y;
    return;
  }
  if((button = controller.buttonHome()) == 1){
    dataOut[1] |= Home;
    return;
  }
  if((button = controller.buttonMinus()) == 1){
    dataOut[1] |= Minus;
    return;
  }
  if((button = controller.buttonPlus()) == 1){  
    dataOut[1] |= Plus;
    return;
  }  
}  
  
void sendData(void){
  Serial.print(dataOut[0]);
  Serial.print(dataOut[1]);
  Serial.print(dataOut[2]);
}  
