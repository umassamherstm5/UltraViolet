// UV Client Header
// 1.10.2014

#ifndef CLIENT_H
#define CLIENT_H

#define MAX_CLIENT_TRANSITIONS 5

int Process_state(void);
int Send_state(void);
int Wait_state(void);
void packControllerData(void);
void sendData(void);

// State indicators
int (*state[])(void) = { Process_state, Send_state, Wait_state };

// State indicators
enum state_codes { Process, Send, Wait };

// Possible return codes
enum return_codes { ok, nok };

// Used by transition table
struct transition {
	enum state_codes src_state;
	enum return_codes ret_code;
	enum state_codes dst_state;
};

// Transition Table
struct transition state_transitions[] = {
	{ Process,		ok,			Send	},
	{ Process,		nok,		Send	},
	{ Send,			ok,			Wait	},
	{ Wait,			ok,			Process },
	{ Wait,			nok,		Send }
};

int lookup_transition(struct transition *currentTransition);

enum MsgOut { NACK = 0x15, ACK = 0x06 };
enum MsgIn  { VOLTAGE = 0x1, NO_PERIODIC = 0x0};

enum ButtonMap {  ZL            =  0x1,
                  ZR            =  0x2,
                  LeftTrigger   =  0x3,
                  RightTrigger  =  0x4,
                  Left          =  0x5,
                  Right         =  0x6,
                  Up            =  0x7,
                  Down          =  0x8,
                  A             =  0x9,
                  B             =  0x10,
                  X             =  0xA,
                  Y             =  0xB,
                  Plus          =  0xC,
                  Minus         =  0xD,
                  Home          =  0xE
};

#endif
