// UV Server HEader
// 1.10.14

// UV Server Header
// 12.31.13

#ifndef SERVER_H
#define SERVER_H

#define MAX_SERVER_TRANSITIONS 6

int Wait_state(void);
int Process_state(void);
int Send_state(void);
void runFuzzyMotors(int L_X, int L_Y);
int returnFuzzyChange(int diff);
void returnTranslated(int *T_X, int *T_Y, int L_X, int L_Y);
void motorHandler(void);

// State indicators
int (*state[])(void) = { Process_state, Wait_state, Send_state };

// State indicators
enum state_codes { Process, Wait, Send };

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
	{ Process,		ok,		Send		},
	{ Process,		nok,	Send		},
	{ Wait,			nok,	Process		},
	{ Wait,			ok,		Process		},
	{ Send, 		nok,	Wait		},
	{ Send,			ok,		Wait		}
	
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
