/* 12.29.2013
 * Based on: http://playground.arduino.cc//Main/WiiClassicController
 * 2009/10/04 Modified Micono Utilities.(http://micono.cocolog-nifty.com)
 * http://www.instructables.com/id/USB-Wii-Classic-Controller/step9/Reading-the-Wii-Classic-Controller/
 *
 *
 * Ported to work on Arduino V1.x
 */
 
#include "Arduino.h"

#ifndef UVWiiClassic_h
#define UVWiiClassic_h

static void setPowerPins(void){
	// Set power and ground pins for I2C
	pinMode(A2, OUTPUT);
	pinMode(A3, OUTPUT);
	digitalWrite(A2, LOW);
	digitalWrite(A3, HIGH);
	
	// Wait for things to stabilize
	delay(500);
	
	return;
}

class WiiClassic{
	private:
		unsigned int data[6];
		int counter;
		
		
        byte _decode_byte (byte x){
            x = (x ^ 0x17) + 0x17;
            return x;
        }
		
		void _send_zero(){
            Wire.beginTransmission(0x52);	// transmit to device 0x52
            Wire.write(0x00);		// sends one byte
            Wire.endTransmission();	// stop transmitting
        }		
	
	public:
		void begin(void){
			Wire.begin();
			Wire.beginTransmission(0x52); // Transmit to device 0x52
			Wire.write(0x40); // Sends memory address
			Wire.write(0x00); // Sends memory address
			Wire.endTransmission(); // Stop transmitting
			
			update();
		}
		
		void update(void){
			// Request 6 bytes from controller at address 0x52
			Wire.requestFrom(0x52, 6);
			
			// Reset counter
			counter = 0;
			
			while(Wire.available()){
				data[counter++] = _decode_byte(Wire.read());
				
				
				if(counter == 6){
					_send_zero(); // Kill connection
					counter = 0;
				}
			}
		}
		
		unsigned int leftStickX(void){
			// Byte 0, bits 0-5
			return (data[0] & 0x3F);
		}
		
		unsigned int leftStickY(void){
			// Byte 1, bits 0-5
			return (data[1] & 0x3F);
		}
		
		unsigned int rightStickX(void){
		   /* Byte 2, bit 7
			* Byte 1, bits 6-7
			* Byte 0, bits 6-7
			*/
			return (((data[2] & 0x80) >> 7) | 
					((data[1] & 0xC0) >> 5) |
					((data[0] & 0xC0) >> 3));
		}
		
		unsigned int rightStickY(void){
			// Byte 2, bits 0-4
			return (data[2] & 0x1F);
		}
		
		unsigned int leftZTrigger(void){
			/* Byte 3, bits 5-7
			 * Byte 2, bits 5-6
			 */
			if((((data[3] & 0xE0) >> 5) |
					 ((data[2] & 0x60) >> 2)) > 0){
			
				return 1;
			}
			else{
				return 0;
			}
		}
		
		unsigned int rightZTrigger(void){
			// Byte 3, bits 0-4
			if((data[3] & 0x1F) > 0){
				return 1;
			}
			else{
				return 0;
			}
		}
		
		unsigned int buttonUp(void){
			// Byte 5, bit 0
			return !(data[5] & 0x01);
		}
		
		unsigned int buttonDown(void){
			// Byte 4, bit 6
			return !((data[4] & 0x40) >> 6);
		}
		
		unsigned int buttonRight(void){
			// Byte 4, bit 7
			return !((data[4] & 0x80) >> 7);
		}
		
		unsigned int buttonLeft(void){
			// Byte 5, bit 1
			return !((data[5] & 0x2) >> 1);
		}
		
		unsigned int leftTrigger(void){
			// Byte 4, bit 5
			return !((data[4] & 0x20) >> 5);
		}
		
		unsigned int rightTrigger(void){
			// Byte 4, bit 1
			return !((data[4] & 0x2) >> 1);
		}
		
		unsigned int buttonA(void){
			// Byte 5, bit 4
			return !((data[5] & 0x10) >> 4);
		}
		
		unsigned int buttonB(void){
			// Byte 5, bit 6
			return !((data[5] & 0x40) >> 6);
		}
		
		unsigned int buttonX(void){
			// Byte 5, bit 3
			return !((data[5] & 0x8) >> 3);
		}
		
		unsigned int buttonY(void){
			// Byte 5, bit 5
			return !((data[5] & 0x20) >> 5);
		}
		
		unsigned int buttonHome(void){
			// Byte 4, bit 3
			return !((data[4] & 0x8) >> 3);
		}
		
		unsigned int buttonMinus(void){
			// Byte 4, bit 4
			return !((data[4] & 0x10) >> 4);
		}
		
		unsigned int buttonPlus(void){
			// Byte 4, bit 2
			return !((data[4] & 0x4) >> 2);
		}
};

#endif












