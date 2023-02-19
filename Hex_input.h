/*
 * Hex_input.h
 *
 *  Created on: Feb 11, 2023
 *      Author: baotriet
 */

#ifndef HEX_INPUT_H_
#define HEX_INPUT_H_

#define BUTTON_PRESSED(stat, mask) (stat & (mask))

// Byte 0 Checksum
// Byte 1
#define INPUT_TOGGLE_ON_OFF 		0X1
#define INPUT_TOGGLE_MODE 	 		0x2
#define INPUT_TOGGLE_BALANCE 		0X4
#define INPUT_TOGGLE_BODY_HEIGHT	0x8
#define INPUT_BODY_UP				0x10
#define INPUT_BODY_DOWN				0x20	
#define INPUT_SPEED_UP				0x40
#define INPUT_SPEED_DOWN			0x80
// Byte 2
#define INPUT_GRIP_OPEN				0x100
#define INPUT_GRIP_CLOSE			0x200
#define INPUT_OPT_SEL				0x400
#define	INPUT_OPT_R1				0x800
#define INPUT_OPT_R2				0x1000
#define INPUT_BODY_BALANCE          0x2000

// Byte 3 Lx
// Byte 4 Ly
// Byte 5 Rx
// Byte 6 Ry

#define CONFIG_TRAVEL_DEAD_ZONE 4      //The deadzone for the analog input from the remote

#endif /* HEX_INPUT_H_ */
