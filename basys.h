/**
 * I/O Definitions for the Basys AVR
 *
 * This file defines each of the "variables" used to
 * interact with the I/O on the Basys board.
 *
 * Each of the "variables" is really a direct reference
 * to a the special function register at the address ("register number")
 * noted below.
 */


#pragma once

#ifndef __BASYS_H__
#define __BASYS_H__

//Ensure we're working with an ATmega103
#ifndef __AVR_ATmega103__
#define __AVR_ATmega103__
#endif

#include <avr/io.h>


/** 
 * Create easy names for each of the pin numbers in the left ports.
 */ 
#define LEFT_MOTOR_DIRECTION    7
#define LEFT_MOTOR_ENABLE       6
#define LEFT_MOTOR_A            5
#define LEFT_MOTOR_B            4
#define RIGHT_MOTOR_DIRECTION   3
#define RIGHT_MOTOR_ENABLE      2
#define RIGHT_MOTOR_A           1
#define RIGHT_MOTOR_B           0


/**
 * Create meaningful shortcuts to the special function registers of each of the ports and pins.
 */
#define LEDS           _SFR_IO8(0x1B)
#define LEFT_PORT_OUT  _SFR_IO8(0x18)
#define LEFT_PORT_IN   _SFR_IO8(0x16)
#define SWITCHES       _SFR_IO8(0x10)
#define SSEG_LEFT      _SFR_IO8(0x03)
#define SSEG_RIGHT     _SFR_IO8(0x07) 
#define BUTTONS        _SFR_IO8(0x13)

/**
 * Sets up all of the Basys board's I/O for use with the robot.
 *
 * Note that while function definitions are normally placed in .c files; 
 * this function is different. You can read more about inline functions at
 * http://en.wikipedia.org/wiki/Inline_function.
 */ 
static inline void set_up_basys_io() {
  DDRA = 0xFF;
  DDRB = ~(1 << LEFT_MOTOR_A | 1 << LEFT_MOTOR_B | 1 << RIGHT_MOTOR_A  | 1 << RIGHT_MOTOR_B);
  _SFR_IO8(0x14) = 0x00;
  DDRD = 0x00;
  DDRE = 0xFF;
  _SFR_IO8(0x08) = 0xFF;
}

#endif
