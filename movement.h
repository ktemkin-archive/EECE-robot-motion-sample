/**
 * EECE 252 Robot Movement Library
 * Example Code for Project 1
 */

#ifndef __MOVEMENT_H__
#define __MOVEMENT_H__

#include "basys.h"

/**
 * Structure which stores a cartesian point.
 * Typcially stores two displacements in inches.
 */ 
typedef struct {
  char x;
  char y;
} cartesian_point;

/**
 * Structure which stores a polar point.
 * Typically stores an angle in degress and a distance in inches.
 */
typedef struct {
  unsigned int angle;
  unsigned char distance;
} polar_point;

/**
 * Initialize the robot's movement routines. 
 * Should be called before either of the functions below are used.
 */
void set_up_movement_functions();

/**
 * Moves to the location to the given cartesian point, which is
 * _relative_ to the robot's current position.
 */
void drive_to_relative_cartesian_point(cartesian_point point);

/**
 * Drive to the specified polar point, relative to the
 * robot's current position.
 */
void drive_to_relative_polar_point(polar_point point);

#endif
