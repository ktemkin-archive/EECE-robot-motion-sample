
#define F_CPU 25000000UL

#include <stdlib.h>
#include <util/delay.h>

#include "movement.h"

//Default motor speed, out of 255.
//#define BASE_MOTOR_SPEED 220 
static const unsigned char BASE_MOTOR_SPEED = 220;

//Proportional constant for control.
static const unsigned char PROPORTIONAL_CORRECTION_FACTOR = 5;

//Maximum correction that can be made.
#define MAXIMUM_ALLOWED_CORRECTION (255 - BASE_MOTOR_SPEED) / PROPORTIONAL_CORRECTION_FACTOR

//Conversion factor between quad encoder ticks and actual distance.
static const unsigned char TICKS_PER_INCH = 20; 

//Conversion factor between quad encoder ticks and angle.
static const unsigned char TICKS_PER_DEGREE_NUMERATOR = 7;
static const unsigned char TICKS_PER_DEGREE_DENOMINATOR = 5;

//An approximation of the robot's stopping distance from the BASE_MOTOR_SPEED speed, in quadrature encoder ticks.
static const unsigned char STOPPING_DISTANCE_IN_TICKS = 5;

#define LEFT_MOTOR_PORT         LEFT_PORT_OUT
#define RIGHT_MOTOR_PORT        LEFT_PORT_OUT
#define QUADRATURE_ENCODER_PORT LEFT_PORT_IN

//The period for the pulse width modulation.
static const unsigned char PWM_INTERVAL = 3; // 3us

//The amount of milliseconds it takes for the robot to come to a full stop.
static const unsigned int FULL_STOP_TIME = 500;

//An enumerative tive that defines two arbitrary constants: CLOCKWISE, and COUNTERCLOCKWISE.
typedef enum {
  CLOCKWISE, 
  COUNTERCLOCKWISE
} direction_of_rotation;


/***
 * Prototypes for internal functions.
 *
 * Each of these functions is used only inside of this source file; publically used functions are defined in movement.h.
 */

//Basic "high-level" movement functions, which operate in terms of real-world degrees.
static void rotate_by_degrees(unsigned int angle, direction_of_rotation direction);
static void move_forward_by_inches(char distance);

//High level turn functions, which allow the robot to turn relative to its current position.
static void turn_around();
static void turn_left();
static void turn_right();

//Core method which runs the motors at the same speed until they've traveled a given (linear) distance.
static void run_motors_for_set_distance(unsigned int ticks_needed);

//Adjustment methods, which are used to drive the robot with both motors set to the same true speed.
static unsigned char determine_motor_speed_from_offset(int distance_motor_is_lagging_by);
static void move_motors_at_speed(unsigned char left_time_high, unsigned char right_time_high);

//Funtions which determine the robot's position relative to a desired point. 
static char point_is_to_the_robots_right(cartesian_point point);
static char point_is_behind_robot(cartesian_point point);

//Functions that determine the correct locations/distances for travel.
static cartesian_point rotate_point_location_by_180_degrees(cartesian_point point);
static int correct_for_stopping_distance(unsigned int distance_in_quad_ticks);
static inline char wheel_direction_for_robot_rotation(char direction);

//Functions that adjust the motor's directions.
static inline void set_left_motor_rotation(direction_of_rotation direction);
static inline void set_right_motor_rotation(direction_of_rotation direction);
static inline void set_motor_rotation(volatile unsigned char * port, unsigned char bit_number, direction_of_rotation direction);
static inline void set_motor_directions_to_forward();

//Functions that turn the motors on and off.
static inline void stop_left_motor();
static inline void stop_right_motor();
static inline void stop_both_motors();
static inline void start_left_motor();
static inline void start_right_motor();
static inline void full_stop_from_moving();

//Functions to read the quadrature encoder signals.
static inline unsigned char read_left_quadrature_encoder();
static inline unsigned char read_right_quadrature_encoder();
static inline unsigned char read_quadrature_encoder(volatile unsigned char * port, unsigned char b_channel_bit_number);

//Low level functions which get and set bits within a byte.
static inline void set_bit(volatile unsigned char * byte, unsigned char bit);
static inline void clear_bit(volatile unsigned char * byte, unsigned char bit);

//Basic math functions.
static inline int max(int a, int b);
static inline int min(int a, int b);


/**
 **
 ** Public API: these are the functions which can be called from outside code.
 **
 **/

/**
 * Initialize the robot's movement routines. 
 * Should be called before either of the functions below are used.
 */
void set_up_movement_functions() {
    stop_both_motors();
}

/**
 * Moves to the location to the given cartesian point, which is
 * _relative_ to the robot's current position.
 */
void drive_to_relative_cartesian_point(cartesian_point point) {

  if(point_is_behind_robot(point)) {
    turn_around();
    point = rotate_point_location_by_180_degrees(point);
  }

  move_forward_by_inches(point.y);

  if(point_is_to_the_robots_right(point)) {
      turn_right();
  } else {
      turn_left();
  }

  move_forward_by_inches(point.x);
}

/**
 * Drive to the specified polar point, relative to the
 * robot's current position.
 */
void drive_to_relative_polar_point(polar_point point) {
  rotate_by_degrees(point.angle, CLOCKWISE);
  move_forward_by_inches(point.distance);
}

/**
 **
 ** Private functions; for internal use only.
 **
 **/

/**
 * Moves the robot (straight) forward by the specified number of inches.
 */
static void move_forward_by_inches(char distance) {
   
    unsigned int distance_in_quad_ticks = abs((int)distance) * TICKS_PER_INCH;
    distance_in_quad_ticks = correct_for_stopping_distance(distance_in_quad_ticks);
    
    set_motor_directions_to_forward();
    run_motors_for_set_distance(distance_in_quad_ticks);
}

/**
 * Turn the robot completely around, 180 degrees.
 */
static void turn_around() {
    rotate_by_degrees(180, CLOCKWISE);
}

/**
 * Turn the robot 90 degrees counter-clockwise, to its "left".
 */
static void turn_left() {
    rotate_by_degrees(90, COUNTERCLOCKWISE);
}

/**
 * Turn the robot 90 degrees clockwise, to its "right".
 */
static void turn_right() {
    rotate_by_degrees(90, CLOCKWISE);
}

/**
 * Rotates the robot by the specified number of inches in the specified direction.
 */
static void rotate_by_degrees(unsigned int angle, direction_of_rotation direction) {

    unsigned int angle_in_quad_ticks = (angle * TICKS_PER_DEGREE_NUMERATOR) / TICKS_PER_DEGREE_DENOMINATOR;
    char wheel_direction = wheel_direction_for_robot_rotation(direction);

    set_left_motor_rotation(wheel_direction);
    set_right_motor_rotation(wheel_direction);
    run_motors_for_set_distance(angle_in_quad_ticks);

}

/**
 * Moves the left and right motor for the specified distance, in terms of quadrature
 * encoder ticks. The direction of the motor is not changed; so you should call
 * set_motor_rotation for each motor before calling this function.
 */
static void run_motors_for_set_distance(unsigned int target_distance_in_ticks)
{
    unsigned int distance_traveleled_left = 0, distance_traveleled_right = 0;
    unsigned char left_encoder_value  = read_left_quadrature_encoder();
    unsigned char right_encoder_value = read_right_quadrature_encoder();
    unsigned char left_motor_speed, right_motor_speed;
    unsigned char previous_left_encoder_value, previous_right_encoder_value;

    while ((distance_traveleled_left < target_distance_in_ticks) || (distance_traveleled_right < target_distance_in_ticks)) {

        left_motor_speed = determine_motor_speed_from_offset(distance_traveleled_right - distance_traveleled_left);
        right_motor_speed = determine_motor_speed_from_offset(distance_traveleled_left - distance_traveleled_right);

        move_motors_at_speed(left_motor_speed, right_motor_speed);

        //Save the previous quadrature encoder values as a basis for comparison.
        previous_left_encoder_value = left_encoder_value;
        previous_right_encoder_value = right_encoder_value;
        left_encoder_value = read_left_quadrature_encoder();
        right_encoder_value = read_right_quadrature_encoder();
       
        //If either of our quadrature encoders have change value, that means we've moved a single
        //"tick" of distance. Increment the distance traveled.
        if (left_encoder_value != previous_left_encoder_value) {
            ++distance_traveleled_left;
        }

        if (right_encoder_value != previous_right_encoder_value) {
            ++distance_traveleled_right;
        }
    }  

    full_stop_from_moving();
}

/**
 * Determines the speed for a given motor (in terms of duty cycle out of 255) 
 * by using the total distance that the given motor is _behind_ in ticks.
 */
static unsigned char determine_motor_speed_from_offset(int distance_motor_is_lagging_by_in_ticks) {
    unsigned char amount_to_correct_by; 

    //If the motor is laggging...
    if(distance_motor_is_lagging_by_in_ticks > 0) {
      //... speed it up by a factor proportional to how behind it is.
      amount_to_correct_by = PROPORTIONAL_CORRECTION_FACTOR * min(MAXIMUM_ALLOWED_CORRECTION, distance_motor_is_lagging_by_in_ticks);
    } 
    //Otherwise, don't correct.
    else {
      amount_to_correct_by = 0;
    }

    return BASE_MOTOR_SPEED + amount_to_correct_by;
}

/**
 * Performs a short burst of movement using Pulse Width Modulation. 
 *
 * The left_time_high and right_time_high specify the duty cycles for the left and right
 * motors respectively, and should be expressed out of 255.
 */
static void move_motors_at_speed(unsigned char left_time_high, unsigned char right_time_high) {
    int time;

    //Spend approximately 255 pwm intervals sending a single period of PWM.
    //
    //Assuming PWM_INTERVAL is long enough, this should be much longer than the time we spend
    //performing computations in the calling function.
    for (time = 0; time < 255; ++time) {

        if (time < left_time_high)
            start_left_motor();
        else
            stop_left_motor();


        if (time < right_time_high)
            start_right_motor();
        else
            stop_right_motor();

  
        _delay_us(PWM_INTERVAL);
    }
}

/**
 * Returns true iff the specified point is behind our robot.
 */ 
static char point_is_behind_robot(cartesian_point point) {
  return point.x < 0;
}

/**
 * Returns true iff the specified point is to the robot's right.
 */ 
static char point_is_to_the_robots_right(cartesian_point point) {
  return point.y < 0;
}

/**
 * Returns a new point which represents the rotation of the given point:
 * that is, where the point would be relative to the robot if the robot were
 * to turn 180 degrees.
 */ 
static cartesian_point rotate_point_location_by_180_degrees(cartesian_point point) {
  point.y *= -1;
  point.x *= -1;
  return point;
}

/**
 * Adjusts a given distance (in quad ticks) to account for the approxiamte stopping distance of the robot.
 */
static int correct_for_stopping_distance(unsigned int distance_in_quad_ticks) {
    if(distance_in_quad_ticks > STOPPING_DISTANCE_IN_TICKS) {
      return distance_in_quad_ticks - STOPPING_DISTANCE_IN_TICKS;
    } else {
      return 0;
    }
}

/**
 * Determines the direction that _both_ wheels would need to rotate in order to
 * rotate the robot in the given direction. Directions are gagued as if one were 
 * looking at the robot from above.
 */ 
static inline char wheel_direction_for_robot_rotation(char direction) {
  return (direction == CLOCKWISE) ? COUNTERCLOCKWISE : CLOCKWISE;
}

/**
 * Sets the direction in which the robot's left wheel should rotate.
 */
static inline void set_left_motor_rotation(direction_of_rotation direction) {
  set_motor_rotation(&LEFT_MOTOR_PORT, LEFT_MOTOR_DIRECTION, direction);
}

/**
 * Sets the direction in which the robot's right wheel should rotate.
 */
static inline void set_right_motor_rotation(direction_of_rotation direction) {
  set_motor_rotation(&RIGHT_MOTOR_PORT, RIGHT_MOTOR_DIRECTION, direction);
}

/**
 * Sets the direction in which the given wheel should rotate.
 *
 * The wheel's location should be expressed via a pointer to a port register (port),
 * and the bit number which controls the wheel's direction.
 */
static inline void set_motor_rotation(volatile unsigned char * port, unsigned char bit_number, direction_of_rotation direction) {
  if(direction == CLOCKWISE) {
    set_bit(port, bit_number);  
  } else {
    clear_bit(port, bit_number); 
  }
}

/**
 * Convenience function which sets the robot's motors to move forward.
 */ 
static inline void set_motor_directions_to_forward() {
    set_left_motor_rotation(COUNTERCLOCKWISE);
    set_right_motor_rotation(CLOCKWISE);
}

/**
 * Stops both motors, and waits for a short period to allow the robot to come to a full stop.
 */
static inline void full_stop_from_moving() {
  stop_both_motors();
  _delay_ms(FULL_STOP_TIME);
}

/**
 * Reads the current state of the left quadrature encoder, and returns it as a two-bit unsigned integer.
 */
static inline unsigned char read_left_quadrature_encoder() {
  return read_quadrature_encoder(&QUADRATURE_ENCODER_PORT, LEFT_MOTOR_B);
}

/**
 * Reads the current state of the right quadrature encoder, and returns it as a two-bit unsigned integer.
 */
static inline unsigned char read_right_quadrature_encoder() {
  return read_quadrature_encoder(&QUADRATURE_ENCODER_PORT, RIGHT_MOTOR_B);
}

/**
 * Reads the current state of the given quadrature encoder, and returns it as a two-bit unsigned integer.
 * Assumes a contiguous quadrature encoder.
 *
 * The location of the specified quadrature encoder should be specified as a pointer to its port, and the bit number of the
 * _lower_ bit of the two-bit value.
 */
static inline unsigned char read_quadrature_encoder(volatile unsigned char * port, unsigned char lower_channel_bit_number) {
    unsigned char quadrature_encoder_unmasked = *port >> lower_channel_bit_number;
    return quadrature_encoder_unmasked & 0b11;
}

/**
 * Turns off the left motor, and returns immediately.
 * Does not wait for the motor to come to a full stop.
 */
static inline void stop_left_motor() {
  clear_bit(&LEFT_MOTOR_PORT, LEFT_MOTOR_ENABLE);
}

/**
 * Turns off the right motor, and returns immediately.
 * Does not wait for the motor to come to a full stop.
 */
static inline void stop_right_motor() {
  clear_bit(&RIGHT_MOTOR_PORT, RIGHT_MOTOR_ENABLE);
}

/**
 * Turns off both motors, and returns immediately.
 * Does not wait for the motor to come to a full stop.
 */
static inline void stop_both_motors() {
  stop_left_motor();
  stop_right_motor();
}

/**
 * Turns on the left motor.
 */
static inline void start_left_motor() {
  set_bit(&LEFT_MOTOR_PORT, LEFT_MOTOR_ENABLE);
}

/**
 * Turns on the right motor.
 */
static inline void start_right_motor() {
  set_bit(&RIGHT_MOTOR_PORT, RIGHT_MOTOR_ENABLE);
}


/**
 * Clears the given bit of the specified byte in-place.
 * Arguments should be a pointer to the byte to be cleared, and the bit number.
 */
static inline void clear_bit(volatile unsigned char * byte, unsigned char bit) {
  *byte &= ~(1 << bit);
}

/**
 * Sets the given bit of the specified byte in-place.
 * Arguments should be a pointer to the byte to be cleared, and the bit number.
 */
static inline void set_bit(volatile unsigned char * byte, unsigned char bit) {
  *byte |= 1 << bit;
}

/**
 * Returns whichever of its two arguments is greater.
 */
static inline int max(int a, int b) {
  return (a > b) ? a : b;
}

/**
 * Returns whichever of its two arguments is lesser.
 */
static inline int min(int a, int b) {
  return (a < b) ? a : b;
}
