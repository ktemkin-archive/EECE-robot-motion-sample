
#define F_CPU 25000000UL 

#include <util/delay.h>

#include "basys.h"
#include "movement.h"

//Patterns to display when awaiting user input.
static unsigned char WAITING_FOR_MODE_PATTERN           = 0x01;
static unsigned char WAITING_FOR_X_DISPLACEMENT_PATTERN = 0x02;
static unsigned char WAITING_FOR_Y_DISPLACEMENT_PATTERN = 0x03;
static unsigned char WAITING_FOR_ANGLE_PATTERN          = 0x04;
static unsigned char WAITING_FOR_DISTANCE_PATTERN       = 0x05;

//Button number with upon which to wait for user input.
static unsigned char ENTRY_BUTTON = 1;

//Local "high-level" user input entry functions.
cartesian_point get_cartesian_point_from_user();
polar_point get_polar_point_from_user();
static inline char is_dual_displacement_mode_requested();

//Local "low_level" user input entry functions.
static inline unsigned char get_user_input(unsigned char led_pattern);
static inline unsigned char get_signed_user_input(unsigned char led_pattern);
static unsigned char read_switches_on_press_of(unsigned char button_num);
static void wait_for_button_press(unsigned char button_num);


/**
 * Main code for the Project 1 demonstration.
 */
int main() {

    set_up_basys_io();
    set_up_movement_functions();

    while(1) {

        LEDS = WAITING_FOR_MODE_PATTERN;

        if(is_dual_displacement_mode_requested()) {
            cartesian_point desired_destination = get_cartesian_point_from_user();
            drive_to_relative_cartesian_point(desired_destination);
        } else {
            polar_point desired_destination = get_polar_point_from_user();
            drive_to_relative_polar_point(desired_destination);
        }
    }

    return 0;
}

/**
 * Reads a signed cartesian point from the user via the following process:
 * - The x value is entered on the switches, and loaded via button 1.
 * - The y value is entered on the switches, and loaded via button 1.
 */ 
cartesian_point get_cartesian_point_from_user() {
  cartesian_point point;

  //Request that the user enter the X and Y corrdinates for a cartesian point.
  point.x = get_user_input(WAITING_FOR_X_DISPLACEMENT_PATTERN);
  point.y = get_user_input(WAITING_FOR_Y_DISPLACEMENT_PATTERN);

  //... and return that point.
  return point;
}

/**
 * Reads a polar point from the user via the following process.
 * - The x value is entered on the switches, and loaded via button 1.
 * - The y value is entered on the switches, and loaded via button 1.
 */
polar_point get_polar_point_from_user() {
  polar_point point;

  //Request that the user enter the angle and distance for a polar point...
  point.angle    = get_signed_user_input(WAITING_FOR_ANGLE_PATTERN) * 2;
  point.distance = get_user_input(WAITING_FOR_DISTANCE_PATTERN);

  //... and return that point.
  return point;
}

/**
 * Determines if "dual displacement" mode is desired.
 * Waits for the user to enter a value on the switches and press BTN1.
 * If the switch value is non-zero, we use dual displacement mode.
 */ 
inline char is_dual_displacement_mode_requested () {
  return read_switches_on_press_of(ENTRY_BUTTON) != 0; 
}

/**
 * Waits until BTN1 is pressed, and then reads a single byte of unsigned data from the user.
 * Displays a status pattern on the board's LEDs.
 */
inline unsigned char get_user_input(unsigned char led_pattern) {

  //Indicate the status of the operation by displaying an LED pattern.
  LEDS = led_pattern;

  //And read the raw port data into the given byte.
  return read_switches_on_press_of(ENTRY_BUTTON);

}

/**
 * Waits until BTN1 is pressed, and then reads a single byte of signed data from the user.
 * Displays a status pattern on the board's LEDs.
 */
inline unsigned char get_signed_user_input(unsigned char led_pattern) {
  return (char)get_user_input(led_pattern); 
}

/**
 * Waits until BTN1 is pressed, and then reads a single byte of signed data from the user.
 * Does not display status to the user.
 */
unsigned char read_switches_on_press_of(unsigned char button_num) {
    wait_for_button_press(button_num);
    return SWITCHES;
}

/**
 * Waits until the rising edge of a button press.
 */ 
void wait_for_button_press(unsigned char button_num) {
    loop_until_bit_is_clear(BUTTONS, button_num);
    loop_until_bit_is_set(BUTTONS, button_num);
}


