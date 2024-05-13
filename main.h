#pragma once 

#include "mbed.h"


// pes board pin map
#include "pm2_drivers/PESBoardPinMap.h"

// drivers
#include "pm2_drivers/DebounceIn.h"
#include <cstdint>
#include <cstdio>
#include <ctime>


#include "pm2_drivers/FastPWM/FastPWM.h"
#include "pm2_drivers/DCMotor.h"

//#include "pm2_drivers/UltrasonicSensor.h"


// The Laser-lib is from: http://os.mbed.com/users/joelvonrotz/code/VL53L0X/
//#include <VL53L0X.h>

# define M_PI 3.14159265358979323846 // number pi, an example in case you need it

// ---- Vehicle Variables ----
//# define WHEEL_DIAMETER 30.0 
# define GEAR_DIAMETER 20.3


bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and shows how you can run a code segment only once
bool start_pressed = false;

// objects for user button (blue button) handling on nucleo board
//DebounceIn user_button(PC_13);  // create InterruptIn interface object to evaluate user button falling and rising edge (no blocking code in ISR)
DebounceIn user_button(USER_BUTTON);  // create InterruptIn interface object to evaluate user button falling and rising edge (no blocking code in ISR)
//DebounceIn btn_cmd1();
//DebounceIn btn_cmd2();
//DebounceIn btn_cmd3();



//DigitalOut Led_1(PC_13);
//DigitalOut Led_2(PC_14);
//DigitalOut Led_3(PC_15);

//define Sensors
// ultra sonic sensor
//UltrasonicSensor us_sensor(PB_D3);
//float us_distance_cm = 0.0f;





// define functions
void toggle_do_execute_main_fcn();   // custom function which is getting executed when user
                                     // button gets pressed, definition below

bool read_cap_color();
bool drive_belt(void);
int read_liquid_level(void);
bool drive_belt_forward();
bool drive_belt_backward();
void show_LED(int error_code);
bool decap();
bool read_button(float);











