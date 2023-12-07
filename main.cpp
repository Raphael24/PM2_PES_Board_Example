/*
* Project: Medical Ventilator
* Date: 22.11.2023
* Authors: Raphael Romann
* Modul: PM3
* Version: 1.0.0.0
*/
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <stdlib.h>

#include <mbed.h>
#include <ctime>
#include <math.h>

#include "PM2_Drivers.h"

# define M_PI 3.14159265358979323846 // number pi, an example in case you need it

/* Versuch: 1
Durchmesser des Wicklers:           5mm
Winkel der der Motor fahren muss:   280 Grad
Geschwindigkeit:                    0.8rpm
*/

/* Versuch: 2
Durchmesser des Wicklers:           10mm
Winkel der der Motor fahren muss:   200 Grad
Geschwindigkeit:                    0.2rpm
*/

#include <main.h>
bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and shows how you can run a code segment only once
bool start_pressed = false;

// main runs as an own thread

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(PC_13);  // create InterruptIn interface object to evaluate user button falling and rising edge (no blocking code in ISR)
void user_button_pressed_fcn();
 
 // custom functions which get executed when user button gets pressed, definition below
 float angle_in_degree(float angle)
    {
        float value;
        value = (1.0f/360.0f)*angle;
        return value;
    }


int main()
{
    // states and actual state for state machine
    const int ROBOT_STATE_INIT     = 0;
    const int compress_bag         = 1;
    const int release_bag          = 2;
    int robot_state_actual = ROBOT_STATE_INIT;
 
 
    // attach button fall function to user button object, button has a pull-up resistor
    user_button.fall(&user_button_pressed_fcn);
 
    // while loop gets executed every main_task_period_ms milliseconds (simple aproach to repeatedly execute main)
    const int main_task_period_ms = 10; // define main task period time in ms e.g. 50 ms -> main task runs 20 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task every main_task_period_ms
 
 
    // led on nucleo board
    DigitalOut user_led(LED1);       // create DigitalOut object to command user led
 
    // additional led
    DigitalOut additional_led(PB_9); // create DigitalOut object to command extra led (you need to add an aditional resistor, e.g. 220...500 Ohm)
 
 
    // mechanical button
    DigitalIn mechanical_button(PC_5); // create DigitalIn object to evaluate extra mechanical button, you need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);    // set pullup mode: sets pullup between pin and 3.3 V, so that there is a defined potential
 


    //================================== Motor Definitions ===================================================

    // 78:1, 100:1, ... Metal Gearmotor 20Dx44L mm 12V CB
    DigitalOut enable_motors(PB_15); // create DigitalOut object to enable dc motors

    //================================== Definitions of Motor 1 (open-loop) ==================================
 
    FastPWM pwm_M1(PB_13); // motor M1 is used open-loop
    EncoderCounter  encoder_M1(PA_6, PC_7); // create encoder objects to read in the encoder counter values, since M1 is used open-loop no encoder would be needed for operation, this is just an example

    const float counts_per_turn = 20.0f * 488.0f;
    const float max_voltage = 12.0f;  
    const float k_gear_M1= 78.0f/78.125f;
    const float knM1 = 55.0f/12.0f;     
    const float kp_M1 = 0.1;

    PositionController positionController_M1(counts_per_turn *k_gear_M1, knM1 / k_gear_M1, max_voltage, pwm_M1, encoder_M1); 
    // parameters adjusted to 100:1 gear, we need a different speed controller gain here
    positionController_M1.setSpeedCntrlGain(kp_M1 * k_gear_M1);   // adjust internal speed controller gain, this is just an example

    //================================== Definitions of Motor 2 (open-loop) ==================================

    FastPWM pwm_M2(PA_9);  // motor M2 is closed-loop speed controlled (angle velocity)
    EncoderCounter  encoder_M2(PB_6, PB_7);

    const float counts_per_turn_M2  = 20.0f * 195.3125f;        // 20 * übersetzung
    const float max_voltage_M2      = 6.0f;                    // auf 12V lassen
    const float kn_M2               = 75.0f/6.0f;             // maximale Geschwindidkeit in RPM (ratio per minute) / max volatage
    const float kp_M2               = 0.1f;                     // korrektur faktor

    PositionController positionController_M2(counts_per_turn_M2, kn_M2, max_voltage_M2, pwm_M2, encoder_M2); 
    // parameters adjusted to 100:1 gear, we need a different speed controller gain here
    //positionController_M2.setSpeedCntrlGain(kp_M2 * k_gear_M2);   // adjust internal speed controller gain, this is just an example


//================================== End of motor definitions  ==================================

    FastPWM pwm_M3(PA_10); // motor M3 is closed-loop position controlled (angle controlled)
    EncoderCounter  encoder_M3(PA_0, PA_1);
 

/*
   const float max_voltage = 12.0f;               // define maximum voltage of battery packs, adjust this to 6.0f V if you only use one batterypack
    const float counts_per_turn = 20.0f * 78.125f; // define counts per turn at gearbox end: counts/turn * gearratio
    const float kn = 180.0f / 12.0f;               // define motor constant in RPM/V
    const float k_gear = 100.0f / 78.125f;         // define additional ratio in case you are using a dc motor with a different gear box, e.g. 100:1
    const float kp = 0.2f;                         // define custom kp, this is the default speed controller gain for gear box 78.125:1
    */
    
    /*//const float counts_per_turn_M1 = 20.0f*250.0f;
    const float counts_per_turn = 20.0f * 78.125f;
    const float max_voltage = 12.0f;  
    const float k_gear_M1= 250.0f/78.125f;
    const float knM1 = 55.0f/12.0f;     
    const float kp_M1 = 0.1;
       
    */

    float max_speed_rps = 0.1f; // define maximum speed that the position controller is changig the speed, has to be smaller or equal to kn * max_voltage
    float max_speed_rps_m2 = 0.2;
    /*
    positionController_M1.setMaxVelocityRPS(max_speed_rps); // adjust max velocity for internal speed controller
    */
    main_task_timer.start();
 
 
    float offset_M1 = 0;
    float offset_M2 = 0;
    float angle_M1 = 0.0f;
    float angle_M2 = 200.0f;
    
    
    // this loop will run forever
    while (true) {
         // printf("Angle: %f\n", positionController_M1.getRotation()*360.0f);
    
        main_task_timer.reset();
 
        if (do_execute_main_task)
        {  
            additional_led = 1;

            // state machine
            switch (robot_state_actual) {
                case ROBOT_STATE_INIT:
                {
                    if(mechanical_button.read())
                    {
                    enable_motors=1;
                    
                    robot_state_actual=compress_bag;
                    }
                    break;
                }
                case compress_bag:
                {
                   //const float counts_per_turn_M1 = 20.0f*250.0f;
                    max_speed_rps = 0.35f; // define maximum speed that the position controller is changig the speed, has to be smaller or equal to kn * max_voltage
                    
                    
                    // Run Motor 1
                    positionController_M1.setMaxVelocityRPS(max_speed_rps); // adjust max velocity for internal speed controller
                    //positionController_M1.setDesiredRotation(offset_M1+angle_in_degree(angle_M1));
                    // Run Motor 2
                    positionController_M2.setMaxVelocityRPS(max_speed_rps_m2); // adjust max velocity for internal speed controller
                    positionController_M2.setDesiredRotation(offset_M2+angle_in_degree(angle_M2));

                    printf("Angle M2: %f\n", positionController_M2.getRotation()*360.0f);
                    if(positionController_M2.getRotation() >= offset_M2 + angle_in_degree(angle_M2)-0.01f) {

                        robot_state_actual=release_bag;
                        //offset_M1=positionController_M1.getRotation();
                        offset_M2=positionController_M2.getRotation();
                    }
                    break;
                }
 
                case release_bag:
                {
                        max_speed_rps = 0.25f; // define maximum speed that the position controller is changig the speed, has to be smaller or equal to kn * max_voltage
                        // Run Motor 1
                        //positionController_M1.setMaxVelocityRPS(max_speed_rps); // adjust max velocity for internal speed controller
                        //positionController_M1.setDesiredRotation(0.0f);
                        // Run Motor 2
                        positionController_M2.setMaxVelocityRPS(max_speed_rps); // adjust max velocity for internal speed controller
                        positionController_M2.setDesiredRotation(0.0f);

                        printf("Angle M2: %f\n", positionController_M2.getRotation()*360.0f);

                        if(positionController_M2.getRotation() <= 0.01f)
                        {  
                            robot_state_actual=compress_bag;
                            offset_M1=positionController_M1.getRotation(); 
                            offset_M2=positionController_M2.getRotation();
                        }
                        break;
                    
                }
                default:
                {
                    enable_motors=0;
                }         
        }      

        // toggling the user led
        user_led = !user_led;
 
        // do only output via serial what's really necessary, this makes your code slow
        /*printf("IR sensor (mV): %3.3f, Encoder M1: %3d, Speed M2 (rps) %3.3f, Position M3 (rot): %3.3f, Servo S1 angle (normalized): %3.3f, Servo S2 angle (normalized): %3.3f\r\n",
               ir_distance_mV,
               encoder_M1.read(),
               speedController_M2.getSpeedRPS(),
               positionController_M3.getRotation(),
               servo_S1_angle,
               servo_S2_angle);*/
 
        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
    }
}
 
void user_button_pressed_fcn()
{
    // do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    if (do_execute_main_task) do_reset_all_once = true;
 
}
 
 