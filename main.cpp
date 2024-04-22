#include "main.h"
#include <cstdint>
// Ivo here
// bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
// bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
// DebounceIn user_button(USER_BUTTON); // create DebounceIn object to evaluate the user button
                                     // falling and rising edge
// void toggle_do_execute_main_fcn();   // custom function which is getting executed when user
                                     // button gets pressed, definition below

// main runs as an own thread

// ------------- System Variables -------------
// for exapmle:
// int length_cap = 10; // in mm
// int hight_cap = 100; // in mm

// ------------- Operation Variables -------------

int8_t sum_endstop = 0;










int main()
{
    user_button.fall(&toggle_do_execute_main_fcn);


    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, there for
    Timer main_task_timer;              // create Timer object which we use to run the main task every main_task_period_ms
                                         
    // led on nucleo board
    DigitalOut user_led(USER_LED);

    // additional led
    // create DigitalOut object to command extra led, you need to add an aditional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via a resistor
    DigitalOut led1(PB_9);

    // ------------- States and actual state for the machine -------------
    const int CAPTOR_STATE_INIT = 0; // Alle Endstops auf 0
    const int CAPTOR_STATE_000 = 1; // Alle Endstops auf 0
    const int CAPTOR_STATE_100 = 2; 
    const int CAPTOR_STATE_110 = 3; 
    const int CAPTOR_STATE_111 = 4; 
    const int CAPTOR_STATE_011 = 5; 
    const int CAPTOR_STATE_001 = 6; 
    const int CAPTOR_STATE_010_error = 7; // Error case
    const int CAPTOR_STATE_101_error = 8; // Erorr case
    const int CAPTOR_STATE_error = 9; 
   
    int captor_state_actual = CAPTOR_STATE_INIT;



    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;


            // ------------- State machine -------------
            switch (captor_state_actual){

                case CAPTOR_STATE_INIT:
                    sum_endstop = 0;
                    sum_endstop |= endstop1.read(); 
                    sum_endstop |= endstop2.read(); 
                    sum_endstop |= endstop3.read(); 
                    /* Mapping table:
                    000 : 0
                    100 : 4
                    110 : 6
                    111 : 7
                    011 : 3
                    001 : 1
                    101 : 5
                    010 : 2
                    */

                    if(sum_endstop == 0) {
                        captor_state_actual = CAPTOR_STATE_000;
                        break;
                        
                    } else if (sum_endstop == 4){
                        captor_state_actual = CAPTOR_STATE_100;
                        break;

                    } else if (sum_endstop == 6){
                        captor_state_actual = CAPTOR_STATE_110;
                        break;

                    }else if (sum_endstop == 7){
                        captor_state_actual = CAPTOR_STATE_111;
                        break;

                    }else if (sum_endstop == 3){
                        captor_state_actual = CAPTOR_STATE_011;
                        break;

                    }else if (sum_endstop == 1){
                        captor_state_actual = CAPTOR_STATE_001;
                        break;

                    }else if (sum_endstop == 5){
                        captor_state_actual = CAPTOR_STATE_101_error;
                        break;

                    }else if (sum_endstop == 2){
                        captor_state_actual = CAPTOR_STATE_010_error;
                        break;

                    }
                    break;
                
                case CAPTOR_STATE_000:
                    captor_state_actual = CAPTOR_STATE_100;
                    break;
                
                case CAPTOR_STATE_100:
                    captor_state_actual = CAPTOR_STATE_110;
                    break;
                
                case CAPTOR_STATE_110:
                    captor_state_actual = CAPTOR_STATE_111;
                    break;
                
                case CAPTOR_STATE_111:
                    captor_state_actual = CAPTOR_STATE_011;
                    break;
                
                case CAPTOR_STATE_011:
                    captor_state_actual = CAPTOR_STATE_001;
                    break;
                
                case CAPTOR_STATE_001:
                    captor_state_actual = CAPTOR_STATE_010_error;
                    break;
                
                case CAPTOR_STATE_010_error:
                    // Error case
                    captor_state_actual = CAPTOR_STATE_error;
                    break;
                
                case CAPTOR_STATE_101_error:
                    // Error case
                    captor_state_actual = CAPTOR_STATE_error;
                    break;
                
                case CAPTOR_STATE_error:
                    // Error case
                    captor_state_actual = CAPTOR_STATE_INIT;
                    break;
                
                default:
                    // do nothing 
                    captor_state_actual = CAPTOR_STATE_INIT;
                    break;



            }
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects
                led1 = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}

// Farbsensor
bool read_cap_coulor(void){
    // return if level is correct
    return true;
}

// Decapper
bool drive_belt(void);
// return 1 if decapping is succesful
// return 0 if decapping is not succesful

// Ultrasonic / liquid level
bool read_liquid_level(void) {
    // return if level is correct
    return true;
}

// Foerderband fahren
int drive_belt(int velocity, int cylecounter);

// show Error messages on LED's
void show_LED(int error_code);