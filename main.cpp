#include "main.h"
#include <cstdint>

#include "pm2_drivers/FastPWM/FastPWM.h"
#include "pm2_drivers/DCMotor.h"


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

int8_t sum_endstop = 0x0;




int main()
{
    user_button.fall(&toggle_do_execute_main_fcn);


    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, there for
    Timer main_task_timer;              // create Timer object which we use to run the main task every main_task_period_ms

    DebounceIn endstop1(PB_2, PullDown);
    DebounceIn endstop2(PC_8, PullDown);
    DebounceIn endstop3(PC_6, PullDown);

    // led on nucleo board
    DigitalOut user_led(USER_LED);

    // additional led
    // create DigitalOut object to command extra led, you need to add an aditional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via a resistor
    DigitalOut led1(PB_9);


    // ------------- Motoren -------------
    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                    // 6.0f V if you only use one battery pack

    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    // motor M1
    FastPWM pwm_M2(PB_PWM_M2); // create FastPWM object to command motor M1
    enable_motors = 1; // setting this once would actually be enough
    const float gear_ratio_M2 = 78.125f; // gear ratio
    const float kn_M2 = 180.0f / 12.0f;  // motor constant [rpm/V]
    // it is assumed that only one motor is available, there fore
    // we use the pins from M1, so you can leave it connected to M1
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio_M2, kn_M2, voltage_max);
    // enable the motion planner for smooth movement
    motor_M2.enableMotionPlanner(true);
    // limit max. acceleration to half of the default acceleration
    motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);
    motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.5f);

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
                    motor_M2.setRotation(motor_M2.getRotation());
                    sum_endstop = 0;
                    sum_endstop |= (!(endstop1.read()) << 2); 
                    sum_endstop |= (!(endstop2.read()) << 1); 
                    sum_endstop |= !(endstop3.read()); 
                    printf("Endstop1: %d, Endstop2: %d, Endstop3: %d\n", endstop1.read(), endstop2.read(), endstop3.read());
                    printf("Summeendstops: %d \n", sum_endstop);
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
                        printf("Schritt CAPTOR_STATE_000 wurde aktiviert\n");
                        break;
                        
                    } else if (sum_endstop == 4){
                        captor_state_actual = CAPTOR_STATE_100;
                        printf("Schritt CAPTOR_STATE_100 wurde aktiviert\n");
                        break;

                    } else if (sum_endstop == 6){
                        captor_state_actual = CAPTOR_STATE_110;
                        printf("Schritt CAPTOR_STATE_110 wurde aktiviert\n");
                        break;

                    }else if (sum_endstop == 7){
                        captor_state_actual = CAPTOR_STATE_111;
                        printf("Schritt CAPTOR_STATE_111 wurde aktiviert\n");
                        break;

                    }else if (sum_endstop == 3){
                        captor_state_actual = CAPTOR_STATE_011;
                        printf("Schritt CAPTOR_STATE_011 wurde aktiviert\n");
                        break;

                    }else if (sum_endstop == 1){
                        captor_state_actual = CAPTOR_STATE_001;
                        printf("Schritt CAPTOR_STATE_001 wurde aktiviert\n");
                        break;

                    }else if (sum_endstop == 5){
                        captor_state_actual = CAPTOR_STATE_101_error;
                        printf("Schritt CAPTOR_STATE_101_error wurde aktiviert\n");
                        break;

                    }else if (sum_endstop == 2){
                        captor_state_actual = CAPTOR_STATE_010_error;
                        printf("Schritt CAPTOR_STATE_010_error wurde aktiviert\n");
                        break;

                    }
                    break;
                
                case CAPTOR_STATE_000:
                    printf("Run CAPTOR_STATE_000\n");
                    motor_M2.setRotation(3.0f);
                    printf("Position: %f", motor_M2.getRotation());
                    if(motor_M2.getRotation() >= 3.0){
                        motor_M2.setRotation(0.0f);
                        captor_state_actual = CAPTOR_STATE_INIT;
                    } 
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