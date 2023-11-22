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

// ------------- Vehicle Variables -------------
# define WHEEL_DIAMETER      34.5   // in mm
# define ARM_LENGTH         147.0   // in mm
# define GRYPER_HIGHT        30.0   // in mm
# define AXE_HEIGHT          45.0   // in mm


// ------------- Operation Variables -------------
# define DISTANCE_1         500      // in mm (gemessen auf der Bahn 440mm)
# define DISTANCE_2         340      // in mm    
# define ANGEL_SET_ARM      45      // in Grad (wird nicht verwendet)
# define HURDLE_HIGHT       100     // in mm

bool arm_0_position = 0;
bool claw_down = 0;
bool claw_up = 0;
bool forward_1 = 0;
bool forward_2 = 0;

float angle_claws_forward = 0.0;
float angle_claws_backword = 0.0;


#include <main.h>
// main runs as an own thread

int main()
{
    // attach button fall function to user button object, button has a pull-up resistor
    user_button.fall(&user_button_pressed_fcn); //Blue User Button on Nucleo Board
    // while loop gets executed every main_task_period_ms milliseconds (simple aproach to repeatedly execute main)
    const int main_task_period_ms = 50; // define main task period time in ms e.g. 50 ms -> main task runs 20 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task every main_task_period_ms
    
    
    // ---------- Buttons ----------
    DigitalIn mechanical_button(PC_13); // old PC_5
    mechanical_button.mode(PullUp);

    DigitalIn btn_start(PB_2);   // create DigitalIn object to evaluate extra mechanical button, you need to specify the mode for proper usage, see below
    btn_start.mode(PullDown); // set pullup mode: sets pullup between pin and 3.3 V, so that there is a defined potential
   // btn_start.fall(&start_button_pressed_fcn);

    DigitalIn btn_reset_vehicle(PC_8);
    btn_reset_vehicle.mode(PullDown); // set pullup mode: sets pullup between pin and 3.3 V, so that there is a defined potential
    
    DigitalIn btn_reset_all(PC_6);
    btn_reset_all.mode(PullDown); // set pullup mode: sets pullup between pin and 3.3 V, so that there is a defined potential
    
    bool btn_reserve = 0;
    //DigitalIn btn_reserve(PB_12);
    //btn_reserve.mode(PullDown); // set pullup mode: sets pullup between pin and 3.3 V, so that there is a defined potential

    // ------------- Leds -------------
    DigitalOut user_led(LED1);       // create DigitalOut object to command user led
    // additional led
    DigitalOut additional_led(PB_9); // create DigitalOut object to command extra led (you need to add an aditional resistor, e.g. 220...500 Ohm)


    // ------------- Motoren -------------
    DigitalOut enable_motors(PB_15);                    // create DigitalOut object to enable dc motors
    const float max_voltage = 6.0f;                    // Voltage for DC-Motors


    // ------------- M1 (closed-loop position controlled) -------------
    float max_speed_rps_M1 = 0.25f;
    const int M1_gear = 195;
    const float maxAccelerationRPS_M1 = 4.0f;

    const float counts_per_turn_M1 = 20.0f * 78.125f;      // define counts per turn at gearbox end: counts/turn * gearratio
    const float kn_M1 = 180.0f / 12.0f;                    // define motor constant in RPM/V
    const float k_gear_M1 = M1_gear / 78.125f;             // define additional ratio in case you are using a dc motor with a different gear box, e.g. 100:1
    const float kp_M1 = 0.1f;

    FastPWM pwm_M1(PB_13);                              // Pin is correct!
    EncoderCounter  encoder_M1(PA_6, PC_7);             // Pin is correct!
    PositionController positionController_M1(counts_per_turn_M1 * k_gear_M1, kn_M1 / k_gear_M1, max_voltage, pwm_M1, encoder_M1);
    positionController_M1.setSpeedCntrlGain(kp_M1 * k_gear_M1);   // adjust internal speed controller gain, this is just an example
    positionController_M1.setMaxVelocityRPS(max_speed_rps_M1); // adjust max velocity for internal speed controller
    // fuer ruck beseitigung; maximale beschleunigung festssetzen
    positionController_M1.setMaxAccelerationRPS(maxAccelerationRPS_M1);


    // ------------- M2 (closed-loop position controlled) -------------
    float max_speed_rps_M2 = 0.040f;
    const int M2_gear = 195;
    const float maxAccelerationRPS_M2 = 1.0f;

    const float counts_per_turn_M2 = 20.0f * 78.125f;     // define counts per turn at gearbox end: counts/turn * gearratio
    const float kn_M2 = 50.0f / 12.0f;                    // define motor constant in RPM/V
    const float k_gear_M2 = M2_gear / 78.125f;            // define additional ratio in case you are using a dc motor with a different gear box, e.g. 100:1
    const float kp_M2 = 0.1f;

    FastPWM pwm_M2(PA_9);                       // Pin is correct!
    EncoderCounter  encoder_M2(PB_6, PB_7);     // Pin is correct!
    PositionController positionController_M2(counts_per_turn_M2 * k_gear_M2, kn_M2 / k_gear_M2, max_voltage, pwm_M2, encoder_M2);
    positionController_M2.setSpeedCntrlGain(kp_M2 * k_gear_M2);   // adjust internal speed controller gain, this is just an example
    positionController_M2.setMaxVelocityRPS(max_speed_rps_M2); // adjust max velocity for internal speed controller
    // fuer ruck beseitigung; maximale beschleunigung festssetzen
    positionController_M2.setMaxAccelerationRPS(maxAccelerationRPS_M2);


    const int VENTILATOR_STATE_INIT = 0; 
    const int VENTILATOR_STATE_FORWARD = 1;
    const int VENTILATOR_STATE_BACKWORD = 2;
    const int VENTILATOR_STATE_FINAL = 8;
    const int VENTILATOR_STATE_RESET = 9;
    const int VENTILATOR_TEST = 11;
    int ventilator_state_actual = VENTILATOR_STATE_INIT;

     // this loop will run forever
    while (true) {

        main_task_timer.reset();

        //printf("DO_EXECUTE_MAIN_TASK:  %d :\n", do_execute_main_task);
        printf("BTN USER: %d", user_button.read(), 
        "BTN MECHANICAL:  %d", mechanical_button.read(), 
        "BTN START:  %d\n", btn_start.read(),
        "BTN RESET:  %d\n", btn_reset_vehicle.read());
        //printf("BTN MECHANICAL:  %d ", mechanical_button.read());
        //printf("BTN START:  %d \n", btn_start.read());
        
        //if (do_execute_main_task) {
    
            // visual feedback that the main task is executed, setting this once would actually be enough
            additional_led = 1;

            //state_machine
            switch(ventilator_state_actual){

                case VENTILATOR_STATE_INIT:
                

                    if(!btn_start.read()){ //btn_start
                        // Start the loop
                        printf("SET START MODE\n");
                        enable_motors = 1;
                        ventilator_state_actual = VENTILATOR_STATE_FORWARD;                    

                    } else if(!btn_reset_vehicle.read()) {
                        // for the resetloop
                        printf("SET RESET MODE\n");
                        ventilator_state_actual = VENTILATOR_STATE_RESET;
                        
                    } else if(btn_reset_all.read()) {
                        printf("SET RESET ALL MODE\n");
                        // Reset All
                        // muss negiert werden wenn der Button angeschlossen wird

                    } else if(btn_reserve){
                        //Reserve Button
                        printf("SET RESERVE MODE\n");
                    } else {
                        // set state to init state
                        ventilator_state_actual = VENTILATOR_STATE_INIT;
                    }
                    break;


                case VENTILATOR_STATE_FORWARD:
                    printf("Run VENTILATOR_STATE_FORWARD\n");
                    /* Fahre die Claws zusammen */
                    angle_claws_forward = 1.57;

                    if (!claw_down){
                        positionController_M1.setDesiredRotation(angle_claws_forward); // 1.0f = 360°, 0.222f= 80°   
                    }
                    if(positionController_M1.getRotation() <= angle_claws_forward){  
                        claw_down = 1;  
                        ventilator_state_actual = VENTILATOR_STATE_BACKWORD;
                        printf("Set STATE_BACKWORD_1\n");
                    }
                    break;

                case VENTILATOR_STATE_BACKWORD:
                    printf("Run VENTILATOR_STATE_BACKWORD\n");
                    /* Fahre die Claws auseinander */
                    angle_claws_backword = 0;

                    if (!claw_up){
                        positionController_M1.setDesiredRotation(0.0f); // 1.0f = 360°, 0.222f= 80°   
                    }
                    if(positionController_M1.getRotation() <= 0.01f){  
                        claw_up = 1;  
                        ventilator_state_actual = VENTILATOR_STATE_FINAL;
                        printf("Set STATE_FORWARD_1\n");
                    }
                    break;

          

                case VENTILATOR_STATE_FINAL:
                    printf("Run STATE_FINAL\n");
                    claw_up = 0;
                    claw_down = 1;
                    ventilator_state_actual = VENTILATOR_STATE_INIT;
                    break;

                case VENTILATOR_STATE_RESET:
                    printf("Run STATE_RESET\n");

                    positionController_M1.setDesiredRotation(0.0f); //M1 auf Position 0.0 fahren
                    positionController_M2.setDesiredRotation(0.0f); //M2 auf position 0.0 fahren

                    
                    if(positionController_M1.getRotation() <= 0.01f ){
                        ventilator_state_actual = VENTILATOR_STATE_INIT;
                    }
                    break;
            }

            //printf("\nEncoder M1: %3d\tPosition M1: %3.3f\tEncoder M2: %3d\tPosition M2: %3.3f", encoder_M1.read(), positionController_M1.getRotation(), encoder_M2.read(), positionController_M2.getRotation());


        // toggling the user led
        user_led = !user_led;

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
        //}
    }
}



void user_button_pressed_fcn() {
    // do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    printf("User button pressed\n");
    if(do_execute_main_task) do_reset_all_once = true;
}

void start_button_pressed_fcn() {
    printf("START Button pressed\n");
    start_pressed = !start_pressed;
}


