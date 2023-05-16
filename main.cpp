/*
* Project: pm2_pes_board_example
* Date: 22.03.2023
* Authors: Raphael Romann, Yves Guldimann
* Modul: PM2
* Version: 0.0.0.0
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
# define DISTANCE_2         320      // in mm    
# define ANGEL_SET_ARM      45      // in Grad (wird nicht verwendet)
# define HURDLE_HIGHT       100     // in mm

bool arm_0_position = 0;
bool arm_down = 0;
bool arm_set_m1 = 0;
bool arm_down_2 = 0;
bool forward_1 = 0;
bool forward_2 = 0;
bool rotate_full = 0;
bool adjust_ok = 0;
bool detach_ok = 0;
bool detach_ok_2 = 0;
bool detach_forward_ok = 0;
bool arm_down_3 = 0;
// ruetler
bool ruetler_ok = 0;
int i = 0;
int range = 0;
float one_step = 0.0;
bool r_backwards = 0;
bool r_forwards = 0;

float angle_arm_down_1 = 0.0;

// for GRYPER_STATE_SET_ARM
double angle_B = 0.0, angle_rot = 0.0;
float bogenlaenge = 0.0, rotation = 0.0, act_pos = 0.0, act_pos_m1;

// for GRYPER_STATE_DETACH
double angle_detach = 0;
double angle_adjust = 0;
double angle_detach_2 = 0;

#include <main.h>
// main runs as an own thread

int main()
{
    // attach button fall function to user button object, button has a pull-up resistor
    user_button.fall(&user_button_pressed_fcn); //Blue User Button on Nucleo Board
    // while loop gets executed every main_task_period_ms milliseconds (simple aproach to repeatedly execute main)
    const int main_task_period_ms = 50; // define main task period time in ms e.g. 50 ms -> main task runs 20 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task every main_task_period_ms
    // led on nucleo board
    
    // ------------- Variablen -------------

    // ---------- Buttons ----------
    DigitalIn mechanical_button(PC_5); 
    mechanical_button.mode(PullUp);

    DigitalIn btn_start(PB_2);   // create DigitalIn object to evaluate extra mechanical button, you need to specify the mode for proper usage, see below
    btn_start.mode(PullDown); // set pullup mode: sets pullup between pin and 3.3 V, so that there is a defined potential
    //btn_start.fall(&start_button_pressed_fcn);

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

    // ------------- Vehicle geometry -------------

    // ------------- Motoren -------------
    DigitalOut enable_motors(PB_15);                    // create DigitalOut object to enable dc motors
    const float max_voltage = 12.0f;                    // Voltage for DC-Motors


    // ------------- M1 (closed-loop position controlled) -------------
    float max_speed_rps_M1 = 0.2f;
    const int M1_gear = 100;
    const float maxAccelerationRPS_M1 = 4.0f;

    const float counts_per_turn_M1 = 20.0f * 78.125f;      // define counts per turn at gearbox end: counts/turn * gearratio
    const float kn_M1 = 180.0f / 12.0f;                    // define motor constant in RPM/V
    const float k_gear_M1 = M1_gear / 78.125f;              // define additional ratio in case you are using a dc motor with a different gear box, e.g. 100:1
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
    const int M2_gear = 488;
    const float maxAccelerationRPS_M2 = 1.0f;

    const float counts_per_turn_M2 = 20.0f * 78.125f;      // define counts per turn at gearbox end: counts/turn * gearratio
    const float kn_M2 = 50.0f / 12.0f;                    // define motor constant in RPM/V
    const float k_gear_M2 = M2_gear / 78.125f;              // define additional ratio in case you are using a dc motor with a different gear box, e.g. 100:1
    const float kp_M2 = 0.1f;

    FastPWM pwm_M2(PA_9);                       // Pin is correct!
    EncoderCounter  encoder_M2(PB_6, PB_7);     // Pin is correct!
    PositionController positionController_M2(counts_per_turn_M2 * k_gear_M2, kn_M2 / k_gear_M2, max_voltage, pwm_M2, encoder_M2);
    positionController_M2.setSpeedCntrlGain(kp_M2 * k_gear_M2);   // adjust internal speed controller gain, this is just an example
    positionController_M2.setMaxVelocityRPS(max_speed_rps_M2); // adjust max velocity for internal speed controller
    // fuer ruck beseitigung; maximale beschleunigung festssetzen
    positionController_M2.setMaxAccelerationRPS(maxAccelerationRPS_M2);

    // ------------- M3 (closed-loop position controlled) -------------
   /* const int M3_gear = 0;
    FastPWM pwm_M3(PA_10);                      // Pin is correct!
    EncoderCounter  encoder_M3(PA_0, PA_1);     // Pin is correct!
    */

    // ------------- Sensoren -------------
    // ------------- States and actual state for the machine -------------

    const int GRYPER_STATE_INIT = 0; 
    const int GRYPER_STATE_ARM_DOWN_1 = 1;
    const int GRYPER_STATE_FORWARD_1 = 2;
    const int GRYPER_STATE_SET_ARM = 3;
    const int GRYPER_STATE_ROTATE = 4;
    const int GRYPER_STATE_DETACH = 5;
    const int GRYPER_STATE_ARM_DOWN_2 = 6;
    const int GRYPER_STATE_FORWARD_2 = 7;
    const int GRYPER_STATE_FINAL = 8;
    const int GRYPER_STATE_RESET = 9;
    const int GRYPER_TEST = 11;
    int gryper_state_actual = GRYPER_STATE_INIT;

     // this loop will run forever
    while (true) {

        main_task_timer.reset();

        //printf("DO_EXECUTE_MAIN_TASK:  %d :\n", do_execute_main_task);
        //printf("BTN RESET Vehicle:  %d :", btn_reset_vehicle.read());
        //printf("BTN RESET ALL:  %d \n", btn_reset_all.read());
        
        if (do_execute_main_task) {
    
            // visual feedback that the main task is executed, setting this once would actually be enough
            additional_led = 1;

            //state_machine
            switch(gryper_state_actual){

                case GRYPER_STATE_INIT:

                    if(!btn_start.read()){ //btn_start
                        // Start the loop
                        printf("SET START MODE\n");
                        enable_motors = 1;
                        gryper_state_actual = GRYPER_STATE_DETACH;                    

                    } else if(!btn_reset_vehicle.read()) {
                        // for the resetloop
                        printf("SET RESET MODE\n");
                        gryper_state_actual = GRYPER_STATE_RESET;
                        
                    } else if(btn_reset_all.read()) {
                        printf("SET RESET ALL MODE\n");
                        // Reset All
                        // muss negiert werden wenn der Button angeschlossen wird

                    } else if(btn_reserve){
                        //Reserve Button
                        printf("SET RESERVE MODE\n");
                    } else {
                        // set state to init state
                        gryper_state_actual = GRYPER_STATE_INIT;
                    }
                    break;


                case GRYPER_STATE_ARM_DOWN_1:
                    printf("Run STATE_ARM_DOWN_1\n");
                    /* Test1: Winkel ein bisschen kleiner machen
                    //  geschwindikeit i.o, übergang zu STATE 2 i.o */
                    angle_arm_down_1 = -0.25;

                    if (!arm_down){
                        positionController_M2.setDesiredRotation(angle_arm_down_1); // 1.0f = 360°, 0.222f= 80°   
                    }
                    if(positionController_M2.getRotation() <= angle_arm_down_1){  
                        arm_down = 1;  
                        gryper_state_actual = GRYPER_STATE_FORWARD_1;
                        printf("Set STATE_FORWARD_1\n");
                    }
                    break;
                
                case GRYPER_STATE_FORWARD_1:
                    printf("Run STATE_FORWARD_1\n");

                    positionController_M1.setDesiredRotation(convertDistanceToRotation(DISTANCE_1, WHEEL_DIAMETER)); // 1 Radumdrehung 94.25
                    printf("\n1RAD: %f\tact_pos %f\n", convertDistanceToRotation(DISTANCE_1, WHEEL_DIAMETER), positionController_M1.getRotation());

                    if(positionController_M1.getRotation() >= convertDistanceToRotation(DISTANCE_1, WHEEL_DIAMETER)-0.1f){
                        forward_1 = 1;
                        //gryper_state_actual = GRYPER_STATE_SET_ARM;
                        printf("Set STATE_SET_ARM\n");
                        gryper_state_actual = GRYPER_STATE_SET_ARM;
                    }
                    break;
                
                case GRYPER_STATE_SET_ARM:
                    printf("Run STATE_SET_ARM\n");
                    /*  lift the arm and set it to the hurdle 
                        1. drive arm in 0 position
                        2. calculate angle
                        3. drive angel
                        4. drive a little bit forward
                    */
                    
                    // 1. drive arm in 0 position
                    if (!arm_0_position) {
                        positionController_M2.setDesiredRotation(0.0f);
                    }
                    // Check if arm is in 0 position
                    if(positionController_M2.getRotation()>= -0.005f && positionController_M2.getRotation() <= 0.005f) {
                        printf("Reset 0 Position------------------------------------------------------------\n");
                        arm_0_position = 1;
                        
                    }               

                    // 2. calculate angle
                    angle_B = calcAngleSetArm();                                      // Fkt. get angle in [rad] -> 0.948
                    bogenlaenge = get_way_from_rad(angle_B);                          // in mm
                    //rotation = convertDistanceToRotation(bogenlaenge, ARM_LENGTH);  // = 0.301 rot
                    rotation = convertRadToRotation(angle_B) + 0.05;
                    printf("ANGLE: %f [m]\tROT: %f\tact_pos: %f\n",angle_B, rotation, positionController_M2.getRotation());

                    // 3. Drive angle 
                    if (arm_0_position) {
                        printf("SET_ARM: drive angle\n");
                        positionController_M2.setDesiredRotation(rotation);
                    }

                    // 4. Drive a little bit backwords at the end of the set-arm-rotation (0.2)
                    if (positionController_M2.getRotation() >= rotation-0.09  && !arm_set_m1) {    // RR: 0.1 muss noch angepasst werden
                        printf("SET_ARM: drive forward-----------------------------------\n");
                        act_pos_m1 = positionController_M1.getRotation();
                        positionController_M1.setDesiredRotation(act_pos_m1-0.16);            // Distanz welche M1 zurueck fährt
                        arm_set_m1 = 1;
                    }
                    
                    // Set further STEP  
                    if(positionController_M2.getRotation() >= rotation-0.01f){
                        printf("SET_ARM: set next step\n");
                        gryper_state_actual = GRYPER_STATE_ROTATE;
                    }                               
                    break;
                
                case GRYPER_STATE_ROTATE:
                    printf("Run STATE_ROTATE\n");
                    /* drive angle over the hurdle
                        1. calc angle/dist: (360 - 2*angle_B)
                        2. drive angle 
                        3. drive adjust angle
                    */

                    // 1. calculate angle
                    angle_rot =  M_PI + 2 * calcAngleSetArm(); // Fkt. get angle in [rad] -> 0.65
                    // bogenlaenge = get_way_from_rad(angle_rot); // in mm
                    //rotation = convertDistanceToRotation(bogenlaenge, ARM_LENGTH);
                    rotation = convertRadToRotation(angle_rot) - 0.20; // 0.05

                    //rotation = 0.5; // RR: Nur zu test zwecken
                    printf("ANGLE ROT: %f \tROT: %f act_pos: %f\n",angle_rot, rotation, positionController_M2.getRotation());
                    
                    // 2. drive angle
                    if (!rotate_full) {
                        positionController_M2.setDesiredRotation(rotation); // RR: + aktuelle Position
                        printf("ROTATE: gryper is rotating\n");
                    }
                    
                    // check if arm reaches the end position
                    printf("STATE ROTATE: Angle %f\n", positionController_M2.getRotation());
                    if (positionController_M2.getRotation() >= rotation - 0.001f) {
                        rotate_full = 1;
                        printf("ROTATE: full rotate OK\n");
                    }

                    // 3. adjust angle
                    angle_adjust = 0.01;
                    if (rotate_full && !adjust_ok) {
                        act_pos = positionController_M2.getRotation();
                        positionController_M2.setDesiredRotation(act_pos + angle_adjust);
                        adjust_ok = 1;
                        printf("ROTATE: gryper is adjusting\n");
                    }

                    // Set further STEP 
                    if (positionController_M2.getRotation() >= act_pos + angle_adjust && adjust_ok && rotate_full) {
                        //gryper_state_actual = GRYPER_STATE_DETACH;
                        gryper_state_actual = GRYPER_STATE_DETACH;
                    }
                    break;

                case GRYPER_STATE_DETACH:
                    printf("Run STATE_DETACH\n");
                    /*  lift the arm and detach it from the hurdle 
                        1. lift arm and drive backword
                        2. drive 5cm forward*/
                    angle_detach = 0.07;

                    // 1. Setze greifer waagrecht
                    if (!detach_ok) {
                        printf("DETACH: is detaching\n");
                        act_pos = positionController_M2.getRotation();
                        positionController_M2.setDesiredRotation(act_pos - angle_detach);
                        //osDelay(1000);
                        //printf("Delay is finish");
                        detach_ok = 1;
                    }
                    angle_detach_2 = 0.70;


                    // Ruetler
                     if (positionController_M2.getRotation() <= act_pos - angle_detach + 0.001 && detach_ok && !ruetler_ok ){
                         printf("START: RUETLER------\n");
                         i = 0;
                         range = 10;
                         one_step = 0.0;
                         r_backwards = 0, r_forwards = 0;
                         angle_detach_2 = 0.5;
                         

                         while (i < range) {
                            one_step = angle_detach_2/range;
                            printf("STEP: %d : onestep: %f : act_pos_save %f : act_pos : %f\n", i, one_step, act_pos, positionController_M2.getRotation());

                            if(!r_backwards && !r_forwards) {
                                printf("R: Forward: -------");
                                act_pos = positionController_M2.getRotation();
                                positionController_M2.setDesiredRotation(act_pos + one_step); 
                                r_forwards = 1;
                            }
                            
                            if(positionController_M2.getRotation() >= act_pos + one_step - 0.001f && !r_backwards && r_forwards) {
                                printf("R: Backward ---------\n");
                                act_pos = positionController_M2.getRotation();
                                positionController_M2.setDesiredRotation(act_pos - one_step + 0.03);
                                r_backwards = 1;
                                
                            }

                            if(i >= 2){
                                act_pos_m1 = positionController_M1.getRotation();
                                positionController_M1.setDesiredRotation(act_pos_m1 - 0.2);
                            }

                            if(positionController_M2.getRotation() <= act_pos - one_step + 0.03 + 0.001f && r_backwards){
                                printf("R: Nextstep ---------\n");
                                i += 1;
                                r_backwards = 0;
                                r_forwards = 0;
                            
                            
                            }
                         }
                         printf("Juhuuu de scheiss het klappet:)\n");
                         ruetler_ok = 1;
                         positionController_M2.setDesiredRotation(positionController_M2.getRotation());
                         positionController_M1.setDesiredRotation(positionController_M1.getRotation());
                         
                     }
                    /*
                    // detach
                    if (positionController_M2.getRotation() <= act_pos - angle_detach +0.001 && detach_ok && !detach_ok_2 && ruetler_ok){
                        positionController_M1.setDesiredRotation(convertDistanceToRotation(-100, WHEEL_DIAMETER) + positionController_M1.getRotation());
                        positionController_M2.setDesiredRotation(act_pos + angle_detach_2);
                        detach_ok_2 = 1;
                    }*/
                    
                    // 2. drive 5cm forward
                    printf("DETACH: act_pos_m1 %f\t", positionController_M1.getRotation() );
                    printf("DETACH: act_pos_m2 %f\n", positionController_M2.getRotation() );

                    if (ruetler_ok && detach_ok && !detach_forward_ok) {
                        printf("DETACH: drive forward---------------\n");
                        positionController_M1.setDesiredRotation(convertDistanceToRotation(120, WHEEL_DIAMETER) + positionController_M1.getRotation()); 
                        act_pos_m1 = positionController_M1.getRotation();
                        detach_forward_ok = 1;
                        // RR: evt. ganz nach hinten fahren bis an das hinderniss und nur von dort aus eine bestimmte distanz fahren
                    }
                    
                    // Set further STEP 
                    if (detach_forward_ok && detach_ok && positionController_M1.getRotation() >= act_pos_m1 + convertDistanceToRotation(50, WHEEL_DIAMETER) -0.01f) {
                        printf("DETACH: Finish\n");
                        //gryper_state_actual = GRYPER_STATE_ARM_DOWN_2;
                        gryper_state_actual = GRYPER_STATE_ARM_DOWN_2;
                    }
                    
                    break;

                case GRYPER_STATE_ARM_DOWN_2:
                    printf("Run STATE_ARM_DOWN_2\n");
                    angle_arm_down_1 = -0.25;

                    if(!arm_down_3) {
                        printf("Huere scheiss");
                        act_pos = positionController_M2.getRotation();
                        positionController_M2.setDesiredRotation(act_pos + angle_arm_down_1); // 1.0f = 360°, 0.222f = 80°
                        arm_down_3 = 1; 
                    }
                
                    if(positionController_M2.getRotation() <= act_pos + angle_arm_down_1 + 0.001f && arm_down_3){
                        printf("Arm down 2");
                        gryper_state_actual = GRYPER_STATE_FORWARD_2;
                        //gryper_state_actual = GRYPER_STATE_INIT;
                    }
                    break;
                    

                case GRYPER_STATE_FORWARD_2:
                    printf("Run STATE_FORWARD_2\n");

                    if(!forward_2){
                        act_pos_m1 = positionController_M1.getRotation();
                        positionController_M1.setDesiredRotation(act_pos_m1 + convertDistanceToRotation(DISTANCE_2, WHEEL_DIAMETER));
                        forward_2 = 1;
                    }
                    
                    printf("\n2RAD: %f\n", convertDistanceToRotation(DISTANCE_2, WHEEL_DIAMETER));

                    if(positionController_M1.getRotation() >= act_pos_m1 + convertDistanceToRotation(DISTANCE_2, WHEEL_DIAMETER)-0.1f && forward_2){
                        gryper_state_actual = GRYPER_STATE_INIT;
                        //gryper_state_actual = GRYPER_STATE_INIT;
                    }
                    break;

                case GRYPER_STATE_FINAL:
                    printf("Run STATE_FINAL\n");

                    if (!arm_down_2){ // Move Gryper back vertical to the car
                        positionController_M2.setDesiredRotation(1.0f); // 1.0f = 360° 
                        arm_down_2 = 1;
                        
                    }
                    if (positionController_M2.getRotation() >= 1.0f - 0.1f && arm_down_2){
                        gryper_state_actual = GRYPER_STATE_INIT;
                    }   
                    break;

                case GRYPER_STATE_RESET:
                    printf("Run STATE_RESET\n");

                    positionController_M1.setDesiredRotation(0.0f); //M1 auf Position 0.0 fahren
                    positionController_M2.setDesiredRotation(0.0f); //M2 auf position 0.0 fahren

                    arm_0_position = 0; arm_down = 0; arm_set_m1 = 0; arm_down_2 = 0; forward_1 = 0; forward_2 = 0; rotate_full = 0; 
                    adjust_ok = 0; detach_ok = 0; detach_ok_2 = 0; detach_forward_ok = 0; ruetler_ok = 0; r_backwards = 0; r_forwards = 0;arm_down_3 = 0;

                    if(positionController_M1.getRotation() <= 0.01f && positionController_M2.getRotation() <= 0.01f){
                        gryper_state_actual = GRYPER_STATE_INIT;
                    }
                    break;
            }

            //printf("\nEncoder M1: %3d\tPosition M1: %3.3f\tEncoder M2: %3d\tPosition M2: %3.3f", encoder_M1.read(), positionController_M1.getRotation(), encoder_M2.read(), positionController_M2.getRotation());

        } else {

            if (do_reset_all_once) {
                printf("\nALL DONE\n");
                do_reset_all_once = false;

                additional_led = 0;
               // positionController_M1.setDesiredRotation(0.0f);
                //positionController_M2.setDesiredRotation(0.0f);
            }            
        }

        // toggling the user led
        user_led = !user_led;

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}



void user_button_pressed_fcn() {
    // do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    if (do_execute_main_task) do_reset_all_once = true;
}

void start_button_pressed_fcn() {
    printf("START Button pressed\n");
    start_pressed = !start_pressed;
}

// General Functions
float convertDistanceToRadians(float distanceInMillimeters, float diameter) {
    float u = diameter * M_PI;
    return ((u / 2) * M_PI / distanceInMillimeters)*0.727; // 1.375 Getriebe (22/16)

}

float get_way_from_rad(float angle){
     return angle * ARM_LENGTH;
}


float convertDistanceToRotation(float distanceInMillimeters, float diameter) {
    return (distanceInMillimeters / (diameter * M_PI))*0.727;
}

// Functions for STEP 3
double calcAngleSetArm(void) {
    return asin((HURDLE_HIGHT + GRYPER_HIGHT - AXE_HEIGHT)/ARM_LENGTH);
}


double convertRadToRotation(double angle) {
    // Angle in [rad]
    return (1 / (2 * M_PI) * angle) *2 ;
}
