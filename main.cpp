#include "main.h"
#include <VL53L0X.h>


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


// define range sensor
//DigitalOut hsens_power(PB_7,1); // power on hSens terminals on CORE2 romrap: funktioniert nicht
//DevI2C i2c(PB_9, PB_7); // SDA, SC
//VL53L0X sensor(PB_9, PB_8); //PB_9 = Data, PB_8 = Clock:

// ------------- Motoren -------------
const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                // 6.0f V if you only use one battery pack
DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
// motor M2 (position controlled)
FastPWM pwm_M2(PB_PWM_M2); // create FastPWM object to command motor M1

const float gear_ratio_M2 = 78.125f; // gear ratio
const float kn_M2 = 180.0f / 12.0f;  // motor constant [rpm/V]
// it is assumed that only one motor is available, there fore
// we use the pins from M1, so you can leave it connected to M1
DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio_M2, kn_M2, voltage_max);

// motor M1 (speed controlled, openloop)
FastPWM pwm_M1(PB_PWM_M1); // create FastPWM object to command motor M1

// ------------- Sensoren -------------
I2C         i2c(PB_9, PB_8);
VL53L0X     vl_sensor(&i2c);
DigitalOut  vl_shutdown(PB_12); //romrap: eventuell löschen



int main()
{
    printf("Program init\n\n\r");
    //DevI2C i2c(PB_9, PB_8);
    Timer zeit;
    //PinName NC_1 = NC;
    // Das könnte die Speisung (PowerPin) des Sensors sein
    //DigitalOut gpio0(PB_12); //romrap: nochmal überprüfen
    int8_t dev_addr = 0x52;
    uint32_t distance = 0;
    
    //I2C         i2c(PB_9, PB_8);
    //VL53L0X     vl_sensor(&i2c);
    //DigitalOut  vl_shutdown(PB_12); //romrap: eventuell löschen
    //Serial      usb(USBTX, USBRX, 115200);

    //vl_shutdown = 1;  //turn VL53L0X on
    //vl_sensor.setDeviceAddress(0x52);
    vl_sensor.init();
    //vl_sensor.setModeContinuous();    
    //vl_sensor.startContinuous();
    
    // while loop for testcase
    //while(0)
    //{
    //    wait_us(10000);
    //    printf("%4imm\r\n", vl_sensor.getRangeMillimeters());
    //}

    user_button.fall(&toggle_do_execute_main_fcn);
    

    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, there for
    //Timer main_task_timer;              // create Timer object which we use to run the main task every main_task_period_ms

    DebounceIn endstop1(PB_2, PullDown);
    DebounceIn endstop2(PC_8, PullDown);
    DebounceIn endstop3(PC_6, PullDown);

    // led on nucleo board
    DigitalOut user_led(USER_LED);

    // additional led
    // create DigitalOut object to command extra led, you need to add an aditional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via a resistor
    DigitalOut led1(PB_9);

    // ============  Initialisiere Motoren ============
    enable_motors = 1; // setting this once would actually be enough

    // motor M2 (position controlled)
    // enable the motion planner for smooth movement
    motor_M2.enableMotionPlanner(true);
    // limit max. acceleration to half of the default acceleration
    motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.2f);
    //motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.5f);


    // motor M1 (speed controlled, openloop)
    //pwm_M1.write(0.0f); //   0V is applied to the motor
    //pwm_M1.write(0.5f); //  12V is applied to the motor
    //pwm_M1.write(0.0f); // -12V is applied to motor





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
    const int CAPTOR_STATE_TEST = 20;
   
    int captor_state_actual = CAPTOR_STATE_INIT;
    int act_step_activ = 0;
    bool check_decap = false;



    // start timer
    Timer main_task_timer;              // create Timer object which we use to run the main task every main_task_period_ms
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();
        //printf("THIS is the Mainloop3: %d\n", do_execute_main_task);

        if (do_execute_main_task) {

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;

            // ------------- State machine -------------
            switch(captor_state_actual){

                case CAPTOR_STATE_INIT:
                    printf("CAPTOR_STATE_INIT\n");
                    motor_M2.setRotation(motor_M2.getRotation());   //motor stop
                    pwm_M1.write(0.5f);                             //motor stop
                    sum_endstop = 0;
                    sum_endstop |= (!(endstop1.read()) << 2); 
                    sum_endstop |= (!(endstop2.read()) << 1); 
                    sum_endstop |= !(endstop3.read()); 
                    //sum_endstop = CAPTOR_STATE_TEST;    //to Force Step
                    //printf("Endstop1: %d, Endstop2: %d, Endstop3: %d\n", endstop1.read(), endstop2.read(), endstop3.read());
                    //printf("Summeendstops: %d \n", sum_endstop);
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
                    if(sum_endstop == 0 or (act_step_activ and sum_endstop != 0)) {
                        if (act_step_activ and sum_endstop == 0) {
                            act_step_activ  = 0;
                        }
                        
                        captor_state_actual = CAPTOR_STATE_000;
                        printf("Schritt CAPTOR_STATE_000 wurde aktiviert\n");
                        break;
                        
                    } else if (sum_endstop == 4 and !act_step_activ){
                        pwm_M1.write(0.5f);                             //motor stop
                        captor_state_actual = CAPTOR_STATE_100;
                        printf("Schritt CAPTOR_STATE_100 wurde aktiviert\n");
                        break;

                    } else if (sum_endstop == 6 and !act_step_activ){
                        pwm_M1.write(0.5f);                             //motor stop
                        captor_state_actual = CAPTOR_STATE_110;
                        printf("Schritt CAPTOR_STATE_110 wurde aktiviert\n");
                        break;

                    }else if (sum_endstop == 7 and !act_step_activ){
                        pwm_M1.write(0.5f);                             //motor stop
                        captor_state_actual = CAPTOR_STATE_111;
                        printf("Schritt CAPTOR_STATE_111 wurde aktiviert\n");
                        break;

                    }else if (sum_endstop == 3 and !act_step_activ){
                        pwm_M1.write(0.5f);                             //motor stop
                        captor_state_actual = CAPTOR_STATE_011;
                        printf("Schritt CAPTOR_STATE_011 wurde aktiviert\n");
                        break;

                    }else if (sum_endstop == 1 and !act_step_activ){
                        pwm_M1.write(0.5f);                             //motor stop
                        captor_state_actual = CAPTOR_STATE_001;
                        printf("Schritt CAPTOR_STATE_001 wurde aktiviert\n");
                        break;

                    }else if (sum_endstop == 5 and !act_step_activ){
                        pwm_M1.write(0.5f);                             //motor stop
                        captor_state_actual = CAPTOR_STATE_101_error;
                        printf("Schritt CAPTOR_STATE_101_error wurde aktiviert\n");
                        break;

                    }else if (sum_endstop == 2 and !act_step_activ){
                        pwm_M1.write(0.5f);                             //motor stop
                        captor_state_actual = CAPTOR_STATE_010_error;
                        printf("Schritt CAPTOR_STATE_010_error wurde aktiviert\n");
                        break;

                    } else if (sum_endstop == 20){
                        pwm_M1.write(0.5f);                             //motor stop
                        captor_state_actual = CAPTOR_STATE_TEST;
                        printf("Schritt CAPTOR_STATE_TEST wurde aktiviert\n");
                    } else {
                        printf("Schritt CAPTOR_STATE_INIT wurde aktiviert\n");
                        captor_state_actual = CAPTOR_STATE_INIT;
                    }
                    break;



                case CAPTOR_STATE_TEST:
                    printf("Run TEST: END\n");
                    // Test something
                    printf("Run TEST: END\n");
                    captor_state_actual = CAPTOR_STATE_INIT;
                    break;

                case CAPTOR_STATE_000:
                    drive_belt_forward();
                    printf("Run CAPTOR_STATE_000\n");
                    captor_state_actual = CAPTOR_STATE_INIT;
                    break;
                
                case CAPTOR_STATE_100:
                    read_liquid_level();
                    drive_belt_forward();
                    act_step_activ = 1;
                    captor_state_actual = CAPTOR_STATE_INIT;
                    break;
                
                case CAPTOR_STATE_110:
                    read_liquid_level();
                    check_decap =  decap();
                    drive_belt_forward();
                    act_step_activ = 1;
                    captor_state_actual = CAPTOR_STATE_INIT;
                    break;
                
                case CAPTOR_STATE_111:
                    read_liquid_level();
                    check_decap =  decap();
                    read_liquid_level();
                    drive_belt_forward();
                    act_step_activ = 1;
                    captor_state_actual = CAPTOR_STATE_INIT;
                    break;
                
                case CAPTOR_STATE_011:
                    check_decap =  decap();
                    read_liquid_level();
                    drive_belt_forward();
                    act_step_activ = 1;
                    captor_state_actual = CAPTOR_STATE_INIT;
                    break;
                
                case CAPTOR_STATE_001:
                    read_liquid_level();
                    drive_belt_forward();
                    act_step_activ = 1;
                    printf("Motor drive Forward\n");
                    captor_state_actual = CAPTOR_STATE_INIT;
                    break;
                
                case CAPTOR_STATE_010_error:
                    // Error case
                    captor_state_actual = CAPTOR_STATE_INIT;
                    break;
                
                case CAPTOR_STATE_101_error:
                    // Error case
                    captor_state_actual = CAPTOR_STATE_INIT;
                    break;
                
                case CAPTOR_STATE_error:
                    // Error case
                    captor_state_actual = CAPTOR_STATE_INIT;
                    break;
                
                default:
                    // do nothing 
                    printf("STEP DEFAULT\n");
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
        //int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        //thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
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
bool read_cap_color(void){
    // return if level is correct
    return true;
}


bool decap(void){
    float rot_decap = 3.0f;
    bool decap_ok = 0;
    bool tornado_is_down = 0;
    bool tornado_is_up = 1;

    while (!decap_ok) {
        //printf("Position: %f\n", motor_M2.getRotation());
        if (tornado_is_up) {
            motor_M2.setRotation(rot_decap);
            tornado_is_up = 0;
        }

        if(motor_M2.getRotation() >= rot_decap-0.01f and !tornado_is_up){
            tornado_is_down = 1;
            motor_M2.setRotation(0.0f);
        }

        if(tornado_is_down and motor_M2.getRotation() <= 0.01f){
            decap_ok = 1;
            tornado_is_up = 1;
        }
        wait_us(1000);
    }
    return true;
}


bool read_cap_coulor(){
    return true;
}
// Decapper
bool drive_belt(void);
// return 1 if decapping is succesful
// return 0 if decapping is not succesful

// Ultrasonic / liquid level
bool read_liquid_level() {
    // return True if level is correct
    int counter = 0;
    uint16_t sum_liq_level = 0;
    printf("FUN: read liquidlevel: START\n");
    vl_sensor.setModeContinuous();
    vl_sensor.startContinuous();

    while(counter < 100) {
        wait_us(10000);
        if (vl_sensor.getRangeMillimeters() >= 2000) {
            counter = counter;
        }else {
            sum_liq_level +=  vl_sensor.getRangeMillimeters();
            counter++;
        }
        printf("%4d mm Act: %4d mm\r\n", sum_liq_level/counter, vl_sensor.getRangeMillimeters());
    }
    sum_liq_level /= counter;
    printf("FUN: read liquidlevel: END %d\n", sum_liq_level);
    return true;
}

// Foerderband fahren
bool drive_belt_forward(){
    pwm_M1.write(1.0f);
    printf("Motor drive Forward\n");
    return true;
}

bool drive_belt_backward() {
    pwm_M1.write(0.0f);
    printf("Motor drive Backward\n");
    return true;
}

// show Error messages on LED's
//void show_LED(int error_code);