/* ========================================
 *
 * Copyright ECE3091 Group 5, 2019
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF ECE3091 Group 5
 *
 * ========================================
*/
#include "project.h"
#include <stdlib.h>
#include <stdio.h>

char test_string[200]; //debug purposes

//Ultrasonic Sensor
float Distance = 100, Distance2 = 100, Distance3 = 100, Distance4 = 100;
float DistanceOld = 0, Distance2Old = 0, Distance3Old = 0, Distance4Old = 0;
uint16 count = 0, count2 = 0, count3 = 0, count4 = 0;
int flagFront = 0, flagLeft = 0, flagRight = 0, flagStop = 0, flagMove = 0;
int flagPrelim = 0, flagPuck = 0, flagBurst = 0, flagStart = 0;
int i=0;

int cnt_left, cnt_right, diff_enc, stopint, DistanceB;
uint16 encoder = 0;


// Servo
#define five_PC 50
#define ten_PC 100
char ambientC[10];



// Servo Motor
    void ServoFunction(int startAngle,int stopAngle){
        
        //PWM_SERVO_Enable();
        //PWM operates on 5%(-90 or to the left) to 10%(+90 or to the right) duty cycle
         for(int i = startAngle; i<stopAngle;i++){
            PWM_t_WriteCompare(i);
            
            UART_PutString("1"); 
            CyDelay(100);
       
        }
        //PWM_SERVO_Stop();
    }
    
 //Ultrasonic Sensor
    // send high logic to trigger to 10us
    // read high level time of echo
    // divide time (us) by 58 for cm
    // will not trigger if flagStop raised
    void Burst1(){ //send out ultrasonic pulses
        if (ECHO1_Read()==0){ // && flagStop == 0){
            UART_PutString("Burst 1\n");
            TRIG1_Write(1);
            CyDelayUs(10);
            TRIG1_Write(0);  
        }
    CyDelay(100);
    }
    
    void Burst2(){ //send out ultrasonic pulses
        if (ECHO2_Read()==0 && flagStop == 0){ 
           UART_PutString("Burst 2\n");
            TRIG2_Write(1);
            CyDelayUs(10);
            TRIG2_Write(0);  
        }
    CyDelay(100);
    }
        
    void Burst3(){ //send out ultrasonic pulses
        if (ECHO3_Read()==0 && flagStop == 0){
           UART_PutString("Burst 3\n");
            TRIG3_Write(1);
            CyDelayUs(10);
            TRIG3_Write(0);  
        }
    CyDelay(100);
    }
    
    void Burst4(){ //send out ultrasonic pulses
        if (ECHO4_Read()==0 && flagStop == 0){
           UART_PutString("Burst 4\n");
            TRIG4_Write(1);
            CyDelayUs(10);
            TRIG4_Write(0);  
        }
    CyDelay(100);
    }


// Movement
     void stop(){
        PWM1_OUT_Write(0);
        PWM1_OUT2_Write(0);
        PWM2_OUT_Write(0);
        PWM2_OUT2_Write(0);
        flagMove = 0;
    }
    // Each wheel has a PWM1 & PWM2 signal, OUT & OUT2 are the separate wheels
    // need PWM_Enable and Stop so that can change between directions and doesn't run continuously
    void MoveBack(){ //check direction
        // straight line
        QuadDec_L_SetCounter(0);
        QuadDec_R_SetCounter(0);
        cnt_left = 0;
        cnt_right = 0;
        Distance2 = 0;
        Burst2(); // Distance2 is not resetting with this Burst but is still working as desired
        
        // arena 120cm *120cm, robot 30cm long
        DistanceB = 80 - (Distance2 + 30);
        if (DistanceB > 10){
            // PWM1 pin write - OUT & OUT2 must be complementary
            PWM1_OUT_Write(0);
            PWM1_OUT2_Write(1);
            // PWM2 pin write - inverse of PWM1, OUT & OUT2 complementary
            PWM2_OUT_Write(1);
            PWM2_OUT2_Write(0);
        }
        
        stopint = (DistanceB-5)*200; // move back the same distance it just moved forward
        sprintf(test_string, "stop count = %d\n", stopint);
        UART_PutString(test_string);
        
        while (abs(cnt_right) <= stopint && abs(cnt_left) <= stopint){
            cnt_left = QuadDec_L_GetCounter();
            cnt_right = QuadDec_R_GetCounter(); 
        }
        stop();
        sprintf(test_string, "left = %d\n", cnt_left);
        UART_PutString(test_string);
        sprintf(test_string, "right = %d\n", cnt_right); // value will be negative
        UART_PutString(test_string);
        Burst2();
    }

    void MoveForward(){ //check direction
        // straight line
        QuadDec_L_SetCounter(0);
        QuadDec_R_SetCounter(0);
        cnt_left = 0;
        cnt_right = 0;
        //Burst3();
        
        while(i<3){
        
        
        
        Burst2(); // check is safe to move a distance then use encoder to make sure distance not exceeded
        Burst3();// need to incorporate encoder to read more frequently - separate function? 
                            // -> stop when counter reaches appropriate count = polling loop?
        Burst4();
       // Correct();
        
        
        
        if (Distance2 > 10){ // may add puck and stop flags, increase distance to account for where pucks are
            // PWM1 pin write - OUT & OUT2 must be complementary
             PWM_WriteCompare(2);
            PWM1_OUT_Write(1);
            PWM1_OUT2_Write(0);
            // PWM2 pin write - inverse of PWM1, OUT & OUT2 complementary
            PWM2_OUT_Write(0);
            PWM2_OUT2_Write(1);
            flagMove = 1;
        }
        
        stopint = (Distance2-5)*200;
        sprintf(test_string, "stop count = %d\n", stopint);
        UART_PutString(test_string);
        
        while (abs(cnt_right) <= stopint && abs(cnt_left) <= stopint && flagMove == 1){ // currently won't exit loop if less than 10cm from wall as encoders stay 0
        cnt_left = QuadDec_L_GetCounter();
        cnt_right = QuadDec_R_GetCounter(); 
        //Burst2();
        }
        stop();
        sprintf(test_string, "left = %d\n", cnt_left);
        UART_PutString(test_string);
        sprintf(test_string, "right = %d\n", cnt_right); // value will be negative
        UART_PutString(test_string); 
    }
        i++;
    }

    void MoveRight(){ //check direction
        // turning on axis
        // PWM1 pin write
        PWM1_OUT_Write(1);
        PWM1_OUT2_Write(1);
        // PWM2 pin write
        PWM2_OUT_Write(0);
        PWM2_OUT2_Write(0);
    }

    void MoveLeft(){ //check direction
        // turning on axis
        // PWM1 pin write
        PWM1_OUT_Write(0);
        PWM1_OUT2_Write(0);
        // PWM2 pin write
        PWM2_OUT_Write(1);
        PWM2_OUT2_Write(1);
    }
    
    void CorrectLeft(){
        PWM_WriteCompare(2);
        PWM1_OUT_Write(1);
        PWM2_OUT_Write(0);
    }
    
    void CorrectRight(){
        PWM_WriteCompare(2);
        PWM1_OUT2_Write(0);
        PWM2_OUT2_Write(1);
    }
    



    CY_ISR(TimerUS1_Handler) //read distance from ultrasonic sensor - front low (puck)
    {
        Timer_1_ReadStatusRegister();
        Distance = 0; 
        count = 0;
        count = Timer_1_ReadCounter();
        Distance = (float)(65535 - count)/58.0; //cm
        
        sprintf(test_string, "puck = %0.1f\n", Distance);
        UART_PutString(test_string);
        //ServoFunction();
        //CyDelay(1000);
            
        if (Distance <= 10){ 
            //flagFront = 1; // raise both flags as will need to turn once stop removed
            flagStop = 1; // must add check everywhere
            stop();
            UART_PutString("Stop, pucks detected\n");
            flagPuck = 1;
            UART_PutString("Do work with puck\n");
            ServoFunction(40,70); // servo will open and close fully on repeat 
            
            CyDelay(5000);
  
            flagPuck = 0;
            flagStop = 0;
            // add other functions to perform once pucks are detected - once complete, remove flagStop and flagPuck
            
        }
         // check distance from front puck to wall
//        else if (Distance > 10 && Distance <= 40){ // && flagStop == 0) {
//             add flag and work out how to proceed slowly
//            flagStop = 0;
//            flagFront = 0;
//            UART_PutString(" possible pucks - forward slowly\n");
//            flagPuck = 2;
//            PWM_WriteCompare(46); // check value
//            MoveForward();     
//        } 
//        else if (Distance > 10 && flagFront != 0){
//         //   PWM_WriteCompare(30);
//            UART_PutString("no pucks but wall\n");   
//            MoveBack();
//        }
    }

    CY_ISR(TimerUS2_Handler) //read distance from ultrasonic sensor - front high (wall)
    {
        Distance2 = 0; 
        count2 = 0;
        
        Timer_2_ReadStatusRegister();
        
        count2 = Timer_2_ReadCounter();

        Distance2 = (float) (65535 - count2)/58.0; //cm
        
        sprintf(test_string, "front = %0.1f\n", Distance2);
        UART_PutString(test_string);
        //CyDelay(1000);
        /*    
        if (Distance2 <= 30 && flagStop == 0){ // check turn radius now that turns on centre // flagstop?
            flagFront = 1;

            UART_PutString(" wall front - turn\n"); // trigger burst(side)?
            // ?prelim: use flags to follow set path- eg if (flag1) turn right, if (flag2,3) turn left...
                // turn right (first movement below), move out of home base, turn left on detection of wall, move forward until detect wall or pucks,
                // turn left if wall, stop if pucks, after picked up puck, turn left, move forward(?), then turn right, 
                // move forward until wall, turn left, foward until wall, turn left then move forward until distance
                // front ~= 50cm (inside home base)
            // ?add later: sense each side, raise flag which one is closer, turn other way
            if (flagPrelim == 1){
                MoveLeft();
                flagPrelim = 2;
            }
            else if (flagPrelim == 2){ // check distance condition
                MoveLeft();
                if (flagPuck == 0){
                    flagPrelim = 3;
                }
                else if (flagPuck == 1){
                    flagPrelim = 5;
                }
            }
            else if (flagPrelim == 3 && flagPuck == 0){ // check condition inside this distance range
                MoveLeft();
                flagPrelim = 4;
            }
            // moveRight
            else if (flagPrelim == 5){
                MoveLeft();
                flagPrelim = 6;
            }
        
        else if (flagPrelim == 6){
            //if (Distance2 > 20){
            PWM_WriteCompare(35);
                //MoveForward();
                CyDelay(500);
            //}
           // else {
                MoveLeft();
                flagPrelim = 7;
           // }
            }
        }
        else if (flagPrelim == 4){
            MoveRight();
            flagPrelim = 5;
        }
        else if (flagPrelim == 7){
            if (Distance2 > 50){
                //MoveForward();   
            }
            else {
                stop();
                flagStop = 1;
                UART_PutString("Home base\n");
            }
        }
        else if (flagStop == 0 && flagPrelim != 0 && flagPuck == 0) {
            flagFront = 0;
            UART_PutString(" forward\n");
          
               //PWM_WriteCompare(30);
            
            //MoveForward();
        }
        
        sprintf(test_string, "flagPrelim = %d\n", flagPrelim);
        UART_PutString(test_string);
        */
    }
    
    CY_ISR(TimerUS3_Handler) //read distance from ultrasonic sensor right (not connected yet)
    {
        Timer_3_ReadStatusRegister();
        Distance3 = 0; 
        count3 = 0;
        count3 = Timer_3_ReadCounter();
        Distance3 = (float) (65535 - count3)/58.0; //cm
        
        sprintf(test_string, "distance right = %0.1f\n", Distance3);
        UART_PutString(test_string);
            
        if (Distance3 <= 25 && flagLeft != 0 && flagStop == 0){ //check turn radius, change condition (add another statement?)
            stop();
            flagRight = 1;
            UART_PutString("obstacle right - Turn left\n");
           // MoveLeft();
            //MoveLeft();    
        }
        else if (flagStop == 0) {
            flagRight = 0;
            UART_PutString("safe to turn right\n");  
        }            
    }
    
    
    CY_ISR(TimerUS4_Handler) //read distance from ultrasonic sensor left
    {
        Distance4Old = Distance4;
        Distance4 = 0; 
        count4 = 0;
        Timer_4_ReadStatusRegister();
        count4 = Timer_4_ReadCounter();
        Distance4 = (float) (65535 - count4)/58.0; //cm
        
        sprintf(test_string, "distance left = %0.1f\n", Distance4);
        UART_PutString(test_string);
        
                if ((Distance4Old - Distance4 >= 2) && Distance3 - Distance3Old >=2){
                    stop();
                    CyDelay(100);
                    CorrectRight();
                }
                else if ((Distance4 - Distance4Old >=2)&& Distance3Old - Distance3 >= 2){
                    stop();
                    CyDelay(100);
                    CorrectLeft();
                }   
       
             sprintf(test_string, "Distance4 = %0.1f\n", Distance4);
            UART_PutString(test_string);
            
            sprintf(test_string, "Distance4Old = %0.1f\n", Distance4Old);
            UART_PutString(test_string);
        
       } 
         
       
        
//        if (Distance4 <= 25 && flagRight != 0 && flagStop == 0){ // check turn radius, change condition (add another statement?)
//            stop();
//            flagLeft = 1;
//            UART_PutString("obstacle left - Turn right\n");
//            //MoveRight();
//            //MoveLeft();    
//        }
//        else if (flagStop == 0){
//            flagLeft = 0;
//            UART_PutString("safe to turn left\n");  
//        }           
    
    
// function that takes both old and new distance readings from  left and right ultrasonics and compares them.
        // if one side is staying constant, thats okay, as might have detected the obstacle on the other side
        //  but if both have deviated, correct
        // need to decide where is this called and how
/*void Correct(){
    if ((Distance4Old - Distance4 >= 2) && Distance3 - Distance3Old >=2){
        stop();
        CyDelay(100);
        CorrectRight();
    }
    else if ((Distance4 - Distance4Old >=2)&& Distance3Old - Distance3 >= 2){
        stop();
        CyDelay(100);
        CorrectLeft();
    }   
   
    sprintf(test_string, "Distance4 = %d\n", Distance4);
        UART_PutString(test_string);
        
        sprintf(test_string, "Distance4Old = %d\n", Distance4Old);
        UART_PutString(test_string);
}*/

int main(void)
{
    /* Place your initialization/startup code here */
    UART_Start();
    
    // Ultrasonic Module
    Timer_1_Start();
    Timer_2_Start();
    Timer_3_Start();
    Timer_4_Start();

    // Motor Control
    // EN gets output of PWM - need to find correct duty cycle and period //
    PWM_Start();
    ENB_Write(0);

    
    QuadDec_R_Start();
    QuadDec_L_Start();
    
    // Servo
    PWM_SERVO_Start();
    PWM_t_Start();
    PWM_t_WriteCompare(40);

   
    //Burst();
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    isr_1_StartEx(TimerUS1_Handler);
    isr_2_StartEx(TimerUS2_Handler);
    isr_3_StartEx(TimerUS3_Handler);
    isr_4_StartEx(TimerUS4_Handler);

    UART_PutString("Reset\n");
    
    //initialise servo angle to closed/open
    
    
    for(;;)
    {   
        // Ultrasonic Sensor
                //consider push button start for now
                //eg. set manoeuvres, then move around obstacle, then move to find correct puck
                        
            if (SW1_Read() == 0){
                UART_PutString("Start\n");
                PWM_WriteCompare(2); //adjust speed using WriteCompare - higher value is faster, up to period value
                MoveForward();
                //CyDelay(1000);
                //MoveBack();
                //CyDelay(2000);
                //stop();
                //CyDelay(1000);
                //CorrectRight();
                //UART_PutString("correct");
                //CyDelay(1000);
                //stop();
                //MoveRight();
                //flagPrelim = 1;
                //Burst2();
                flagBurst = 0;
            }
            
            
            if (flagBurst == 1){ // burst forward when moving forward etc, removes polling loop aspect
                //Burst1();
                //CyDelay(500);
                Burst2();
           
                //CyDelay(500);
                //Burst3();
                //CyDelay(200);
                //Burst4();
                //CyDelay(500);   
            }
            

            // want to add way to become parallel again - encoders and ultrasonics
        
        // Motor Encoder
            //encoderA1 = Counter_ReadCounter();
            //encoder = ENC_A_Read();
            //float readenc = encoder;
            //sprintf(test_string, "Encoder = %d", encoderA1);
            //UART_PutString(test_string);
            //UART_PutString("\n");
//            MoveForward();
//            encoder = QuadDec_1_GetCounter();
//            sprintf(test_string, " encoder = %d\n", encoder);
//            UART_PutString(test_string);
            
            
            
        // Servo Motor
    }
}

/* [] END OF FILE */

