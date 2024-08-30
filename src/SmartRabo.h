#ifndef SMART_RABO_H
#define SMART_RABO_H

#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

#include <EEPROM.h>




class SmartRabo {
public:
    SmartRabo( char* ssid,  char* password,  char* agent_ip);
    void setup();
   
    void initialize();  
    void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
    void loop(volatile int &posR, volatile int &posL, float wr, float wl, float &v1FiltR,  float &v1FiltL);

    
private:
    
    
    float deltaT;
    long loop_timer;

    //Controllr variable
    float ePreviousR; 
    float eIntegralR; 
    float eDerivativeR;
    float eR;
    
    float ePreviousL; 
    float eIntegralL; 
    float eDerivativeL;
    float eL;
    
    
    float kpR=50; 
    float kdR=30;
    float kiR=40; 
    
    
    float kpL=50;
    float kdL=30;
    float kiL=40; 
    
    float uR,uL ;
    int pwmr,pwml;
    
    //Filter
    float velocity1R;
    float velocity1L;
    
    float v1PrevR;
   
    float v1PrevL;

   
    //Motor deriver Pin
    uint enPinL;
    uint IN1PinL;
    uint IN2PinL;
    
    
    uint enPinR;
    uint IN1PinR;
    uint IN2PinR;

    const int EEPROM_SIZE = 64;
    char storedPassword[32];  // Buffer to store the password read from EEPROM

    char* ssid_;
    char* password_;
    char* agent_ip_;
 

};

#endif // SMART_RABO_H
