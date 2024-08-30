#include "SmartRabo.h"
#include <micro_ros_arduino.h>
#include <ESP32Servo.h>


// create servo object to control a servo
Servo myservo;  
int servoPin = 13;


// create myRobot object to connect to Smart-Rabo
SmartRabo myRobot("Rana", "k146617829", "192.168.43.173");

rcl_subscription_t LEDs_subscriber;
std_msgs__msg__Int8 LEDs_msg;

rcl_subscription_t servo_subscriber;
std_msgs__msg__Int8 servo_msg;

rcl_publisher_t left_encoder_publisher;
std_msgs__msg__Int32 left_encoder_msg;


rcl_publisher_t right_encoder_publisher;
std_msgs__msg__Int32 right_encoder_msg;

rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;

rcl_publisher_t sonar_left_publisher;
std_msgs__msg__Int32 sonar_left_msg;

rcl_publisher_t sonar_right_publisher;
std_msgs__msg__Int32 sonar_right_msg;

rcl_publisher_t sonar_front_publisher;
std_msgs__msg__Int32 sonar_front_msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;



float wR =0.0; //Speed Set_point 10
float wL =0.0; //Speed Set_point 10
    

#define LEFT_LED_PIN 32
#define RIGHT_LED_PIN 27


// Encoder Pins
#define LeftEncoder_C1 22
#define LeftEncoder_C2 23


#define RightEncoder_C1 4
#define RightEncoder_C2 15


// Variables to store encoder states
volatile int LeftEncoderCount = 0;
volatile int lastEncoderAStateL = LOW;

volatile int RightEncoderCount = 0;
volatile int lastEncoderAStateR = LOW;

void LeftEncoderCallback();
void RightEncoderCallback();


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


volatile int encoderPosL = 0;
volatile int encoderPosR = 0;

float vR = 0;
float vL = 0;

//Ultrasonic Sensor

// Define pins
#define TRIG_PIN_FRONT 12
#define ECHO_PIN_FRONT 14
 int distance_front=0;

//___________________________________________________
//Functions
void error_loop(){
  while(1){
    delay(100);
  }
}

//_______
int limitToMaxValue(int value, int maxLimit) {
  if (value > maxLimit) {
    return maxLimit;
  } else {
    return value;
  }
}

//_______

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    right_encoder_msg.data = RightEncoderCount;
    left_encoder_msg.data = LeftEncoderCount;
    
    
    // Publish the distance
  
    sonar_front_msg.data = distance_front;
 
    
    RCSOFTCHECK(rcl_publish(&left_encoder_publisher, &left_encoder_msg, NULL));
    RCSOFTCHECK(rcl_publish(&right_encoder_publisher, &right_encoder_msg, NULL));
   
    RCSOFTCHECK(rcl_publish(&sonar_front_publisher, &sonar_front_msg, NULL));
    //Serial.print("Distance right: ");
    //Serial.print(distance_right);
   ///Serial.println(" cm");

    
  }
}

//_______

void LEDs_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msgin;

  int8_t value = msg->data;

  switch (value) {
    case 0:
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;
    case 1:
      digitalWrite(LEFT_LED_PIN, HIGH);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;
    case 2:
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, HIGH);
      break;
    case 3:
      digitalWrite(LEFT_LED_PIN, HIGH);
      digitalWrite(RIGHT_LED_PIN, HIGH);
      break;
    default:
      break;
  }
}

//_______
void servo_callback(const void* msgin) {
  const std_msgs__msg__Int8* msg = (const std_msgs__msg__Int8*)msgin;
  int8_t angle = msg->data;
  int servo_position;
  servo_position = limitToMaxValue(angle, 50);
  myservo.write(servo_position);
}

//_______
//twist message callback
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // Calculate Motor speeds based on Twist message 
  float linear =msg->linear.x;
  float angular=msg->angular.z;
  //clculate indivitual motor speed *********************
  wL =((2*linear+angular*0.134)/(2*0.03));
  wR =((2*linear-angular*0.134)/(2*0.03));
}


float calculateDistance(int TRIG_PIN, int ECHO_PIN) {
  // Clears the trigPin condition
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  float dis=(pulseIn(ECHO_PIN, HIGH)/2.0)*0.0343;

      if(dis>100){
      dis=100;
    }
  return dis;
}




void setup() {

  myRobot.setup();

  Serial.begin(115200);
  // Initialize pins
 
  pinMode(TRIG_PIN_FRONT, OUTPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);



  pinMode(LEFT_LED_PIN, OUTPUT);
  digitalWrite(LEFT_LED_PIN, HIGH);  

  pinMode(RIGHT_LED_PIN, OUTPUT);
  digitalWrite(RIGHT_LED_PIN, HIGH);  
  delay(2000);

  pinMode(LeftEncoder_C1, INPUT_PULLUP);
  pinMode(LeftEncoder_C2, INPUT_PULLUP);
  pinMode(RightEncoder_C1, INPUT_PULLUP);
  pinMode(RightEncoder_C2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LeftEncoder_C1), LeftEncoderCallback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightEncoder_C1), RightEncoderCallback, CHANGE);

 

// Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object

  
 // Initialize MicroROS
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "SMART_RABO", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &LEDs_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "LEDs"));

    //servo subscriber
  RCCHECK(rclc_subscription_init_default(
      &servo_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
      "/servo"));

        // create twist subscriber
  RCCHECK(rclc_subscription_init_default(
    &twist_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
    &left_encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_motor_ticks"));

  RCCHECK(rclc_publisher_init_default(
    &right_encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "right_motor_ticks"));





  RCCHECK(rclc_publisher_init_default(
    &sonar_front_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "distance_front"));



   // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
    

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context,4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &LEDs_subscriber, &LEDs_msg, &LEDs_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));

  left_encoder_msg.data = 0;
  right_encoder_msg.data = 0;
}

void loop() {

distance_front = (int) calculateDistance(TRIG_PIN_FRONT,ECHO_PIN_FRONT);
myRobot.loop(encoderPosR,encoderPosL,wR,wL,vR,vL);

  Serial.print(vR);//v1FiltR
  Serial.print(" ");
  Serial.print(vL);//v1FiltR
  Serial.print(" ");
  Serial.println(wR);

RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

//____________________
// Function to update encoder position
void LeftEncoderCallback() {
  int encoderAState = digitalRead(LeftEncoder_C1);
  int encoderBState = digitalRead(LeftEncoder_C2);

  if ((encoderAState == HIGH) && (lastEncoderAStateL == LOW)) {
    if (encoderBState == LOW) {
      LeftEncoderCount++;
      encoderPosL++;
    } else {
      LeftEncoderCount--;
      encoderPosL--;
    }
  }

  lastEncoderAStateL = encoderAState;
}
//____________________
void RightEncoderCallback() {
  int encoderAState = digitalRead(RightEncoder_C1);
  int encoderBState = digitalRead(RightEncoder_C2);

  if ((encoderAState == HIGH) && (lastEncoderAStateR == LOW)) {
    if (encoderBState == LOW) {
      RightEncoderCount++;
      encoderPosR++;
    } else {
      RightEncoderCount--;
      encoderPosR--;
    }
  }

  lastEncoderAStateR = encoderAState;
}
