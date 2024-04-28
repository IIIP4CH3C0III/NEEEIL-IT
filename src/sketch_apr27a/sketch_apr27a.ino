#include <Arduino.h>
#include <math.h>
#include "pico/multicore.h"

// TIMER DEFINITIONS
#define TIMER_MOTORS           100
#define TIMER_SENSORS          100

// STRUCT TIMER
typedef struct{
  uint32_t curr_time = 0;                       // Current time since beginning of the execution
  uint32_t loop_interval = TIMER_MOTORS;        // Interval between each loop iteration
  uint32_t last_loop_time = 0;                  // Time of the last loop iteration
} TIME_CTRL;   


// MOTOR AND ENCODER PINS
#define IN1                    27
#define IN2                    22
#define IN3                    26
#define IN4                    28
#define ENA                    15
#define ENB                    14
#define ENCODERRIGHT_PIN_A     0
#define ENCODERRIGHT_PIN_B     1
#define ENCODERLEFT_PIN_A      2
#define ENCODERLEFT_PIN_B      3

#define SPEED_BASE             65
#define SPEED_TURN             65

#define SLCOMP                 1.20            // SLIP COMPENSATION
	
// STRUCT TO MOTOR CONTROL
typedef struct {
  float vel_left, vel_right;                    // Current velocity for both motors
  float pwm_offset=50;                          // Minimum pwm to avoid motors not moving
  float pwm_left, pwm_right;                    // PWM values for both motors     
  float vel_2_pwm=255/100;                      // Conversion factor from velocity to PWM in percentage
  float slip_compensation=SLCOMP;               // How much the right motor is faster than the left motor
} MOTORS;

// REFERENCE PLOT 
typedef struct {
  float x=0, y=0, theta=0;                     // Position in x, y and orientation according wall
} POSE;


// SENSORS PROXIMITY
#define N_ITERATIONS            3
#define N_ULTRASENSORS          3 
#define DIREITA                 0
#define FRENTE                  1
#define ESQUERDA                2
#define THRESHOLD_DISTANCE      25

typedef struct{
  uint8_t echoPin;
  uint8_t triggerPin;
  float reading;  
  uint8_t dr;
} ULTRASOUND;


// MEMORY ALOCATION 
MOTORS motors;
TIME_CTRL time_ctrl;
ULTRASOUND ultrasound_sensor[ N_ULTRASENSORS ];
POSE robot_pose;

// TICK COUNTER
volatile int encoderLeftCount = 0;  
volatile int encoderRightCount = 0;

////////// CODE USED FOR MOTOR //////////

/**
 * Converts velocity to PWM values for both motors.
 * PWM value is calculated based on the velocity, slip compensation, and velocity-to-PWM conversion factor.
 * The calculated PWM value is constrained between the PWM offset and 255.
 */
void vel_to_pwm() {
  if ( motors.vel_left == 0 )
    motors.pwm_left = 0;
  else
    motors.pwm_left = constrain(motors.pwm_offset + (2-motors.slip_compensation) * abs(motors.vel_left) * motors.vel_2_pwm,motors.pwm_offset,255);

  if ( motors.vel_right == 0 )
    motors.pwm_right = 0;
  else
    motors.pwm_right = constrain(motors.pwm_offset + motors.slip_compensation * abs(motors.vel_right) * motors.vel_2_pwm,motors.pwm_offset,255);

  return; 
}

/**
 * Resets the motor values to zero.
 * 
 * @param motors The MOTORS struct containing the motor values.
 */
void reset_motors(MOTORS& motors) {
  motors.vel_left = 0;
  motors.vel_right = 0;
  motors.pwm_left = 0;
  motors.pwm_right = 0;
}

/**
 * Initializes the motors.
 * Sets the initial values for velocity, PWM, slip compensation, and velocity to PWM conversion.
 * Configures the pin modes for motor control.
 */
void init_motors() {
  motors.vel_left = 0;
  motors.vel_right = 0;
  motors.pwm_left = 0;
  motors.pwm_right = 0;
  motors.slip_compensation = SLCOMP;  // Right / Left
  motors.vel_2_pwm = 255/100;

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void encoderISR() {
  for( ; ; ){
    if (digitalRead(ENCODERRIGHT_PIN_B)) {
      encoderRightCount--;
    } else {
      encoderRightCount++;
    }

    if (digitalRead(ENCODERLEFT_PIN_B)) {
      encoderLeftCount++;
    } else {
      encoderLeftCount--;
    }
  }
}

void reset_encoders(){
  encoderRightCount = 0;
  encoderLeftCount = 0;
}


/**
 * Initializes the encoders.
 * This function launches the `encoderISR` function on core 1.
 */
void init_encoders() {
  multicore_launch_core1(encoderISR);
}

/**
 * Sets the outputs for controlling the motors based on the current velocity values.
 * This function determines the direction (rotating front or backwards) of each motor and sets the corresponding input pins accordingly.
 * It also sets the PWM values for each motor to control their speed.
 */
void set_outputs() {
  vel_to_pwm();
  if (motors.vel_left > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  if (motors.vel_right > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  
  analogWrite(ENA, motors.pwm_left);
  analogWrite(ENB, motors.pwm_right);
}

////////// CODE USED FOR SENSORS //////////

void
init_ultrsensors(){
  uint8_t startPinTrigger = 6; 
  uint8_t startPinEcho = 7; 
  for( int i = 0 ;  i < N_ULTRASENSORS ; ++i ){
    startPinTrigger += 2;
    startPinEcho += 2;
    ultrasound_sensor[i].triggerPin = startPinTrigger;
    ultrasound_sensor[i].echoPin = startPinEcho;
    pinMode( ultrasound_sensor[i].triggerPin , OUTPUT );
    pinMode( ultrasound_sensor[i].echoPin , INPUT );
    ultrasound_sensor[i].reading = 0;
  } return;
}

void
map_sonar(void){
  float sum = 0 ;
  for( int i = 0 ; i < N_ULTRASENSORS ; ++i ){
    for( int j = 0 ; j < N_ITERATIONS ; ++j ){
      digitalWrite( ultrasound_sensor[i].triggerPin, LOW);
      delayMicroseconds(2);
      digitalWrite( ultrasound_sensor[i].triggerPin, HIGH);
      delayMicroseconds(10);
      digitalWrite( ultrasound_sensor[i].triggerPin, LOW);
      sum += pulseIn( ultrasound_sensor[i].echoPin, HIGH) * 0.034 / 2;
    }
    ultrasound_sensor[i].reading = sum / N_ITERATIONS;
    if( THRESHOLD_DISTANCE >= ultrasound_sensor[i].reading )
      ultrasound_sensor[i].dr = 0;
    else ultrasound_sensor[i].dr = 1 ;

    sum = 0;
  } return;
} 

////////// CODE USED FOR REFERENCIAL //////////

void init_pose() {
  robot_pose.x = 0;
  robot_pose.y = 0;
  robot_pose.theta = 0;
}



////////// TEMPLATE CODE FROM ARDUINO IDE //////////

void
decision(){
  if( ultrasound_sensor[DIREITA].dr && ultrasound_sensor[FRENTE].dr && ultrasound_sensor[ESQUERDA].dr  ){
    motors.vel_left  = SPEED_BASE;
    motors.vel_right = SPEED_BASE;
  } else if ( !ultrasound_sensor[DIREITA].dr && ultrasound_sensor[FRENTE].dr && ultrasound_sensor[ESQUERDA].dr ){
    motors.vel_left  = SPEED_BASE;
    motors.vel_right = SPEED_BASE;
  } else if ( ultrasound_sensor[DIREITA].dr && !ultrasound_sensor[FRENTE].dr && ultrasound_sensor[ESQUERDA].dr ){
    motors.vel_left  = SPEED_BASE;
    motors.vel_right = -SPEED_BASE;  
  } else if ( !ultrasound_sensor[DIREITA].dr && !ultrasound_sensor[FRENTE].dr && ultrasound_sensor[ESQUERDA].dr ){
    motors.vel_left  = -SPEED_BASE;
    motors.vel_right = SPEED_BASE;  
  } else if ( ultrasound_sensor[DIREITA].dr && ultrasound_sensor[FRENTE].dr && !ultrasound_sensor[ESQUERDA].dr ){
    motors.vel_left  = SPEED_BASE;
    motors.vel_right = SPEED_BASE;  
  } else if ( !ultrasound_sensor[DIREITA].dr && ultrasound_sensor[FRENTE].dr && !ultrasound_sensor[ESQUERDA].dr ){
    motors.vel_left  = SPEED_BASE;
    motors.vel_right = SPEED_BASE;  
  } else if ( ultrasound_sensor[DIREITA].dr && !ultrasound_sensor[FRENTE].dr && !ultrasound_sensor[ESQUERDA].dr ){
    motors.vel_left  = SPEED_BASE;
    motors.vel_right = -SPEED_BASE;  
  } else {
    motors.vel_left  = SPEED_BASE*1.1;
    motors.vel_right = -SPEED_BASE*1.1;   
  }
  return;
}

void setup() {
  Serial.begin(115200);
  init_motors();
  init_encoders();
  init_pose();
  reset_encoders();
  init_ultrsensors();
}

void loop( ){
  time_ctrl.curr_time = millis();
  
  if(time_ctrl.curr_time - time_ctrl.last_loop_time >= time_ctrl.loop_interval) {
    time_ctrl.last_loop_time = time_ctrl.curr_time;

    (void)map_sonar();
    (void)decision();
    set_outputs();
    reset_encoders();

  }
}
