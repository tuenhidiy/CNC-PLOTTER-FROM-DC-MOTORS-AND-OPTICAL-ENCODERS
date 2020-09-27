/* 
 * This Arduino program is refered from: https://github.com/cswiger/dcservodrive by cswiger.
 * In this project, DC motors can be simulated as same as stepper motors and we can control them via GRBL firmware for CNC application.  
 */
   
// Timer2 library
#include "FlexiTimer2.h"

// PID library
#include <PID_v1.h>

// AFMotor library
#include <AFMotor.h>

// Quadrature Encoder Library
#include "Encoder.h"

// Create the motor driver instances
AF_DCMotor motorX(1, MOTOR12_8KHZ);
AF_DCMotor motorY(2, MOTOR12_8KHZ);

// Set up pins for the quadrature encoders - Arduino MEGA2560 has 6 interrupt pins.
#define EncoderX_ChannelA   18  // Interrupt 5
#define EncoderX_ChannelB   22
#define EncoderY_ChannelA   20  // Interrupt 3
#define EncoderY_ChannelB   24

// Set up STEP & DIRECTION pins for X and Y axis
#define STEP_XPIN           19  // Interrupt 4
#define STEP_YPIN           21  // Interrupt 2
#define DIR_XPIN            23
#define DIR_YPIN            25

// Turn on/ off debugging for X/Y motor
#define DEBUG_X             0
#define DEBUG_Y             0

// For calculating the actual movements
#define STEPSPERMM_X      24.5    // STEP/mm is used in the GRBL for DC motor X axis.
#define DEADBW_X          4.5     // Deadband width = 4.5 --> Acceptable error for positioning in mm: 0.18mm.

#define STEPSPERMM_Y      192.0   // STEP/mm is used in the GRBL for DC motor Y axis.
#define DEADBW_Y          19.2    // Deadband width = 19.2 --> Acceptable error for positioning in mm: 0.1mm.


// Set up Input
double INPUT_X;
double INPUT_Y;

double OLD_INPUT_X = 0;
double OLD_INPUT_Y = 0;

// Set up Actual value
double ACTUAL_X_MM;
double ACTUAL_Y_MM;

double OLD_ACTUAL_X_MM;
double OLD_ACTUAL_Y_MM;


// PID controller constants
double KP_X = 20.0;     // P for X motor
double KI_X = 0.03;     // I for X motor
double KD_X = 0.01;     // D for X motor

double KP_Y = 9.0;      // P for Y motor
double KI_Y = 0.02;     // I for Y motor
double KD_Y = 0.01;     // D for Y motor

// The Output variable motor speed to the motor driver
double OUTPUT_X;
double OUTPUT_Y;

double OLD_OUTPUT_X = 0;
double OLD_OUTPUT_Y = 0;

// Setpoint
double SETPOINT_X = 0;
double SETPOINT_Y = 0;

double OLD_SETPOINT_X = 0;
double OLD_SETPOINT_Y = 0;

double ERROR_X = 0;
double ERROR_Y = 0;

// PID controller
PID myPID_X(&INPUT_X, &OUTPUT_X, &SETPOINT_X, KP_X, KI_X, KD_X, DIRECT);
PID myPID_Y(&INPUT_Y, &OUTPUT_Y, &SETPOINT_Y, KP_Y, KI_Y, KD_Y, DIRECT);

// Setup optical encoders
Encoder XEncoder(EncoderX_ChannelA, EncoderX_ChannelB);
Encoder YEncoder(EncoderY_ChannelA, EncoderY_ChannelB);

void setup()
{
  // For debugging
  Serial.begin(115200);

  pinMode(STEP_XPIN, INPUT);
  pinMode(STEP_YPIN, INPUT);
  pinMode(DIR_XPIN, INPUT);
  pinMode(DIR_YPIN, INPUT);  

  // The stepper simulator
  attachInterrupt(4, doXstep, RISING);  // PIN 19 (Interrupt 4) - Interrupt X step at rising edge pulses
  attachInterrupt(2, doYstep, RISING);  // PIN 21 (Interrupt 2) - Interrupt Y step at rising edge pulses
  
  // Outpout PWM limits
  myPID_X.SetOutputLimits(-255,255);
  myPID_Y.SetOutputLimits(-255,255);
  
  // Compute output every 1ms
  myPID_X.SetSampleTime(1);
  myPID_Y.SetSampleTime(1);
  
  // Setup PID mode
  myPID_X.SetMode(AUTOMATIC);
  myPID_Y.SetMode(AUTOMATIC);
  
  // Apply PID every 1ms by FlexiTimer2
  FlexiTimer2::set(1, 1.0/1000, doPID);
  FlexiTimer2::start();
}

void loop()
{
  // Read X and Y axis optical encoders
  INPUT_X = XEncoder.read();
  INPUT_Y = YEncoder.read();
  
  // Calculating the error
  ERROR_X = (INPUT_X - SETPOINT_X);
  ERROR_Y = (INPUT_Y - SETPOINT_Y);
  
  // For debugging
 if (DEBUG_X)
 {
  ACTUAL_X_MM = INPUT_X / STEPSPERMM_X;
 // Debugging X motor actual position in mm
 if (OLD_ACTUAL_X_MM != ACTUAL_X_MM)
 {
    Serial.print("ACTUAL X(MM): ");
    Serial.println(ACTUAL_X_MM);    
    OLD_ACTUAL_X_MM = ACTUAL_X_MM;
  }  
 // Debugging position X encoders
 if (OLD_INPUT_X != INPUT_X)
 {
    Serial.print("POSITION X: ");
    Serial.println(INPUT_X);    
    OLD_INPUT_X = INPUT_X;
  }
  // Debugging X stepping input
  if ( SETPOINT_X != OLD_SETPOINT_X )
  {
    Serial.print("SETPOINT X: ");
    Serial.println(SETPOINT_X);    
    OLD_SETPOINT_X = SETPOINT_X;
  }
  // Debugging X motor PWM output
  if ( OUTPUT_X != OLD_OUTPUT_X )
  {
    Serial.print("OUTPUT X: ");
    Serial.println(OUTPUT_X);    
    OLD_OUTPUT_X = OUTPUT_X;
  }
  
 }
 if (DEBUG_Y)
 {
    ACTUAL_Y_MM = INPUT_Y / STEPSPERMM_Y;
 // Debugging Y motor actual position in mm
 if (OLD_ACTUAL_Y_MM != ACTUAL_Y_MM)
 {
    Serial.print("ACTUAL Y(MM): ");
    Serial.println(ACTUAL_Y_MM);    
    OLD_ACTUAL_Y_MM = ACTUAL_Y_MM;
  }      
  // Debugging Y stepping input
  if ( SETPOINT_Y != OLD_SETPOINT_Y )
  {
    Serial.print("SETPOINT Y: ");
    Serial.println(SETPOINT_Y);
    OLD_SETPOINT_Y = SETPOINT_Y;
  }
  // Debugging position Y encoders
  if (OLD_INPUT_Y != INPUT_Y)
  {
    Serial.print("POSITION Y: ");
    Serial.println(INPUT_Y);
    OLD_INPUT_Y = INPUT_Y;
  }
  // Debugging Y motor PWM output
  if ( OUTPUT_Y != OLD_OUTPUT_Y )
  {
    Serial.print("OUTPUT Y: ");
    Serial.println(OUTPUT_Y);    
    OLD_OUTPUT_Y = OUTPUT_Y;
  }
  
 }
}

void doXstep() 
{
  if ( digitalRead(DIR_XPIN) == HIGH ) SETPOINT_X--;
  else SETPOINT_X++;
}

void doYstep()
{
  if ( digitalRead(DIR_YPIN) == HIGH ) SETPOINT_Y--;
  else SETPOINT_Y++;
}

void doPID() 
{
  interrupts();
  myPID_X.Compute();
  myPID_Y.Compute();
  if (abs(ERROR_X) < DEADBW_X) // If the motor X is in position within the deadband width (acceptable error)
    {
      motorX.setSpeed(0); // Turn off the X motor
    }
  else
    {
      motorX.setSpeed(abs(int(OUTPUT_X)));  // X Motor is regulated by PID controller ouput
    }
  if (abs(ERROR_Y) < DEADBW_Y) // If the motor Y is in position within the deadband width (acceptable error)
    {
      motorY.setSpeed(0); // Turn off the Y motor
    }
  else
    {  
      motorY.setSpeed(abs(int(OUTPUT_Y)));  // Y Motor is regulated by PID controller ouput
    }
  int directionX;
  int directionY;
  
  if(OUTPUT_X > 0)
    {
      directionX = FORWARD;
    }
  if(OUTPUT_X < 0)
    {
      directionX = BACKWARD;
    }
    
  if(OUTPUT_Y > 0)
    {
      directionY = FORWARD;
    }
  if(OUTPUT_Y < 0)
    {
      directionY = BACKWARD;
    }
  
  motorX.run(directionX);
  motorY.run(directionY);
}
