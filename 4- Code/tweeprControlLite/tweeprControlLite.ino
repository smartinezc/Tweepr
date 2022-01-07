/*
 *                Tweepr Control Code
 *              Proyecto de Grado IELE 
 *              
 * Code to control the Two Wheel Electronic Educational 
 * Platform for Robotics (TWEEPR) system
 *                Santiago Martínez
 *           
 * Version V0.4
 * January 6,2022
*/

// TODO: Write documentation

/* ----------------------------------------------------
 * LIBRARIES
 * ----------------------------------------------------
*/

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

/* ----------------------------------------------------
 * CONSTANTS
 * ----------------------------------------------------
*/

// ESP32 pins used
#define EN_M    5         // Pin for Motors Enable
#define STEP_L  16         // Pin for Left Motor Step
#define DIR_L   17         // Pin for Left Motor Direction
#define STEP_R  18         // Pin for Right Motor Step
#define DIR_R   19         // Pin for Right Motor Direction


// Sensor Address
#define MPU6050_ADDR    0x68      // MPU6050 I2C Address (Sometimes 0x69)

// Absolute limits
#define MIN_ANGLE 5     // Min angle to enable control over motors
#define MAX_ANGLE 35    // Max allowed angle deviation from the center
#define MAX_PID   400   // Max PID value output allowed

// Loop period constants
#define period    20    // Period for signal sampling         [ms]

// Tunned PID gains
const float Kp = 15;    // Proporcional gain (~15)
const float Ki = 0;     // Integral gain ()
const float Kd = 15;    // Derivative gain ()

/* ----------------------------------------------------
 * OBJETOS
 * ----------------------------------------------------
*/

// Objeto que representa el sensor MPU6050
MPU6050 mpu(Wire);

// Hardware timer for Motor Control Loop
hw_timer_t * motorControlTimer = NULL;

/* ----------------------------------------------------
 * VARIABLES
 * ----------------------------------------------------
*/

// Tiempo de ejecución del programa
unsigned long timeAcc = 0;  //                                  [ms]

bool active = false;        // If true, motors control would be active

// Variables de estado y mediciones
long offsetsXY[2];          // Gyro Offsets for X and Y Axis    [deg]
long gyroXY[2];             // Gyro readings for X and Y Axis   [deg]
int accOffset = 1045;       // Accelerometer offset             [m/s^2]
float setPoint = 90.0;      // SetPoint deseado del sistema     [cm]
float angleR = 0.0;         // Angle reading from MPU sensor    [cm]
float error = 0.0;          // Error en de la planta            [cm]

// Memoria de Control
float prop = 0.0;
float inte = 0.0;
float deri = 0.0;
float PIDout = 0.0;
float anglPrev = 0.0;       // Previous angle reading           [cm]
float errorPrev = 0.0;      // Error previo de la planta        [cm]

// Motor Control variables
unsigned long timeMC = 0;   // Execution time for Motor Control [us]
float PIDoutLeft = 0.0;     // PID value for Left Motor
float PIDoutRight = 0.0;    // PID value for Right Motor
volatile int leftMotorPulseP;        // Left motor pulse period (*20us)  [us]
volatile int rightMotorPulseP;       // Right motor pulse period (*20us) [us]
volatile int leftMotorPulseP_Prev;   // Left motor pulse period (*20us)  [us]
volatile int rightMotorPulseP_Prev;  // Right motor pulse period (*20us) [us]
volatile int countSpeedMotorL = 0;   // Counter for pulse duration, Left Motor
volatile int countSpeedMotorR = 0;   // Counter for pulse duration, Right Motor

/* ----------------------------------------------------
 * INTERRUPT ROUTINE
 * ----------------------------------------------------
*/

// Motor Control Loop (ticks every 50us)
void IRAM_ATTR motorControl()
{
  // Left Motor pulse control
  countSpeedMotorL++;             // Increase counter every time this routine is executed
  
  // Check if the counter is greater than the previous motor pulse period
  if(countSpeedMotorL > leftMotorPulseP_Prev)
  {
    countSpeedMotorL = 0;                       // Reset counter if greater
    leftMotorPulseP_Prev = leftMotorPulseP;     // Load next motor pulse period
  
    // Check if the new motor pulse period is negative
    if(leftMotorPulseP_Prev < 0)
    {
      digitalWrite(DIR_L, LOW);                 // Set DIR_L pin to LOW, reverse direction
      leftMotorPulseP_Prev *= -1;               // Invert to count positive period intervals
    }
    else
    {
      digitalWrite(DIR_L, HIGH);                // Set DIR_L pin to HIGH, forward direction
    }
  }
  else if (countSpeedMotorL == 1)digitalWrite(STEP_L, HIGH);   // Create pulse for motor step
  else if (countSpeedMotorL == 2)digitalWrite(STEP_L, LOW);    // End motor step pulse in 20us
  
  
  
  // Right Motor pulse control
  countSpeedMotorR++;             // Increase counter every time this routine is executed
  
  // Check if the counter is greater than the previous motor pulse period
  if(countSpeedMotorR > rightMotorPulseP_Prev)
  {
    countSpeedMotorR = 0;                       // Reset counter if greater
    rightMotorPulseP_Prev = rightMotorPulseP;   // Load next motor pulse period
  
    // Check if the new motor pulse period is negative
    if(rightMotorPulseP_Prev < 0)
    {
      digitalWrite(DIR_R, LOW);                 // Set DIR_R pin to LOW, reverse direction
      rightMotorPulseP_Prev *= -1;              // Invert to count positive period intervals
    }
    else
    {
      digitalWrite(DIR_R, HIGH);                // Set DIR_R pin to HIGH, forward direction
    }
  }
  else if (countSpeedMotorR == 1)digitalWrite(STEP_R, HIGH);   // Create pulse for motor step
  else if (countSpeedMotorR == 2)digitalWrite(STEP_R, LOW);    // End motor step pulse in 20us
}

/* ----------------------------------------------------
 * MAIN
 * ----------------------------------------------------
*/

void setup() 
{
  Serial.begin(115200);     // Inicia el puerto serial
  Wire.begin();             // Inicia la comunicación I2C

  initPins();               // Initialize and define pins mode
  byte stat = mpu.begin();  // Start I2C communication with MPU6050 sensor

  while(stat != 0) { }      // If sensor can't be initianlize exit code

  // Establish the gyro offset on startup
  delay(1000);
  mpu.calcOffsets();

  // Start timer for Motor Control Loop. Using timer 0, Clock period 12.5ns
  motorControlTimer = timerBegin(0, 20, true);      // -> 12.5ns * 20 = 250ns
  timerAttachInterrupt(motorControlTimer, &motorControl, true);
  timerAlarmWrite(motorControlTimer, 200, true);    // 200 * 250ns = 50us, autoreload true
  timerAlarmEnable(motorControlTimer);              // Enable
}

void loop() 
{
  // Update MPU6050 readings 
  mpu.update();

  // TODO: Check for received WiFi commands
  // TODO: Make WiFi commands last for some loops
  
  // Check if the signals reading time period have passed
  if(active && millis() - timeAcc >= period)
  {
    // Read angle calculation from MPU and calculate the error
    angleR = mpu.getAngleX();
    error = angleR - setPoint;

    // Proporcional
    prop = Kp * error;

    // Integral
    inte = inte + ((Ki*period)/2)*(error + errorPrev);

    // Derivativo
    deri = (Kd*(angleR - anglPrev)/period);

    // Salida del PID
    PIDout = prop + inte + deri;
    if(PIDout < -MAX_PID)       {PIDout = -MAX_PID;}
    else if(PIDout > MAX_PID)   {PIDout = MAX_PID;}
    
    // Actualizar la memoria del control
    anglPrev = angleR; 
    errorPrev = error;

    // Dead band where the robot is somewhat balanced (Don't control)
    if(error < MIN_ANGLE && error > -MIN_ANGLE)PIDout = 0;
    
    // Turn off control if angle > MAX_ANGLE (Disable Motors)
    if(error > MAX_ANGLE || error < -MAX_ANGLE)
    {
      active = false;
      PIDout = 0;
      digitalWrite(EN_M, HIGH);
    }

    // TODO: Change PIDout whit WiFI commands for each motor L&R
    PIDoutLeft = -PIDout;
    PIDoutRight = PIDout;
    
    // Refactor PIDout value for each motor, take integrer part as pulse period for control
    if(PIDoutLeft > 0)
    {
      leftMotorPulseP = -5 + (1 / (PIDoutLeft + 9)) * 5500;
    }
    else if(PIDoutLeft < 0)
    {
      leftMotorPulseP = 5 + (1 / (PIDoutLeft - 9)) * 5500;
    }

    if(PIDoutRight > 0)
    {
      rightMotorPulseP = -5 + (1 / (PIDoutRight + 9)) * 5500;
    }
    else if(PIDoutRight < 0)
    {
      rightMotorPulseP = 5 + (1 / (PIDoutRight - 9)) * 5500;
    }

    // Update current execution time
    timeAcc = millis();
  }
  else if(millis() - timeAcc >= period)
  {
    angleR = mpu.getAngleX();

    // Activate the control loop if Tweepr is up
    if(!active && angleR > 89 && angleR < 91)
    {
      active = true;
      digitalWrite(EN_M, LOW);
    }
    
    // Update current execution time
    timeAcc = millis();
  }
}


/* ----------------------------------------------------
 * MÉTODOS Y FUNCIONES
 * ----------------------------------------------------
*/

// Function to initialize and define pins mode
void initPins()
{
  // Define outputs
  pinMode(DIR_L, OUTPUT);
  pinMode(STEP_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(STEP_R, OUTPUT);
  pinMode(EN_M, OUTPUT);

  // Disable Stepper Motors while startup (LOW active)
  digitalWrite(EN_M, HIGH);
}
