/*
 *                Tweepr Control Code
 *              Proyecto de Grado IELE 
 *              
 * Code to control the Two Wheel Electronic Educational 
 * Platform for Robotics (TWEEPR) system
 *                Santiago Martínez
 *           
 * Version V0.2
 * December 28, 2021
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

// TODO: Define pins

// Pines a utilizar del Arduino
#define DIR_L   3         // Pin for Left Motor Direction
#define STEP_L  3         // Pin for Left Motor Step
#define DIR_R   3         // Pin for Right Motor Direction
#define STEP_R  3         // Pin for Right Motor Step
#define EN_M    2         // Pin for Motors Enable

// Sensor Address
#define MPU6050_ADDR    0x68      // MPU6050 I2C Address (Sometimes 0x69)

// Absolute Max limits
#define MAX_ANGLE 40    // Max allowed angle deviation from the center
#define MAX_PID   400   // Max PID value output allowed

// Period for signal sampling and control sequence, in ms
#define period 20

// Ganancias sintonizadas del PID
const float Kp = 7;       // Constante Proporcional del sistema
const float Ki = 0;       // Constante Integral del sistema
const float Kd = 2500;    // Constante Derivativa del sistema


/* ----------------------------------------------------
 * OBJETOS
 * ----------------------------------------------------
*/

// Objeto que representa el sensor MPU6050
MPU6050 mpu(Wire);

/* ----------------------------------------------------
 * VARIABLES
 * ----------------------------------------------------
*/

// Tiempo de ejecución del programa
unsigned long timeAcc = 0;  //                                [ms]

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

// Ángulo del servo
int angServo = 90;        //                                  [°]


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

    // TODO: Dead band where the robot is somewhat balanced (Don't control)
    // TODO: Turn off control if angle > MAX_ANGLE (active = false)

    // TODO: Change PIDout whit WiFI commands for each motor L&R

    Serial.println(error);
    
    // TODO: Motor control with PIDout values

    // Update current execution time
    timeAcc = millis();
  }
  else if(millis() - timeAcc >= period)
  {
    angleR = mpu.getAngleX();

    // Activate the control loop if Tweepr is up
    if(!active && angleR > 89 && angleR < 91){active = true;}

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
