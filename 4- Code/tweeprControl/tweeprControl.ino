/*
 * Taller de Control - 2020-01
 * Grupo 7
 * Código de control para el sistema Bola-Balanza
 *           Santiago Martínez
 *           
 * Octubre 14, 2021
 */

/* ----------------------------------------------------
 * LIBRERÍAS
 * ----------------------------------------------------
*/

// Control del Servomotor
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

/* ----------------------------------------------------
 * CONSTANTES
 * ----------------------------------------------------
*/

// Pines a utilizar del Arduino
#define DIR_L   2         // Pin for Left Motor Direction
#define STEP_L  3         // Pin for Left Motor Step
#define DIR_R   2         // Pin for Right Motor Direction
#define STEP_R  3         // Pin for Right Motor Step
#define EN_M    2         // Pin for Motors Enable

// Sensor Address
#define MPU6050_ADDR    0x68      // MPU6050 I2C Address (Sometimes 0x69)

// Máximos y Mínimos de la salida del control
#define MAXLIM 40       // Delta de Ángulo máximo permitido para el Servo
#define MINLIM -40      // Delta de Ángulo mínimo permitido para el Servo

// Periodo de muestreo de la señal y ciclo de control, en ms
#define periodo 50

// Ganancias sintonizadas del PID
const float Kp = 7;       // Constante Proporcional del sistema
const float Ki = 0;       // Constante Integral del sistema
const float Kd = 2500;    // Constante Derivativa del sistema


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
float setPoint = 17.3;      // SetPoint deseado del sistema     [cm]
float distancia = 0.0;      // Distancia medida por el sensor   [cm]
float error = 0.0;          // Error en de la planta            [cm]

// Memoria de Control
float prop = 0.0;
float inte = 0.0;
float deri = 0.0;
float PIDout = 0.0;
float distPrev = 0.0;     // Distancia previa medida          [cm]
float errorPrev = 0.0;    // Error previo de la planta        [cm]

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

  pinsInit();               // Initialize and define pins mode
  mpu6050Init();            // Start I2C communication with MPU6050 sensor

  // Establish the gyro offset on startup. Mean of 500 measurements
  getGyroXY(500, offsetsXY);
}

void loop() 
{
  // Comprueba si se ha cumplido el periodo de muestreo de señales
  if(millis() >= timeAcc + periodo)
  {
    // Actualizar tiempo actual de ejecución
    timeAcc = millis();

    // Obtiene la distancia como promedio de 50 mediciones y calcula el error
    //distancia = getDistancia(300);
    error = distancia - setPoint;

    // Proporcional
    prop = Kp * error;

    // Integral
    inte = inte + ((Ki*periodo)/2)*(error + errorPrev);

    // Derivativo
    deri = (Kd*(distancia - distPrev)/periodo);

    // Salida del PID
    PIDout = prop + inte + deri;
    if(PIDout < MINLIM)       {PIDout = MINLIM;}
    else if(PIDout > MAXLIM)  {PIDout = MAXLIM;}
    
    // Actualizar la memoria del control
    distPrev = distancia; 
    errorPrev = error;

    // Actualiza el ángulo con el control calcualdo
    angServo = 90 + PIDout;

    Serial.println(error);
    
    //servo.write(angServo);
  }
  
}


/* ----------------------------------------------------
 * MÉTODOS Y FUNCIONES
 * ----------------------------------------------------
*/

// Function to initialize and define pins mode
void pinsInit()
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

// Function to initialize MPU6050 sensor
void mpu6050Init()
{
  // Start I2C communication with MPU6050 sensor
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);     // Write to 0x6B register the value of 000 -> Activate Gyro
  Wire.endTransmission();

  // Change Gyro Scale to +-250deg/sec
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);     // Write to 0x1B register the value of 000 -> Full Scale
  Wire.endTransmission();

  // Change Accelerometer Scale to +-4g
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x08);     // Write to 0x1C register the value of 008 -> Scale to +-4g
  Wire.endTransmission();

  // Enable Digital Filters
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);     // Write to 0x1A register the value of 003 -> Set Digital low Filter to ~43Hz
  Wire.endTransmission();
}

// Function to obtain Gyro readings. Mean of n measurements
void getGyroXY(int n, long *data)
{
  // Acumulative sum of sensor readings
  long xGyroRead = 0;
  long yGyroRead = 0; 
  
  // Take n measurements and acumulate them
  for(int i=0; i<n; i++)
  {
    // Obtaing reading
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);                     // Write to 0x43 register to prepare a request
    Wire.endTransmission();
  
    Wire.requestFrom(MPU6050_ADDR, 4);    // Request 4 bytes from MPU6050 Sensor
  
    // Acumulate value of reading
    yGyroRead += Wire.read() << 8 | Wire.read();  //Merge high and low byte and get an integer
    xGyroRead += Wire.read() << 8 | Wire.read();  //Merge high and low byte and get an integer
  
    delayMicroseconds(3500);
  }
  
  // Obtain mean value of measurements
  xGyroRead /= n;
  yGyroRead /= n;
  
  // Update *data passed as argument
  data[0] = xGyroRead;
  data[1] = yGyroRead;
}

// Function to obtain angle reading
float getAngle()
{
  // Acumulative sum of sensor readings
  int accRaw = 0;

  // Obtaing reading
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3F);                     // Write to 0x3F register to prepare a request
  Wire.endTransmission();
  
  Wire.requestFrom(MPU6050_ADDR, 2);    // Request 2 bytes from MPU6050 Sensor
  
  // Acumulate value of reading
  accRaw += Wire.read() << 8 | Wire.read();  //Merge high and low byte and get an integer
  accRaw += accOffset;

  // Limit range
  if(accRaw > 8200)
  {
    accRaw = 8200;
  }
  else if(accRaw < -8200)
  {
    accRaw = -8200;
  }

  // Calculate angle according to datasheet
  return asin((float)accRaw / 8200.0) * 57.296;

  //
  /* 
  if(!active && abs(accAngle) < 0.5)
  {
    
  }*/

  // Get gyro readings. Mean of 20 measurements
  getGyroXY(20, gyroXY);
  gyroXY[0] -= offsetsXY[0];
  gyroXY[1] -= offsetsXY[1];
}

 
