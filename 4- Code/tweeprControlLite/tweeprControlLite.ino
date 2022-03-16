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

#include <Arduino.h>            // Basic interface
#include <Wire.h>               // I2C communication manager
#include <MPU6050_light.h>      // MPU sensor manager
#include <WiFi.h>               // WiFi library manager
#include <ESPmDNS.h>            // mDNS library manager
#include <ESPAsyncWebServer.h>  // Web Server library manager
#include <SPIFFS.h>             // File server manager

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
#define MIN_ANGLE 2     // Min angle to enable control over motors
#define MAX_ANGLE 40    // Max allowed angle deviation from the center
#define MAX_PID   400   // Max PID value output allowed

// Loop period constants
#define period    4     // Period for signal sampling         [us]

// WiFi config
#define netSSID "Martinez"          // SSID of WiFi network to be connected to
#define netPASS "eli433Bijou19"     // Password of WiFi network

/* ----------------------------------------------------
 * OBJETOS
 * ----------------------------------------------------
*/

// Objeto que representa el sensor MPU6050
MPU6050 mpu(Wire);

// Hardware timer for Motor Control Loop
hw_timer_t * motorControlTimer = NULL;

// Web Server handler
AsyncWebServer server(80);      // Server listening on 80 PORT

/* ----------------------------------------------------
 * VARIABLES
 * ----------------------------------------------------
*/

// Tiempo de ejecución del programa
unsigned long timeAcc = 0;  //                                  [ms]

bool active = false;        // If true, motors control would be active

// Variables de estado y mediciones
float setPoint = 91.25;     // SetPoint deseado del sistema     [°]
float moveSetP = 0.0;       // Moving comand Setpoint           [°]
float angleR = 0.0;         // Angle reading from MPU sensor    [°]
float error = 0.0;          // Error en de la planta            [°]

// Tunned PID gains
float Kp = 16;              // Proporcional gain (~12)
float Ki = 4.0;             // Integral gain (~0.6)
float Kd = 12;              // Derivative gain (~23)

// Control Memory
float prop = 0.0;
float inte = 0.0;
float deri = 0.0;
float PIDout = 0.0;
float anglPrev = 0.0;       // Previous angle reading           [°]
float anglPrev2 = 0.0;      // Previous 2 angle reading         [°]
float anglPrev3 = 0.0;      // Previous 3 angle reading         [°]
float errorPrev = 0.0;      // Error previo de la planta        [°]

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
  Wire.begin();             // Start I2C communication
  Wire.setClock(400000);    // Change I2C clock frecuency 400kHz

  initPins();               // Initialize and define pins mode
  byte stat = mpu.begin();  // Start I2C communication with MPU6050 sensor
  while(stat != 0) { }      // If sensor can't be initianlize exit code
  mpu6050Init();            // Adjust resolution and filters for MPU6050
  delay(1000);
  mpu.calcOffsets();        // Establish the gyro offset on startup

  // WiFi conection process
  WiFi.begin(netSSID, netPASS);
  while(WiFi.status() != WL_CONNECTED)delay(800);   // Wait for connection to be establish
  if(!MDNS.begin("TweeprControl"))return;           // Start mDNS service
  if(!SPIFFS.begin(true))return;                    // Initialize SPIFFS

  // Server routing
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });
  server.on("/app.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/app.js", "text/css");
  });
  
  // Receive a GET request to tweeprcontrol.local/get?pidgains=<values>
  // Receive a GET request to tweeprcontrol.local/get?move=<values>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String cmdR;    // Text message containing the command recieved
    if(request->hasParam("pidgains")) 
    {
      // Extract PID Gains values
      cmdR = request->getParam("pidgains")->value();
      Kp = split(cmdR, ';', 0).toFloat();
      Ki = split(cmdR, ';', 1).toFloat();
      Kd = split(cmdR, ';', 2).toFloat();
      request->send(200);
    }
    else if(request->hasParam("move"))
    {
      // Change setPoint value
      cmdR = request->getParam("move")->value();
      setPoint += cmdR.toFloat();
      request->send(200, "text/plain", String(setPoint, 2));
    }
  });
  server.begin();

  // Start timer for Motor Control Loop. Using timer 0, Clock period 12.5ns
  motorControlTimer = timerBegin(0, 20, true);      // -> 12.5ns * 20 = 250ns
  timerAttachInterrupt(motorControlTimer, &motorControl, true);
  timerAlarmWrite(motorControlTimer, 80, true);     // 80 * 250ns = 20us, autoreload true
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
    angleR = (angleR + anglPrev + anglPrev2 + anglPrev3)/4.0;
    error = angleR - setPoint - moveSetP;
    if(PIDout > 10 || PIDout < -10) {error += PIDout * 0.015 ;}

    // Proporcional
    prop = Kp * error;

    // Integral
    inte = inte + ((Ki*(period/1000))/2)*(error + errorPrev);
    //inte = inte + (Ki * error);

    // Derivativo
    deri = (Kd*(angleR - anglPrev)/(period));
    //deri = Kd * (error - errorPrev);

    // Salida del PID
    PIDout = prop + inte + deri;
    if(PIDout < -MAX_PID)       {PIDout = -MAX_PID;}
    else if(PIDout > MAX_PID)   {PIDout = MAX_PID;}
    
    // Actualizar la memoria del control
    moveAngleR(); 
    errorPrev = error;

    // Dead band where the robot is somewhat balanced (Don't control)
    if(error < MIN_ANGLE && error > -MIN_ANGLE)PIDout = 0;
    
    // Turn off control if angle > MAX_ANGLE (Disable Motors)
    if(error > MAX_ANGLE || error < -MAX_ANGLE)
    {
      active = false;
      PIDout = 0;
      inte = 0;
      moveSetP = 0;
      digitalWrite(EN_M, HIGH);
    }

    // TODO: Change PIDout whit WiFI commands for each motor L&R
    PIDoutLeft = -PIDout;
    PIDoutRight = PIDout;
    if(setPoint > 91) 
    {
      if(PIDout < 0)moveSetP += 0.001;
      if(PIDout > 0)moveSetP -= 0.001;
    }
    
    // Refactor PIDout value for each motor, take integrer part as pulse period for control
    if(PIDoutLeft > 0)
    {
      leftMotorPulseP = -5 + (1 / (PIDoutLeft + 9)) * 5500;
    }
    else if(PIDoutLeft < 0)
    {
      leftMotorPulseP = 5 + (1 / (PIDoutLeft - 9)) * 5500;
    }
    else{leftMotorPulseP = 0;}

    if(PIDoutRight > 0)
    {
      rightMotorPulseP = -5 + (1 / (PIDoutRight + 9)) * 5500;
    }
    else if(PIDoutRight < 0)
    {
      rightMotorPulseP = 5 + (1 / (PIDoutRight - 9)) * 5500;
    }
    else{rightMotorPulseP = 0;}

    //Serial.println(millis() - timeAcc);
    Serial.println(moveSetP);

    // Update current execution time
    timeAcc = millis();
  }
  else if(millis() - timeAcc >= period)
  {
    angleR = mpu.getAngleX();
    angleR = (angleR + anglPrev + anglPrev2 + anglPrev3)/4.0;
    moveAngleR();
    //Serial.println(millis() - timeAcc);

    // Activate the control loop if Tweepr is up
    if(!active && angleR > 90 && angleR < 92)
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

// Function to initialize MPU6050 sensor
void mpu6050Init()
{
  // Enable Digital Filters
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);     // Write to 0x1A register the value of 003 -> Set Low Filter to ~43Hz
  Wire.endTransmission();
}

// Funtion to Move the 4-window filter for the MPU6050 angle reading
void moveAngleR()
{
  anglPrev3 = anglPrev2;
  anglPrev2 = anglPrev;
  anglPrev = angleR;
}

// Split String function into individual values ("" if not found)
String split(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
