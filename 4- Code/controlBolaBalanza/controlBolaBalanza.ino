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
#include <Servo.h>
#include <Wire.h>


/* ----------------------------------------------------
 * CONSTANTES
 * ----------------------------------------------------
 */

// Pines a utilizar del Arduino
#define servoP  9       // Pin para el control del Servo (PWM)
#define sensor  A0      // Pin para lectura del Sensor SHARP

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
 * OBJETOS
 * ----------------------------------------------------
 */

// Objeto que representa el servo
Servo servo;


/* ----------------------------------------------------
 * VARIABLES
 * ----------------------------------------------------
 */

// Tiempo de ejecución del programa
unsigned long timeAcc = 0;  //                                [ms]

// Variables de estado y mediciones
float setPoint = 17.3;    // SetPoint deseado del sistema     [cm]
float distancia = 0.0;    // Distancia medida por el sensor   [cm]
float error = 0.0;        // Error en de la planta            [cm]

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
  Serial.begin(9600);     // Inicia el puerto serial

  servo.attach(servoP);   // Establece el pin PWM por le cual se controla el Servo
  servo.write(angServo);  // Pone horizontal el riel

  // Obtiene la distancia y el error para evitar un inicio brusco
  distancia = getDistancia(50);
  error = distancia - setPoint;

  timeAcc = millis();     // Actualiza el tiempo de ejecución
}

void loop() 
{
  // Comprueba si se ha cumplido el periodo de muestreo de señales
  if(millis() >= timeAcc + periodo)
  {
    // Actualizar tiempo actual de ejecución
    timeAcc = millis();

    // Obtiene la distancia como promedio de 50 mediciones y calcula el error
    distancia = getDistancia(300);
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
    
    servo.write(angServo);
  }
  
}


/* ----------------------------------------------------
 * MÉTODOS Y FUNCIONES
 * ----------------------------------------------------
 */

// Método que retorna la distancia leída por el sensor. Promedio de n mediciones
float getDistancia(int n)
{
  // Suma del valor digital del ADC obtenido
  float sum=0;
  for(int i=0;i<n;i++)
  {
    // Conversión de la lectura a voltaje
    sum += analogRead(sensor) * 0.0048828125;
  }

  // Fórmula para convertir de voltaje a distancia según Datasheet
  float d = 13*pow(sum/n, -1);            // d = 13*voltaje^(-1)

  // Limitar a máximos del rango del sensor
  if(d <= 4)
  {
    d = 4.0;
  }
  else if(d >= 30)
  {
    d = 30.0;
  }

  return d;
}

 
