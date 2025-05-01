#include "pkg/motor.h"
#include "pkg/car.h"
#include "pkg/comm.h"
//#include "utils/DRV8833.h"
#include "Arduino.h"

// -- Definiciones --
//Recordar que los pines digitales 0 y 1 son RX y TX, respectivamente. Se usan para el moóulo Bluetooth 
#define MOTOR_A_PIN1 3 
#define MOTOR_A_PIN2 5
#define MOTOR_B_PIN1 6
#define MOTOR_B_PIN2 9
#define PLANT_PIN 7
// --------------------

//-- Variables Globales-- 
float posX;
float posY;
float theta; //Rotación del carro 
//-----------------------

comm bt(2, 8, 0, 1);
Coords CoordsIniciales = {0, 0, 0};

//-- Funciones --------------------------------------------
void procesarComando(String cmd) {
   if (cmd.startsWith("POS:")) {
     int indexTheta = cmd.indexOf("THETA:");
     if (indexTheta != -1) {
       String posStr = cmd.substring(4, indexTheta - 1);
       int coma = posStr.indexOf(',');
       posX = posStr.substring(0, coma).toFloat();
       posY = posStr.substring(coma + 1).toFloat();
 
       String thetaStr = cmd.substring(indexTheta + 6);
       theta = thetaStr.toFloat();
 
       Serial.print("X: "); Serial.println(posX);
       Serial.print("Y: "); Serial.println(posY);
       Serial.print("θ: "); Serial.println(theta);
     }
   }
 }
//---------------------------------------------------------

// ------------ Constructor de los motores -------------
DRV8833 driver;
Motor motorA = {MOTOR_A_PIN1, MOTOR_A_PIN2, A};
Motor motorB = {MOTOR_B_PIN1, MOTOR_B_PIN2, B};
// -------------------------------------------------------

void setup() {

   pinMode(PLANT_PIN, INPUT);
   
   //float initCarSpeed  = 0;

   // -- Inicio de los motores --
   initMotor(driver, &motorA); 
   initMotor(driver, &motorB);
   // ---------------------------

   // -- Logica del carro -- 
   /*Car car(carCoords, 0);
   car.initRoute(car.getCoords(), destinationA);*/
}  

void loop() {

   //Espera a que la planta se coloque en su lugar
   while (digitalRead(PLANT_PIN)==LOW)
   {
      driver.motorAStop();
      driver.motorBStop();
      delay(200);
   }
   
   //Si detecta la planta 
   //...

} //FIN