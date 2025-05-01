#include "car.h"
#include "motor.h"

// Metodos getters
Coords Car::getCoords() {
  return this->coord;
}

float Car::getSpeed() {
  return this->speed;
}

// Metodos de movimiento
void Car::stopCar(DRV8833 driver){
  driver.motorAStop(); 
  driver.motorBStop();
} 

void Car::moveCarForward(DRV8833 driver){ 
  driver.motorAForward();
  driver.motorBForward();
}

void Car::moveCarBack(DRV8833 driver){
  driver.motorAReverse();
  driver.motorBReverse();
}

//Depende de como giren los motores, es decir que está sujeto a revisión 
//Metodos para rotar el carro

void Car::RotateL(DRV8833 driver){
  driver.motorAForward();
  driver.motorBReverse();
}

void Car::RotateR(DRV8833 driver){
  driver.motorBForward();
  driver.motorAReverse();
}

void Car::initRoute(Coords carCoord, Coords destinationCoord) {
  //logica del recorrido 
}
