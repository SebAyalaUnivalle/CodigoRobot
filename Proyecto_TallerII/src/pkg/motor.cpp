#include "motor.h"

//Pines del controlador DRV8833 conectados al Arduino
#define AIN1 4
#define AIN2 5
#define BIN1 6
#define BIN2 12 

motor::motor(){}

motor::~motor(){}

void motor::init(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  //Conectar el pin STBY a +5V
}

void motor::Detener() {
  analogWrite(AIN1, LOW);
  analogWrite(AIN2, LOW);
  analogWrite(BIN1, LOW);
  analogWrite(BIN2, LOW);
}

void motor::Adelante() {
    analogWrite(AIN2, 130);
    analogWrite(BIN2, 130);
  }

void motor::GirarDerecha() {
    analogWrite(AIN2, 130);
    analogWrite(BIN1, 130);
  }

void motor::GirarIzquierda() {
    analogWrite(AIN1, 130);
    analogWrite(BIN2, 130);
  }