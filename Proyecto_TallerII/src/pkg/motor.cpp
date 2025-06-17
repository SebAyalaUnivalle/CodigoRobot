#include "motor.h"

//Pines del controlador DRV8833 conectados al Arduino
#define AIN2 4
#define AIN1 5

#define BIN2 6
#define BIN1 12 

//cambie el orden porque los conecte mal xd atte:santiago

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
    analogWrite(AIN2, 200);
    analogWrite(BIN2, 200);
  }

void motor::GirarDerecha() {
    analogWrite(AIN1, 200);
    analogWrite(BIN2, 200);
  }

void motor::GirarIzquierda() {
    analogWrite(AIN2, 200);
    analogWrite(BIN1, 200);
  }