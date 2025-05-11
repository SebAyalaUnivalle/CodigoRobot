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
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void motor::Adelante() {
    digitalWrite(AIN1, HIGH);
    digitalWrite(BIN1, HIGH);
  }

void motor::GirarDerecha() {
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
  }

void motor::GirarIzquierda() {
    digitalWrite(AIN1, HIGH);
    digitalWrite(BIN2, HIGH);
  }

void motor::Adelante(int speed) {
    analogWrite(AIN1, speed);
    digitalWrite(AIN2, LOW);
    analogWrite(BIN1, speed);
    digitalWrite(BIN2, LOW);
  }

void motor::GirarDerecha(int speed) {
    analogWrite(AIN2, speed);
    digitalWrite(AIN1, LOW);
    analogWrite(BIN1, speed);
    digitalWrite(BIN2, LOW);
  }

void motor::GirarIzquierda(int speed) {
    analogWrite(AIN1, speed);
    digitalWrite(AIN2, LOW);
    analogWrite(BIN2, speed);
    digitalWrite(BIN1, LOW);
  }