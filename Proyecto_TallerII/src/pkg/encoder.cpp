#include "encoder.h"
#include "Arduino.h"

encoder::encoder(){}

encoder::~encoder(){}

void encoder::init(){
  DistanciaRecorrida = 0;
  ultimoTiempo = 0;
}

void encoder::IncrementarDistancia(){
  unsigned long tiempoActual = micros();

  // Hacer que el tiempo minimo entre cada medicion sea 250us
  if (tiempoActual - ultimoTiempo > 250) {
    DistanciaRecorrida = DistanciaRecorrida + LongitudArco;
  }
  
  ultimoTiempo = tiempoActual;
}

void encoder::ResetDistancia(){DistanciaRecorrida = 0;}

double encoder::GetDistancia(){return DistanciaRecorrida;}