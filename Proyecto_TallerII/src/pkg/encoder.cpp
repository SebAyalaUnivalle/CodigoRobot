#include "encoder.h"
#include "Arduino.h"

encoder::encoder(){}

encoder::~encoder(){}

void encoder::init(){
  DistanciaRecorrida = 0;
  ultimoTiempo = 0;
}

void encoder::IncrementarDistancia(){
  unsigned int tiempoActual = millis();

  // Hacer que el tiempo minimo entre cada medicion sea 17ms
  if (tiempoActual - ultimoTiempo > 10) {
    DistanciaRecorrida = DistanciaRecorrida + LongitudArco;
  }
  
  ultimoTiempo = tiempoActual;
}

void encoder::ResetDistancia(){DistanciaRecorrida = 0;}

double encoder::GetDistancia(){return DistanciaRecorrida;}