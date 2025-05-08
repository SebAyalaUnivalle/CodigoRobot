#include "encoder.h"
#include "Arduino.h"

#define PIN_ENCODER 2

encoder::encoder(){
    LongitudArco = 0.00816814; //La distancia en metros que recorre el robot cada vez que el encoder es activado.
    DistanciaRecorrida = 0;
    ultimoTiempo = 0;
}

encoder::~encoder(){}

void encoder::IncrementarDistancia(){
  int tiempoActual = micros();

  // Hacer que el tiempo minimo entre cada medicion sea 250us
  if (tiempoActual - ultimoTiempo > 250) {
    DistanciaRecorrida = DistanciaRecorrida + LongitudArco;
  }
    
}

void encoder::ResetDistancia(){DistanciaRecorrida = 0;}

double encoder::GetDistancia(){return DistanciaRecorrida;}