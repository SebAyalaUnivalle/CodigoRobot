#include "encoder.h"
#include "Arduino.h"

#define PIN_ENCODER 2

encoder::encoder(){
    LongitudArco = 1; //La distancia en metros que recorre el robot cada vez que el encoder es activado.
    DistanciaRecorrida = 0;
}

encoder::~encoder(){}

void encoder::IncrementarDistancia(){
    DistanciaRecorrida = DistanciaRecorrida + LongitudArco;
}

void encoder::ResetDistancia(){DistanciaRecorrida = 0;}

double encoder::GetDistancia(){return DistanciaRecorrida;}