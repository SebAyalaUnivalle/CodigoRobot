#include <math.h>

#include "Wire.h"
#include "I2Cdev.h"
#include "HMC5883L.h"
#include "magneto.h"

//Inicializa el magnetometro y los 3 componentes de la fuerza magnetica que detecta el componente.
magneto::magneto(){
    magnX = 0;
    magnY = 0;
    magnZ = 0;
    Serial.begin(9600);
    Wire.begin();
    magnetometro.initialize();
}

magneto::~magneto(){}

//FALTA CORREGIR LOS ANGULOS GENERADOS POR ESTA FUNCION, AHORA MISMO GENERA UN ANGULO CON RESPECTO AL NORTE MAGNETICO, NO EL EJE X POSITIVO GLOBAL
float magneto::DireccionActual(){ //En Radianes
    float angulo;
    magnetometro.getHeading(&magnX, &magnY, &magnZ);
    angulo = atan2(magnY, magnX);
    if (angulo < 0){angulo = angulo + 6.28319;}
    return angulo;
}