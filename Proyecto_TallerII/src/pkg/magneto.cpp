#include <math.h>
#include "magneto.h"
#include "../utils/QMC5883LCompass.h"

//Inicializa el magnetometro y los 3 componentes de la fuerza magnetica que detecta el componente.
magneto::magneto(){}

magneto::~magneto(){}

void magneto::init(){
    magnetometro.init();
    magnetometro.setCalibration(-1211, 523, -1271, 580, -805, 865); // X_min, X_max, Y_min, Y_max, z_min,z_max  
}

//Retorna el angulo entre la direccion hacia el frente del robot, y el eje X positivo del sistema global.
float magneto::DireccionMagnetica(){ //En Radianes
    float angulo = 0;
    for(int i=0; i<3; i++){
        magnetometro.read();
        float anguloTemp;
        magnX = -magnetometro.getX();
        magnY = magnetometro.getY();
        anguloTemp = atan2(magnY, magnX);
        if (anguloTemp < 0){anguloTemp += 6.28319;}
        angulo += anguloTemp;
        delay(1);
    }
    return (angulo/3);
}