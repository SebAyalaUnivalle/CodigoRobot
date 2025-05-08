#include <math.h>
#include "magneto.h"
#include "../utils/QMC5883LCompass.h"

//Inicializa el magnetometro y los 3 componentes de la fuerza magnetica que detecta el componente.
magneto::magneto(){}

magneto::~magneto(){}

void magneto::init(){
    Offset = 0;
    magnetometro.init();
    magnetometro.setCalibration(-700, 503, -58, 1160,-513,-373); // X_min, X_max, Y_min, Y_max, z_min,z_max  
}

//Retorna el angulo entre la direccion hacia el frente del robot, y el eje X positivo del sistema global.
float magneto::DireccionActual(){ //En Radianes
    float angulo;
    magnX = magnetometro.getX();
    magnY = magnetometro.getY();
    angulo = atan2(magnY, magnX);
    if (angulo < 0){angulo = angulo + 6.28319;}
    angulo = angulo - Offset;
    if (angulo < 0){angulo = angulo + 6.28319;}
    else if (angulo > 6.28319){angulo = angulo - 6.28319;}
    return angulo;
}

void magneto::SetOffsetMagnetometro(float AnguloInicial){
    float AnguloMagnetico;
    magnX = magnetometro.getX();
    magnY = magnetometro.getY();
    AnguloMagnetico = atan2(magnY, magnX);
    if (AnguloMagnetico < 0){AnguloMagnetico = AnguloMagnetico + 6.28319;}
    Offset = AnguloMagnetico + AnguloInicial - 1.5708;
}