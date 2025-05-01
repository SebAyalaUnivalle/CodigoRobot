#ifndef CAR_H
#define CAR_H

#include "motor.h"

struct Coords {
  float x, y, r; //(posición en x, posición en y, rotación)
  };

// Clase principal
class Car {
  public:
    // Constructor
    Car(Coords c, float s) : coord(c), speed(s) {}

    // Metodos generales                                       // [Parametros]
    Coords getCoords();                                           
    float getSpeed();                                             
     
    // Metodos de movimiento
    void stopCar(DRV8833 driver);
    void moveCarForward(DRV8833 driver);                         // [Driver]
    void moveCarBack(DRV8833 driver);                            // [Driver]

    // Metodos para rotar el carro (SOBRE SU PROPIO EJE)
    void RotateR(DRV8833 driver);
    void RotateL(DRV8833 driver);

    
    void initRoute(Coords carCoord, Coords destinationCoord);    // [Coordenadas del carro y de destino]

  
  private:
    // [Atributos]
    Coords coord;
    float speed;
};

#endif // CAR_H