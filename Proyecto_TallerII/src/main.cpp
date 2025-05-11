#include "pkg/motor.h"
#include "pkg/comm.h"
#include "pkg/magneto.h"
#include "pkg/encoder.h"
#include "Arduino.h"

// -- Definiciones --
//Recordar que los pines digitales 0 y 1 son RX y TX, respectivamente. Se usan para el moóulo Bluetooth

#define PLANT_PIN 7
#define PIN_ENCODER_A 2
#define PIN_ENCODER_B 3

//Los motores usan los pines digitales 4, 5, 6, 12, y 8.
//El magnetometro usa los pines analogos A5 y A4.

//---------------------------------------------------------

//-- Variables Globales -----------------------------------

//Variables que entran por el bluetooth
float Xcultivo = 0, Ycultivo = 0, RotacionCultivo = 0, RotacionInicial = 0;
float PuntoInicial[2] = {0,0}, PuntoA[2] = {0.1,0.1}, PuntoB [2] = {0,0.2}; //Indice 0 es X, indice 1 es Y

//Variables que son calculadas dentro del codigo
float DistanciaObjetivo, RotacionObjetivo, PosicionActual[2];

//Inicializacion de los objetos de los componentes
motor rueda;
encoder regla_A;
encoder regla_B;
magneto brujula;

// -------------------------------------------------------


//-- Funciones --------------------------------------------

//Entra un numero dado en grados y devuelve el mismo en radianes.
float GradosToRad(float grados){
   return grados * 0.017453;
 }

//Entra un numero dado en radianes y devuelve el mismo en grados.
float RadToGrados(float rad){
   return rad * 57.29578;
 }

double DistPromedioEncoders(){
   return ((regla_A.GetDistancia() + regla_B.GetDistancia())/2.0);
}

//Funcion que toma el vector objetivo como entrada, calculando la distancia a recorrer y el angulo objetivo (En radianes)
void DefinirObjetivos(float Vector[2]){
   DistanciaObjetivo = sqrt(pow((Vector[1] - PosicionActual[1]) ,2) + pow((Vector[0] - PosicionActual[0]) ,2));
   Serial.print("Distancia Objetivo:"); Serial.println(DistanciaObjetivo);
   RotacionObjetivo = atan2(Vector[1] - PosicionActual[1] , Vector[0] - PosicionActual[0]);
   if (RotacionObjetivo < 0) {RotacionObjetivo = RotacionObjetivo + 6.2832;}
   Serial.print("Rotacion Objetivo:"); Serial.println(RotacionObjetivo);
 }

void IrHaciaObjetivos(){
   delay(500);
   //Comenzar rotacion, girando los motores en direcciones opuestas
   if((brujula.DireccionActual() - RotacionObjetivo) > 0.034907){ //Rotar a la derecha
      rueda.GirarDerecha();
      Serial.println("Girando a la derecha...");
   }
   else if((brujula.DireccionActual() - RotacionObjetivo) < -0.034907){ //Rotar a la izquierda
      rueda.GirarIzquierda();
      Serial.println("Girando a la izquierda...");
   }

   //Continuar rotacion hasta que el robot este a menos de 3° (0.034907 Radianes) del angulo objetivo
   while(abs(brujula.DireccionActual() - RotacionObjetivo) > 0.034907){
      delay(10);
   }

   //Detener el robot y esperar un momento
   regla_A.ResetDistancia();
   regla_B.ResetDistancia();
   rueda.Detener();
   delay(500);

   //Moverse hacia adelante hasta estar a menos de 5cm de la distancia objetivo
   rueda.Adelante();
   Serial.println("Moviendo hacia adelante...");
   while((DistanciaObjetivo - DistPromedioEncoders()) > 0.05){
      delay(10);
   }

   //Detener los motores, y reiniciar el contador de distancia
   rueda.Detener();
   regla_A.ResetDistancia();
   regla_B.ResetDistancia();
}

//Pasa un vector del sistema local al sistema global
void Local_a_Global(float (*Vector)[2]){
   float SavedVector[2] = {(*Vector)[0], (*Vector)[1]};
   
   (*Vector)[0] = (SavedVector[0] * cos(RotacionCultivo)) - (SavedVector[1] * sin(RotacionCultivo)) + Xcultivo;
   (*Vector)[1] = (SavedVector[0] * sin(RotacionCultivo)) + (SavedVector[1] * cos(RotacionCultivo)) + Ycultivo;
 }

//Cada vez que se activa el encoder, llama a la funcion del objeto.
void IncrementarDistEncoder_A() {regla_A.IncrementarDistancia();}
void IncrementarDistEncoder_B() {regla_B.IncrementarDistancia();}

//---------------------------------------------------------


void setup() {
   Serial.begin(9600);
   Serial.println("Inicializando el robot...");
   brujula.init();
   rueda.init();
   regla_A.init();
   regla_B.init();

   //Inicializar el pin del encoder, y detectar cada vez que este se activa.
   pinMode(PIN_ENCODER_A, INPUT_PULLUP);
   pinMode(PIN_ENCODER_B, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), IncrementarDistEncoder_A, FALLING);
   attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), IncrementarDistEncoder_B, FALLING);
   pinMode(PLANT_PIN, INPUT_PULLUP); //El componente debe ser conectado entre el pin de la planta, y un pin GND.
   Serial.println("Pines inicializados!");

   //Determina el valor del offset, que arregla los valores de la funcion DireccionActual
   brujula.SetOffsetMagnetometro(GradosToRad(RotacionInicial)); //La rotacion inicial entra por el bluetooth con un valor en grados.
   Serial.println("Offset del magnetometro inicializado!");
   Serial.println("Robot inicializado!");
}  

void loop() {

   // !!!   EL CODIGO DESPUES DE ESTE COMENTARIO SOLO SE DEBE EJECUTAR DESPUES DE REALIZAR LA CONEXION
   //   BLUETOOTH, Y HABER RECIBIDO LOS DATOS NECESARIOS PARA LA NAVEGACION POR MEDIO DE ESTA   !!!

   Local_a_Global(&PuntoA);
   Local_a_Global(&PuntoB);
   RotacionCultivo = GradosToRad(RotacionCultivo); //El dato entra desde el bluetooth con un valor en grados.
   PosicionActual[0] = PuntoInicial[0];
   PosicionActual[1] = PuntoInicial[1];
   Serial.println("Calculos de navegacion completados!");

   //Espera a que la planta se coloque en su lugar
   while (digitalRead(PLANT_PIN)==HIGH)
   {
      Serial.println("Esperando a la planta...");
      delay(500);
   }
   
   //Si detecta la planta
   Serial.println("Planta detectada!");
   Serial.println("Dirigiendose al punto A...");
   DefinirObjetivos(PuntoA);
   IrHaciaObjetivos();
   PosicionActual[0] = PuntoA[0];
   PosicionActual[1] = PuntoA[1];
   Serial.println("Dirigiendose al punto B...");
   DefinirObjetivos(PuntoB);
   IrHaciaObjetivos();
   PosicionActual[0] = PuntoB[0];
   PosicionActual[1] = PuntoB[1];
}

//FIN