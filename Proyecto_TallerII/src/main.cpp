#include "pkg/motor.h"
#include "pkg/magneto.h"
#include "pkg/encoder.h"
#include "Arduino.h"
#include <SoftwareSerial.h>

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
float Xcultivo = 0, Ycultivo = 0, RotacionCultivo = 0, AnguloInicial = 0;
float PuntoInicial[2] = {0,0}, PuntoB[2] = {0,0.5}, PuntoC [2] = {0.5,0.5}; //Indice 0 es X, indice 1 es Y

//Variables que son calculadas dentro del codigo
float DistanciaObjetivo, AnguloObjetivo, AnguloActual, PosicionActual[2];

//Inicializacion de los objetos de los componentes
motor rueda;
encoder regla_A;
encoder regla_B;
magneto brujula;
SoftwareSerial BT(1,0); //(RX TX) Inicalización del módulo Bluetooth HC05

// -------------------------------------------------------


//-- Funciones --------------------------------------------

//Entran los datos de los puntos y del cultivo como un String con varios caracteres. La función los separa

void procesarComando(String cmd) {
  // Verificar si es un comando válido (debe empezar con $ y terminar con @)
  if (cmd.startsWith("$") && cmd.endsWith("@")) {
    // Eliminar los caracteres de inicio y fin
    cmd = cmd.substring(1, cmd.length() - 1);
    
    // Dividir la cadena por punto y coma para obtener cada segmento
    int separador1 = cmd.indexOf(';');
    int separador2 = cmd.indexOf(';', separador1 + 1);
    int separador3 = cmd.indexOf(';', separador2 + 1);
    
    if (separador1 != -1) {
      // Procesar segmento 1: CoordXA, CoordYA, AnguloRobot
      String segmento1 = cmd.substring(0, separador1);
      int coma1 = segmento1.indexOf(',');
      int coma2 = segmento1.indexOf(',', coma1 + 1);
      
      if (coma1 != -1 && coma2 != -1) {
        PuntoInicial[0] = segmento1.substring(0, coma1).toFloat();
        PuntoInicial[1] = segmento1.substring(coma1 + 1, coma2).toFloat();
        AnguloInicial = segmento1.substring(coma2 + 1).toFloat();
      }
      
      // Procesar segmento 2: CoordXB, CoordYB
      if (separador2 != -1) {
        String segmento2 = cmd.substring(separador1 + 1, separador2);
        int coma3 = segmento2.indexOf(',');
        
        if (coma3 != -1) {
          PuntoB[0] = segmento2.substring(0, coma3).toFloat();
          PuntoB[1] = segmento2.substring(coma3 + 1).toFloat();
        }
        
        // Procesar segmento 3: CoordXC, CoordYC
        if (separador3 != -1) {
          String segmento3 = cmd.substring(separador2 + 1, separador3);
          int coma4 = segmento3.indexOf(',');
          
          if (coma4 != -1) {
            PuntoC[0] = segmento3.substring(0, coma4).toFloat();
            PuntoC[1] = segmento3.substring(coma4 + 1).toFloat();
          }
          
          // Procesar segmento 4: CoordXCultivo, CoordYCultivo, AnguloCultivo
          String segmento4 = cmd.substring(separador3 + 1);
          int coma5 = segmento4.indexOf(',');
          int coma6 = segmento4.indexOf(',', coma5 + 1);
          
          if (coma5 != -1 && coma6 != -1) {
            Xcultivo = segmento4.substring(0, coma5).toFloat();
            Ycultivo = segmento4.substring(coma5 + 1, coma6).toFloat();
            RotacionCultivo = segmento4.substring(coma6 + 1).toFloat();
          }
        }
      }
    }
  } 
  else if (cmd == "S") {
    // Comando de parada
    rueda.Detener();
  }
}

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

float GetVelocidad()
{
    static double ultimaDistancia = 0;
    static unsigned long ultimoTiempo = micros();

    double distanciaActual = DistPromedioEncoders();
    unsigned long tiempoActual = micros();

    double velocidad = (distanciaActual - ultimaDistancia) / ((tiempoActual - ultimoTiempo) / 1000000); // Convertimos us a s

    ultimaDistancia = distanciaActual;
    ultimoTiempo = tiempoActual;

    return velocidad; // Expresada en m/s
}

//Manda los datos de movimiento del robot a la App, para ser visualizados
void enviarPosicionActual() {
  float velocidad = GetVelocidad();
  String mensaje = "";

  mensaje += String(velocidad, 2) + ","; // Velocidad
  mensaje += String(RadToGrados(brujula.DireccionMagnetica()), 2) + ","; // Ángulo
  mensaje += String(PosicionActual[0], 2) + ","; // X
  mensaje += String(PosicionActual[1], 2); // Y

  BT.println(mensaje); // '\n' se usa como delimitador
}


//Funcion que toma el vector objetivo como entrada, calculando la distancia a recorrer y el angulo objetivo (En radianes)
void DefinirObjetivos(float Vector[2]){
   DistanciaObjetivo = sqrt(pow((Vector[1] - PosicionActual[1]) ,2) + pow((Vector[0] - PosicionActual[0]) ,2));
   Serial.print("Distancia Objetivo:"); Serial.println(DistanciaObjetivo);

   AnguloObjetivo = atan2(Vector[1] - PosicionActual[1] , Vector[0] - PosicionActual[0]);
   if (AnguloObjetivo < 0) {AnguloObjetivo = AnguloObjetivo + 6.2832;}
   Serial.print("Angulo Objetivo:"); Serial.println(AnguloObjetivo);
 }

void IrHaciaObjetivos(){
   delay(500);

   float AnguloMagneticoObjetivo = brujula.DireccionMagnetica() + (AnguloActual - AnguloObjetivo);
   if(AnguloMagneticoObjetivo<0){AnguloMagneticoObjetivo += 6.2832;}
   Serial.println(AnguloMagneticoObjetivo);

   //Comenzar rotacion, girando los motores en direcciones opuestas
   if((AnguloMagneticoObjetivo - brujula.DireccionMagnetica()) > 0.06){ //Rotar a la derecha
      rueda.GirarDerecha();
      Serial.println("Girando a la derecha...");
   }
   else if((AnguloMagneticoObjetivo - brujula.DireccionMagnetica()) < -0.06){ //Rotar a la izquierda
      rueda.GirarIzquierda();
      Serial.println("Girando a la izquierda...");
   }

   //Continuar rotacion hasta que el robot este a alrededor de 3° del angulo objetivo
   while(abs(AnguloMagneticoObjetivo - brujula.DireccionMagnetica()) > 0.06){
      Serial.print("Direccion actual: "); Serial.println(brujula.DireccionMagnetica());
      Serial.print("Diferencia: "); Serial.println(abs(AnguloMagneticoObjetivo - brujula.DireccionMagnetica()));
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
    Serial.print("Distancia Recorrida:  "); Serial.println(DistPromedioEncoders());
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
   BT.begin(9600);
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

   Serial.println("Robot inicializado!");
}  

void loop() {
    bool detener = false;

   //Recepción de datos de los puntos y del cultivo
   static String Comando = "";
   while (BT.available()){
      char C = BT.read();
      if (C == '@'){
         Comando += C;
         procesarComando(Comando);
         Comando = "";
      }
      else 
      Comando += C; 
   }

   // !!!   EL CODIGO DESPUES DE ESTE COMENTARIO SOLO SE DEBE EJECUTAR DESPUES DE REALIZAR LA CONEXION
   //   BLUETOOTH, Y HABER RECIBIDO LOS DATOS NECESARIOS PARA LA NAVEGACION POR MEDIO DE ESTA   !!!

   Local_a_Global(&PuntoB);
   Local_a_Global(&PuntoC);
   RotacionCultivo = GradosToRad(RotacionCultivo); //El dato entra desde el bluetooth con un valor en grados.
   AnguloActual = GradosToRad(AnguloInicial);
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
   //Parte de un Punto Global A, y se dirige a dos puntos Locales, primero el B, y luego el C.
   Serial.println("Planta detectada!");
   Serial.println("Dirigiendose al punto B...");
   DefinirObjetivos(PuntoB);
   IrHaciaObjetivos();
   PosicionActual[0] = PuntoB[0];
   PosicionActual[1] = PuntoB[1];
   AnguloActual = AnguloObjetivo;
   Serial.println("Dirigiendose al punto C...");
   DefinirObjetivos(PuntoC);
   IrHaciaObjetivos();
   PosicionActual[0] = PuntoC[0];
   PosicionActual[1] = PuntoC[1];
   AnguloActual = AnguloObjetivo;
   Serial.println("Dirigiendose al punto B...");
   DefinirObjetivos(PuntoB);
   IrHaciaObjetivos();
   PosicionActual[0] = PuntoB[0];
   PosicionActual[1] = PuntoB[1];
   AnguloActual = AnguloObjetivo;
   Serial.println("Volviendo a la posicion inicial...");
   DefinirObjetivos(PuntoInicial);
   IrHaciaObjetivos();
   PosicionActual[0] = PuntoInicial[0];
   PosicionActual[1] = PuntoInicial[1];
   AnguloActual = AnguloObjetivo;
   while(detener==false){delay(100);}
}

//FIN