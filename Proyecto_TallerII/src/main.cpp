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
//#define LED_green 9
//#define LED_red 8
//#define intensidad 120

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
SoftwareSerial BT(11,10); //(RX TX) Inicalización del módulo Bluetooth HM10

// -------------------------------------------------------

//-- Funciones --------------------------------------------

//Entran los datos de los puntos y del cultivo como un String con varios caracteres. La función los separa

void procesarDatosAB(String cmd) {
  String cmdSinDelim = cmd.substring(1, cmd.length() - 1);
  // Separar por ';'
  int separador = cmdSinDelim.indexOf(';');
  if (separador == -1) {
    Serial.println("ERROR: Separador ';' no encontrado");
    return;
  }
  
  String segmentoA = cmdSinDelim.substring(0, separador);     // Punto A: x,y,θ
  String segmentoB = cmdSinDelim.substring(separador + 1);    // Punto B: x,y
  int c1 = segmentoA.indexOf(',');
  int c2 = segmentoA.indexOf(',', c1 + 1);
  if (c1 != -1 && c2 != -1) {
    PuntoInicial[0] = segmentoA.substring(0, c1).toFloat();
    PuntoInicial[1] = segmentoA.substring(c1 + 1, c2).toFloat();
    AnguloInicial = segmentoA.substring(c2 + 1).toFloat();
  } 
  // Procesar Punto B
  int c3 = segmentoB.indexOf(',');
  if (c3 != -1) {
    PuntoB[0] = segmentoB.substring(0, c3).toFloat();
    PuntoB[1] = segmentoB.substring(c3 + 1).toFloat();
  }
}

void procesarDatosC_Cultivo(String cmd) {  
  // Eliminar delimitadores
  String cmdSinDelim = cmd.substring(1, cmd.length() - 1);
  // Separar por ';'
  int separador = cmdSinDelim.indexOf(';');
  if (separador == -1) {
    Serial.println("ERROR: Separador ';' no encontrado");
    return;
  }
  
  String segmentoC = cmdSinDelim.substring(0, separador);         // Punto C: x,y
  String segmentoCultivo = cmdSinDelim.substring(separador + 1);  // Cultivo: x,y,θ


  int c1 = segmentoC.indexOf(',');
  if (c1 != -1) {
    PuntoC[0] = segmentoC.substring(0, c1).toFloat();
    PuntoC[1] = segmentoC.substring(c1 + 1).toFloat();
  } 

  // Procesar Punto Cultivo
  int c2 = segmentoCultivo.indexOf(',');
  int c3 = segmentoCultivo.indexOf(',', c2 + 1);
  if (c2 != -1 && c3 != -1) {
    Xcultivo = segmentoCultivo.substring(0, c2).toFloat();
    Ycultivo = segmentoCultivo.substring(c2 + 1, c3).toFloat();
    RotacionCultivo = segmentoCultivo.substring(c3 + 1).toFloat();
  }
}

bool validarFormato(String cmd) {
  // Validar formato básico
  if (cmd.startsWith("$") && cmd.endsWith("@")) {
    // Debe tener al menos un ';' para separar los dos puntos
    return (cmd.indexOf(';') > 0);
  } 
  else if (cmd.startsWith("&") && cmd.endsWith("#")) {
    // Debe tener al menos un ';' para separar punto C y cultivo
    return (cmd.indexOf(';') > 0);
  }
  return false;
}

void procesarComando(String cmd) {  
  if (cmd.startsWith("$") && cmd.endsWith("@")) {
      procesarDatosAB(cmd);
  } 
  else if (cmd.startsWith("&") && cmd.endsWith("#")) {
      procesarDatosC_Cultivo(cmd);
  }
  else if (cmd.startsWith("S")) {
    //Serial.println("-> Comando de parada recibido");
   // rueda.Detener();  // Comando de parada
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
   Serial.print("Angulo Magnetico Objetivo: "); Serial.println(AnguloMagneticoObjetivo);

   //Comenzar rotacion, girando los motores en direcciones opuestas
   if((AnguloMagneticoObjetivo - brujula.DireccionMagnetica()) > 0.08){ //Rotar a la derecha
      rueda.GirarDerecha();
      //void enviarPosicionActual();
      //Serial.println("Girando a la derecha...");
   }
   else if((AnguloMagneticoObjetivo - brujula.DireccionMagnetica()) < -0.08){ //Rotar a la izquierda
      rueda.GirarIzquierda();
      //void enviarPosicionActual();
      //Serial.println("Girando a la izquierda...");
   }

   //Continuar rotacion hasta que el robot este a alrededor de 5° del angulo objetivo
   while(abs(AnguloMagneticoObjetivo - brujula.DireccionMagnetica()) > 0.08){
      //Serial.print("Direccion actual: "); Serial.println(RadToGrados(brujula.DireccionMagnetica()));
      //Serial.print("Diferencia: "); Serial.println(RadToGrados(abs(AnguloMagneticoObjetivo - brujula.DireccionMagnetica())));
      //void enviarPosicionActual();
      delay(5);
   }

   //Detener el robot y esperar un momento
   regla_A.ResetDistancia();
   regla_B.ResetDistancia();
   rueda.Detener();

   delay(500);

   //Moverse hacia adelante hasta estar a menos de 5cm de la distancia objetivo
   rueda.Adelante();
   Serial.println("Moviendo hacia adelante...");
   while((DistanciaObjetivo - DistPromedioEncoders()) > 0.08){
   //void enviarPosicionActual();
   //Serial.print("Distancia Recorrida:  "); Serial.println(DistPromedioEncoders());
   delay(5);
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
   //pinMode(LED_green,OUTPUT);
   //pinMode(LED_red, OUTPUT);
   attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), IncrementarDistEncoder_A, FALLING);
   attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), IncrementarDistEncoder_B, FALLING);
   pinMode(PLANT_PIN, INPUT_PULLUP); //El componente debe ser conectado entre el pin de la planta, y un pin GND.
   Serial.println("Pines inicializados!");

   Serial.println("Robot inicializado!");
}  

void loop() {
   // bool detener = false;
    Serial.println("Iniciando loop...");
    //digitalWrite(LED_red, HIGH);
    //digitalWrite(LED_green, LOW);


   //Recepción de datos de los puntos y del cultivo
   static String Comando = "";
   static bool comandoIniciado = false;
   
   while(BT.available()==0){delay(500); Serial.println("Esperando datos...");}

   while (BT.available()>0){
      char C = BT.read();
      
      // Detectar inicio de comando
      if (C == '$' || C == '&') {
        // Si ya teníamos un comando iniciado, lo descartamos
        if (comandoIniciado) {
          Serial.println("ADVERTENCIA: Comando anterior incompleto descartado");
        }
        Comando = String(C);  // Iniciar nuevo comando
        comandoIniciado = true;
        Serial.println("-> Nuevo comando iniciado con: " + String(C));
      }
      // Solo procesar caracteres si tenemos un comando iniciado
      else if (comandoIniciado) {
        // Verificar delimitadores de fin de comando
        if (C == '@' || C == '#'){
           Comando += C;  // Añadir el delimitador final
           Comando.trim(); // Eliminar espacios en blanco
                    
           
           if (Comando.length() > 2 && validarFormato(Comando)) {
             procesarComando(Comando);
           } else {
             Serial.println("ERROR: Comando con formato inválido o demasiado corto");
           }
           
           // Limpiar buffer y estado
           Comando = "";
           comandoIniciado = false;
        }
        else {
          Comando += C;
          if (Comando.length() > 100) {
            Comando = "";
            comandoIniciado = false;
          }
        }
      }
   }
   
   Serial.println("---Puntos Pre-Transformacion---");
   Serial.print("Punto B: X: "); Serial.print(PuntoB[0]); Serial.print("   Y: "); Serial.println(PuntoB[1]);
   Serial.print("Punto C: X: "); Serial.print(PuntoC[0]); Serial.print("   Y: "); Serial.println(PuntoC[1]);
   // !!!   EL CODIGO DESPUES DE ESTE COMENTARIO SOLO SE DEBE EJECUTAR DESPUES DE REALIZAR LA CONEXION
   //   BLUETOOTH, Y HABER RECIBIDO LOS DATOS NECESARIOS PARA LA NAVEGACION POR MEDIO DE ESTA   !!!

   Local_a_Global(&PuntoB);
   Local_a_Global(&PuntoC);
   RotacionCultivo = GradosToRad(RotacionCultivo); //El dato entra desde el bluetooth con un valor en grados.
   AnguloActual = GradosToRad(AnguloInicial);
   PosicionActual[0] = PuntoInicial[0];
   PosicionActual[1] = PuntoInicial[1];
   //Serial.println("Calculos de navegacion completados!");

   Serial.println("---Puntos Post-Transformacion---");
   Serial.print("Punto B: X: "); Serial.print(PuntoB[0]); Serial.print("   Y: "); Serial.println(PuntoB[1]);
   Serial.print("Punto C: X: "); Serial.print(PuntoC[0]); Serial.print("   Y: "); Serial.println(PuntoC[1]);

   Serial.println("Esperando planta...");


   //Espera a que la planta se coloque en su lugar
   while (digitalRead(PLANT_PIN)==HIGH)
   {
      Serial.println("Esperando a la planta...");
      delay(500);
   }
    
   //Si detecta la planta
   //Parte de un Punto Global A, y se dirige a dos puntos Locales, primero el B, y luego el C.
      //digitalWrite(LED_red,LOW);
      //digitalWrite(LED_green,HIGH);
      Serial.println("Planta detectada!");
      Serial.println("Dirigiendose al punto B...");
      DefinirObjetivos(PuntoB);    //  Dentro de estas dos funciones deben incluirse la funcion encargada de enviar los datos del robot
      IrHaciaObjetivos();          //
      PosicionActual[0] = PuntoB[0];
      PosicionActual[1] = PuntoB[1];
      AnguloActual = AnguloObjetivo;
      Serial.println("Dirigiendose al punto C...");
      DefinirObjetivos(PuntoC);
      IrHaciaObjetivos();
      PosicionActual[0] = PuntoC[0];
      PosicionActual[1] = PuntoC[1];
      AnguloActual = AnguloObjetivo;

   //Antes de volver al origen, la planta debe ser recolectada en el punto C.
      void enviarPosicionActual();
      //digitalWrite(LED_red,HIGH);
      while(digitalRead(PLANT_PIN)==LOW){
        Serial.println("Esperando Recolección de planta...");
        delay(500);
      }

      //digitalWrite(LED_red,LOW);
      
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
      //while(detener==false){delay(100);} Quite esto por que la idea es que el robot pueda volver a ponerse operativo
      Serial.println("Final del loop alcanzado!");
      delay(500);
}

//FIN