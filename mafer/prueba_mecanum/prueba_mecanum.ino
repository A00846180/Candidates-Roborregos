// =====================================================================
//  Prueba de las mecanum - Candidates 2026
//  Solo mueve el robot: adelante, atras, a los lados y girar.
//  Pines y forma de mover los motores igual que en nuestro codigo que
//  ya funcionaba (IN1 con pwm, IN2 en alto o bajo segun el sentido).
//
//  Se controla desde el monitor serie (115200):
//    w = adelante        s = atras
//    a = izquierda       d = derecha     (de lado, sin girar)
//    q = girar izq       e = girar der   (en su lugar)
//    x = parar
//    + / - = mas o menos velocidad
//    p = demo: hace todos los movimientos solito, 1 seg cada uno
//    1 2 3 4 = mover solo un motor, adelante y luego atras (para revisar)
//
//  Cada movimiento dura "tiempoMovimiento" y luego se para solo,
//  para que no se nos vaya de la mesa.
// =====================================================================

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ----------------------------- PINES ---------------------------------
// asi quedaron despues de probar llanta por llanta:
// el driver de los pines 16-19 mueve las llantas de ATRAS
// y el de los pines 23-27 las de ADELANTE
#define DEL_IZQ_IN1   23
#define DEL_IZQ_IN2   25
#define DEL_DER_IN1   26
#define DEL_DER_IN2   27
#define TRAS_IZQ_IN1  16
#define TRAS_IZQ_IN2  17
#define TRAS_DER_IN1  18
#define TRAS_DER_IN2  19

#define PWM_FREQ        1000
#define PWM_RESOLUTION  8

//                         del.izq      del.der      tras.izq      tras.der
const uint8_t pinIN1[4] = { DEL_IZQ_IN1, DEL_DER_IN1, TRAS_IZQ_IN1, TRAS_DER_IN1 };
const uint8_t pinIN2[4] = { DEL_IZQ_IN2, DEL_DER_IN2, TRAS_IZQ_IN2, TRAS_DER_IN2 };
const char* nombreMotor[4] = {"delantero izq", "delantero der", "trasero izq", "trasero der"};

// la delantera derecha giraba al reves, por eso va en true
bool alReves[4] = {false, true, false, false};

// --------------------------- AJUSTES ---------------------------------
int velocidad = 160;                      // 0 a 255
unsigned long tiempoMovimiento = 1000;    // ms que dura cada comando

// ---------------------------- PWM ------------------------------------
// el core 3.x del ESP32 cambio como se usa el pwm, por eso esto
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define CONECTAR_PWM(pin, canal)        ledcAttach((pin), PWM_FREQ, PWM_RESOLUTION)
  #define ESCRIBIR_PWM(pin, canal, valor) ledcWrite((pin), (valor))
#else
  #define CONECTAR_PWM(pin, canal)        do { ledcSetup((canal), PWM_FREQ, PWM_RESOLUTION); ledcAttachPin((pin), (canal)); } while (0)
  #define ESCRIBIR_PWM(pin, canal, valor) ledcWrite((canal), (valor))
#endif

// --------------------------- MOTORES ---------------------------------
// vel de -255 (atras) a 255 (adelante), igual que el setMotor que ya teniamos:
// adelante -> IN2 en bajo y pwm en IN1
// atras    -> IN2 en alto y pwm "al reves" en IN1
void moverMotor(int i, int vel) {
  if (alReves[i]) vel = -vel;
  vel = constrain(vel, -255, 255);
  if (vel >= 0) {
    digitalWrite(pinIN2[i], LOW);
    ESCRIBIR_PWM(pinIN1[i], i, vel);
  } else {
    digitalWrite(pinIN2[i], HIGH);
    ESCRIBIR_PWM(pinIN1[i], i, 255 + vel);   // vel es negativa aqui
  }
}

// la formula de las mecanum:
//   adelante: + adelante, - atras
//   lado:     + derecha,  - izquierda
//   giro:     + a la derecha, - a la izquierda
void mover(int adelante, int lado, int giro) {
  int delIzq  = adelante + lado + giro;
  int delDer  = adelante - lado - giro;
  int trasIzq = adelante - lado + giro;
  int trasDer = adelante + lado - giro;

  // si alguno se pasa de 255 bajamos todos parejo
  int mayor = max(max(abs(delIzq), abs(delDer)), max(abs(trasIzq), abs(trasDer)));
  if (mayor > 255) {
    delIzq  = delIzq  * 255 / mayor;
    delDer  = delDer  * 255 / mayor;
    trasIzq = trasIzq * 255 / mayor;
    trasDer = trasDer * 255 / mayor;
  }

  moverMotor(0, delIzq);
  moverMotor(1, delDer);
  moverMotor(2, trasIzq);
  moverMotor(3, trasDer);
}

void parar() {
  for (int i = 0; i < 4; i++) moverMotor(i, 0);
}

// los 6 movimientos, para usarlos facil despues
void irAdelante()      { mover( velocidad, 0, 0); }
void irAtras()         { mover(-velocidad, 0, 0); }
void irDerecha()       { mover(0,  velocidad, 0); }
void irIzquierda()     { mover(0, -velocidad, 0); }
void girarDerecha()    { mover(0, 0,  velocidad); }
void girarIzquierda()  { mover(0, 0, -velocidad); }

// hace el movimiento un rato y se para
void moverUnRato(void (*movimiento)(), const char* nombre) {
  Serial.println(nombre);
  movimiento();
  delay(tiempoMovimiento);
  parar();
}

void demo() {
  Serial.println("--- demo ---");
  moverUnRato(irAdelante, "adelante");       delay(500);
  moverUnRato(irAtras, "atras");             delay(500);
  moverUnRato(irDerecha, "derecha");         delay(500);
  moverUnRato(irIzquierda, "izquierda");     delay(500);
  moverUnRato(girarDerecha, "girar der");    delay(500);
  moverUnRato(girarIzquierda, "girar izq");
  Serial.println("--- fin demo ---");
}

// mueve un solo motor hacia adelante y luego hacia atras
void probarUnMotor(int i) {
  Serial.printf("%s: hacia ADELANTE\n", nombreMotor[i]);
  moverMotor(i, velocidad);
  delay(tiempoMovimiento);
  parar();
  delay(700);
  Serial.printf("%s: hacia ATRAS\n", nombreMotor[i]);
  moverMotor(i, -velocidad);
  delay(tiempoMovimiento);
  parar();
}

void ayuda() {
  Serial.println("");
  Serial.println("w adelante | s atras | a izquierda | d derecha");
  Serial.println("q girar izq | e girar der | x parar");
  Serial.println("+ / - velocidad | p demo | 1-4 un motor (adelante y atras)");
  Serial.printf("velocidad actual: %d\n", velocidad);
}

// ------------------------- SETUP Y LOOP ------------------------------
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(pinIN2[i], OUTPUT);
    digitalWrite(pinIN2[i], LOW);
    CONECTAR_PWM(pinIN1[i], i);
  }
  parar();
  delay(500);
  Serial.println("listo, prueba de mecanum");
  ayuda();
}

void loop() {
  if (!Serial.available()) return;
  char letra = Serial.read();
  if (letra == '\n' || letra == '\r' || letra == ' ') return;

  switch (letra) {
    case 'w': moverUnRato(irAdelante, "adelante"); break;
    case 's': moverUnRato(irAtras, "atras"); break;
    case 'a': moverUnRato(irIzquierda, "izquierda"); break;
    case 'd': moverUnRato(irDerecha, "derecha"); break;
    case 'q': moverUnRato(girarIzquierda, "girar izq"); break;
    case 'e': moverUnRato(girarDerecha, "girar der"); break;
    case 'x': parar(); Serial.println("parado"); break;
    case 'p': demo(); break;
    case '+': velocidad = min(velocidad + 20, 255); Serial.printf("velocidad %d\n", velocidad); break;
    case '-': velocidad = max(velocidad - 20, 60);  Serial.printf("velocidad %d\n", velocidad); break;
    case '1': case '2': case '3': case '4': probarUnMotor(letra - '1'); break;
    default: ayuda(); break;
  }
}
