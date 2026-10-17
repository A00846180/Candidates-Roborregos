// =====================================================================
//  Candidates 2026 - Pista B (Niveles)
//  Robot: ESP32 DevKit V1, 2 drivers DRV8833, llantas mecanum
//
//  Librerias que hay que instalar:
//    - Adafruit TCS34725 (sensor de color)
//    - Adafruit SSD1306 + Adafruit GFX (pantallita OLED)
//  El MPU6050 lo leemos directo por I2C, no ocupa libreria.
//
//  Cosas que dimos por hecho (si algo cambia, se ajusta aqui abajo):
//    - En I2C estan el TCS34725 (0x29), el MPU6050 (0x68) y la OLED (0x3C)
//    - El IR de la pelota va en el pin 15 y manda LOW cuando la ve
//    - El sensor de color esta abajo, mas o menos en el centro del robot
//    - En INICIO el robot va viendo hacia la cuadricula de la seccion 1
//      (a eso le decimos "norte") y el checkpoint 1 queda a la derecha
//      de la fila de en medio, como en la imagen 2.1 del reglamento
//    - En un checkpoint (lack of progress) lo ponemos viendo hacia la
//      siguiente seccion, con la pelota enfrente
//    - No tenemos encoders, asi que las distancias van por tiempo y las
//      corregimos con los ultrasonicos y el giroscopio
//
//  Como se usa en la pista:
//    Se prende con el switch y NO se mueve como 2 seg (calibra el giro).
//    Lee el color del piso: si es verde arranca en la seccion 1, si es
//    rojo se va a la seccion que sigue del ultimo checkpoint que guardo.
//    Si escogio mal (ej. el 4o lack of progress), tapar el ultrasonico
//    de enfrente como 1 seg para cambiar de seccion.
//
//  Para probar cosas: monitor serie a 115200 y escribir "ayuda".
//  Orden que nos funciono para calibrar:
//    m -> motores, g -> giroscopio, s <angulo> -> garra,
//    cal <color> en cada color, mov f 90 / mov d 90 -> distancias,
//    giro 90, y luego run 1 / run 2 / run 3
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_TCS34725.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ----------------------------- PINES ---------------------------------
// motores en este orden: del. izq, del. der, tras. izq, tras. der
// pinMotorA = el pin que lo hace ir hacia ADELANTE, pinMotorB = hacia atras
// (lo probamos motor por motor, el delantero der venia al reves: 27 adelante, 25 atras)
//                             del.izq  del.der  tras.izq  tras.der
const uint8_t pinMotorA[4] = {   17,      27,      16,       23  };
const uint8_t pinMotorB[4] = {   19,      25,      18,       26  };
bool motorAlReves[4] = {false, false, false, false};   // si uno gira al reves, ponerlo en true
const char* nombreMotor[4] = {"delantero izq", "delantero der", "trasero izq", "trasero der"};

// ultrasonicos: 0 = enfrente, 1 = izquierda, 2 = derecha
const uint8_t trigUltra[3] = {4, 5, 33};
const uint8_t echoUltra[3] = {34, 35, 36};
#define ULTRA_FRENTE 0
#define ULTRA_IZQ    1
#define ULTRA_DER    2

const uint8_t lineaIzq = 13, lineaDer = 14, lineaCentro = 39;   // TCRT5000
const uint8_t pinServo = 32;
const int8_t  pinIRPelota = 15;   // revisar que si sea este!!
const uint8_t pinSDA = 21, pinSCL = 22;

// --------------------------- AJUSTES ---------------------------------
const float CASILLA = 30.0;   // cm, una unidad de la pista

// del centro del robot a la cara de cada ultrasonico (frente, izq, der)
float offsetUltra[3] = {8.0, 10.0, 10.0};

// lo que le restamos a cada lectura para que salga bien (lo medimos nosotros)
float ajusteUltra[3] = {3.0, 2.7, 2.7};   // frente, izq, der

// velocidades (pwm de 0 a 255)
int velNormal    = 160;   // con esta se calibra el tiempo por cm
int velLenta     = 135;
int velMuyLenta  = 120;   // para acercarse a la pelota (mas bajo y algunos motores no arrancan)
int velSeccion2  = 135;
int velGiro      = 160;
int pwmMinGiro   = 110;
int pwmMuerto    = 70;    // abajo de esto los motores casi no se mueven
const int frecuenciaPWM = 1000;   // la misma que nos funciono en la pista A

// cuanto tarda en avanzar 1 cm a velNormal (se mide con "mov")
float msPorCmAdelante = 30.0;
float msPorCmLado     = 38.0;

// giroscopio
bool  giroInvertido = false;
float kpRumbo = 3.0;      // que tan fuerte corrige mientras avanza
float giroMax = 40.0;     // si es muy grande, una llanta casi se para al corregir
float kpGiro  = 2.2;
float toleranciaGiro = 2.0;   // grados

// garra: 90 es el primer movimiento de la prueba del servo (90, 60, 120)
int servoCerrado = 90;
int servoAbierto = 120;   // revisar si abre con 120 o con 60

// sensores digitales
int  nivelBlanco = LOW;      // lo que da el TCRT cuando ve blanco
bool irDaLowConPelota = true;

// seccion 1
float minAbierto = 20.0;     // si el ultrasonico lee mas que esto, ese lado esta abierto
float maxAbierto = 75.0;
float maxAvancePelota = 30.0;
float extraAlAgarrar = 2.0;
float correccionMin = 1.5;   // errores mas chicos los ignoramos
float correccionMax = 8.0;   // mas grandes ya no le creemos al ultrasonico
float extraCheckpoint = 12.0;   // para quedar mas del 50% adentro

// seccion 2
float ignorarAlInicio = 15.0;   // la rayita blanca que separa el checkpoint del verde
float buscarRojoDesde = 105.0;
float limiteSeccion2  = 200.0;
float retrocesoLinea  = 4.0;
float pasoLateral     = 4.0;
float avancePrueba    = 8.0;
float margenHueco     = 4.0;
float paredMuyCerca   = 4.0;
int   intentosLinea   = 20;

// seccion 3
int maxPasosSeccion3 = 16;

// arranque
uint32_t tiempoEspera = 3000;
float distanciaMano = 5.0;

// --------------------------- VARIABLES -------------------------------
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
Adafruit_TCS34725 sensorColor(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
Preferences memoria;
bool hayOled = false, hayColor = false, hayGiro = false;

// direcciones de la pista
#define NORTE 0
#define ESTE  1
#define SUR   2
#define OESTE 3
// direcciones respecto al robot
#define ADELANTE  0
#define DERECHA   1
#define ATRAS     2
#define IZQUIERDA 3

int   haciaDondeVe = NORTE;   // para donde ve el robot
float angulo = 0;             // grados, positivo = vuelta a la derecha
float offsetGiro = 0;
uint32_t ultimaLecturaGiro = 0;
uint32_t ultimaVezPelota = 0;
int checkpointGuardado = 0;
int intentosRecuperar = 0;   // para que no se quede atorado tratando de agarrar la pelota

enum Color { BLANCO = 0, VERDE, ROJO, CIAN, AMARILLO, NARANJA, ROSA, FIN, NUM_COLORES };
const char* nombreColor[NUM_COLORES] = {"blanco", "verde", "rojo", "cian", "amarillo", "naranja", "rosa", "fin"};

struct MuestraColor { float r, g, b, brillo; };
// valores a ojo, hay que calibrarlos con "cal <color>"
MuestraColor coloresGuardados[NUM_COLORES] = {
  {0.36, 0.34, 0.30, 8.0},  // blanco
  {0.25, 0.45, 0.30, 6.8},  // verde
  {0.62, 0.20, 0.20, 7.0},  // rojo
  {0.20, 0.40, 0.42, 7.6},  // cian
  {0.45, 0.40, 0.17, 7.8},  // amarillo
  {0.55, 0.28, 0.17, 7.4},  // naranja
  {0.50, 0.22, 0.30, 7.4},  // rosa
  {0.32, 0.48, 0.20, 7.6},  // fin
};
float pesoBrillo = 0.01;
float distanciaMaxColor = 0.02;

struct LecturaColor { float r, g, b, brillo; uint16_t total; };

typedef bool (*CondicionParo)();

// las declaramos aqui porque tienen valores por defecto
void  moverMotor(int i, int vel);
void  mover(float adelante, float lado, float giro);
void  frenar();
void  esperar(uint32_t ms);
void  actualizarAngulo();
float diferenciaAngulo(float a, float b);
float avanzarHacia(int direccion, float cm, int vel, CondicionParo parar = nullptr);
float avanzar(int relativa, float cm, int vel, CondicionParo parar = nullptr);
void  girarA(int direccion);
float distancia(int ultra);
float distanciaHacia(int direccion);
bool  hayPelota();
bool  vioLinea();
LecturaColor leerColor();
int   queColor(const LecturaColor& lectura, float* qtanCerca = nullptr);
int   colorSeguro(int veces = 5);
void  mostrar(const String& titulo, const String& linea1 = "", const String& linea2 = "");
bool  cuidarPelota();

// ---------------------------- MOTORES --------------------------------
// el core 3.x cambio como se usa el pwm, por eso esto
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define CONECTAR_PWM(pin, canal, frec, bits) ledcAttach((pin), (frec), (bits))
  #define ESCRIBIR_PWM(pin, canal, valor)      ledcWrite((pin), (valor))
#else
  #define CONECTAR_PWM(pin, canal, frec, bits) do { ledcSetup((canal), (frec), (bits)); ledcAttachPin((pin), (canal)); } while (0)
  #define ESCRIBIR_PWM(pin, canal, valor)      ledcWrite((canal), (valor))
#endif
const int canalServo = 8;

void iniciarMotores() {
  for (int i = 0; i < 4; i++) {
    CONECTAR_PWM(pinMotorA[i], 2 * i, frecuenciaPWM, 8);
    CONECTAR_PWM(pinMotorB[i], 2 * i + 1, frecuenciaPWM, 8);
  }
  for (int i = 0; i < 4; i++) moverMotor(i, 0);
}

// vel va de -255 a 255
void moverMotor(int i, int vel) {
  if (motorAlReves[i]) vel = -vel;
  vel = constrain(vel, -255, 255);
  if (vel > 0) {
    ESCRIBIR_PWM(pinMotorA[i], 2 * i, vel);
    ESCRIBIR_PWM(pinMotorB[i], 2 * i + 1, 0);
  } else if (vel < 0) {
    ESCRIBIR_PWM(pinMotorA[i], 2 * i, 0);
    ESCRIBIR_PWM(pinMotorB[i], 2 * i + 1, -vel);
  } else {
    ESCRIBIR_PWM(pinMotorA[i], 2 * i, 0);
    ESCRIBIR_PWM(pinMotorB[i], 2 * i + 1, 0);
  }
}

// formula de las mecanum: adelante, lado (+ = derecha) y giro (+ = derecha)
void mover(float adelante, float lado, float giro) {
  float di = adelante + lado + giro;
  float dd = adelante - lado - giro;
  float ti = adelante - lado + giro;
  float td = adelante + lado - giro;
  float mayor = max(max(fabsf(di), fabsf(dd)), max(fabsf(ti), fabsf(td)));
  if (mayor > 255) {   // si alguno se pasa, bajamos todos parejo
    float k = 255 / mayor;
    di *= k; dd *= k; ti *= k; td *= k;
  }
  moverMotor(0, di);
  moverMotor(1, dd);
  moverMotor(2, ti);
  moverMotor(3, td);
}

void frenar() {
  // en el DRV8833 las dos entradas en alto = freno
  for (int i = 0; i < 4; i++) {
    ESCRIBIR_PWM(pinMotorA[i], 2 * i, 255);
    ESCRIBIR_PWM(pinMotorB[i], 2 * i + 1, 255);
  }
  esperar(70);
  for (int i = 0; i < 4; i++) moverMotor(i, 0);
}

// ----------------------------- GARRA ---------------------------------
void iniciarServo() { CONECTAR_PWM(pinServo, canalServo, 50, 16); }

void moverServo(int grados) {
  grados = constrain(grados, 0, 180);
  // mismo rango que en nuestro codigo de pruebas (servo.attach(pin, 1000, 2000))
  uint32_t pulso = map(grados, 0, 180, 1000, 2000);   // microsegundos
  uint32_t duty = (uint32_t)((uint64_t)pulso * 65535ULL / 20000ULL);
  ESCRIBIR_PWM(pinServo, canalServo, duty);
}
void abrirGarra()  { moverServo(servoAbierto); }
void cerrarGarra() { moverServo(servoCerrado); }

// -------------------------- GIROSCOPIO -------------------------------
const uint8_t dirMPU = 0x68;

void escribirMPU(uint8_t registro, uint8_t valor) {
  Wire.beginTransmission(dirMPU);
  Wire.write(registro);
  Wire.write(valor);
  Wire.endTransmission();
}

void iniciarGiro() {
  Wire.beginTransmission(dirMPU);
  Wire.write(0x6B);
  Wire.write(0);   // despertarlo
  hayGiro = (Wire.endTransmission() == 0);
  if (!hayGiro) return;
  escribirMPU(0x1A, 0x03);   // filtro
  escribirMPU(0x1B, 0x08);   // +-500 grados/s
}

int16_t leerGiroZ() {
  Wire.beginTransmission(dirMPU);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom((int)dirMPU, 2);
  int16_t alto = Wire.read();
  int16_t bajo = Wire.read();
  return (int16_t)((alto << 8) | (bajo & 0xFF));
}

// promedia un rato quieto para saber cuanto "se mueve" el sensor solo
void calibrarGiro() {
  if (!hayGiro) return;
  double suma = 0;
  for (int i = 0; i < 600; i++) {
    suma += leerGiroZ();
    delay(2);
  }
  offsetGiro = suma / 600;
  ultimaLecturaGiro = micros();
}

void actualizarAngulo() {
  if (!hayGiro) return;
  uint32_t ahora = micros();
  float dt = (ahora - ultimaLecturaGiro) / 1000000.0;
  ultimaLecturaGiro = ahora;
  if (dt <= 0 || dt > 0.2) return;
  float vel = (leerGiroZ() - offsetGiro) / 65.5;   // grados por segundo
  float aLaDerecha = giroInvertido ? vel : -vel;
  if (fabsf(aLaDerecha) < 0.3) aLaDerecha = 0;    // quitar ruidito
  angulo += aLaDerecha * dt;
}

// un delay que no deja de leer el giroscopio
void esperar(uint32_t ms) {
  uint32_t inicio = millis();
  while (millis() - inicio < ms) {
    actualizarAngulo();
    delay(2);
  }
}

// diferencia entre dos angulos, siempre entre -180 y 180
float diferenciaAngulo(float a, float b) {
  return fmodf(a - b + 540.0, 360.0) - 180.0;
}

float correccionRumbo() {
  float error = diferenciaAngulo(haciaDondeVe * 90.0, angulo);
  return constrain(kpRumbo * error, -giroMax, giroMax);
}

// --------------------------- SENSORES --------------------------------
float distanciaUnaVez(int u) {
  digitalWrite(trigUltra[u], LOW);
  delayMicroseconds(3);
  digitalWrite(trigUltra[u], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigUltra[u], LOW);
  unsigned long tiempo = pulseIn(echoUltra[u], HIGH, 25000UL);
  if (tiempo == 0) return 999;   // no regreso el eco
  float cm = tiempo * 0.01715 - ajusteUltra[u];
  return cm < 0 ? 0 : cm;
}

// tomamos 3 y nos quedamos con la de en medio por si sale una loca
float distancia(int u) {
  float m[3];
  for (int k = 0; k < 3; k++) {
    m[k] = distanciaUnaVez(u);
    esperar(8);
  }
  if (m[0] > m[1]) { float t = m[0]; m[0] = m[1]; m[1] = t; }
  if (m[1] > m[2]) { float t = m[1]; m[1] = m[2]; m[2] = t; }
  if (m[0] > m[1]) { float t = m[0]; m[0] = m[1]; m[1] = t; }
  return m[1];
}

// convierte una direccion de la pista a una del robot
int relativaA(int direccion) { return ((direccion - haciaDondeVe) % 4 + 4) % 4; }

int ultraQueVe(int relativa) {
  if (relativa == ADELANTE)  return ULTRA_FRENTE;
  if (relativa == DERECHA)   return ULTRA_DER;
  if (relativa == IZQUIERDA) return ULTRA_IZQ;
  return -1;   // atras no tenemos
}

// lee el ultrasonico que apunta hacia esa direccion de la pista
float distanciaHacia(int direccion) {
  int u = ultraQueVe(relativaA(direccion));
  if (u < 0) return -1;
  return distancia(u);
}

float offsetHacia(int direccion) {
  int u = ultraQueVe(relativaA(direccion));
  return u < 0 ? 0 : offsetUltra[u];
}

bool esBlanco(uint8_t pin) { return digitalRead(pin) == nivelBlanco; }
bool vioLinea() { return esBlanco(lineaIzq) || esBlanco(lineaCentro) || esBlanco(lineaDer); }

bool hayPelota() {
  if (pinIRPelota < 0) return true;
  bool la_ve = digitalRead(pinIRPelota) == (irDaLowConPelota ? LOW : HIGH);
  if (la_ve) ultimaVezPelota = millis();
  return la_ve;
}
// le damos un cuarto de segundo antes de decir que se perdio
bool perdimosPelota() {
  hayPelota();
  return millis() - ultimaVezPelota > 250;
}

// ---- color ----
LecturaColor leerColor() {
  LecturaColor l = {0, 0, 0, 0, 0};
  if (!hayColor) return l;
  uint16_t r, g, b, c;
  sensorColor.getRawData(&r, &g, &b, &c);
  float total = c > 0 ? c : 1;
  l.total = c;
  l.r = r / total;   // normalizado para que no importe tanto la luz
  l.g = g / total;
  l.b = b / total;
  l.brillo = logf(total);
  return l;
}

// busca cual color guardado se parece mas a la lectura
int queColor(const LecturaColor& lectura, float* qtanCerca) {
  if (!hayColor) return -1;
  int mejor = -1;
  float menor = 1e9;
  for (int k = 0; k < NUM_COLORES; k++) {
    float dr = lectura.r - coloresGuardados[k].r;
    float dg = lectura.g - coloresGuardados[k].g;
    float db = lectura.b - coloresGuardados[k].b;
    float dl = lectura.brillo - coloresGuardados[k].brillo;
    float d = dr * dr + dg * dg + db * db + pesoBrillo * dl * dl;
    if (d < menor) { menor = d; mejor = k; }
  }
  if (qtanCerca) *qtanCerca = menor;
  if (menor > distanciaMaxColor) return -1;   // no se parece a ninguno
  return mejor;
}

// lee varias veces y se queda con el que gane por mayoria
int colorSeguro(int veces) {
  int votos[NUM_COLORES] = {0};
  for (int i = 0; i < veces; i++) {
    int c = queColor(leerColor());
    if (c >= 0) votos[c]++;
    actualizarAngulo();
  }
  int ganador = -1, maxVotos = 0;
  for (int k = 0; k < NUM_COLORES; k++) {
    if (votos[k] > maxVotos) { maxVotos = votos[k]; ganador = k; }
  }
  if (maxVotos * 2 > veces) return ganador;
  return -1;
}

bool esVerde(int c) { return c == VERDE || c == FIN; }
bool pararEnRojo()   { return queColor(leerColor()) == ROJO; }
bool pararConPelota(){ return hayPelota(); }
bool pararEnLinea()  { return vioLinea(); }

// ---- memoria (para no perder los colores ni el checkpoint al apagar) ----
void cargarMemoria() {
  memoria.begin("pistaB", true);
  if (memoria.getBytesLength("colores") == sizeof(coloresGuardados))
    memoria.getBytes("colores", coloresGuardados, sizeof(coloresGuardados));
  checkpointGuardado = memoria.getInt("cp", 0);
  memoria.end();
}

void guardarColores() {
  memoria.begin("pistaB", false);
  memoria.putBytes("colores", coloresGuardados, sizeof(coloresGuardados));
  memoria.end();
}

void guardarCheckpoint(int cp) {
  checkpointGuardado = cp;
  memoria.begin("pistaB", false);
  memoria.putInt("cp", cp);
  memoria.end();
  Serial.printf("checkpoint guardado: %d\n", cp);
}

// ---- pantalla ----
void mostrar(const String& titulo, const String& linea1, const String& linea2) {
  Serial.println(titulo + " | " + linea1 + " | " + linea2);
  if (!hayOled) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(0, 0);
  oled.println(titulo);
  oled.setTextSize(1);
  oled.setCursor(0, 28);
  oled.println(linea1);
  oled.setCursor(0, 44);
  oled.println(linea2);
  oled.display();
}

// -------------------------- MOVIMIENTO -------------------------------
// cm por milisegundo segun la velocidad (aprox, suponiendo que es lineal)
float cmPorMs(int vel, bool deLado) {
  float base = 1.0 / (deLado ? msPorCmLado : msPorCmAdelante);
  float factor = (float)(vel - pwmMuerto) / (velNormal - pwmMuerto);
  if (factor < 0.05) factor = 0.05;
  return base * factor;
}

// mueve el robot hacia una direccion de la pista sin girarlo (gracias a
// las mecanum) y mantiene el rumbo con el giroscopio. regresa cuanto avanzo
float avanzarHacia(int direccion, float cm, int vel, CondicionParo parar) {
  if (cm <= 0) return 0;
  int rel = relativaA(direccion);
  float adelante = 0, lado = 0;
  if (rel == ADELANTE) adelante = vel;
  else if (rel == ATRAS) adelante = -vel;
  else if (rel == DERECHA) lado = vel;
  else lado = -vel;

  float rapidez = cmPorMs(vel, rel == DERECHA || rel == IZQUIERDA);
  float recorrido = 0;
  uint32_t inicio = millis(), antes = inicio;
  actualizarAngulo();

  while (recorrido < cm) {
    actualizarAngulo();
    mover(adelante, lado, correccionRumbo());
    uint32_t ahora = millis();
    recorrido += (ahora - antes) * rapidez;
    antes = ahora;
    if (parar && parar()) break;
    if (ahora - inicio > 15000) break;   // por si algo sale mal
    delay(4);
  }
  frenar();
  return recorrido;
}

// igual pero la direccion es respecto al robot (adelante, derecha...)
float avanzar(int relativa, float cm, int vel, CondicionParo parar) {
  return avanzarHacia((haciaDondeVe + relativa) % 4, cm, vel, parar);
}

void girarA(int direccion) {
  haciaDondeVe = ((direccion % 4) + 4) % 4;
  float objetivo = haciaDondeVe * 90.0;
  uint32_t inicio = millis();
  int vecesBien = 0;
  while (millis() - inicio < 4000) {
    actualizarAngulo();
    float error = diferenciaAngulo(objetivo, angulo);
    if (fabsf(error) < toleranciaGiro) {
      mover(0, 0, 0);
      vecesBien++;
      if (vecesBien > 10) break;   // que se quede quieto un ratito antes de salir
    } else {
      vecesBien = 0;
      float fuerza = constrain(fabsf(kpGiro * error), (float)pwmMinGiro, (float)velGiro);
      mover(0, 0, error > 0 ? fuerza : -fuerza);
    }
    delay(4);
  }
  frenar();
}

// compara lo que lee el ultrasonico con lo que deberia leer y se acomoda
// paredEsperada = cm del centro del robot a la pared (-1 si no aplica)
bool acomodarCon(int direccion, float paredEsperada) {
  if (paredEsperada < 0) return false;
  float leido = distanciaHacia(direccion);
  if (leido < 0 || leido > 300) return false;
  float error = leido - (paredEsperada - offsetHacia(direccion));
  if (fabsf(error) < correccionMin) return true;    // ya estamos bien
  if (fabsf(error) > correccionMax) return false;   // no le creemos
  if (error > 0) avanzarHacia(direccion, error, velLenta);
  else           avanzarHacia((direccion + 2) % 4, -error, velLenta);
  return true;
}

// ---- pelota ----
// abre, avanza hasta que el IR la ve, y cierra. regresa cuanto avanzo
float agarrarPelota(float maxCm) {
  abrirGarra();
  esperar(350);
  float avance = avanzar(ADELANTE, maxCm, velMuyLenta, pararConPelota);
  if (hayPelota()) avance += avanzar(ADELANTE, extraAlAgarrar, velMuyLenta);
  cerrarGarra();
  esperar(450);

  if (!hayPelota()) {   // otro intentito
    abrirGarra();
    esperar(300);
    avance += avanzar(ADELANTE, 5, velMuyLenta, pararConPelota);
    cerrarGarra();
    esperar(450);
  }
  Serial.printf("pelota: %s (avanzo %.1f cm)\n", hayPelota() ? "si" : "no", avance);
  return avance;
}

// si se nos fue la pelota intenta agarrarla otra vez enfrente
bool cuidarPelota() {
  if (pinIRPelota < 0 || !perdimosPelota()) return true;
  // si ya lo intento varias veces, seguro el IR no la ve; mejor seguir avanzando
  if (intentosRecuperar >= 2) return false;
  intentosRecuperar++;
  frenar();
  mostrar("PELOTA", "se cayo, recuperando");
  abrirGarra();
  esperar(300);
  avanzar(ADELANTE, 8, velMuyLenta, pararConPelota);
  cerrarGarra();
  esperar(400);
  return hayPelota();
}

// cuando reiniciamos en un checkpoint el juez pone la pelota enfrente
void agarrarPelotaDeEnfrente() {
  mostrar("PELOTA", "agarrando la pelota", "de enfrente");
  agarrarPelota(10);
}

// ========================== SECCION 1 =================================
// cuadricula de 3x3: x es la columna (0 oeste a 2 este), y es la fila
// (0 sur a 2 norte). La pelota esta en (1,1) y INICIO abajo de (columnaInicio, 0)
int posX = 0, posY = 0, columnaInicio = 0;

// las 8 casillas alrededor del centro, en orden
const int vuelta[8][2] = {{0,0}, {0,1}, {0,2}, {1,2}, {2,2}, {2,1}, {2,0}, {1,0}};

// lados de la celda de la pelota: 0 sur, 1 oeste, 2 este, 3 norte
const int casillaParaRevisar[4][2] = {{1,0}, {0,1}, {2,1}, {1,2}};
const int haciaElCentro[4] = {NORTE, ESTE, OESTE, SUR};
const char* nombreLado[4] = {"sur", "oeste", "este", "norte"};

int lugarEnLaVuelta(int x, int y) {
  for (int i = 0; i < 8; i++)
    if (vuelta[i][0] == x && vuelta[i][1] == y) return i;
  return -1;
}

// cuanto deberia haber del centro del robot a la pared en esa direccion.
// -1 cuando en medio esta la celda de la pelota, el inicio o el checkpoint
float paredSeccion1(int x, int y, int dir) {
  if (dir == NORTE) {
    if (x == 1 && y == 0) return -1;
    return (2.5 - y) * CASILLA;
  }
  if (dir == SUR) {
    if (x == 1 && y == 2) return -1;
    if (y == 0 && x == columnaInicio) return -1;
    return (y + 0.5) * CASILLA;
  }
  if (dir == ESTE) {
    if (y == 1) return -1;
    return (2.5 - x) * CASILLA;
  }
  if (dir == OESTE) {
    if (y == 1 && x == 2) return -1;
    return (x + 0.5) * CASILLA;
  }
  return -1;
}

// se acomoda en la casilla usando las paredes de afuera
void acomodarseSeccion1() {
  if (!acomodarCon(OESTE, paredSeccion1(posX, posY, OESTE)))
    acomodarCon(ESTE, paredSeccion1(posX, posY, ESTE));
  if (!acomodarCon(NORTE, paredSeccion1(posX, posY, NORTE)))
    acomodarCon(SUR, paredSeccion1(posX, posY, SUR));
}

void moverUnaCasilla(int dir) {
  avanzarHacia(dir, CASILLA, velNormal);
  if (dir == NORTE) posY++;
  else if (dir == SUR) posY--;
  else if (dir == ESTE) posX++;
  else posX--;
  acomodarseSeccion1();
}

// le da la vuelta a la celda de la pelota por el camino mas corto
void irA(int x, int y) {
  int aqui = lugarEnLaVuelta(posX, posY);
  int destino = lugarEnLaVuelta(x, y);
  if (aqui < 0 || destino < 0) return;
  int paso = ((destino - aqui + 8) % 8 <= (aqui - destino + 8) % 8) ? 1 : -1;
  while (aqui != destino) {
    int sig = (aqui + paso + 8) % 8;
    int dx = vuelta[sig][0] - vuelta[aqui][0];
    int dy = vuelta[sig][1] - vuelta[aqui][1];
    int dir;
    if (dx > 0) dir = ESTE;
    else if (dx < 0) dir = OESTE;
    else if (dy > 0) dir = NORTE;
    else dir = SUR;
    moverUnaCasilla(dir);
    aqui = sig;
  }
}

// si ve lejos hacia el centro es que ese lado no tiene pared
bool ladoAbierto(int lado) {
  float d = distanciaHacia(haciaElCentro[lado]);
  Serial.printf("lado %s: %.1f cm\n", nombreLado[lado], d);
  return d >= minAbierto && d <= maxAbierto;
}

void seccion1() {
  mostrar("SECCION 1", "buscando la pelota");
  cerrarGarra();
  haciaDondeVe = NORTE;
  avanzarHacia(NORTE, CASILLA, velNormal);   // salir del inicio

  // con las paredes de los lados sacamos en que columna quedamos
  float izq = distanciaHacia(OESTE);
  float der = distanciaHacia(ESTE);
  float colIzq = (izq + offsetUltra[ULTRA_IZQ]) / CASILLA - 0.5;
  float colDer = 2.0 - ((der + offsetUltra[ULTRA_DER]) / CASILLA - 0.5);
  float col;
  if (izq < 300 && der < 300) col = (colIzq + colDer) / 2;
  else if (izq < 300) col = colIzq;
  else col = colDer;
  columnaInicio = constrain((int)roundf(col), 0, 2);
  posX = columnaInicio;
  posY = 0;
  Serial.printf("columna %d (izq %.1f, der %.1f)\n", columnaInicio, izq, der);
  acomodarseSeccion1();

  // primero el sur, luego el lado mas cercano. si ninguno, es el norte
  int orden[3] = {0, 1, 2};
  if (columnaInicio == 2) { orden[1] = 2; orden[2] = 1; }
  int entrada = -1;
  for (int k = 0; k < 3 && entrada < 0; k++) {
    int lado = orden[k];
    irA(casillaParaRevisar[lado][0], casillaParaRevisar[lado][1]);
    if (ladoAbierto(lado)) entrada = lado;
  }
  if (entrada < 0) entrada = 3;
  mostrar("SECCION 1", String("entrada por el ") + nombreLado[entrada]);

  irA(casillaParaRevisar[entrada][0], casillaParaRevisar[entrada][1]);
  girarA(haciaElCentro[entrada]);   // la garra hacia la pelota
  acomodarseSeccion1();
  float loQueEntro = agarrarPelota(maxAvancePelota);
  avanzar(ATRAS, loQueEntro, velLenta);   // salir de la celda con la pelota
  acomodarseSeccion1();

  // el checkpoint esta al este de la fila de en medio
  irA(2, 1);
  girarA(ESTE);
  mostrar("SECCION 1", "al checkpoint");
  avanzarHacia(ESTE, CASILLA + 15, velLenta, pararEnRojo);
  avanzarHacia(ESTE, extraCheckpoint, velLenta);
  guardarCheckpoint(1);
}

// ========================== SECCION 2 =================================
// cuando ve una linea blanca se hace para atras y se va de lado hasta
// encontrar el hueco. regresa cuanto avanzo neto hacia adelante
float esquivarLinea(bool izq, bool centro, bool der) {
  float neto = 0;
  int haciaDonde;
  if (izq && !der) haciaDonde = DERECHA;
  else if (der && !izq) haciaDonde = IZQUIERDA;
  else haciaDonde = (distancia(ULTRA_IZQ) > distancia(ULTRA_DER)) ? IZQUIERDA : DERECHA;   // hacia donde haya mas espacio
  Serial.printf("linea (izq %d centro %d der %d), me voy a la %s\n",
                izq, centro, der, haciaDonde == DERECHA ? "derecha" : "izquierda");

  for (int i = 0; i < intentosLinea; i++) {
    neto -= avanzar(ATRAS, retrocesoLinea, velLenta);

    // si ya llegamos a la pared, el hueco esta del otro lado
    float pared = distancia(haciaDonde == DERECHA ? ULTRA_DER : ULTRA_IZQ);
    if (pared < paredMuyCerca) haciaDonde = (haciaDonde == DERECHA) ? IZQUIERDA : DERECHA;

    avanzar(haciaDonde, pasoLateral, velLenta);
    neto += avanzar(ADELANTE, avancePrueba, velMuyLenta, pararEnLinea);
    if (!vioLinea()) {
      // ya paso, nos hacemos tantito mas para quedar centrados en el hueco
      avanzar(haciaDonde, margenHueco, velLenta);
      return neto;
    }
  }
  return neto;
}

void seccion2() {
  mostrar("SECCION 2", "esquivando lineas");
  haciaDondeVe = ESTE;
  float avanzado = 0;
  float rapidez = cmPorMs(velSeccion2, false);
  uint32_t antes = millis();

  while (true) {
    actualizarAngulo();
    if (avanzado >= buscarRojoDesde && pararEnRojo()) break;   // llegamos al checkpoint 2
    if (avanzado > limiteSeccion2) break;

    bool izq = esBlanco(lineaIzq), centro = esBlanco(lineaCentro), der = esBlanco(lineaDer);
    bool hayLinea = izq || centro || der;
    if (hayLinea && avanzado > ignorarAlInicio && avanzado < buscarRojoDesde) {
      frenar();
      avanzado += esquivarLinea(izq, centro, der);
      antes = millis();
      continue;
    }

    if (!cuidarPelota()) Serial.println("sin pelota, sigo de todos modos");
    mover(velSeccion2, 0, correccionRumbo());
    uint32_t ahora = millis();
    avanzado += (ahora - antes) * rapidez;
    antes = ahora;
    delay(3);
  }
  frenar();
  avanzarHacia(ESTE, extraCheckpoint, velLenta);
  guardarCheckpoint(2);
}

// ========================== SECCION 3 =================================
// cian = derecha, amarillo = izquierda, naranja = adelante (alejandose del
// checkpoint), rosa = atras. Todo respecto a la pista, por eso el robot
// no gira, solo se desliza para los lados
void acomodarseSeccion3(int x, int y, int entrada) {
  acomodarCon(entrada, (2.5 - x) * CASILLA);
  if (y >= 0) acomodarCon((entrada + 3) % 4, (1.5 + y) * CASILLA);   // pared izq
  else        acomodarCon((entrada + 1) % 4, (1.5 - y) * CASILLA);   // pared der
}

// si no reconoce el color se mueve poquito para todos lados a ver
int buscarColor() {
  const int direcciones[4] = {ADELANTE, ATRAS, DERECHA, IZQUIERDA};
  const float cuanto[4] = {5, 10, 5, 10};
  for (int i = 0; i < 4; i++) {
    avanzar(direcciones[i], cuanto[i], velLenta);
    int c = colorSeguro();
    if (c >= 0 && c != BLANCO) return c;
  }
  return -1;
}

void seccion3() {
  mostrar("SECCION 3", "siguiendo colores");
  int entrada = haciaDondeVe;
  int x = 0, y = 0;   // y positivo = a la derecha
  avanzar(ADELANTE, CASILLA, velNormal);   // del checkpoint a la primera casilla

  for (int paso = 0; paso < maxPasosSeccion3; paso++) {
    cuidarPelota();
    acomodarseSeccion3(x, y, entrada);
    int c = colorSeguro();
    if (c < 0 || c == BLANCO || c == ROJO) c = buscarColor();
    Serial.printf("casilla (%d,%d): %s\n", x, y, c >= 0 ? nombreColor[c] : "??");

    if (c >= 0 && esVerde(c)) {
      mostrar("LLEGAMOS!", hayPelota() ? "con pelota :)" : "sin pelota");
      guardarCheckpoint(0);
      return;
    }

    int hacia;
    if (c == NARANJA) hacia = ADELANTE;
    else if (c == CIAN) hacia = DERECHA;
    else if (c == ROSA) hacia = ATRAS;
    else if (c == AMARILLO) hacia = IZQUIERDA;
    else {
      mostrar("SECCION 3", "no se que color es");
      return;
    }

    mostrar("SECCION 3", String("color: ") + nombreColor[c]);
    avanzarHacia((entrada + hacia) % 4, CASILLA, velNormal);
    if (hacia == ADELANTE) x++;
    else if (hacia == ATRAS) x--;
    else if (hacia == DERECHA) y++;
    else y--;
  }
  mostrar("SECCION 3", "ya di muchos pasos");
}

// ========================== RECORRIDO =================================
void correrDesde(int seccion) {
  ultimaVezPelota = millis();
  if (seccion == 1) {
    haciaDondeVe = NORTE;
    angulo = 0;
    seccion1();
    seccion = 2;
  } else {
    // en un checkpoint el robot ya ve hacia la siguiente seccion
    haciaDondeVe = ESTE;
    angulo = 90;
    agarrarPelotaDeEnfrente();
  }
  if (seccion == 2) {
    seccion2();
    seccion = 3;
  }
  if (seccion == 3) seccion3();
  frenar();
}

// decide en que seccion empezar segun el color del piso
int seccionInicial() {
  int piso = colorSeguro(7);
  Serial.printf("piso: %s, ultimo checkpoint: %d\n", piso >= 0 ? nombreColor[piso] : "??", checkpointGuardado);
  if (piso >= 0 && esVerde(piso)) {   // estamos en el inicio
    guardarCheckpoint(0);
    return 1;
  }
  if (piso == ROJO) return checkpointGuardado >= 2 ? 3 : 2;
  // si no se reconocio el color le creemos a lo guardado
  if (checkpointGuardado == 0) return 1;
  return min(checkpointGuardado + 1, 3);
}

// ======================== MENU DE PRUEBAS =============================
bool tecleoAlgo() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    return true;
  }
  return false;
}

void ayuda() {
  Serial.println(F(
    "\n---- menu pista B ----\n"
    "m               probar cada motor\n"
    "mov f|a|d|i cm  moverse adelante/atras/derecha/izquierda\n"
    "giro grados     girar (ej. giro 90)\n"
    "g               ver el angulo del giroscopio\n"
    "u               ver los ultrasonicos\n"
    "l               ver los sensores de linea\n"
    "i               ver el IR de la pelota\n"
    "c               ver el sensor de color\n"
    "cal <color>     guardar un color (blanco verde rojo cian amarillo naranja rosa fin)\n"
    "s <angulo>      mover el servo / abrir / cerrar\n"
    "i2c             ver que hay conectado en I2C\n"
    "cp <0|1|2>      cambiar el checkpoint guardado\n"
    "run <1|2|3>     correr desde una seccion\n"
    "(con cualquier tecla se paran las lecturas)"));
}

void buscarI2C() {
  for (uint8_t d = 1; d < 127; d++) {
    Wire.beginTransmission(d);
    if (Wire.endTransmission() == 0) Serial.printf("  encontre algo en 0x%02X\n", d);
  }
  Serial.printf("oled: %d  color: %d  giro: %d\n", hayOled, hayColor, hayGiro);
}

void comando(String texto) {
  texto.trim();
  texto.toLowerCase();
  if (texto.length() == 0) return;
  int espacio = texto.indexOf(' ');
  String cmd = espacio < 0 ? texto : texto.substring(0, espacio);
  String arg = espacio < 0 ? "" : texto.substring(espacio + 1);
  arg.trim();

  if (cmd == "ayuda" || cmd == "help") {
    ayuda();
  }
  else if (cmd == "m") {
    for (int i = 0; i < 4; i++) {
      Serial.printf("motor %d: %s\n", i, nombreMotor[i]);
      moverMotor(i, velNormal);
      delay(1000);
      moverMotor(i, 0);
      delay(500);
    }
  }
  else if (cmd == "mov") {
    char letra = arg.length() ? arg[0] : 'f';
    float cm = arg.length() > 2 ? arg.substring(2).toFloat() : 30;
    int dir = ADELANTE;
    if (letra == 'd') dir = DERECHA;
    else if (letra == 'a') dir = ATRAS;
    else if (letra == 'i') dir = IZQUIERDA;
    haciaDondeVe = NORTE; angulo = 0; ultimaLecturaGiro = micros();
    avanzar(dir, cm, velNormal);
    Serial.printf("debio moverse %.1f cm, mide cuanto fue de verdad\n", cm);
  }
  else if (cmd == "giro") {
    haciaDondeVe = NORTE; angulo = 0; ultimaLecturaGiro = micros();
    girarA((int)roundf(arg.toFloat() / 90.0));
    Serial.printf("quedo en %.1f grados\n", angulo);
  }
  else if (cmd == "g") {
    angulo = 0; ultimaLecturaGiro = micros();
    while (!tecleoAlgo()) {
      esperar(200);
      Serial.printf("angulo %.1f\n", angulo);
    }
  }
  else if (cmd == "u") {
    while (!tecleoAlgo()) {
      Serial.printf("frente %.1f  izq %.1f  der %.1f\n", distancia(ULTRA_FRENTE), distancia(ULTRA_IZQ), distancia(ULTRA_DER));
      delay(150);
    }
  }
  else if (cmd == "l") {
    while (!tecleoAlgo()) {
      Serial.printf("izq %s  centro %s  der %s\n",
                    esBlanco(lineaIzq) ? "BLANCO" : "-",
                    esBlanco(lineaCentro) ? "BLANCO" : "-",
                    esBlanco(lineaDer) ? "BLANCO" : "-");
      delay(150);
    }
  }
  else if (cmd == "i") {
    while (!tecleoAlgo()) {
      Serial.println(hayPelota() ? "PELOTA" : "-");
      delay(150);
    }
  }
  else if (cmd == "c") {
    while (!tecleoAlgo()) {
      LecturaColor l = leerColor();
      float cerca;
      int c = queColor(l, &cerca);
      Serial.printf("r %.3f g %.3f b %.3f brillo %.2f (%u) -> %s  [%.4f]\n",
                    l.r, l.g, l.b, l.brillo, l.total, c >= 0 ? nombreColor[c] : "??", cerca);
      delay(150);
    }
  }
  else if (cmd == "cal") {
    int c = -1;
    for (int i = 0; i < NUM_COLORES; i++) if (arg == nombreColor[i]) c = i;
    if (c < 0) { Serial.println("ese color no lo conozco"); return; }
    MuestraColor suma = {0, 0, 0, 0};
    for (int i = 0; i < 20; i++) {
      LecturaColor l = leerColor();
      suma.r += l.r; suma.g += l.g; suma.b += l.b; suma.brillo += l.brillo;
    }
    coloresGuardados[c] = {suma.r / 20, suma.g / 20, suma.b / 20, suma.brillo / 20};
    guardarColores();
    Serial.printf("listo, %s = r %.3f g %.3f b %.3f brillo %.2f\n", nombreColor[c],
                  coloresGuardados[c].r, coloresGuardados[c].g, coloresGuardados[c].b, coloresGuardados[c].brillo);
  }
  else if (cmd == "s")      { moverServo(arg.toInt()); Serial.printf("servo en %d\n", (int)arg.toInt()); }
  else if (cmd == "abrir")  abrirGarra();
  else if (cmd == "cerrar") cerrarGarra();
  else if (cmd == "i2c")    buscarI2C();
  else if (cmd == "cp")     guardarCheckpoint(constrain((int)arg.toInt(), 0, 2));
  else if (cmd == "run") {
    int s = constrain((int)arg.toInt(), 1, 3);
    mostrar("PRUEBA", String("seccion ") + s);
    delay(1500);
    ultimaLecturaGiro = micros();
    correrDesde(s);
  }
  else {
    Serial.println("no entendi, escribe 'ayuda'");
  }
}

// ========================= SETUP Y LOOP ===============================
bool enMenu = false;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) {
    pinMode(trigUltra[i], OUTPUT);
    digitalWrite(trigUltra[i], LOW);
    pinMode(echoUltra[i], INPUT);
  }
  pinMode(lineaIzq, INPUT);
  pinMode(lineaDer, INPUT);
  pinMode(lineaCentro, INPUT);
  if (pinIRPelota >= 0) pinMode(pinIRPelota, INPUT_PULLUP);

  iniciarMotores();
  iniciarServo();
  cerrarGarra();

  Wire.begin(pinSDA, pinSCL);
  Wire.setClock(400000);
  hayOled = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  hayColor = sensorColor.begin();
  iniciarGiro();
  cargarMemoria();

  mostrar("NO MOVER", "calibrando giro...",
          String("oled ") + hayOled + " color " + hayColor + " giro " + hayGiro);
  delay(300);
  calibrarGiro();

  int seccion = seccionInicial();

  // cuenta regresiva. tapar el ultrasonico de enfrente 1 seg cambia la
  // seccion, y si escriben algo en el monitor serie se va al menu
  uint32_t inicio = millis(), manoDesde = 0;
  while (millis() - inicio < tiempoEspera) {
    if (Serial.available()) { enMenu = true; break; }

    if (distanciaUnaVez(ULTRA_FRENTE) < distanciaMano) {
      if (manoDesde == 0) manoDesde = millis();
      if (millis() - manoDesde > 1000) {
        seccion = seccion % 3 + 1;
        manoDesde = 0;
        inicio = millis();   // reiniciar la cuenta
        while (distanciaUnaVez(ULTRA_FRENTE) < distanciaMano) delay(50);   // esperar a que quite la mano
      }
    } else {
      manoDesde = 0;
    }

    int faltan = (tiempoEspera - (millis() - inicio)) / 1000 + 1;
    mostrar(String("SECCION ") + seccion, String("arranca en ") + faltan,
            hayColor ? "tapa enfrente = cambiar" : "NO HAY SENSOR DE COLOR");
    esperar(60);
  }

  if (enMenu) {
    mostrar("MENU", "monitor serie 115200");
    ayuda();
    return;
  }
  ultimaLecturaGiro = micros();
  correrDesde(seccion);
}

void loop() {
  if (Serial.available()) {
    enMenu = true;
    comando(Serial.readStringUntil('\n'));
  }
  actualizarAngulo();
  delay(5);
}
